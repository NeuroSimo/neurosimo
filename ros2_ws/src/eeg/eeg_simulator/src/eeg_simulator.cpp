#include <chrono>
#include <cmath>
#include <fstream>
#include <sstream>
#include <vector>
#include <memory>
#include <thread>
#include <string>
#include <filesystem>

#include "realtime_utils/utils.h"

#include <sys/inotify.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

#include <nlohmann/json.hpp>

#include "eeg_simulator.h"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace std::placeholders;

const std::string EEG_RAW_TOPIC = "/eeg/raw";
const std::string DATASET_LIST_TOPIC = "/eeg_simulator/dataset/list";

const std::string PROJECTS_DIRECTORY = "projects/";
const std::string EEG_SIMULATOR_DATA_SUBDIRECTORY = "eeg_simulator/";

const milliseconds SESSION_PUBLISHING_INTERVAL = 1ms;
const milliseconds SESSION_PUBLISHING_INTERVAL_TOLERANCE = 2ms;

/* Have a long queue to avoid dropping messages. */
const uint16_t EEG_QUEUE_LENGTH = 65535;


/* TODO: Simulating the EEG device to the level of sending UDP packets not implemented on the C++
     side yet. For a previous Python reference implementation, see commit c0afb515b. */
EegSimulator::EegSimulator() : Node("eeg_simulator") {
  callback_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  /* Create a single SubscriptionOptions object */
  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.callback_group = callback_group;

  /* Subscriber for EEG bridge healthcheck. */
  this->eeg_bridge_healthcheck_subscriber = create_subscription<system_interfaces::msg::Healthcheck>(
    "/eeg/healthcheck",
    10,
    std::bind(&EegSimulator::handle_eeg_bridge_healthcheck, this, std::placeholders::_1),
    subscription_options);

  /* Publisher for EEG simulator healthcheck. */
  this->healthcheck_publisher = this->create_publisher<system_interfaces::msg::Healthcheck>(
    "/eeg_simulator/healthcheck",
    10);

  /* Subscriber for active project. */
  auto qos_persist_latest = rclcpp::QoS(rclcpp::KeepLast(1))
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

  this->active_project_subscriber = create_subscription<std_msgs::msg::String>(
    "/projects/active",
    qos_persist_latest,
    std::bind(&EegSimulator::handle_set_active_project, this, std::placeholders::_1),
    subscription_options);

  /* Publisher for EEG datasets. */
  dataset_list_publisher = this->create_publisher<project_interfaces::msg::DatasetList>(
    DATASET_LIST_TOPIC,
    qos_persist_latest);

  /* Service for changing dataset. */
  this->set_dataset_service = this->create_service<project_interfaces::srv::SetDataset>(
    "/eeg_simulator/dataset/set",
    std::bind(&EegSimulator::handle_set_dataset, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    callback_group);

  /* Service for changing playback. */
  this->set_playback_service = this->create_service<std_srvs::srv::SetBool>(
    "/eeg_simulator/playback/set",
    std::bind(&EegSimulator::handle_set_playback, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    callback_group);

  /* Service for start time. */
  this->start_time_service = this->create_service<project_interfaces::srv::SetStartTime>(
    "/eeg_simulator/start_time/set",
    std::bind(&EegSimulator::handle_set_start_time, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    callback_group);

  /* Publisher for dataset. */
  this->dataset_publisher = this->create_publisher<std_msgs::msg::String>(
    "/eeg_simulator/dataset",
    qos_persist_latest);

  /* Publisher for playback. */
  this->playback_publisher = this->create_publisher<std_msgs::msg::Bool>(
    "/eeg_simulator/playback",
    qos_persist_latest);

  /* Publisher for start time. */
  this->start_time_publisher = this->create_publisher<std_msgs::msg::Float64>(
    "/eeg_simulator/start_time",
    qos_persist_latest);

  /* QOS for session */
  const auto DEADLINE_NS = std::chrono::nanoseconds(SESSION_PUBLISHING_INTERVAL + SESSION_PUBLISHING_INTERVAL_TOLERANCE);

  auto qos_session = rclcpp::QoS(rclcpp::KeepLast(1))
      .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
      .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
      .deadline(DEADLINE_NS)
      .lifespan(DEADLINE_NS);

  /* Subscriber for session.

     NB: It is crucial to not use the shared, re-entrant callback group for this subscriber, as the session
       messages are time-critical and using the shared callback group seems to hinder the performance. */
  this->session_subscriber = create_subscription<system_interfaces::msg::Session>(
    "/system/session",
    qos_session,
    std::bind(&EegSimulator::handle_session, this, std::placeholders::_1));

  /* Client for stopping session. */
  this->stop_session_client = this->create_client<system_interfaces::srv::StopSession>(
    "/system/session/stop",
    rmw_qos_profile_services_default,
    callback_group);

  /* Publisher for EEG samples.

     NB: It is crucial to not use the shared, re-entrant callback group for this publisher, as EEG sample messages
       are time-critical and using the shared callback group seems to hinder the performance. */
  eeg_publisher = this->create_publisher<eeg_interfaces::msg::Sample>(
    EEG_RAW_TOPIC,
    EEG_QUEUE_LENGTH);

  /* Initialize inotify. */
  this->inotify_descriptor = inotify_init();
  if (this->inotify_descriptor == -1) {
      RCLCPP_ERROR(this->get_logger(), "Error initializing inotify");
      exit(1);
  }

  /* Set the inotify descriptor to non-blocking. */
  int flags = fcntl(inotify_descriptor, F_GETFL, 0);
  fcntl(inotify_descriptor, F_SETFL, flags | O_NONBLOCK);

  /* Create a timer callback to poll inotify. */
  this->inotify_timer = this->create_wall_timer(std::chrono::milliseconds(100),
                                                std::bind(&EegSimulator::inotify_timer_callback, this),
                                                callback_group);

  /* Create a timer for publishing healthcheck. */
  this->healthcheck_publisher_timer = this->create_wall_timer(
      std::chrono::milliseconds(500),
      [this] { publish_healthcheck(); },
      callback_group);
}

EegSimulator::~EegSimulator() {
  inotify_rm_watch(inotify_descriptor, watch_descriptor);
  close(inotify_descriptor);
}

void EegSimulator::publish_healthcheck() {
  auto healthcheck = system_interfaces::msg::Healthcheck();

  if (this->eeg_bridge_available) {
    healthcheck.status.value = system_interfaces::msg::HealthcheckStatus::NOT_READY;
    healthcheck.status_message = "Not ready";
    healthcheck.actionable_message = "Turn off the EEG bridge to use the EEG simulator.";
  } else if (this->is_streaming) {
    healthcheck.status.value = system_interfaces::msg::HealthcheckStatus::READY;
    healthcheck.status_message = "Streaming...";
    healthcheck.actionable_message = "End the session to stop streaming.";
  } else if (this->eeg_simulator_state == EegSimulatorState::LOADING) {
    healthcheck.status.value = system_interfaces::msg::HealthcheckStatus::READY;
    healthcheck.status_message = "Loading...";
    healthcheck.actionable_message = "Wait until loading is finished.";
  } else if (this->eeg_simulator_state == EegSimulatorState::ERROR_LOADING) {
    // Healthcheck status is set to READY, as it is not a critical error.
    healthcheck.status.value = system_interfaces::msg::HealthcheckStatus::READY;
    healthcheck.status_message = this->error_message;
    healthcheck.actionable_message = "An error occurred while reading a file. Please check the logs.";
  } else {
    healthcheck.status.value = system_interfaces::msg::HealthcheckStatus::READY;
    healthcheck.status_message = "Ready";
    healthcheck.actionable_message = "Start a session to stream data.";
  }
  this->healthcheck_publisher->publish(healthcheck);
}

void EegSimulator::handle_eeg_bridge_healthcheck(const std::shared_ptr<system_interfaces::msg::Healthcheck> msg) {
  this->eeg_bridge_available = msg->status.value == system_interfaces::msg::HealthcheckStatus::READY;
  if (eeg_bridge_available) {
    this->set_playback(false);
    RCLCPP_INFO(this->get_logger(), "EEG simulator disabled because EEG bridge is available.");
  }
}

std::tuple<bool, int, double, bool> EegSimulator::get_dataset_info(const std::string& data_file_path) {
  std::ifstream data_file(data_file_path);
  int sampling_frequency = 0;
  double duration = 0.0;
  bool samples_dropped = false;
  bool success = true;

  if (!data_file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Error opening file: %s", data_file_path.c_str());

    success = false;
    return std::make_tuple(success, sampling_frequency, duration, samples_dropped);
  }

  std::string line;
  double first_timestamp, second_timestamp;

  /* Read the first timestamp. */
  if (std::getline(data_file, line)) {
    std::stringstream ss(line);
    ss >> first_timestamp;
  }

  /* Read the second timestamp to determine sampling frequency. */
  if (std::getline(data_file, line)) {
    std::stringstream ss(line);
    ss >> second_timestamp;

    /* Calculate and round the sampling frequency to the nearest integer. */
    sampling_frequency = static_cast<int>(std::round(1.0 / (second_timestamp - first_timestamp)));
  }

  double_t previous_timestamp = second_timestamp;
  double_t expected_difference = second_timestamp - first_timestamp;

  double_t internal_timestamp;

  /* Read until the last line to get the last timestamp and check for dropped samples. */
  while (std::getline(data_file, line)) {
    std::stringstream ss(line);
    ss >> internal_timestamp;

    /* Check if the current sample interval is equal to the expected interval (second - first). */
    if (std::abs((internal_timestamp - previous_timestamp) - expected_difference) > TOLERANCE_S) {
      RCLCPP_WARN(this->get_logger(), "Warning: Dropped samples found in dataset %s.", data_file_path.c_str());
      RCLCPP_WARN(this->get_logger(), "Previous timestamp: %.4f, current timestamp: %.4f, expected difference: %.4f",
        previous_timestamp,
        internal_timestamp,
        expected_difference);

      samples_dropped = true;
    }
    previous_timestamp = internal_timestamp;
  }

  /* Calculate the duration using the first and last timestamp. */
  duration = internal_timestamp - first_timestamp;

  return std::make_tuple(success, sampling_frequency, duration, samples_dropped);
}

std::vector<project_interfaces::msg::Dataset> EegSimulator::list_datasets(const std::string& path) {
  std::vector<project_interfaces::msg::Dataset> datasets;

  /* Check that the directory exists. */
  if (!std::filesystem::exists(path) || !std::filesystem::is_directory(path)) {
    RCLCPP_WARN(this->get_logger(), "Warning: Directory does not exist: %s.", path.c_str());
    return datasets;
  }

  /* List all .json files in the directory and fetch their attributes. */
  RCLCPP_INFO(this->get_logger(), "Reading datasets...");
  RCLCPP_INFO(this->get_logger(), " ");

  for (const auto &entry : std::filesystem::directory_iterator(path)) {
    if (entry.is_regular_file() && entry.path().extension() == ".json") {
      std::ifstream file(entry.path());
      nlohmann::json json_data;

      std::string filename = entry.path().filename().string();
      RCLCPP_INFO(this->get_logger(), "%s", filename.c_str());

      try {
        project_interfaces::msg::Dataset dataset_msg;
        file >> json_data;

        dataset_msg.json_filename = filename;

        /* Validate "name" field */
        if (json_data.contains("name") && json_data["name"].is_string()) {
          dataset_msg.name = json_data["name"];
        } else {
          RCLCPP_ERROR(this->get_logger(), "  • Mandatory field 'name' is missing or invalid");
          continue;
        }

        /* Validate "data_file" field. */
        if (json_data.contains("data_file") && json_data["data_file"].is_string()) {
          dataset_msg.data_filename = json_data["data_file"];
        } else {
          RCLCPP_ERROR(this->get_logger(), "  • Mandatory field 'data_file' is missing or invalid");
          continue;
        }

        /* Validate "channels" object and its fields "eeg" and "emg". */
        if (json_data.contains("channels") && json_data["channels"].is_object()) {
          if (json_data["channels"].contains("eeg") && json_data["channels"]["eeg"].is_number_integer()) {
            dataset_msg.num_of_eeg_channels = json_data["channels"]["eeg"];
          } else {
            RCLCPP_ERROR(this->get_logger(), "  • Mandatory field 'channels.eeg' is missing or invalid");
            continue;
          }

          if (json_data["channels"].contains("emg") && json_data["channels"]["emg"].is_number_integer()) {
            dataset_msg.num_of_emg_channels = json_data["channels"]["emg"];
          } else {
            RCLCPP_ERROR(this->get_logger(), "  • Mandatory field 'channels.emg' is missing or invalid");
            continue;
          }
        } else {
          RCLCPP_ERROR(this->get_logger(), "  • Mandatory object 'channels' is missing or invalid");
          continue;
        }



        /* Calculate the sampling frequency and duration from the data file. */
        std::string data_file_path = entry.path().parent_path().string() + "/" + dataset_msg.data_filename;
        auto [success, sampling_frequency, duration, samples_dropped] = get_dataset_info(data_file_path);

        if (!success) {
          RCLCPP_ERROR(this->get_logger(), "  • Error reading the dataset, skipping...");
          continue;
        }

        if (samples_dropped) {
          /* TODO: Should this fail harder? */
          RCLCPP_WARN(this->get_logger(), "  • Warning: Dropped samples found");
        }

        dataset_msg.sampling_frequency = sampling_frequency;
        dataset_msg.duration = duration;

        datasets.push_back(dataset_msg);

      } catch (const nlohmann::json::parse_error& ex) {
        RCLCPP_ERROR(this->get_logger(), "  • JSON parse error: %s", ex.what());
      }
      RCLCPP_INFO(this->get_logger(), " ");
    }
  }

  /* Sort datasets. */
  std::sort(datasets.begin(), datasets.end(), [](const auto& a, const auto& b) {
    return a.name < b.name;
  });

  return datasets;
}

void EegSimulator::update_dataset_list() {
  auto datasets = this->list_datasets(this->data_directory);

  /* Publish datasets. */
  project_interfaces::msg::DatasetList msg;
  msg.datasets = datasets;

  this->dataset_list_publisher->publish(msg);

  /* Store datasets internally. */
  for (const auto& dataset : datasets) {
    dataset_map[dataset.json_filename] = dataset;
  }

  /* XXX: Maybe not the cleanest way to pass on the default dataset to the caller or whoever needs it. */
  this->default_dataset_json = datasets.size() > 0 ? datasets[0].json_filename : "";
}

void EegSimulator::handle_set_active_project(const std::shared_ptr<std_msgs::msg::String> msg) {
  this->active_project = msg->data;

  std::ostringstream oss;
  oss << PROJECTS_DIRECTORY << this->active_project << "/" << EEG_SIMULATOR_DATA_SUBDIRECTORY;
  this->data_directory = oss.str();

  RCLCPP_INFO(this->get_logger(), "Active project set to: %s", this->active_project.c_str());

  /* Use a lock here to ensure that dataset is not reset by set_dataset while the list is still
     being updated. This can happen when the project manager first sets a new project using pub-sub
     and then immediately after that sets a new dataset with a service call. However, this is just
     a workaround for now - a more robust but architecturally more complex solution would be to
     ensure that the project manager does not set a new dataset until the list is updated,
     e.g., by acknowledging that the project has been reset. (The current solution with a lock
     still does not guarantee a correct order between setting the project and the dataset.) */
  {
    std::lock_guard<std::mutex> lock(dataset_mutex);
    update_dataset_list();
  }

  update_inotify_watch();
}

bool EegSimulator::set_dataset(std::string json_filename) {
  /* See comment above for why a lock is used here. */
  std::lock_guard<std::mutex> lock(dataset_mutex);

  if (dataset_map.find(json_filename) == dataset_map.end()) {
    RCLCPP_ERROR(this->get_logger(), "Dataset not found: %s.", json_filename.c_str());
    RCLCPP_ERROR(this->get_logger(), "Available datasets:");
    for (const auto& dataset : dataset_map) {
      RCLCPP_ERROR(this->get_logger(), "  • %s", dataset.first.c_str());
    }
    return false;
  }

  /* Update ROS state variable. */
  auto msg = std_msgs::msg::String();
  msg.data = json_filename;

  this->dataset_publisher->publish(msg);

  /* Update dataset internally. */
  this->dataset = dataset_map[json_filename];

  RCLCPP_INFO(this->get_logger(), "Dataset selected: %s", json_filename.c_str());

  /* Reset the simulator state. */
  this->eeg_simulator_state = EegSimulatorState::READY;

  /* If playback is set to true, re-initialize streaming. */
  if (this->playback) {
    initialize_streaming();
  }

  return true;
}

void EegSimulator::handle_set_dataset(
      const std::shared_ptr<project_interfaces::srv::SetDataset::Request> request,
      std::shared_ptr<project_interfaces::srv::SetDataset::Response> response) {

  response->success = set_dataset(request->filename);
}

void EegSimulator::set_playback(bool playback) {
  this->playback = playback;

  RCLCPP_INFO(this->get_logger(), "Playback %s.", this->playback ? "enabled" : "disabled");

  /* Update ROS state variable. */
  auto msg = std_msgs::msg::Bool();
  msg.data = this->playback;

  this->playback_publisher->publish(msg);

  /* If playback is set to true, initialize streaming. */
  if (this->playback) {
    initialize_streaming();
  }
}

void EegSimulator::handle_set_playback(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response) {

  set_playback(request->data);
  response->success = true;
  response->message = "";
}

void EegSimulator::handle_set_start_time(
      const std::shared_ptr<project_interfaces::srv::SetStartTime::Request> request,
      std::shared_ptr<project_interfaces::srv::SetStartTime::Response> response) {

  this->play_dataset_from = request->start_time;

  RCLCPP_INFO(this->get_logger(), "Start time set to: %.1f", this->play_dataset_from);

  /* Update ROS state variable. */
  auto msg = std_msgs::msg::Float64();
  msg.data = this->play_dataset_from;

  this->start_time_publisher->publish(msg);

  response->success = true;
}

void EegSimulator::initialize_streaming() {
  this->sampling_frequency = this->dataset.sampling_frequency;
  this->num_of_eeg_channels = this->dataset.num_of_eeg_channels;
  this->num_of_emg_channels = this->dataset.num_of_emg_channels;

  /* Initialize variables. */
  this->total_channels = this->num_of_eeg_channels + this->num_of_emg_channels;
  this->sampling_period = 1.0 / this->sampling_frequency;

  /* Open and read data file. */
  std::string data_filename = this->dataset.data_filename;
  std::string data_file_path = data_directory + data_filename;

  RCLCPP_INFO(this->get_logger(), " ");
  RCLCPP_INFO(this->get_logger(), "Initializing streaming of dataset: %s", data_filename.c_str());

  data_file.open(data_file_path, std::ios::in);

  if (!data_file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Error opening file: %s", data_file_path.c_str());
    return;
  }

  if (this->current_data_file_path != data_file_path) {
    this->eeg_simulator_state = EegSimulatorState::LOADING;

    dataset_buffer.clear();
    std::string line;
    uint32_t line_number = 0;
    while (std::getline(data_file, line)) {
      line_number++;
      std::stringstream ss(line);
      std::string number;
      std::vector<double_t> data;
      while (std::getline(ss, number, ',')) {
        double_t value;
        try {
          value = std::stod(number);
        } catch (const std::invalid_argument& e) {
          RCLCPP_ERROR(this->get_logger(), "Error converting string to double on line %d", line_number);
          RCLCPP_ERROR(this->get_logger(), "Line contents: %s", line.c_str());

          this->eeg_simulator_state = EegSimulatorState::ERROR_LOADING;
          this->error_message = "Error on line " + std::to_string(line_number);
          data_file.close();
          this->set_playback(false);

          return;
        }
        data.push_back(value);
      }
      dataset_buffer.push_back(data);
    }
  }
  this->current_index = 0;
  this->time_offset = 0.0;

  this->current_data_file_path = data_file_path;

  RCLCPP_INFO(this->get_logger(), "Finished loading data.");

  data_file.close();

  this->eeg_simulator_state = EegSimulatorState::READY;
}

void EegSimulator::handle_session(const std::shared_ptr<system_interfaces::msg::Session> msg) {
  this->session_time = msg->time;

  /* Return if session is not ongoing. */
  if (msg->state.value != system_interfaces::msg::SessionState::STARTED) {
    /* Re-initialize streaming already after stopping the previous session. */
    if (this->session_started) {
      RCLCPP_INFO(this->get_logger(), "Session stopped, stopping streaming.");
      this->initialize_streaming();
    }

    this->session_started = false;
    this->is_streaming = false;
    return;
  }

  /* Check if the session is started for the first time. */
  if (!this->session_started) {
    RCLCPP_INFO(this->get_logger(), "Session started, starting streaming.");
    this->session_started = true;
  }

  /* Return if the simulator is not set to playback dataset. */
  if (!playback) {
    this->is_streaming = false;
    return;
  }

  /* Return if the simulator is still loading the dataset. */
  if (this->eeg_simulator_state == EegSimulatorState::LOADING) {
    return;
  }
  this->is_streaming = true;

  /* Calculate the target time up to which we should publish samples */
  double_t target_time = this->play_dataset_from + this->session_time;

  /* Publish samples until target time */
  bool success = this->publish_until(target_time);
  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to publish samples until target time. Stopping streaming.");
    this->is_streaming = false;
    return;
  }
}

/* Publish a single sample at the given index. Returns true if the sample was published successfully. */
bool EegSimulator::publish_single_sample(size_t sample_index) {
  if (sample_index >= dataset_buffer.size()) {
    RCLCPP_ERROR(this->get_logger(), "Sample index %zu out of bounds (max: %zu)", sample_index, dataset_buffer.size() - 1);
    return false;
  }

  const std::vector<double_t>& data = dataset_buffer[sample_index];
  double_t sample_time = data.front();

  if (total_channels > static_cast<int>(data.size() - 1)) {
    RCLCPP_ERROR(this->get_logger(), "Total # of EEG and EMG channels (%d) exceeds # of channels in data (%zu)", total_channels, data.size() - 1);
    return false;
  }

  double_t time = sample_time + this->time_offset;

  /* Create the sample message. */
  eeg_interfaces::msg::Sample msg;

  /* Note: + 1 below is to skip the time column. */
  msg.eeg.insert(msg.eeg.end(), data.begin() + 1, data.begin() + 1 + num_of_eeg_channels);
  msg.emg.insert(msg.emg.end(), data.begin() + 1 + num_of_eeg_channels, data.end());

  msg.metadata.sampling_frequency = this->sampling_frequency;
  msg.metadata.num_of_eeg_channels = this->num_of_eeg_channels;
  msg.metadata.num_of_emg_channels = this->num_of_emg_channels;
  msg.metadata.is_simulation = true;
  msg.metadata.system_time = this->get_clock()->now();

  msg.time = time;

  /* Mark the sample as valid by default. The preprocessor can later mark it as invalid if needed. */
  msg.valid = true;

  eeg_publisher->publish(msg);

  RCLCPP_INFO_THROTTLE(this->get_logger(),
                       *this->get_clock(),
                       2000,
                       "Published sample at %.1f s.", time);

  return true;
}

/* Publish samples from start_index until (but not including) the first sample that is after until_time.
   Returns true if the samples were published successfully. */
bool EegSimulator::publish_until(double_t until_time) {
  if (dataset_buffer.empty()) {
    RCLCPP_ERROR(this->get_logger(), "No data available to publish");
    return false;
  }

  double_t sample_time = 0.0;
  size_t samples_published = 0;
  
  while (true) {
    const std::vector<double_t>& data = dataset_buffer[this->current_index];
    sample_time = data.front();
    double_t adjusted_time = sample_time + this->time_offset;

    // If we've reached a sample after until_time, stop
    if (adjusted_time > until_time) {
      break;
    }

    // Publish this sample
    bool success = publish_single_sample(this->current_index);
    if (!success) {
      RCLCPP_ERROR(this->get_logger(), "Failed to publish sample at index %zu", current_index);
      break;
    }

    samples_published++;

    // Move to next sample
    this->current_index = (this->current_index + 1) % dataset_buffer.size();

    // If we've wrapped around and LOOP is enabled, update time offset
    if (LOOP && this->current_index == 0) {
      RCLCPP_INFO(this->get_logger(), "Reached end of dataset, looping back to beginning.");
      this->time_offset = this->time_offset + dataset_buffer.back().front() + sampling_period;
    }

    // Safety check: prevent infinite loops if not in loop mode
    if (!LOOP && this->current_index == 0) {
      RCLCPP_INFO(this->get_logger(), "Reached end of dataset without loop mode. Stopping session.");

      // Call the stop session service
      auto request = std::make_shared<system_interfaces::srv::StopSession::Request>();
      auto result = stop_session_client->async_send_request(request);

      break;
    }

    // Prevent infinite loops in case of edge cases
    if (samples_published > dataset_buffer.size() * 2) {
      RCLCPP_WARN(this->get_logger(), "Published too many samples, breaking to prevent infinite loop.");
      break;
    }
  }

  return true;
}

/* Inotify functions */

void EegSimulator::update_inotify_watch() {
  /* Remove the old watch. */
  inotify_rm_watch(inotify_descriptor, watch_descriptor);

  /* Add a new watch. */
  watch_descriptor = inotify_add_watch(inotify_descriptor, this->data_directory.c_str(), IN_MODIFY | IN_CREATE | IN_DELETE | IN_MOVE);
  if (watch_descriptor == -1) {
      RCLCPP_ERROR(this->get_logger(), "Error adding watch for: %s", this->data_directory.c_str());
      return;
  }
}

void EegSimulator::inotify_timer_callback() {
  int length = read(inotify_descriptor, inotify_buffer, 1024);

  if (length < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      /* No events, return early. */
      return;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Error reading inotify");
      return;
    }
  }

  int i = 0;
  while (i < length) {
    struct inotify_event *event = (struct inotify_event *)&inotify_buffer[i];
    if (event->len) {
      std::string event_name = event->name;
      if (event->mask & (IN_CREATE | IN_DELETE | IN_MOVE)) {
        RCLCPP_INFO(this->get_logger(), "File '%s' created, deleted, or moved, updating dataset list.", event_name.c_str());
        this->update_dataset_list();
      }
    }
    i += sizeof(struct inotify_event) + event->len;
  }
}


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto logger = rclcpp::get_logger("eeg_simulator");

  realtime_utils::MemoryConfig mem_config;
  mem_config.enable_memory_optimization = true;
  mem_config.preallocate_size = 10 * 1024 * 1024; // 10 MB

  realtime_utils::SchedulingConfig sched_config;
  sched_config.enable_scheduling_optimization = true;
  sched_config.scheduling_policy = SCHED_RR;
  sched_config.priority_level = realtime_utils::PriorityLevel::HIGHEST_REALTIME;

  try {
    realtime_utils::initialize_scheduling(sched_config, logger);
    realtime_utils::initialize_memory(mem_config, logger);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(logger, "Initialization failed: %s", e.what());
    return -1;
  }

  auto node = std::make_shared<EegSimulator>();

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node);
  executor->spin();

  rclcpp::shutdown();
  return 0;
}
