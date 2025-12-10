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

const milliseconds STREAMING_INTERVAL = 1ms;
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

  /* Publisher for streamer state. */
  streamer_state_publisher = this->create_publisher<system_interfaces::msg::StreamerState>(
    "/eeg_simulator/state",
    qos_persist_latest);

  /* Service for changing dataset. */
  this->set_dataset_service = this->create_service<project_interfaces::srv::SetDataset>(
    "/eeg_simulator/dataset/set",
    std::bind(&EegSimulator::handle_set_dataset, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    callback_group);

  /* Service for start time. */
  this->start_time_service = this->create_service<project_interfaces::srv::SetStartTime>(
    "/eeg_simulator/start_time/set",
    std::bind(&EegSimulator::handle_set_start_time, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    callback_group);

  /* Services for starting/stopping the streamer. */
  this->start_streamer_service = this->create_service<std_srvs::srv::Trigger>(
    "/eeg_simulator/start",
    std::bind(&EegSimulator::handle_start_streamer, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    callback_group);

  this->stop_streamer_service = this->create_service<std_srvs::srv::Trigger>(
    "/eeg_simulator/stop",
    std::bind(&EegSimulator::handle_stop_streamer, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    callback_group);

  /* Publisher for dataset. */
  this->dataset_publisher = this->create_publisher<std_msgs::msg::String>(
    "/eeg_simulator/dataset",
    qos_persist_latest);

  /* Publisher for start time. */
  this->start_time_publisher = this->create_publisher<std_msgs::msg::Float64>(
    "/eeg_simulator/start_time",
    qos_persist_latest);

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

  /* Timer to drive streaming. */
  this->stream_timer = this->create_wall_timer(STREAMING_INTERVAL,
                                               std::bind(&EegSimulator::stream_timer_callback, this));

  /* Create a timer for publishing healthcheck. */
  this->healthcheck_publisher_timer = this->create_wall_timer(
      std::chrono::milliseconds(500),
      [this] { publish_healthcheck(); },
      callback_group);

  /* Publish initial streamer state. */
  publish_streamer_state();
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
  } else if (this->streamer_state == system_interfaces::msg::StreamerState::RUNNING) {
    healthcheck.status.value = system_interfaces::msg::HealthcheckStatus::READY;
    healthcheck.status_message = "Streaming...";
    healthcheck.actionable_message = "End streaming to stop data.";
  } else if (this->streamer_state == system_interfaces::msg::StreamerState::LOADING) {
    healthcheck.status.value = system_interfaces::msg::HealthcheckStatus::READY;
    healthcheck.status_message = "Loading...";
    healthcheck.actionable_message = "Wait until loading is finished.";
  } else if (this->streamer_state == system_interfaces::msg::StreamerState::ERROR) {
    // Healthcheck status is set to READY, as it is not a critical error.
    healthcheck.status.value = system_interfaces::msg::HealthcheckStatus::READY;
    healthcheck.status_message = this->error_message;
    healthcheck.actionable_message = "An error occurred while reading a file. Please check the logs.";
  } else {
    healthcheck.status.value = system_interfaces::msg::HealthcheckStatus::READY;
    healthcheck.status_message = "Ready";
    healthcheck.actionable_message = "Start streaming to stream data.";
  }
  this->healthcheck_publisher->publish(healthcheck);
}

void EegSimulator::publish_streamer_state() {
  system_interfaces::msg::StreamerState msg;
  msg.state = this->streamer_state;
  this->streamer_state_publisher->publish(msg);
}

void EegSimulator::handle_eeg_bridge_healthcheck(const std::shared_ptr<system_interfaces::msg::Healthcheck> msg) {
  this->eeg_bridge_available = msg->status.value == system_interfaces::msg::HealthcheckStatus::READY;
}

std::tuple<bool, size_t> EegSimulator::get_sample_count(const std::string& data_file_path) {
  std::ifstream data_file(data_file_path);

  if (!data_file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Error opening file: %s", data_file_path.c_str());
    return std::make_tuple(false, 0);
  }

  size_t line_count = 0;
  std::string line;
  while (std::getline(data_file, line)) {
    line_count++;
  }

  return std::make_tuple(true, line_count);
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

        /* Validate "session" object with sampling_frequency, num_eeg_channels, num_emg_channels. */
        if (json_data.contains("session") && json_data["session"].is_object()) {
          auto& session = json_data["session"];

          if (session.contains("sampling_frequency") && session["sampling_frequency"].is_number_integer()) {
            dataset_msg.sampling_frequency = session["sampling_frequency"];
          } else {
            RCLCPP_ERROR(this->get_logger(), "  • Mandatory field 'session.sampling_frequency' is missing or invalid");
            continue;
          }

          if (session.contains("num_eeg_channels") && session["num_eeg_channels"].is_number_integer()) {
            dataset_msg.num_eeg_channels = session["num_eeg_channels"];
          } else {
            RCLCPP_ERROR(this->get_logger(), "  • Mandatory field 'session.num_eeg_channels' is missing or invalid");
            continue;
          }

          if (session.contains("num_emg_channels") && session["num_emg_channels"].is_number_integer()) {
            dataset_msg.num_emg_channels = session["num_emg_channels"];
          } else {
            RCLCPP_ERROR(this->get_logger(), "  • Mandatory field 'session.num_emg_channels' is missing or invalid");
            continue;
          }
        } else {
          RCLCPP_ERROR(this->get_logger(), "  • Mandatory object 'session' is missing or invalid");
          continue;
        }

        /* Read optional "loop" field, defaulting to false. */
        if (json_data.contains("loop") && json_data["loop"].is_boolean()) {
          dataset_msg.loop = json_data["loop"];
        } else {
          dataset_msg.loop = false;
        }

        /* Read optional "pulse_times" array. */
        std::vector<double_t> parsed_pulse_times;
        if (json_data.contains("pulse_times") && json_data["pulse_times"].is_array()) {
          for (const auto& pulse_time : json_data["pulse_times"]) {
            if (pulse_time.is_number()) {
              parsed_pulse_times.push_back(pulse_time.get<double>());
            } else {
              RCLCPP_WARN(this->get_logger(), "  • Non-numeric value in pulse_times array, skipping");
            }
          }

          /* Warn if loop is enabled with pulse_times - this combination is not supported. */
          if (dataset_msg.loop && !parsed_pulse_times.empty()) {
            RCLCPP_WARN(this->get_logger(), "  • Warning: pulse_times with loop=true is not supported, pulse_times will be ignored");
            parsed_pulse_times.clear();
          } else if (!parsed_pulse_times.empty()) {
            RCLCPP_INFO(this->get_logger(), "  • Loaded %zu pulse times", parsed_pulse_times.size());
          }
        }
        dataset_msg.pulse_count = parsed_pulse_times.size();
        pulse_times_map[filename] = parsed_pulse_times;

        /* Get sample count from the data file and compute duration. */
        std::string data_file_path = entry.path().parent_path().string() + "/" + dataset_msg.data_filename;
        auto [success, sample_count] = get_sample_count(data_file_path);

        if (!success) {
          RCLCPP_ERROR(this->get_logger(), "  • Error reading the dataset, skipping...");
          continue;
        }

        dataset_msg.duration = static_cast<double>(sample_count) / dataset_msg.sampling_frequency;

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
  this->pulse_times = pulse_times_map[json_filename];

  RCLCPP_INFO(this->get_logger(), "Dataset selected: %s", json_filename.c_str());

  /* Reset the simulator state. */
  this->eeg_simulator_state = EegSimulatorState::READY;

  /* Re-initialize streaming. */
  initialize_streaming();

  return true;
}

void EegSimulator::handle_set_dataset(
      const std::shared_ptr<project_interfaces::srv::SetDataset::Request> request,
      std::shared_ptr<project_interfaces::srv::SetDataset::Response> response) {

  response->success = set_dataset(request->filename);
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

void EegSimulator::handle_start_streamer(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void) request;
  initialize_streaming();
  this->streaming_start_time = this->get_clock()->now().seconds();
  this->streaming_sample_index = 0;
  this->streamer_state = system_interfaces::msg::StreamerState::RUNNING;
  response->success = true;
  response->message = "EEG simulator streaming started.";
  publish_streamer_state();
}

void EegSimulator::handle_stop_streamer(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void) request;
  this->streamer_state = system_interfaces::msg::StreamerState::READY;
  this->streaming_start_time = UNSET_TIME;
  this->streaming_sample_index = 0;
  this->current_index = 0;
  this->current_pulse_index = 0;
  this->time_offset = 0.0;
  response->success = true;
  response->message = "EEG simulator streaming stopped.";
  publish_streamer_state();
}

void EegSimulator::initialize_streaming() {
  this->sampling_frequency = this->dataset.sampling_frequency;
  this->num_eeg_channels = this->dataset.num_eeg_channels;
  this->num_emg_channels = this->dataset.num_emg_channels;

  /* Initialize variables. */
  this->total_channels = this->num_eeg_channels + this->num_emg_channels;
  this->sampling_period = 1.0 / this->sampling_frequency;

  /* Open and read data file. */
  std::string data_filename = this->dataset.data_filename;
  std::string data_file_path = data_directory + data_filename;

  RCLCPP_INFO(this->get_logger(), " ");
  RCLCPP_INFO(this->get_logger(), "Initializing streaming of dataset: %s", data_filename.c_str());

  this->streamer_state = system_interfaces::msg::StreamerState::LOADING;
  publish_streamer_state();

  data_file.open(data_file_path, std::ios::in);

  if (!data_file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Error opening file: %s", data_file_path.c_str());
    return;
  }

  if (this->current_data_file_path != data_file_path) {
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

          this->streamer_state = system_interfaces::msg::StreamerState::ERROR;
          this->error_message = "Error on line " + std::to_string(line_number);
          publish_streamer_state();
          data_file.close();

          return;
        }
        data.push_back(value);
      }
      dataset_buffer.push_back(data);
    }
  }
  this->current_index = 0;
  this->current_pulse_index = 0;
  this->time_offset = 0.0;

  this->current_data_file_path = data_file_path;

  RCLCPP_INFO(this->get_logger(), "Finished loading data.");

  /* Log pulse times info if present. */
  if (!this->pulse_times.empty()) {
    RCLCPP_INFO(this->get_logger(), "Dataset has %zu pulse times. First pulse at %.4f s.",
                this->pulse_times.size(), this->pulse_times[0]);
  }

  data_file.close();

  this->streamer_state = system_interfaces::msg::StreamerState::READY;
  publish_streamer_state();
}

void EegSimulator::stream_timer_callback() {
  if (this->streamer_state != system_interfaces::msg::StreamerState::RUNNING) {
    return;
  }

  if (std::isnan(this->streaming_start_time)) {
    return;
  }

  double_t elapsed = this->get_clock()->now().seconds() - this->streaming_start_time;
  double_t target_time = this->play_dataset_from + elapsed;

  bool success = this->publish_until(target_time);
  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to publish samples until target time. Stopping streaming.");
    this->streamer_state = system_interfaces::msg::StreamerState::ERROR;
    publish_streamer_state();
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

  if (total_channels > static_cast<int>(data.size())) {
    RCLCPP_ERROR(this->get_logger(), "Total # of EEG and EMG channels (%d) exceeds # of channels in data (%zu)", total_channels, data.size());
    return false;
  }

  /* Compute sample time from index and sampling frequency. */
  double_t sample_time = sample_index * this->sampling_period;
  double_t time = sample_time + this->time_offset;

  /* Create the sample message. */
  eeg_interfaces::msg::Sample msg;

  msg.eeg.insert(msg.eeg.end(), data.begin(), data.begin() + num_eeg_channels);
  msg.emg.insert(msg.emg.end(), data.begin() + num_eeg_channels, data.end());

  msg.session.sampling_frequency = this->sampling_frequency;
  msg.session.num_eeg_channels = this->num_eeg_channels;
  msg.session.num_emg_channels = this->num_emg_channels;
  msg.session.is_simulation = true;
  msg.session.start_time = this->streaming_start_time;

  msg.time = time;

  /* Set the sample index. */
  msg.sample_index = this->streaming_sample_index;
  this->streaming_sample_index++;

  /* Mark the sample as valid by default. The preprocessor can later mark it as invalid if needed. */
  msg.valid = true;

  /* Check if a pulse should be delivered at this sample time. */
  msg.pulse_delivered = false;
  if (!this->pulse_times.empty() && this->current_pulse_index < this->pulse_times.size()) {
    double_t next_pulse_time = this->pulse_times[this->current_pulse_index];
    if (sample_time >= next_pulse_time) {
      msg.pulse_delivered = true;
      this->current_pulse_index++;

      RCLCPP_INFO(this->get_logger(), "Pulse delivered at %.4f s (pulse %zu/%zu).",
                  sample_time, this->current_pulse_index, this->pulse_times.size());

      /* Log next pulse time if there is one. */
      if (this->current_pulse_index < this->pulse_times.size()) {
        RCLCPP_INFO(this->get_logger(), "Next pulse at %.4f s.",
                    this->pulse_times[this->current_pulse_index]);
      } else {
        RCLCPP_INFO(this->get_logger(), "No more pulses in this dataset.");
      }
    }
  }

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

  size_t samples_published = 0;

  while (true) {
    /* Compute sample time from index and sampling frequency. */
    double_t sample_time = this->current_index * this->sampling_period;
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

    // If we've wrapped around and loop is enabled, update time offset
    if (this->dataset.loop && this->current_index == 0) {
      RCLCPP_INFO(this->get_logger(), "Reached end of dataset, looping back to beginning.");
      double_t dataset_duration = dataset_buffer.size() * this->sampling_period;
      this->time_offset = this->time_offset + dataset_duration;
    }

    // If not in loop mode and we've reached the end, stop streaming
    if (!this->dataset.loop && this->current_index == 0) {
      RCLCPP_INFO(this->get_logger(), "Reached end of dataset. Stopping session.");

      this->streamer_state = system_interfaces::msg::StreamerState::READY;
      publish_streamer_state();

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
