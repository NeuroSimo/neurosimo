#include <chrono>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <ctime>
#include <sstream>

#include "eeg_recorder.h"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace std::placeholders;

const std::string EEG_RAW_TOPIC = "/eeg/raw";
const std::string EEG_PREPROCESSED_TOPIC = "/eeg/preprocessed";

const std::string PROJECTS_DIRECTORY = "projects";
const std::string PREPROCESSED_EEG_SUBDIRECTORY = "/eeg_recorder/preprocessed";
const std::string RAW_EEG_SUBDIRECTORY = "/eeg_recorder/raw";

const milliseconds SESSION_PUBLISHING_INTERVAL = 1ms;
const milliseconds SESSION_PUBLISHING_INTERVAL_TOLERANCE = 2ms;

const milliseconds MAX_TIME_SINCE_LAST_SAMPLE = 1000ms;

/* Have a long queue to avoid dropping messages. */
const uint16_t EEG_QUEUE_LENGTH = 65535;

EegRecorder::EegRecorder() : Node("eeg_recorder") {
  /* Subscriber for active project. */
  auto qos_persist_latest = rclcpp::QoS(rclcpp::KeepLast(1))
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

  this->active_project_subscriber = create_subscription<std_msgs::msg::String>(
    "/projects/active",
    qos_persist_latest,
    std::bind(&EegRecorder::handle_set_active_project, this, _1));

  /* QOS for session */
  const auto DEADLINE_NS = std::chrono::nanoseconds(SESSION_PUBLISHING_INTERVAL + SESSION_PUBLISHING_INTERVAL_TOLERANCE);

  auto qos_session = rclcpp::QoS(rclcpp::KeepLast(1))
      .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
      .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
      .deadline(DEADLINE_NS)
      .lifespan(DEADLINE_NS);

  /* Subscriber for session. */
  this->session_subscriber = create_subscription<system_interfaces::msg::Session>(
    "/system/session",
    qos_session,
    std::bind(&EegRecorder::handle_session, this, _1));

  /* Subscriber for raw EEG. */
  eeg_raw_subscriber = this->create_subscription<eeg_interfaces::msg::Sample>(
    EEG_RAW_TOPIC,
    EEG_QUEUE_LENGTH,
    std::bind(&EegRecorder::handle_raw_eeg_sample, this, _1));

  /* Subscriber for preprocessed EEG. */
  eeg_preprocessed_subscriber = this->create_subscription<eeg_interfaces::msg::Sample>(
    EEG_PREPROCESSED_TOPIC,
    EEG_QUEUE_LENGTH,
    std::bind(&EegRecorder::handle_preprocessed_eeg_sample, this, _1));

  /* Service for changing record data. */
  this->set_record_data_service = this->create_service<std_srvs::srv::SetBool>(
    "/eeg_recorder/record_data/set",
    std::bind(&EegRecorder::handle_set_record_data, this, _1, _2));

  /* Publisher for status of record data. */
  this->record_data_publisher = this->create_publisher<std_msgs::msg::Bool>(
    "/eeg_recorder/record_data",
    qos_persist_latest);

  /* Timer for writing the buffers periodically. */
  this->timer = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&EegRecorder::write_buffers, this));
}

void EegRecorder::handle_set_active_project(const std::shared_ptr<std_msgs::msg::String> msg) {
  this->active_project = msg->data;

  this->raw_data_directory = PROJECTS_DIRECTORY + "/" + this->active_project + "/" + RAW_EEG_SUBDIRECTORY;
  this->preprocessed_data_directory = PROJECTS_DIRECTORY + "/" + this->active_project + "/" + PREPROCESSED_EEG_SUBDIRECTORY;

  RCLCPP_INFO(this->get_logger(), "Active project set to: %s.", this->active_project.c_str());
}

void EegRecorder::handle_session(const std::shared_ptr<system_interfaces::msg::Session> msg) {
  if (this->active_project.empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(),
                         *this->get_clock(), 5000,
                         "No active project set.");
    return;
  }

  auto session_state = msg->state.value;

  if (session_state == system_interfaces::msg::SessionState::STARTED &&
      current_session_state != system_interfaces::msg::SessionState::STARTED) {

    /* Re-initialize variables for checking dropped samples. */
    previous_sample_time_raw = UNSET_PREVIOUS_TIME;
    previous_sample_time_preprocessed = UNSET_PREVIOUS_TIME;

    /* Create the filenames for recording the data. */
    auto now = std::chrono::system_clock::now();
    auto now_time = std::chrono::system_clock::to_time_t(now);

    std::ostringstream timestamp_stream;
    timestamp_stream << std::put_time(std::localtime(&now_time), "%Y-%m-%d-%H-%M-%S");

    /* TODO: Experiment name and subject name are empty strings for now, make them configurable via UI when using real-time pipeline. */
    std::string is_simulation_str = this->is_simulation ? "_simulation" : "";
    if (experiment_name != "" && subject_name != "") {
      filename = timestamp_stream.str() + "_" + experiment_name + "_" + subject_name + is_simulation_str + ".csv";
    } else {
      filename = timestamp_stream.str() + is_simulation_str + ".csv";
    }
    raw_file_path = raw_data_directory + "/" + filename;
    preprocessed_file_path = preprocessed_data_directory + "/" + filename;

    RCLCPP_INFO(this->get_logger(), "Recording session into file: %s", filename.c_str());
  }
  current_session_state = session_state;
}

void EegRecorder::handle_set_record_data(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response) {

  /* XXX: Slight mismatch in naming; maybe it should be record_simulation_data throughout the code. */
  this->record_simulation_data= request->data;

  RCLCPP_INFO(this->get_logger(), "Storing simulation data %s.", this->record_simulation_data ? "enabled" : "disabled");

  /* Update ROS state variable. */
  auto msg = std_msgs::msg::Bool();
  msg.data = this->record_simulation_data;

  this->record_data_publisher->publish(msg);

  response->success = true;
  response->message = "";
}

void EegRecorder::write_preprocessed_buffer() {
  preprocessed_file << preprocessed_buffer.str();
  preprocessed_file.flush();

  /* Clear the buffer. */
  preprocessed_buffer.str("");
  preprocessed_buffer.clear();
}

void EegRecorder::write_raw_buffer() {
  raw_file << raw_buffer.str();
  raw_file.flush();

  /* Clear the buffer. */
  raw_buffer.str("");
  raw_buffer.clear();
}

void EegRecorder::write_buffers() {
  auto now = std::chrono::system_clock::now();

  if (this->raw_file.is_open()) {
    this->write_raw_buffer();

    /* Close file if samples have not arrived in a while. */
    auto time_since_last_raw_sample = std::chrono::duration_cast<std::chrono::milliseconds>(now - this->previous_clock_time_raw);
    if (time_since_last_raw_sample > MAX_TIME_SINCE_LAST_SAMPLE) {
      RCLCPP_INFO(this->get_logger(), "No raw samples have arrived in a while, closing the output file.");
      this->raw_file.close();
    }
  }
  if (this->preprocessed_file.is_open()) {
    this->write_preprocessed_buffer();

    /* Close file if samples have not arrived in a while. */
    auto time_since_last_preprocessed_sample = std::chrono::duration_cast<std::chrono::milliseconds>(now - this->previous_clock_time_preprocessed);
    if (time_since_last_preprocessed_sample > MAX_TIME_SINCE_LAST_SAMPLE) {
      RCLCPP_INFO(this->get_logger(), "No preprocessed samples have arrived in a while, closing the output file.");
      this->preprocessed_file.close();
    }
  }
}

void EegRecorder::update_eeg_info(const eeg_interfaces::msg::SampleMetadata& msg) {
  this->sampling_frequency = msg.sampling_frequency;
  this->num_eeg_channels = msg.num_eeg_channels;
  this->num_emg_channels = msg.num_emg_channels;
  this->is_simulation = msg.is_simulation;

  this->sampling_period = 1.0 / this->sampling_frequency;
}

/* XXX: Very close to a similar check in other pipeline stages. Unify? */
void EegRecorder::check_dropped_samples(double_t sample_time, double_t previous_time) {
  if (this->sampling_frequency == UNSET_SAMPLING_FREQUENCY) {
    RCLCPP_WARN(this->get_logger(), "Sampling frequency not received, cannot check for dropped samples.");
  }

  if (this->sampling_frequency != UNSET_SAMPLING_FREQUENCY && previous_time) {

    auto time_diff = sample_time - previous_time;
    auto threshold = this->sampling_period + this->TOLERANCE_S;

    if (time_diff > threshold) {
      /* Err if sample(s) were dropped. */
      RCLCPP_ERROR(this->get_logger(),
          "Sample(s) dropped. Time difference between consecutive samples: %.5f, should be: %.5f, limit: %.5f", time_diff, this->sampling_period, threshold);

    } else {
      /* If log-level is set to DEBUG, print time difference for all samples, regardless of if samples were dropped or not. */
      RCLCPP_DEBUG(this->get_logger(),
        "Time difference between consecutive samples: %.5f", time_diff);
    }
  }
}

void EegRecorder::handle_raw_eeg_sample([[maybe_unused]] const std::shared_ptr<eeg_interfaces::msg::Sample> msg) {
  this->previous_clock_time_raw = std::chrono::system_clock::now();

  /* XXX: Not sure if EEG info should be updated every sample. */
  update_eeg_info(msg->metadata);

  double_t sample_time = msg->time;

  this->check_dropped_samples(sample_time, this->previous_sample_time_raw);
  this->previous_sample_time_raw = sample_time;

  /* TODO: Duplicating the code from handle_preprocessed_eeg_sample function for now. Later, unify if it turns out that
       they did not diverge too much. */

  if (!raw_file.is_open() && (!this->is_simulation || this->record_simulation_data)) {
    raw_file.open(raw_file_path);
    /* Check if file was opened successfully. */
    if (!raw_file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Error opening file in path: %s", raw_file_path.c_str());
      return;
    }
  }

  std::ostringstream temp_buffer;
  temp_buffer.str("");
  temp_buffer.clear();
  temp_buffer << std::fixed << std::setprecision(4) << msg->time;

  /* Helper function to concatenate with comma. */
  auto add_value_with_comma = [&temp_buffer](double_t value) {
      temp_buffer << "," << value;
  };

  /* Add EEG data if available. */
  if (!msg->eeg.empty()) {
      std::for_each(msg->eeg.begin(), msg->eeg.end(), add_value_with_comma);
  }

  /* Add EMG data if available. */
  if (!msg->emg.empty()) {
      std::for_each(msg->emg.begin(), msg->emg.end(), add_value_with_comma);
  }

  temp_buffer << "\n";
  raw_buffer << temp_buffer.str();
}

void EegRecorder::handle_preprocessed_eeg_sample(const std::shared_ptr<eeg_interfaces::msg::Sample> msg) {
  this->previous_clock_time_preprocessed = std::chrono::system_clock::now();

  double_t sample_time = msg->time;

  this->check_dropped_samples(sample_time, this->previous_sample_time_preprocessed);
  this->previous_sample_time_preprocessed = sample_time;

  if (!preprocessed_file.is_open() && (!this->is_simulation || this->record_simulation_data)) {
    preprocessed_file.open(preprocessed_file_path);

    /* Check if file was opened successfully. */
    if (!preprocessed_file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Error opening file in path: %s", preprocessed_file_path.c_str());
      return;
    }
  }

  std::ostringstream temp_buffer;
  temp_buffer.str("");
  temp_buffer.clear();

  temp_buffer << std::fixed << std::setprecision(4) << msg->time
              << "," << std::setprecision(6) << msg->preprocessing_duration
              << std::setprecision(4) << "," << msg->valid;

  /* Helper function to concatenate with comma. */
  auto add_value_with_comma = [&temp_buffer](double_t value) {
      temp_buffer << "," << value;
  };

  /* Add EEG data if available. */
  if (!msg->eeg.empty()) {
      std::for_each(msg->eeg.begin(), msg->eeg.end(), add_value_with_comma);
  }

  /* Add EMG data if available. */
  if (!msg->emg.empty()) {
      std::for_each(msg->emg.begin(), msg->emg.end(), add_value_with_comma);
  }

  temp_buffer << "\n";
  preprocessed_buffer << temp_buffer.str();
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<EegRecorder>();

  rclcpp::executors::StaticSingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
