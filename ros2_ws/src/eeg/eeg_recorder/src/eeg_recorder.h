#ifndef EEG_RECORDER_H
#define EEG_RECORDER_H

#include <cmath>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

#include "eeg_interfaces/msg/sample.hpp"
#include "eeg_interfaces/msg/eeg_info.hpp"

#include "std_srvs/srv/set_bool.hpp"

#include "system_interfaces/msg/session.hpp"
#include "system_interfaces/msg/session_state.hpp"


const uint16_t UNSET_SAMPLING_FREQUENCY = 0;
const uint8_t UNSET_NUM_OF_CHANNELS = 255;
const double_t UNSET_PREVIOUS_TIME = std::numeric_limits<double_t>::quiet_NaN();

class EegRecorder : public rclcpp::Node {
public:
  EegRecorder();

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr active_project_subscriber;

  rclcpp::Subscription<system_interfaces::msg::Session>::SharedPtr session_subscriber;

  rclcpp::Subscription<eeg_interfaces::msg::Sample>::SharedPtr eeg_raw_subscriber;
  rclcpp::Subscription<eeg_interfaces::msg::Sample>::SharedPtr eeg_preprocessed_subscriber;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_record_data_service;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr record_data_publisher;

  rclcpp::TimerBase::SharedPtr timer;

  void handle_set_active_project(const std::shared_ptr<std_msgs::msg::String> msg);
  void handle_session(const std::shared_ptr<system_interfaces::msg::Session> msg);

  void update_eeg_info(const eeg_interfaces::msg::SampleMetadata& msg);
  void check_dropped_samples(double_t sample_time, double_t previous_time);

  void handle_raw_eeg_sample(const std::shared_ptr<eeg_interfaces::msg::Sample> msg);
  void handle_preprocessed_eeg_sample(const std::shared_ptr<eeg_interfaces::msg::Sample> msg);

  void handle_set_record_data(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  void write_raw_buffer();
  void write_preprocessed_buffer();

  void write_buffers();

  std::string active_project;

  std::string experiment_name;
  std::string subject_name;

  std::string filename;

  std::string raw_data_directory;
  std::string raw_file_path;
  std::ofstream raw_file;
  std::ostringstream raw_buffer;

  double_t previous_sample_time_raw = UNSET_PREVIOUS_TIME;
  std::chrono::time_point<std::chrono::high_resolution_clock> previous_clock_time_raw;

  std::string preprocessed_data_directory;
  std::string preprocessed_file_path;
  std::ofstream preprocessed_file;
  std::ostringstream preprocessed_buffer;

  double_t previous_sample_time_preprocessed = UNSET_PREVIOUS_TIME;
  std::chrono::time_point<std::chrono::high_resolution_clock> previous_clock_time_preprocessed;

  uint8_t current_session_state;

  uint16_t sampling_frequency = UNSET_SAMPLING_FREQUENCY;
  uint8_t num_eeg_channels = UNSET_NUM_OF_CHANNELS;
  uint8_t num_emg_channels = UNSET_NUM_OF_CHANNELS;
  bool is_simulation = false;

  bool record_simulation_data = false;

  double_t sampling_period;

  /* When determining if samples have been dropped by comparing the timestamps of two consecutive
     samples, allow some tolerance to account for finite precision of floating point numbers. */
  static constexpr double_t TOLERANCE_S = 2 * pow(10, -5);
};

#endif //EEG_RECORDER_H
