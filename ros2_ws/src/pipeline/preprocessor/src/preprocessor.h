//
// Created by alqio on 16.1.2023.
//

#ifndef EEG_PROCESSOR_PREPROCESSOR_H
#define EEG_PROCESSOR_PREPROCESSOR_H

#include <cmath>

#include "rclcpp/rclcpp.hpp"

#include "preprocessor_wrapper.h"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"

#include "eeg_interfaces/msg/sample.hpp"
#include "eeg_interfaces/msg/stream_info.hpp"

#include "project_interfaces/msg/filename_list.hpp"
#include "project_interfaces/srv/set_module.hpp"

#include "std_srvs/srv/set_bool.hpp"

#include "pipeline_interfaces/msg/log_message.hpp"
#include "pipeline_interfaces/msg/log_messages.hpp"
#include "pipeline_interfaces/srv/initialize_preprocessor.hpp"
#include "pipeline_interfaces/srv/finalize_preprocessor.hpp"

#include "ring_buffer.h"

const uint16_t UNSET_SAMPLING_FREQUENCY = 0;
const uint8_t UNSET_NUM_OF_CHANNELS = 255;
const double_t UNSET_TIME = std::numeric_limits<double_t>::quiet_NaN();
const std::string UNSET_STRING = "";

struct DeferredProcessingRequest {
  /* The time when processing should actually occur (after look-ahead samples have arrived). */
  double_t scheduled_time;
  
  /* The sample that triggered the processing request. */
  std::shared_ptr<eeg_interfaces::msg::Sample> triggering_sample;
  
  /* Comparison operator for priority queue (min-heap by scheduled_time). */
  bool operator>(const DeferredProcessingRequest& other) const {
    return scheduled_time > other.scheduled_time;
  }
};

using StreamInfo = eeg_interfaces::msg::StreamInfo;

class PreprocessorWrapper;

class EegPreprocessor : public rclcpp::Node {
public:
  EegPreprocessor();

private:
  void publish_heartbeat();

  bool reset_state();

  void handle_initialize_preprocessor(
    const std::shared_ptr<pipeline_interfaces::srv::InitializePreprocessor::Request> request,
    std::shared_ptr<pipeline_interfaces::srv::InitializePreprocessor::Response> response);
  void handle_finalize_preprocessor(
    const std::shared_ptr<pipeline_interfaces::srv::FinalizePreprocessor::Request> request,
    std::shared_ptr<pipeline_interfaces::srv::FinalizePreprocessor::Response> response);
  void publish_python_logs(double sample_time, bool is_initialization);
  void publish_sentinel_sample(double_t sample_time);

  void process_sample(const std::shared_ptr<eeg_interfaces::msg::Sample> msg);

  bool is_sample_window_valid() const;
  void enqueue_deferred_request(const std::shared_ptr<eeg_interfaces::msg::Sample> msg, double_t sample_time);
  void process_deferred_request(const DeferredProcessingRequest& request, double_t current_sample_time);
  void process_ready_deferred_requests(double_t current_sample_time);

  rclcpp::Logger logger;

  rclcpp::TimerBase::SharedPtr heartbeat_publisher_timer;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr heartbeat_publisher;

  rclcpp::Subscription<eeg_interfaces::msg::Sample>::SharedPtr enriched_eeg_subscriber;
  rclcpp::Publisher<eeg_interfaces::msg::Sample>::SharedPtr preprocessed_eeg_publisher;

  rclcpp::Publisher<pipeline_interfaces::msg::LogMessages>::SharedPtr python_log_publisher;

  /* Service server for initialization */
  rclcpp::Service<pipeline_interfaces::srv::InitializePreprocessor>::SharedPtr initialize_service_server;

  /* Service server for finalization */
  rclcpp::Service<pipeline_interfaces::srv::FinalizePreprocessor>::SharedPtr finalize_service_server;

  /* Initialization state */
  bool is_initialized = false;
  bool is_enabled = false;
  std::string initialized_project_name;
  std::string initialized_module_filename;
  std::filesystem::path initialized_working_directory;

  bool error_occurred = false;

  /* Pending session markers to be carried forward to the next published sample. */
  bool pending_session_start = false;
  bool pending_session_end = false;

  StreamInfo stream_info;

  RingBuffer<std::shared_ptr<eeg_interfaces::msg::Sample>> sample_buffer;
  eeg_interfaces::msg::Sample preprocessed_sample;

  std::unique_ptr<PreprocessorWrapper> preprocessor_wrapper;

  /* Deferred processing queue for handling look-ahead samples. */
  std::priority_queue<DeferredProcessingRequest,
                      std::vector<DeferredProcessingRequest>,
                      std::greater<DeferredProcessingRequest>> deferred_processing_queue;

  /* Healthcheck */
  uint8_t status;
  std::string status_message;
  std::string actionable_message;

  /* When determining if a sample is ready to be processed, allow some tolerance to account
     for finite precision of floating point numbers. */
  static constexpr double_t TOLERANCE_S = 2 * pow(10, -5);
};

#endif //EEG_PROCESSOR_PREPROCESSOR_H
