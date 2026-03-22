#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_storage/storage_options.hpp"

#include "neurosimo_eeg_interfaces/srv/initialize_eeg_replay_stream.hpp"
#include "neurosimo_eeg_interfaces/srv/start_streaming.hpp"
#include "neurosimo_eeg_interfaces/srv/stop_streaming.hpp"
#include "neurosimo_eeg_interfaces/msg/stream_info.hpp"
#include "neurosimo_eeg_interfaces/msg/sample.hpp"
#include "neurosimo_pipeline_interfaces/msg/experiment_state.hpp"
#include "neurosimo_system_interfaces/msg/global_config.hpp"
#include "neurosimo_system_interfaces/msg/data_source_state.hpp"
#include "neurosimo_system_interfaces/srv/abort_session.hpp"


class EegReplayNode : public rclcpp::Node {

public:
  EegReplayNode();
  ~EegReplayNode() override;

private:
  void global_config_callback(const neurosimo_system_interfaces::msg::GlobalConfig::SharedPtr msg);
  void publish_state(uint8_t state);

  void handle_initialize(
    const std::shared_ptr<neurosimo_eeg_interfaces::srv::InitializeEegReplayStream::Request> request,
    std::shared_ptr<neurosimo_eeg_interfaces::srv::InitializeEegReplayStream::Response> response);

  void handle_start_streaming(
    const std::shared_ptr<neurosimo_eeg_interfaces::srv::StartStreaming::Request> request,
    std::shared_ptr<neurosimo_eeg_interfaces::srv::StartStreaming::Response> response);

  void handle_stop_streaming(
    const std::shared_ptr<neurosimo_eeg_interfaces::srv::StopStreaming::Request> request,
    std::shared_ptr<neurosimo_eeg_interfaces::srv::StopStreaming::Response> response);

  void playback_loop();
  void abort_session();
  void cleanup();

  rclcpp::CallbackGroup::SharedPtr callback_group_;

  rclcpp::Publisher<neurosimo_system_interfaces::msg::DataSourceState>::SharedPtr state_publisher_;
  rclcpp::Client<neurosimo_system_interfaces::srv::AbortSession>::SharedPtr abort_session_client_;
  rclcpp::Subscription<neurosimo_system_interfaces::msg::GlobalConfig>::SharedPtr global_config_sub_;

  rclcpp::Service<neurosimo_eeg_interfaces::srv::InitializeEegReplayStream>::SharedPtr initialize_service_;
  rclcpp::Service<neurosimo_eeg_interfaces::srv::StartStreaming>::SharedPtr start_streaming_service_;
  rclcpp::Service<neurosimo_eeg_interfaces::srv::StopStreaming>::SharedPtr stop_streaming_service_;

  neurosimo_system_interfaces::msg::GlobalConfig::SharedPtr global_config_;
  bool is_initialized_ = false;
  std::string bag_filepath_;
  bool play_preprocessed_ = false;
  neurosimo_eeg_interfaces::msg::StreamInfo stream_info_;
  uint64_t data_source_fingerprint_ = 0;

  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> is_streaming_{false};
  std::thread playback_thread_;
  std::mutex playback_mutex_;
};