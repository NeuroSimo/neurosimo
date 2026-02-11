#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <memory>
#include <string>
#include <unistd.h>

#define XXH_INLINE_ALL
#include <xxhash.h>

#include "realtime_utils/utils.h"

#include "eeg_bridge.h"

#include "adapters/eeg_adapter.h"
#include "adapters/neurone_adapter.h"
#include "adapters/turbolink_adapter.h"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace std::placeholders;

/* Publisher topics */
const std::string EEG_RAW_TOPIC = "/eeg/raw";
const std::string DEVICE_INFO_TOPIC = "/eeg_device/info";
const std::string HEARTBEAT_TOPIC = "/eeg_bridge/heartbeat";

/* Have a long queue to avoid dropping messages. */
const uint16_t EEG_QUEUE_LENGTH = 65535;

/* Sample timeout in milliseconds */
const int64_t SAMPLE_TIMEOUT_MS = 200;

EegBridge::EegBridge() : Node("eeg_bridge") {
  RCLCPP_INFO(this->get_logger(), "Initializing EEG bridge...");

  /* Create publishers */
  auto qos_persist_latest = rclcpp::QoS(rclcpp::KeepLast(1))
                                .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                                .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

  this->eeg_sample_publisher =
      this->create_publisher<eeg_interfaces::msg::Sample>(EEG_RAW_TOPIC, EEG_QUEUE_LENGTH);

  this->device_info_publisher =
      this->create_publisher<eeg_interfaces::msg::EegDeviceInfo>(DEVICE_INFO_TOPIC, qos_persist_latest);

  this->data_source_state_publisher =
      this->create_publisher<system_interfaces::msg::DataSourceState>("/eeg_bridge/state", qos_persist_latest);

  this->heartbeat_publisher =
    this->create_publisher<std_msgs::msg::Empty>(HEARTBEAT_TOPIC, 10);

  this->health_publisher =
    this->create_publisher<system_interfaces::msg::ComponentHealth>("/eeg_bridge/health", qos_persist_latest);

  /* Create services */
  this->start_streaming_service = this->create_service<eeg_interfaces::srv::StartStreaming>(
    "/eeg_device/streaming/start",
    std::bind(&EegBridge::handle_start_streaming, this, std::placeholders::_1, std::placeholders::_2));

  this->stop_streaming_service = this->create_service<eeg_interfaces::srv::StopStreaming>(
    "/eeg_device/streaming/stop",
    std::bind(&EegBridge::handle_stop_streaming, this, std::placeholders::_1, std::placeholders::_2));

  this->initialize_service = this->create_service<eeg_interfaces::srv::InitializeEegDeviceStream>(
    "/eeg_device/initialize",
    std::bind(&EegBridge::handle_initialize, this, std::placeholders::_1, std::placeholders::_2));

  /* Service client for session abort. */
  this->abort_session_client = this->create_client<system_interfaces::srv::AbortSession>("/session/abort");

  while (!abort_session_client->wait_for_service(2s)) {
    RCLCPP_INFO(get_logger(), "Service /session/abort not available, waiting...");
  }

  /* Create subscribers */
  this->global_config_subscription = this->create_subscription<system_interfaces::msg::GlobalConfig>(
      "/global_configurator/config",
      qos_persist_latest,
      std::bind(&EegBridge::handle_global_config, this, std::placeholders::_1));

  /* Create heartbeat timer */
  this->heartbeat_publisher_timer = this->create_wall_timer(
    std::chrono::milliseconds(500), [this] { publish_heartbeat(); });

  /* Publish initial state */
  publish_data_source_state();
  publish_health_status(system_interfaces::msg::ComponentHealth::READY, "");
}

void EegBridge::handle_global_config(const system_interfaces::msg::GlobalConfig::SharedPtr msg) {
  /* Ignore config changes during ongoing session */
  if (this->data_source_state == system_interfaces::msg::DataSourceState::RUNNING) {
    RCLCPP_WARN(this->get_logger(), "Ignoring global config update during ongoing session");
    return;
  }

  bool needs_adapter_recreation = false;
  bool needs_socket_recreation = false;

  /* Check if port has changed */
  if (msg->eeg_port != static_cast<int32_t>(this->port)) {
    this->port = static_cast<uint16_t>(msg->eeg_port);
    needs_socket_recreation = true;
    needs_adapter_recreation = true;
  }

  /* Check if device type has changed */
  if (msg->eeg_device != this->eeg_device_type) {
    this->eeg_device_type = msg->eeg_device;
    needs_adapter_recreation = true;
  }

  /* Update maximum dropped samples */
  if (msg->maximum_dropped_samples != static_cast<int32_t>(this->maximum_dropped_samples)) {
    this->maximum_dropped_samples = static_cast<uint8_t>(msg->maximum_dropped_samples);
  }

  /* Check if turbolink parameters have changed */
  if (msg->turbolink_sampling_frequency != static_cast<int32_t>(this->turbolink_sampling_frequency)) {
    this->turbolink_sampling_frequency = static_cast<uint32_t>(msg->turbolink_sampling_frequency);
    if (this->eeg_device_type == "turbolink") {
      needs_adapter_recreation = true;
    }
  }

  if (msg->turbolink_eeg_channel_count != static_cast<int32_t>(this->turbolink_eeg_channel_count)) {
    this->turbolink_eeg_channel_count = static_cast<uint8_t>(msg->turbolink_eeg_channel_count);
    if (this->eeg_device_type == "turbolink") {
      needs_adapter_recreation = true;
    }
  }

  /* Recreate socket if port changed */
  if (needs_socket_recreation) {
    this->socket_ = std::make_shared<UdpSocket>(this->port);
    if (!this->socket_->init_socket()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize UDP socket");
      this->publish_health_status(system_interfaces::msg::ComponentHealth::ERROR,
                                   "Failed to initialize UDP socket");
      return;
    }
  }

  /* Recreate adapter if needed */
  if (needs_adapter_recreation) {
    /* Ensure socket exists before creating adapter */
    if (!this->socket_) {
      this->socket_ = std::make_shared<UdpSocket>(this->port);
      if (!this->socket_->init_socket()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize UDP socket");
        this->publish_health_status(system_interfaces::msg::ComponentHealth::ERROR,
                                     "Failed to initialize UDP socket");
        return;
      }
    }

    if (this->eeg_device_type == "neurone") {
      this->eeg_adapter = std::make_shared<NeurOneAdapter>(this->socket_);
    } else if (this->eeg_device_type == "turbolink") {
      this->eeg_adapter = std::make_shared<TurboLinkAdapter>(this->socket_,
                                                             this->turbolink_sampling_frequency,
                                                             this->turbolink_eeg_channel_count);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Unsupported EEG device: %s", this->eeg_device_type.c_str());
      this->publish_health_status(system_interfaces::msg::ComponentHealth::ERROR,
                                   "Unsupported EEG device type");
      return;
    }

    /* Publish updated device info */
    publish_device_info();
  }

  /* Log final configuration */
  RCLCPP_INFO(this->get_logger(), "EEG bridge configuration:");
  RCLCPP_INFO(this->get_logger(), "  • Port: %d", this->port);
  RCLCPP_INFO(this->get_logger(), "  • EEG device: %s", this->eeg_device_type.c_str());
  RCLCPP_INFO(this->get_logger(), "  • Maximum dropped samples: %d", this->maximum_dropped_samples);
  if (this->eeg_device_type == "turbolink") {
    RCLCPP_INFO(this->get_logger(), "  • Turbolink sampling frequency: %d", this->turbolink_sampling_frequency);
    RCLCPP_INFO(this->get_logger(), "  • Turbolink EEG channel count: %d", this->turbolink_eeg_channel_count);
  }
  RCLCPP_INFO(this->get_logger(), " ");

  this->publish_health_status(system_interfaces::msg::ComponentHealth::READY, "");
}

void EegBridge::publish_device_info() {
  if (!this->eeg_adapter) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Cannot publish device info: EEG adapter not initialized");
    return;
  }
  auto device_info = this->eeg_adapter->get_device_info();
  device_info.is_streaming = (this->device_state == EegDeviceState::EEG_DEVICE_STREAMING);
  this->device_info_publisher->publish(device_info);
}

void EegBridge::publish_data_source_state() {
  system_interfaces::msg::DataSourceState msg;
  msg.state = this->data_source_state;
  this->data_source_state_publisher->publish(msg);
}

void EegBridge::set_device_state(EegDeviceState new_state) {
  EegDeviceState previous_state = this->device_state;
  this->device_state = new_state;
  if (previous_state != this->device_state) {
    publish_device_info();
  }
}

void EegBridge::abort_session(const std::string& reason) {
  auto request = std::make_shared<system_interfaces::srv::AbortSession::Request>();
  request->source = "eeg_bridge";
  request->reason = reason;

  auto result = this->abort_session_client->async_send_request(request);
  RCLCPP_INFO(this->get_logger(), "Requested session abort: %s", reason.c_str());
}

void EegBridge::publish_heartbeat() {
  auto heartbeat = std_msgs::msg::Empty();
  this->heartbeat_publisher->publish(heartbeat);
}

void EegBridge::publish_health_status(uint8_t health_level, const std::string& message) {
  auto health = system_interfaces::msg::ComponentHealth();
  health.health_level = health_level;
  health.message = message;
  this->health_publisher->publish(health);
}

bool EegBridge::reset_state() {
  this->session_sample_index = 0;
  this->time_offset = UNSET_TIME;
  this->previous_device_sample_index = UNSET_PREVIOUS_SAMPLE_INDEX;
  this->is_session_start = false;
  this->last_sample_time = std::chrono::steady_clock::now();
  this->session_data_fingerprint = 0;

  this->data_source_state = system_interfaces::msg::DataSourceState::READY;
  publish_data_source_state();

  this->publish_health_status(system_interfaces::msg::ComponentHealth::READY, "");

  return true;
}

void EegBridge::handle_start_streaming(
      const std::shared_ptr<eeg_interfaces::srv::StartStreaming::Request> [[maybe_unused]] request,
      std::shared_ptr<eeg_interfaces::srv::StartStreaming::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received start streaming request");

  if (this->device_state != EegDeviceState::EEG_DEVICE_STREAMING) {
    response->success = false;
    return;
  }

  this->data_source_state = system_interfaces::msg::DataSourceState::RUNNING;
  publish_data_source_state();

  this->is_session_start = true;
  response->success = true;
}

void EegBridge::handle_stop_streaming(
      const std::shared_ptr<eeg_interfaces::srv::StopStreaming::Request> [[maybe_unused]] request,
      std::shared_ptr<eeg_interfaces::srv::StopStreaming::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Received stop streaming request");

  if (this->data_source_state != system_interfaces::msg::DataSourceState::RUNNING) {
    RCLCPP_ERROR(this->get_logger(), "Not streaming, cannot stop streaming.");

    response->success = false;
    response->data_fingerprint = 0;
    return;
  }

  /* Store the final fingerprint before resetting state */
  response->data_fingerprint = this->session_data_fingerprint;
  RCLCPP_INFO(this->get_logger(), "Session data fingerprint: 0x%016lx", response->data_fingerprint);

  this->reset_state();

  response->success = true;
}

void EegBridge::handle_initialize(
    const std::shared_ptr<eeg_interfaces::srv::InitializeEegDeviceStream::Request> [[maybe_unused]] request,
    std::shared_ptr<eeg_interfaces::srv::InitializeEegDeviceStream::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Initializing EEG device stream");

  // Get device info and create stream info
  auto device_info = this->eeg_adapter->get_device_info();

  response->stream_info.sampling_frequency = device_info.sampling_frequency;
  response->stream_info.num_eeg_channels = device_info.num_eeg_channels;
  response->stream_info.num_emg_channels = device_info.num_emg_channels;

  this->reset_state();
}

bool EegBridge::check_for_dropped_samples(uint64_t device_sample_index) {
  /* Warn if the device sample index wraps around. */
  if (previous_device_sample_index != UNSET_PREVIOUS_SAMPLE_INDEX &&
      device_sample_index == 0 &&
      previous_device_sample_index > 0) {

    RCLCPP_WARN(this->get_logger(), 
                "Device sample index wrapped around. Previous: %lu, current: %lu.",
                previous_device_sample_index,
                device_sample_index);
  }

  /* Check for dropped samples using device indices. */
  if (previous_device_sample_index != UNSET_PREVIOUS_SAMPLE_INDEX &&
      device_sample_index > previous_device_sample_index + 1 + this->maximum_dropped_samples &&
      /* Ignore the case where the sample index wraps around. */
      device_sample_index != 0) {

    RCLCPP_ERROR(this->get_logger(),
                 "Samples dropped. Previous device sample index: %lu, current: %lu.",
                 previous_device_sample_index,
                 device_sample_index);
    this->abort_session("EEG samples dropped");
    return false;  // Samples were dropped
  }
  
  this->previous_device_sample_index = device_sample_index;
  return true;  // No samples dropped
}

void EegBridge::check_for_sample_timeout() {
  /* Only check timeout when streaming is active */
  if (this->data_source_state != system_interfaces::msg::DataSourceState::RUNNING) {
    return;
  }

  auto now = std::chrono::steady_clock::now();
  auto time_since_last_sample = std::chrono::duration_cast<std::chrono::milliseconds>(
    now - this->last_sample_time).count();

  if (time_since_last_sample > SAMPLE_TIMEOUT_MS) {
    RCLCPP_ERROR(this->get_logger(),
                 "Sample timeout: No sample received in %ld ms",
                 time_since_last_sample);
    
    this->abort_session("EEG device sample timeout");
  }
}

eeg_interfaces::msg::Sample EegBridge::create_ros_sample(const AdapterSample& adapter_sample,
                                                    const eeg_interfaces::msg::EegDeviceInfo& [[maybe_unused]] device_info) {
  auto sample = eeg_interfaces::msg::Sample();
  sample.eeg = adapter_sample.eeg;
  sample.emg = adapter_sample.emg;
  sample.time = adapter_sample.time;
  sample.pulse_trigger = adapter_sample.trigger_a;
  sample.loopback_trigger = adapter_sample.trigger_b;
  
  return sample;
}

void EegBridge::handle_sample(eeg_interfaces::msg::Sample sample) {
  set_device_state(EegDeviceState::EEG_DEVICE_STREAMING);

  /* Update last sample time */
  this->last_sample_time = std::chrono::steady_clock::now();

  if (this->data_source_state != system_interfaces::msg::DataSourceState::RUNNING) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                          "Waiting for session to start...");
    return;
  }

  /* If this is the first sample, set the time offset. */
  if (std::isnan(this->time_offset)) {
    this->time_offset = sample.time;
  }

  sample.time -= this->time_offset;

  /* Set session start flag. */
  sample.is_session_start = this->is_session_start;

  /* Set the streaming sample index. */
  sample.sample_index = this->session_sample_index;
  this->session_sample_index++;

  /* Mark the sample as valid by default. The preprocessor can later mark it as invalid if needed. */
  sample.valid = true;

  /* Set the system time when the sample was published. */
  auto now = std::chrono::high_resolution_clock::now();
  uint64_t system_time_data_source_published = std::chrono::duration_cast<std::chrono::nanoseconds>(
    now.time_since_epoch()).count();

  sample.system_time_data_source_published = system_time_data_source_published;

  /* Update data fingerprint with EEG data */
  if (!sample.eeg.empty()) {
    this->session_data_fingerprint = XXH64(sample.eeg.data(), 
                                            sample.eeg.size() * sizeof(double),
                                            this->session_data_fingerprint);
  }
  
  /* Update data fingerprint with EMG data */
  if (!sample.emg.empty()) {
    this->session_data_fingerprint = XXH64(sample.emg.data(),
                                            sample.emg.size() * sizeof(double),
                                            this->session_data_fingerprint);
  }

  this->eeg_sample_publisher->publish(sample);

  // Log latency trigger when present
  if (sample.loopback_trigger) {
    RCLCPP_DEBUG(this->get_logger(), "Received latency trigger at time: %.4f s.", sample.time);
  }
  
  // Log pulse trigger when present
  if (sample.pulse_trigger) {
    RCLCPP_INFO(this->get_logger(), "Received TMS pulse at time: %.4f s", sample.time);
  }

  /* Clear the session start marker after publishing. */
  this->is_session_start = false;
}

void EegBridge::process_eeg_packet() {
  /* Check if adapter is initialized */
  if (!this->eeg_adapter) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Waiting for global configuration to initialize EEG adapter...");
    return;
  }

  /* Check if socket is initialized */
  if (!this->socket_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Waiting for socket initialization...");
    return;
  }

  // Read UDP packet
  if (!this->socket_->read_data(this->buffer, BUFFER_SIZE)) {
    RCLCPP_WARN(this->get_logger(), "Timeout while reading EEG data");

    // Interpret timeout as the end of the stream. This can happen at the end of an actual stream
    // when using TurboLink, as the device does not send any data after the stream ends, or e.g.,
    // if the ethernet cable is disconnected in the middle of a stream.
    set_device_state(EegDeviceState::WAITING_FOR_EEG_DEVICE);

    return;
  }

  // Process the packet
  AdapterPacket packet = this->eeg_adapter->process_packet(this->buffer, BUFFER_SIZE);

  auto device_info = this->eeg_adapter->get_device_info();

  if (device_info.sampling_frequency == UNSET_SAMPLING_FREQUENCY) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No sampling frequency set");
    return;
  }

  switch (packet.result) {

  case SAMPLE: {
    // Check for dropped samples using device sample index
    if (!check_for_dropped_samples(packet.sample.sample_index)) {
      // Dropped samples detected, don't process this sample
      break;
    }

    auto ros_sample = create_ros_sample(packet.sample, device_info);

    // Always handle the sample
    handle_sample(ros_sample);
    break;
  }

  case INTERNAL:
    RCLCPP_DEBUG(this->get_logger(), "Internal adapter packet received.");
    break;

  case ERROR:
    RCLCPP_ERROR(this->get_logger(), "Error reading data packet.");
    set_device_state(EegDeviceState::WAITING_FOR_EEG_DEVICE);
    this->abort_session("Error reading EEG data packet");
    break;

  case END:
    RCLCPP_INFO(this->get_logger(), "EEG device measurement stopped.");
    set_device_state(EegDeviceState::WAITING_FOR_EEG_DEVICE);
    break;

  default:
    RCLCPP_WARN(this->get_logger(), "Unknown result type while reading packet.");
  }
}

void EegBridge::spin() {
  auto base_interface = this->get_node_base_interface();

  try {
    while (rclcpp::ok()) {
      rclcpp::spin_some(base_interface);

      process_eeg_packet();

      check_for_sample_timeout();
    }
  } catch (const rclcpp::exceptions::RCLError &exception) {
    RCLCPP_ERROR(rclcpp::get_logger("eeg_bridge"), "Failed with %s", exception.what());
  }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto logger = rclcpp::get_logger("eeg_bridge");

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

  auto node = std::make_shared<EegBridge>();

  node->spin();
  rclcpp::shutdown();
  return 0;
}
