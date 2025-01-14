#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <memory>
#include <string>
#include <unistd.h>

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
const std::string EEG_INFO_TOPIC = "/eeg/info";
const std::string HEALTHCHECK_TOPIC = "/eeg/healthcheck";
const std::string LATENCY_MEASUREMENT_TRIGGER_TOPIC = "/pipeline/latency_measurement_trigger";

/* Subscriber topics */
const std::string SYSTEM_SESSION_TOPIC = "/system/session";

/* Have a long queue to avoid dropping messages. */
const uint16_t EEG_QUEUE_LENGTH = 65535;

/* Note: Needs to match the values in session_bridge.cpp. */
const milliseconds SESSION_PUBLISHING_INTERVAL = 20ms;
const milliseconds SESSION_PUBLISHING_INTERVAL_TOLERANCE = 5ms;

const double_t SYNC_INTERVAL = 1.0;
const double_t MAXIMUM_TIME_CORRECTION_ADJUSTMENT_PER_SYNC_TRIGGER = 0.001;

EegBridge::EegBridge() : Node("eeg_bridge") {

  /* Port parameter */
  auto port_descriptor = rcl_interfaces::msg::ParameterDescriptor{};

  port_descriptor.description = "Port of the eeg device realtime output";
  port_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  this->declare_parameter("port", NULL, port_descriptor);

  this->get_parameter("port", this->port);

  /* EEG device parameter */
  auto eeg_device_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
  eeg_device_descriptor.description = "EEG device to use";
  eeg_device_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  this->declare_parameter("eeg-device", "", eeg_device_descriptor);

  std::string eeg_device_type;
  this->get_parameter("eeg-device", eeg_device_type);

  /* Is mTMS device enabled. */
  auto mtms_device_enabled_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
  mtms_device_enabled_descriptor.description = "Is mTMS device enabled";
  mtms_device_enabled_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  this->declare_parameter("mtms-device-enabled", false, mtms_device_enabled_descriptor);

  this->get_parameter("mtms-device-enabled", this->mtms_device_enabled);

  /* The number of tolerated dropped samples */
  auto num_of_tolerated_dropped_samples_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
  num_of_tolerated_dropped_samples_descriptor.description = "The number of tolerated dropped samples";
  num_of_tolerated_dropped_samples_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  this->declare_parameter("num-of-tolerated-dropped-samples", NULL, num_of_tolerated_dropped_samples_descriptor);

  this->get_parameter("num-of-tolerated-dropped-samples", this->num_of_tolerated_dropped_samples);

  /* Turbolink sampling frequency */
  auto turbolink_sampling_frequency_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
  turbolink_sampling_frequency_descriptor.description = "Sampling frequency of Turbolink device";
  turbolink_sampling_frequency_descriptor.type =
      rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  this->declare_parameter("turbolink-sampling-frequency", NULL,
                          turbolink_sampling_frequency_descriptor);

  uint32_t turbolink_sampling_frequency;
  this->get_parameter("turbolink-sampling-frequency", turbolink_sampling_frequency);

  /* Turbolink channel count */
  auto turbolink_eeg_channel_count_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
  turbolink_eeg_channel_count_descriptor.description = "EEG channel count of Turbolink device";
  turbolink_eeg_channel_count_descriptor.type =
      rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  this->declare_parameter("turbolink-eeg-channel-count", NULL,
                          turbolink_eeg_channel_count_descriptor);

  uint8_t turbolink_eeg_channel_count;
  this->get_parameter("turbolink-eeg-channel-count", turbolink_eeg_channel_count);

  /* Log configuration. */
  RCLCPP_INFO(this->get_logger(), "EEG bridge configuration:");
  RCLCPP_INFO(this->get_logger(), "  Port: %d", this->port);
  RCLCPP_INFO(this->get_logger(), "  EEG device: %s", eeg_device_type.c_str());
  RCLCPP_INFO(this->get_logger(), "  Number of tolerated dropped samples: %d",
              this->num_of_tolerated_dropped_samples);

  /* TODO: string to enum conversion should be done with cleaner solution at some
     point, maybe. */
  if (eeg_device_type == "neurone") {
    this->eeg_adapter = std::make_shared<NeurOneAdapter>(this->port);
  } else if (eeg_device_type == "turbolink") {
    this->eeg_adapter = std::make_shared<TurboLinkAdapter>(this->port, turbolink_sampling_frequency,
                                                           turbolink_eeg_channel_count);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("eeg_bridge"), "Unsupported EEG device. %s",
                 eeg_device_type.c_str());
  }

  this->create_publishers();
  this->create_subscribers();
}

void EegBridge::create_publishers() {
  auto qos_persist_latest = rclcpp::QoS(rclcpp::KeepLast(1))
                                .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                                .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

  this->eeg_sample_publisher =
      this->create_publisher<eeg_msgs::msg::Sample>(EEG_RAW_TOPIC, EEG_QUEUE_LENGTH);

  this->eeg_info_publisher =
      this->create_publisher<eeg_msgs::msg::EegInfo>(EEG_INFO_TOPIC, qos_persist_latest);

  this->latency_measurement_trigger_publisher =
      this->create_publisher<pipeline_interfaces::msg::TimedTrigger>(LATENCY_MEASUREMENT_TRIGGER_TOPIC, 10);

  this->healthcheck_publisher =
      this->create_publisher<system_interfaces::msg::Healthcheck>(HEALTHCHECK_TOPIC, 10);

  this->healthcheck_publisher_timer = this->create_wall_timer(
      std::chrono::milliseconds(500), [this] { publish_eeg_healthcheck(); });
}

void EegBridge::publish_eeg_info() {
  auto eeg_info = this->eeg_adapter->get_eeg_info();
  this->eeg_info_publisher->publish(eeg_info);
}

void EegBridge::publish_eeg_healthcheck() {
  auto healtcheck = system_interfaces::msg::Healthcheck();

  healtcheck.status.value = this->status;
  healtcheck.status_message = this->status_message;
  healtcheck.actionable_message = this->actionable_message;

  this->healthcheck_publisher->publish(healtcheck);
}

void EegBridge::create_subscribers() {
  this->subscribe_to_session();
}

void EegBridge::subscribe_to_session() {
  this->session_received = false;

  auto session_callback =
      [this](const std::shared_ptr<system_interfaces::msg::Session> message) -> void {

    /* Check if the session state has changed or if this is the first session message. */
    bool session_state_changed = this->session_state.value != message->state.value || !this->session_received;

    /* Update session state. */
    this->session_state = message->state;

    bool session_stopping = session_state.value == system_interfaces::msg::SessionState::STOPPING &&
                            session_state_changed;
    bool session_stopped = session_state.value == system_interfaces::msg::SessionState::STOPPED &&
                           session_state_changed;
    bool session_started = session_state.value == system_interfaces::msg::SessionState::STARTED &&
                           session_state_changed;

    /* Stopping a session takes several seconds, whereas if another session is
       started immediately after the previous one is stopped, the mTMS device
       remains in "stopped" state only for a very short period of time. Hence,
       check both conditions to ensure that we notice if the session is stopped.
     */
    if (session_stopping || session_stopped) {
      RCLCPP_INFO(this->get_logger(), "Session %s.", session_stopping ? "stopping" : "stopped");
      this->stop_session();
    }

    if (session_started) {
      RCLCPP_INFO(this->get_logger(), "Session started.");
      publish_eeg_info();
    }

    this->session_received = true;
  };

  /* HACK: Duplicates code from session_bridge.cpp. */
  const auto DEADLINE_NS =
      std::chrono::nanoseconds(SESSION_PUBLISHING_INTERVAL + SESSION_PUBLISHING_INTERVAL_TOLERANCE);

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
                 .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                 .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
                 .deadline(DEADLINE_NS)
                 .lifespan(DEADLINE_NS);

  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.event_callbacks.deadline_callback =
      [this]([[maybe_unused]] rclcpp::QOSDeadlineRequestedInfo &event) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Session not received within deadline.");
      };

  this->session_subscriber = this->create_subscription<system_interfaces::msg::Session>(
      SYSTEM_SESSION_TOPIC, qos, session_callback, subscription_options);
}

void EegBridge::stop_session() {
  /* Reset error state when session is stopped. */
  this->error_state = ErrorState::NO_ERROR;

  this->wait_for_session_to_stop = false;

  this->first_sample_of_session = true;

  this->first_sync_trigger_timestamp = UNSET_TIME;
  this->previous_sample_index = UNSET_PREVIOUS_SAMPLE_INDEX;

  this->time_correction = UNSET_TIME;
  this->time_offset = UNSET_TIME;
  this->num_of_sync_triggers_received = 0;

  /* Reset number of sample packets received. */
  this->sample_packets_received_since_last_sync = 0;
}

void EegBridge::update_healthcheck(uint8_t status, std::string status_message,
                                   std::string actionable_message) {
  this->status = status;
  this->status_message = status_message;
  this->actionable_message = actionable_message;
}

void EegBridge::handle_sync_trigger(double_t sync_time) {
  this->sample_packets_received_since_last_sync = 0;

  if (std::isnan(first_sync_trigger_timestamp)) {
    RCLCPP_INFO(this->get_logger(), "First sync trigger received at %.4f s.", sync_time);
    first_sync_trigger_timestamp = sync_time;
  }

  double_t new_time_correction =
      (sync_time - first_sync_trigger_timestamp) - num_of_sync_triggers_received * SYNC_INTERVAL;

  this->num_of_sync_triggers_received++;

  if (abs(new_time_correction - this->time_correction) >
      MAXIMUM_TIME_CORRECTION_ADJUSTMENT_PER_SYNC_TRIGGER) {
    RCLCPP_ERROR(this->get_logger(),
                 "Sync triggers received too frequently or infrequently. Check the BNC cable and "
                 "EEG software configuration for double triggers.");

    this->error_state = ERROR_OUT_OF_SYNC;
  }

  this->time_correction = new_time_correction;
}

void EegBridge::handle_sample(eeg_msgs::msg::Sample sample) {
  this->eeg_device_state = EegDeviceState::EEG_DEVICE_STREAMING;

  if (this->wait_for_session_to_stop) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                          "Waiting for session to stop...");
    return;
  }

  if (this->session_state.value != system_interfaces::msg::SessionState::STARTED) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                          "Waiting for session to start...");
    return;
  }

  this->sample_packets_received_since_last_sync++;

  /* Ignore sample packets if in an error state, preventing streaming. */
  if (this->error_state != ErrorState::NO_ERROR) {
    return;
  }

  /* Check for out of sync condition; a similar is needed both here and when an actual sync trigger is received
     to detect both cases where the sync trigger is not received at all and where it is received too frequently
     or infrequently. */
  if (this->mtms_device_enabled) {
    auto sampling_frequency = this->eeg_adapter->get_eeg_info().sampling_frequency;

    if (sampling_frequency != UNSET_SAMPLING_FREQUENCY && this->sample_packets_received_since_last_sync > 2 * sampling_frequency) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "No sync trigger received within two seconds. Please ensure: 1) 'Sync' port in the mTMS "
                            "device is connected to 'Trigger A in' in the EEG device, and 2) sending "
                            "triggers is enabled in the EEG software.");

      this->error_state = ErrorState::ERROR_OUT_OF_SYNC;
      return;
    }
  }

  /* Warn if the sample index wraps around. */
  if (previous_sample_index != UNSET_PREVIOUS_SAMPLE_INDEX &&
      sample.index == 0 &&
      previous_sample_index > 0) {

    RCLCPP_WARN(this->get_logger(), "Sample index wrapped around. Previous sample index: %d, current sample index: %d.",
                previous_sample_index,
                sample.index);
  }

  /* Check for dropped samples */
  if (previous_sample_index != UNSET_PREVIOUS_SAMPLE_INDEX &&
      sample.index > previous_sample_index + 1 + this->num_of_tolerated_dropped_samples &&
      /* Ignore the case where the sample index wraps around. */
      sample.index != 0) {

    this->error_state = ErrorState::ERROR_SAMPLES_DROPPED;

    RCLCPP_ERROR(this->get_logger(), "Samples dropped. Previous sample index: %d, current sample index: %d.",
                 previous_sample_index,
                 sample.index);
    return;
  }
  this->previous_sample_index = sample.index;

  /* If mTMS device is not available, offset samples by the timestamp of the first sample. */
  if (!this->mtms_device_enabled) {

    /* If this is the first sample of the session, set the time offset. */
    if (this->first_sample_of_session) {
      this->first_sample_of_session = false;
      this->time_offset = sample.time;
    }

    sample.time -= this->time_offset;
    this->eeg_sample_publisher->publish(sample);

    /* If the mTMS device is not available, return early. */
    return;
  }

  /* Only start publishing samples after the first sync trigger timestamp is set. */
  if (std::isnan(this->first_sync_trigger_timestamp)) {
    return;
  }

  /* Only start publishing samples after sample times exceed the first sync trigger timestamp. */
  if (!std::isnan(this->first_sync_trigger_timestamp) &&
      sample.time < this->first_sync_trigger_timestamp) {

    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Sample packet arrived %.4f s before first sync trigger. First sync "
                         "trigger timestamp: %.4f, sample timestamp: %.4f.",
                         this->first_sync_trigger_timestamp - sample.time,
                         this->first_sync_trigger_timestamp, sample.time);
    return;
  }
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                       "Streaming...");

  sample.time = sample.time - this->first_sync_trigger_timestamp - time_correction;

  eeg_sample_publisher->publish(sample);
}

void EegBridge::process_eeg_data_packet() {
  auto [result_type, sample, sync_time] = this->eeg_adapter->read_eeg_data_packet();

  /* Ignore the packets if session has not started. Exception: sync trigger packets are always processed if
     mTMS device is available, as they are used for drift correction.

     XXX: This is a bit of a hack, done because the first sync trigger can arrive before the session starts. */
  bool is_sync_trigger = result_type == PacketResult::SYNC_TRIGGER || result_type == PacketResult::SAMPLE_WITH_SYNC;
  bool ignore_packet = this->session_state.value != system_interfaces::msg::SessionState::STARTED &&
                       !(is_sync_trigger && this->mtms_device_enabled);

  if (ignore_packet) {
    return;
  }

  /* TODO: Could timestamp even earlier. */
  sample.metadata.system_time = this->get_clock()->now();

  auto eeg_info = this->eeg_adapter->get_eeg_info();

  if (eeg_info.sampling_frequency == UNSET_SAMPLING_FREQUENCY) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No sampling frequency set");
    return;
  }

  switch (result_type) {

  /* XXX: Misnomer, "sync" refers to sync trigger when mTMS device is enabled, but the same trigger
     is used for automatic latency correction. */
  case PacketResult::SAMPLE_WITH_SYNC:
    if (this->mtms_device_enabled) {
      /* Handle sync before sample, as sync will update time_correction */
      handle_sync_trigger(sync_time);
    } else {
      RCLCPP_INFO(this->get_logger(), "Received latency measurement trigger at %.4f s.", sync_time);

      auto msg = pipeline_interfaces::msg::TimedTrigger();
      msg.time = sync_time - this->time_offset;
      this->latency_measurement_trigger_publisher->publish(msg);
    }
    handle_sample(sample);
    break;

  case PacketResult::SAMPLE:
    handle_sample(sample);
    break;

  case PacketResult::SYNC_TRIGGER:
    if (this->mtms_device_enabled) {
      handle_sync_trigger(sync_time);
    } else {
      /* Without mTMS device, "sync trigger" (i.e., the trigger that comes to input port A in the EEG device) is
         used for automatic latency correction.

         TODO: Implement better abstractions so that it can be renamed properly (such as "trigger_port_a") and used
          for both mTMS and non-mTMS devices. Currently, the problem is that neurone_adapter.cpp knows about the
          "sync trigger" concept, which is specific to mTMS devices. */
      auto msg = pipeline_interfaces::msg::TimedTrigger();
      msg.time = sync_time - this->time_offset;
      this->latency_measurement_trigger_publisher->publish(msg);
    }
    break;

  case PacketResult::INTERNAL:
    RCLCPP_DEBUG(this->get_logger(), "Internal adapter packet received.");
    break;

  case PacketResult::ERROR:
    RCLCPP_ERROR(this->get_logger(), "Error reading data packet.");
    this->eeg_device_state = EegDeviceState::WAITING_FOR_EEG_DEVICE;
    break;

  case PacketResult::END:
    RCLCPP_INFO(this->get_logger(), "EEG device measurement stopped.");
    this->eeg_device_state = EegDeviceState::WAITING_FOR_EEG_DEVICE;
    break;

  default:
    RCLCPP_WARN(this->get_logger(), "Unknown result type while reading packet.");
  }
}

void EegBridge::wait_for_session() {
  RCLCPP_INFO(this->get_logger(), "Waiting for session...");

  auto base_interface = this->get_node_base_interface();

  /* HACK: Ensure that node stops itself gracefully by catching the exception:
     this is due to a known race condition in ROS2, in which if Ctrl-C (SIGINT)
     signal arrives between ok() and spin_some function calls, an exception is
     thrown. This seems to cause eProsima Fast DDS to occasionally go into a bad
     state, in which subscribers stop working properly after node is restarted.

     For more info about the race condition, see:

     https://github.com/ros2/rclcpp/issues/1066
     https://github.com/ros2/system_tests/pull/459
  */
  try {
    while (rclcpp::ok() && !this->session_received) {
      rclcpp::spin_some(base_interface);
    }
  } catch (const rclcpp::exceptions::RCLError &exception) {
    RCLCPP_ERROR(rclcpp::get_logger("eeg_bridge"), "Failed with %s", exception.what());
  }
}

void EegBridge::spin() {
  /* Session has a deadline of 25 ms, but it will only start affecting once the first session
   is received. Hence, wait here until the session is received. */
  wait_for_session();

  RCLCPP_INFO(this->get_logger(), "Waiting for measurement start...");

  auto base_interface = this->get_node_base_interface();

  try {
    while (rclcpp::ok()) {
      rclcpp::spin_some(base_interface);

      process_eeg_data_packet();

      /* Case: the EEG device is not streaming, and the session has not started. */
      if (this->eeg_device_state == EegDeviceState::WAITING_FOR_EEG_DEVICE &&
          this->session_state.value != system_interfaces::msg::SessionState::STARTED &&
          this->error_state == ErrorState::NO_ERROR) {

        this->update_healthcheck(system_interfaces::msg::HealthcheckStatus::NOT_READY,
                                 "Waiting for EEG measurement to start",
                                 "Please start the measurement on the EEG device.");
      }

      /* Case: the EEG device is not streaming, but the session has started. */
      if (this->eeg_device_state == EegDeviceState::WAITING_FOR_EEG_DEVICE &&
          this->session_state.value == system_interfaces::msg::SessionState::STARTED &&
          this->error_state == ErrorState::NO_ERROR) {

        /* If we the EEG device is not streaming, but the session has already been started,
           we need to wait for the session to stop before we can start streaming. */
        this->wait_for_session_to_stop = true;
        this->update_healthcheck(system_interfaces::msg::HealthcheckStatus::NOT_READY,
                                 "Waiting for session to stop",
                                 "Please stop the session.");
      }

      /* Case: the EEG device is streaming and the session has not started. */
      if (this->eeg_device_state == EegDeviceState::EEG_DEVICE_STREAMING &&
          this->session_state.value != system_interfaces::msg::SessionState::STARTED &&
          this->error_state == ErrorState::NO_ERROR) {

        this->update_healthcheck(system_interfaces::msg::HealthcheckStatus::READY,
                                 "Ready",
                                 "Ready");
      }

      /* Case: the EEG device is streaming, the session has started, but we are waiting for the session to stop. */
      if (this->eeg_device_state == EegDeviceState::EEG_DEVICE_STREAMING &&
          this->session_state.value == system_interfaces::msg::SessionState::STARTED &&
          this->wait_for_session_to_stop &&
          this->error_state == ErrorState::NO_ERROR) {

        this->update_healthcheck(system_interfaces::msg::HealthcheckStatus::NOT_READY,
                                 "Waiting for session to stop",
                                 "Please stop the session.");
      }

      /* Case: the EEG device is streaming, the session has started, and we are not waiting for the session to stop. */
      if (this->eeg_device_state == EegDeviceState::EEG_DEVICE_STREAMING &&
          this->session_state.value == system_interfaces::msg::SessionState::STARTED &&
          !this->wait_for_session_to_stop &&
          this->error_state == ErrorState::NO_ERROR) {

        this->update_healthcheck(system_interfaces::msg::HealthcheckStatus::READY,
                                 "Streaming data",
                                 "Streaming data");
      }

      /* Case: Explicit error case: out of sync. */
      if (this->error_state == ErrorState::ERROR_OUT_OF_SYNC) {
        this->update_healthcheck(
            system_interfaces::msg::HealthcheckStatus::ERROR,
            "Out of sync between EEG and mTMS devices",
            /* XXX: Not really an actionable message at this stage; as long as we don't
            actually show the cause of the error in the UI, it is more useful to have the
            error as the 'actionable' message than not to display it at all. Once the causes
            come to the UI, this could be changed to "Please stop the session"
            or similar. */
            "Out of sync between EEG and mTMS devices.");
      }

      /* Case: Explicit error case: samples dropped. */
      if (this->error_state == ErrorState::ERROR_SAMPLES_DROPPED) {
        this->update_healthcheck(system_interfaces::msg::HealthcheckStatus::ERROR,
                                 "Samples dropped in EEG device", "Samples dropped in EEG device.");
      }
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
