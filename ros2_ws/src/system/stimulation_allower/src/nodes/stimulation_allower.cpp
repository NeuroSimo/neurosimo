#include "scheduling_utils.h"
#include "memory_utils.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "mtms_device_interfaces/srv/allow_stimulation.hpp"
#include "mtms_device_interfaces/srv/allow_trigger_out.hpp"
#include "stimulation_interfaces/srv/is_stimulation_allowed.hpp"

using namespace std;
using namespace std::chrono;
using namespace std::chrono_literals;
using std::placeholders::_1;

/* HACK: Needs to match the value in neuronavigation bridge. */
const milliseconds COIL_AT_TARGET_PUBLISHING_INTERVAL = 600ms;

class StimulationAllower : public rclcpp::Node {

public:
  StimulationAllower() : Node("stimulation_allower") {

    auto is_stimulation_allowed_callback = [this](
        [[maybe_unused]] const std::shared_ptr<stimulation_interfaces::srv::IsStimulationAllowed::Request> request,
        std::shared_ptr<stimulation_interfaces::srv::IsStimulationAllowed::Response> response) -> void {

      response->success = true;
      response->stimulation_allowed = this->stimulation_allowed;

      RCLCPP_INFO(rclcpp::get_logger("stimulation_allower"), "Successfully responded to get stimulation allowed request.");
    };

    /* Create QoS to persist the latest sample. */
    auto qos_persist_latest = rclcpp::QoS(rclcpp::KeepLast(1))
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

    /* Create QoS profile for 'coil at target' subscriber. */
    auto qos_deadline = rclcpp::QoS(rclcpp::KeepLast(1))
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
        .deadline(std::chrono::nanoseconds(COIL_AT_TARGET_PUBLISHING_INTERVAL));

    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.event_callbacks.deadline_callback = [this]([[maybe_unused]] rclcpp::QOSDeadlineRequestedInfo & event) -> void {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "'Coil at target' message was not received within deadline, setting to false.");

      this->coil_at_target = false;
      this->update_stimulation_allowed();
    };

    /* Create subscribers and service client. */
    neuronavigation_started_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "/neuronavigation/started", qos_persist_latest, std::bind(&StimulationAllower::neuronavigation_started_callback, this, _1));

    target_mode_enabled_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "/neuronavigation/target_mode/enabled", qos_persist_latest, std::bind(&StimulationAllower::target_mode_enabled_callback, this, _1));

    coil_at_target_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "/neuronavigation/coil_at_target", qos_deadline, std::bind(&StimulationAllower::coil_at_target_callback, this, _1), subscription_options);

    allow_stimulation_client_ = this->create_client<mtms_device_interfaces::srv::AllowStimulation>("/mtms_device/allow_stimulation");
    allow_trigger_out_client_ = this->create_client<mtms_device_interfaces::srv::AllowTriggerOut>("/mtms_device/allow_trigger_out");

    /* Create service for querying status of stimulation allowed.*/
    is_stimulation_allowed_service_ = this->create_service<stimulation_interfaces::srv::IsStimulationAllowed>(
        "/stimulation/allowed", is_stimulation_allowed_callback);
  }

private:
  void call_stimulation_allowed_service(bool value) {
    auto request = std::make_shared<mtms_device_interfaces::srv::AllowStimulation::Request>();
    request->allow_stimulation = value;

    while (!allow_stimulation_client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for service to become available...");
    }
    auto result_future = allow_stimulation_client_->async_send_request(request);
  }

  void call_trigger_out_allowed_service(bool value) {
    auto request = std::make_shared<mtms_device_interfaces::srv::AllowTriggerOut::Request>();
    request->allow_trigger_out = value;

    while (!allow_trigger_out_client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for service to become available...");
    }
    auto result_future = allow_trigger_out_client_->async_send_request(request);
  }

  void update_stimulation_allowed() {
    RCLCPP_INFO(this->get_logger(), "Neuronavigation: %s, target mode: %s, coil at target: %s",
      this->neuronavigation_started ? "started" : "stopped",
      this->target_mode_enabled ? "enabled" : "disabled",
      this->coil_at_target ? "true" : "false");

    /* Allow stimulation if:
          - neuronavigation is not started,
          - neuronavigation is started but target mode is disabled, or
          - neuronavigation is started, target mode is enabled, and coil is at the target. */
    this->stimulation_allowed = !this->neuronavigation_started || !this->target_mode_enabled || this->coil_at_target;

    RCLCPP_INFO(this->get_logger(), "%s stimulation and trigger out.", this->stimulation_allowed ? "Allowing" : "Disallowing");

    this->call_stimulation_allowed_service(this->stimulation_allowed);

    /* If stimulation is disallowed, disallow trigger out, as well.

      Rationale: usually, trigger out is used to either signal that a stimulation pulse has been given or trigger
      stimulation on another TMS device - if stimulation is disallowed, both of these use cases should be
      prevented as well. */
    this->call_trigger_out_allowed_service(this->stimulation_allowed);
  }

  void neuronavigation_started_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    this->neuronavigation_started = msg->data;
    update_stimulation_allowed();
  }

  void target_mode_enabled_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    this->target_mode_enabled = msg->data;
    update_stimulation_allowed();
  }

  void coil_at_target_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    this->coil_at_target = msg->data;
    update_stimulation_allowed();
  }

  bool neuronavigation_started = false;
  bool coil_at_target = false;
  bool target_mode_enabled = false;

  bool stimulation_allowed = true;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr neuronavigation_started_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr target_mode_enabled_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr coil_at_target_subscription_;

  rclcpp::Client<mtms_device_interfaces::srv::AllowStimulation>::SharedPtr allow_stimulation_client_;
  rclcpp::Client<mtms_device_interfaces::srv::AllowTriggerOut>::SharedPtr allow_trigger_out_client_;

  rclcpp::Service<stimulation_interfaces::srv::IsStimulationAllowed>::SharedPtr is_stimulation_allowed_service_;
};


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

#if defined(ON_UNIX) && defined(SCHEDULING_OPTIMIZATION)
  RCLCPP_INFO(rclcpp::get_logger("stimulation_allower"), "Setting thread scheduling");
  set_thread_scheduling(pthread_self(), DEFAULT_SCHEDULING_POLICY, DEFAULT_REALTIME_SCHEDULING_PRIORITY);
#endif

  auto node = std::make_shared<StimulationAllower>();

#if defined(ON_UNIX) && defined(MEMORY_OPTIMIZATION)
  RCLCPP_INFO(rclcpp::get_logger("stimulation_allower"), "Locking memory");
  lock_memory();
  preallocate_memory(1024 * 1024 * 10); //10 MB
#endif

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
