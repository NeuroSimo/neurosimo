#include "get_multipulse_waveforms.h"

#include "memory_utils.h"
#include "scheduling_utils.h"

const uint8_t NUM_OF_COILS = 5;

const uint16_t INITIAL_VOLTAGE = 1500;

const bool ENABLE_CACHING = true;

GetMultipulseWaveforms::GetMultipulseWaveforms() : Node("get_multipulse_waveforms"), logger(rclcpp::get_logger("get_multipulse_waveforms")) {
  callback_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  get_multipulse_waveforms_service = this->create_service<targeting_interfaces::srv::GetMultipulseWaveforms>(
      "/waveforms/get_multipulse_waveforms",
      std::bind(&GetMultipulseWaveforms::handle_get_multipulse_waveforms, this, _1, _2),
      rclcpp::QoS(10),
      callback_group);

  get_target_voltages_client = this->create_client<targeting_interfaces::srv::GetTargetVoltages>(
      "/targeting/get_target_voltages",
      rclcpp::QoS(10),
      callback_group);

  approximate_waveform_client = this->create_client<targeting_interfaces::srv::ApproximateWaveform>(
      "/targeting/approximate_waveform",
      rclcpp::QoS(10),
      callback_group);

  estimate_voltage_after_pulse_client = this->create_client<targeting_interfaces::srv::EstimateVoltageAfterPulse>(
      "/targeting/estimate_voltage_after_pulse",
      rclcpp::QoS(10),
      callback_group);

  reverse_polarity_client = this->create_client<targeting_interfaces::srv::ReversePolarity>(
      "/waveforms/reverse_polarity",
      rclcpp::QoS(10),
      callback_group);
}

std::string generate_md5_hash(const std::shared_ptr<targeting_interfaces::srv::GetMultipulseWaveforms_Request> request) {
    rclcpp::Serialization<targeting_interfaces::srv::GetMultipulseWaveforms_Request> serializer;
    rclcpp::SerializedMessage serialized_msg;

    /* Serialize the request. */
    serializer.serialize_message(request.get(), &serialized_msg);

    /* Compute MD5 hash. */
    md5 hash;
    md5::digest_type digest;
    hash.process_bytes(serialized_msg.get_rcl_serialized_message().buffer, serialized_msg.get_rcl_serialized_message().buffer_length);
    hash.get_digest(digest);

    /* Convert the hash to a hexadecimal string. */
    const auto char_digest = reinterpret_cast<const char*>(&digest);
    std::string result;
    boost::algorithm::hex(char_digest, char_digest + sizeof(md5::digest_type), std::back_inserter(result));

    return result;
}

void GetMultipulseWaveforms::handle_get_multipulse_waveforms(
    const std::shared_ptr<targeting_interfaces::srv::GetMultipulseWaveforms::Request> request,
    std::shared_ptr<targeting_interfaces::srv::GetMultipulseWaveforms::Response> response) {

  std::string key = generate_md5_hash(request);

  /* Check if the response is cached. */
  if (ENABLE_CACHING && cache.find(key) != cache.end()) {
    auto cached_response = cache[key];

    /* Copy the cached response. */
    response->success = cached_response->success;
    response->initial_voltages = cached_response->initial_voltages;
    response->approximated_waveforms = cached_response->approximated_waveforms;

    RCLCPP_INFO(this->get_logger(), "Returning cached result");
    return;
  }

  /* Call the function to handle the request. */
  handle_get_multipulse_waveforms_no_cache(request, response);

  /* Cache the response. */
  if (ENABLE_CACHING) {
    cache[key] = response;

    RCLCPP_INFO(this->get_logger(), "Computed and cached result");
  } else {
    RCLCPP_INFO(this->get_logger(), "Computed result (caching disabled)");
  }
};

void GetMultipulseWaveforms::handle_get_multipulse_waveforms_no_cache(
    const std::shared_ptr<targeting_interfaces::srv::GetMultipulseWaveforms::Request>& request,
    std::shared_ptr<targeting_interfaces::srv::GetMultipulseWaveforms::Response>& response) {

  uint8_t num_of_targets = request->targets.size();

  /* Validate request. */
  if (num_of_targets == 0) {
    RCLCPP_WARN(logger, "Received request with 0 targets.");

    response->success = false;
    return;
  }
  if (request->target_waveforms.size() != num_of_targets) {
    RCLCPP_WARN(logger, "Received request with mismatching number of targets and target waveforms.");

    response->success = false;
    return;
  }
  for (uint8_t i = 0; i < num_of_targets; i++) {
    if (request->target_waveforms[i].waveforms.size() != NUM_OF_COILS) {
      RCLCPP_WARN(logger, "Received request with mismatching number of coils in target waveform.");

      response->success = false;
      return;
    }
  }

  /* Loop through each pulse, storing the target voltages and reversed polarities in a 2d vector. */
  std::vector<std::vector<uint16_t>> target_voltages;
  std::vector<std::vector<bool>> reversed_polarities;

  for (uint8_t i = 0; i < num_of_targets; i++) {
    auto result = get_target_voltages(request->targets[i]);

    if (!result->success) {
      RCLCPP_WARN(logger, "Failed to get target voltages for pulse %d", i + 1);

      response->success = false;
      return;
    }

    /* Store voltages into a vector */
    std::vector<uint16_t> target_voltages_for_pulse;
    for (uint8_t j = 0; j < result->voltages.size(); j++) {
      target_voltages_for_pulse.push_back(result->voltages[j]);
    }
    target_voltages.push_back(target_voltages_for_pulse);

    /* Store reversed polarities into a vector */
    std::vector<bool> reversed_polarities_for_pulse;
    for (uint8_t j = 0; j < result->reversed_polarities.size(); j++) {
      reversed_polarities_for_pulse.push_back(result->reversed_polarities[j]);
    }
    reversed_polarities.push_back(reversed_polarities_for_pulse);
  }

  uint16_t coil_voltages[NUM_OF_COILS];

  /* Initialize coil voltages. */
  for (uint8_t i = 0; i < NUM_OF_COILS; i++) {
    coil_voltages[i] = INITIAL_VOLTAGE;
  }

  /* Populate the initial voltages field in the response. */
  for (uint8_t i = 0; i < NUM_OF_COILS; i++) {
    response->initial_voltages.push_back(coil_voltages[i]);
  }

  /* Loop through each pulse and coil, computing the approximated waveforms. */
  for (uint8_t i = 0; i < num_of_targets; i++) {
    event_interfaces::msg::WaveformsForCoilSet approximated_waveforms_for_coil_set;
    for (uint8_t j = 0; j < NUM_OF_COILS; j++) {
      uint16_t actual_voltage = coil_voltages[j];
      uint16_t target_voltage = target_voltages[i][j];

      /* Print information about the target and coil. */
      RCLCPP_INFO(logger, "Target #: %d, Coil #: %d, Actual Voltage: %d, Target Voltage: %d", i + 1, j + 1, actual_voltage, target_voltage);

      /* Check that the target voltage is not larger than the actual voltage. */
      if (target_voltage > actual_voltage) {
        RCLCPP_WARN(logger, "Failure: Target voltage is larger than actual voltage for coil %d", j + 1);

        response->success = false;
        return;
      }

      auto target_waveform = request->target_waveforms[i].waveforms[j];

      /* If polarity is reversed, call the ROS service to get the reversed waveform. */
      if (reversed_polarities[i][j]) {
        auto result = reverse_polarity(target_waveform);

        if (!result->success) {
          RCLCPP_WARN(logger, "Failure: Failed to reverse polarity for coil %d", j + 1);

          response->success = false;
          return;
        }
        target_waveform = result->waveform;
      }

      uint8_t coil_number = j + 1;

      /* Call the ROS service to approximate the waveform. */
      auto result = approximate_waveform(actual_voltage, target_voltage, target_waveform, coil_number);

      if (!result->success) {
        RCLCPP_WARN(logger, "Failure: Failed to approximate waveform for coil %d", j + 1);

        response->success = false;
        return;
      }

      /* Store the approximated waveform in the response. */
      approximated_waveforms_for_coil_set.waveforms.push_back(result->approximated_waveform);

      /* Estimate drop in coil voltage after pulse. */
      auto result2 = estimate_voltage_after_pulse(actual_voltage, result->approximated_waveform, coil_number);

      if (!result2->success) {
        RCLCPP_WARN(logger, "Failure: Failed to estimate voltage after pulse for coil %d", j + 1);

        response->success = false;
        return;
      }

      /* Update coil voltage. */
      coil_voltages[j] = result2->voltage_after;
    }
    response->approximated_waveforms.push_back(approximated_waveforms_for_coil_set);
  }

  response->success = true;
}

const std::shared_ptr<targeting_interfaces::srv::ApproximateWaveform::Response> GetMultipulseWaveforms::approximate_waveform(
    const uint16_t actual_voltage,
    const uint16_t target_voltage,
    const event_interfaces::msg::Waveform& target_waveform,
    const uint8_t coil_number) {

  auto request = std::make_shared<targeting_interfaces::srv::ApproximateWaveform::Request>();
  request->actual_voltage = actual_voltage;
  request->target_voltage = target_voltage;
  request->target_waveform = target_waveform;
  request->coil_number = coil_number;

  approximate_waveform_client->wait_for_service();

  auto result_future = approximate_waveform_client->async_send_request(request);
  auto result = result_future.get();

  return result;
}

const std::shared_ptr<targeting_interfaces::srv::ReversePolarity::Response> GetMultipulseWaveforms::reverse_polarity(
    const event_interfaces::msg::Waveform& waveform) {

  auto request = std::make_shared<targeting_interfaces::srv::ReversePolarity::Request>();
  request->waveform = waveform;

  reverse_polarity_client->wait_for_service();

  auto result_future = reverse_polarity_client->async_send_request(request);
  auto result = result_future.get();

  return result;
}

const std::shared_ptr<targeting_interfaces::srv::EstimateVoltageAfterPulse::Response> GetMultipulseWaveforms::estimate_voltage_after_pulse(
    const uint16_t voltage_before,
    const event_interfaces::msg::Waveform& waveform,
    const uint8_t coil_number) {

  auto request = std::make_shared<targeting_interfaces::srv::EstimateVoltageAfterPulse::Request>();
  request->voltage_before = voltage_before;
  request->waveform = waveform;
  request->coil_number = coil_number;

  estimate_voltage_after_pulse_client->wait_for_service();

  auto result_future = estimate_voltage_after_pulse_client->async_send_request(request);
  auto result = result_future.get();

  return result;
}

const std::shared_ptr<targeting_interfaces::srv::GetTargetVoltages::Response> GetMultipulseWaveforms::get_target_voltages(
    const targeting_interfaces::msg::ElectricTarget& target) {

  auto request = std::make_shared<targeting_interfaces::srv::GetTargetVoltages::Request>();
  request->target = target;

  get_target_voltages_client->wait_for_service();

  auto result_future = get_target_voltages_client->async_send_request(request);
  auto result = result_future.get();

  return result;
}


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

#if defined(ON_UNIX) && defined(SCHEDULING_OPTIMIZATION)
  RCLCPP_INFO(logger, "Setting thread scheduling");
  set_thread_scheduling(pthread_self(), DEFAULT_SCHEDULING_POLICY, DEFAULT_REALTIME_SCHEDULING_PRIORITY);
#endif

  auto node = std::make_shared<GetMultipulseWaveforms>();

#if defined(ON_UNIX) && defined(MEMORY_OPTIMIZATION)
  RCLCPP_INFO(logger, "Locking memory");
  lock_memory();
  preallocate_memory(1024 * 1024 * 10); //10 MB
#endif

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node);
  executor->spin();

  rclcpp::shutdown();
  return 0;
}
