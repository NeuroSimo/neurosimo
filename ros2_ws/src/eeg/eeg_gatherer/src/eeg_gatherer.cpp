#include <assert.h>

#include "rclcpp/rclcpp.hpp"

#include "headers/eeg_gatherer.h"

using namespace std::chrono_literals;
using namespace std::chrono;

EegGatherer::EegGatherer(std::string goal_id, double_t start_time, double_t end_time) {
  this->goal_id = goal_id;

  this->start_time = start_time;
  this->end_time = end_time;
  this->duration = end_time - start_time;

  this->previous_time = UNSET_PREVIOUS_TIME;

  this->eeg_buffer = std::vector<eeg_interfaces::msg::Sample>();

  this->state = DataGatheringState::CHECK_IF_VALID_REQUEST;

}

void EegGatherer::check_dropped_samples(double_t current_time, uint16_t sampling_frequency) {
  if (this->previous_time) {
    double_t sampling_period = 1.0 / sampling_frequency;

    auto time_diff = current_time - this->previous_time;
    auto threshold = sampling_period + this->TOLERANCE_S;

    if (time_diff > threshold) {

      /* Go to error state if we are gathering data when samples are dropped, otherwise just warn. */
      if (this->state == DataGatheringState::GATHER_DATA) {
        this->state = DataGatheringState::FINAL_STATE__SAMPLES_DROPPED;

        RCLCPP_ERROR(rclcpp::get_logger("eeg_gatherer"),
          "%s: Sample(s) dropped while gathering data. Time difference between consecutive samples: %.5f, should be: %.5f, limit: %.5f", this->goal_id.c_str(), time_diff, sampling_period, threshold);
      } else {
        RCLCPP_WARN(rclcpp::get_logger("eeg_gatherer"),
          "%s: Sample(s) dropped. Time difference between consecutive samples: %.5f, should be: %.5f, limit: %.5f", this->goal_id.c_str(), time_diff, sampling_period, threshold);
      }
    } else {
      /* If log-level is set to DEBUG, print time difference for all samples, regardless of if samples were dropped or not. */
      RCLCPP_DEBUG(rclcpp::get_logger("eeg_gatherer"),
        "%s: Time difference between consecutive samples: %.5f", this->goal_id.c_str(), time_diff);
    }
  }
  this->previous_time = current_time;
}

bool EegGatherer::handle_state__check_if_valid_request(double_t current_time) {
  if (current_time >= this->start_time) {
    RCLCPP_WARN(rclcpp::get_logger("eeg_gatherer"),
      "%s: Failure: Current time (%.2f s) is past the starting time (%.2f s).",
      this->goal_id.c_str(),
      current_time,
      this->start_time);

    this->state = DataGatheringState::FINAL_STATE__LATE;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("eeg_gatherer"), "%s: Waiting for the starting time...", this->goal_id.c_str());
    this->state = DataGatheringState::WAIT_FOR_MEP;
  }
  return false;
}

bool EegGatherer::handle_state__wait_for_mep(double_t current_time) {
  bool next_sample = true;

  if (current_time >= this->start_time) {
    RCLCPP_INFO(rclcpp::get_logger("eeg_gatherer"), "%s: Gathering data...", this->goal_id.c_str());

    this->state = DataGatheringState::GATHER_DATA;
    next_sample = false;
  }
  return next_sample;
}

bool EegGatherer::handle_state__gather_data(double_t current_time, const std::shared_ptr<eeg_interfaces::msg::Sample> msg) {
  bool next_sample = false;

  if (current_time < this->end_time) {
    this->eeg_buffer.push_back(*msg);
    next_sample = true;

  } else {
    RCLCPP_INFO(rclcpp::get_logger("eeg_gatherer"), "%s: Finished gathering data...", this->goal_id.c_str());
    this->state = DataGatheringState::FINAL_STATE__SUCCESS;
  }
  return next_sample;
}

void EegGatherer::handle_eeg_sample(const std::shared_ptr<eeg_interfaces::msg::Sample> msg) {
  auto current_time = msg->time;
  auto sampling_frequency = msg->metadata.sampling_frequency;

  check_dropped_samples(current_time, sampling_frequency);

  bool next_sample = false;

  while (!next_sample) {
    switch (this->state) {
      case DataGatheringState::CHECK_IF_VALID_REQUEST:
        next_sample = handle_state__check_if_valid_request(current_time);
        break;

      case DataGatheringState::WAIT_FOR_MEP:
        next_sample = handle_state__wait_for_mep(current_time);
        break;

      case DataGatheringState::GATHER_DATA:
        next_sample = handle_state__gather_data(current_time, msg);
        break;

      case DataGatheringState::FINAL_STATE__LATE:
        next_sample = true;
        break;

      case DataGatheringState::FINAL_STATE__SAMPLES_DROPPED:
        next_sample = true;
        break;

      case DataGatheringState::FINAL_STATE__SUCCESS:
        next_sample = true;
        break;

      default:
        assert(false);
    }
  }
}

bool EegGatherer::is_finished() {
  return std::any_of(std::begin(FINAL_STATES), std::end(FINAL_STATES), [&](DataGatheringState state) { return state == this->state; });
}

bool EegGatherer::success() {
  return this->state == DataGatheringState::FINAL_STATE__SUCCESS;
}

eeg_interfaces::msg::GatherEegError::SharedPtr EegGatherer::get_error() {
  auto error = std::make_shared<eeg_interfaces::msg::GatherEegError>();
  switch (this->state) {
    case DataGatheringState::FINAL_STATE__SUCCESS:
      error->value = eeg_interfaces::msg::GatherEegError::NO_ERROR;
      break;

    case DataGatheringState::FINAL_STATE__LATE:
      error->value = eeg_interfaces::msg::GatherEegError::LATE;
      break;

    case DataGatheringState::FINAL_STATE__SAMPLES_DROPPED:
      error->value = eeg_interfaces::msg::GatherEegError::SAMPLES_DROPPED;
      break;

    default:
      assert(false);
  }
  return error;
}

std::vector<eeg_interfaces::msg::Sample>& EegGatherer::get_eeg_buffer() {
  assert (is_finished());

  return this->eeg_buffer;
}
