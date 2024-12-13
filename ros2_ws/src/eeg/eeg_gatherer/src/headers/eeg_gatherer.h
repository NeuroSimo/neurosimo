#ifndef EEG_GATHERER_EEG_GATHERER_H
#define EEG_GATHERER_EEG_GATHERER_H

#include "rclcpp/rclcpp.hpp"

#include "eeg_interfaces/msg/sample.hpp"
#include "eeg_interfaces/msg/time_window.hpp"
#include "eeg_interfaces/msg/gather_eeg_error.hpp"

const double_t UNSET_PREVIOUS_TIME = std::numeric_limits<double_t>::quiet_NaN();

enum class DataGatheringState {
  CHECK_IF_VALID_REQUEST,
  WAIT_FOR_MEP,
  GATHER_DATA,
  FINAL_STATE__SUCCESS,
  FINAL_STATE__LATE,
  FINAL_STATE__SAMPLES_DROPPED
};

class EegGatherer {

public:
  EegGatherer(std::string goal_id, double_t start_time, double_t end_time);
  void handle_eeg_sample(const std::shared_ptr<eeg_interfaces::msg::Sample> msg);
  bool is_finished();
  bool success();
  std::vector<eeg_interfaces::msg::Sample>& get_eeg_buffer();
  eeg_interfaces::msg::GatherEegError::SharedPtr get_error();

private:
  void check_dropped_samples(double_t current_time, uint16_t sampling_frequency);

  bool handle_state__check_if_valid_request(double_t current_time);
  bool handle_state__wait_for_mep(double_t current_time);
  bool handle_state__gather_data(double_t current_time, const std::shared_ptr<eeg_interfaces::msg::Sample> msg);

  std::vector<eeg_interfaces::msg::Sample> eeg_buffer;

  std::string goal_id;

  double_t start_time;
  double_t end_time;
  double_t duration;

  DataGatheringState state;
  double_t previous_time;

  static constexpr DataGatheringState FINAL_STATES[3] = {
    DataGatheringState::FINAL_STATE__SUCCESS,
    DataGatheringState::FINAL_STATE__LATE,
    DataGatheringState::FINAL_STATE__SAMPLES_DROPPED
  };

  /* When determining if samples have been dropped by comparing the timestamps of two consecutive
     samples, allow some tolerance to account for finite precision of floating point numbers. */
  static constexpr double_t TOLERANCE_S = 2 * pow(10, -5);
};

#endif //EEG_GATHERER_EEG_GATHERER_H
