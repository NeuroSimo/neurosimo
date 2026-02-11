#include <algorithm>
#include <chrono>
#include <cstdio>
#include <functional>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "eeg_interfaces/msg/sample.hpp"
#include "eeg_interfaces/msg/eeg_statistics.hpp"

using namespace std::chrono_literals;

const std::string EEG_ENRICHED_TOPIC = "/eeg/enriched";
const std::string EEG_PREPROCESSED_TOPIC = "/eeg/preprocessed";
const std::string EEG_STATISTICS_TOPIC = "/eeg/statistics";

const double_t UNSET_TIME = std::numeric_limits<double_t>::quiet_NaN();

class EegMonitor : public rclcpp::Node {

public:
  EegMonitor() : Node("eeg_monitor") {
    num_of_raw_eeg_samples = 0;

    /* Subscriber for raw EEG. */
    auto eeg_enriched_subscriber_callback = [this](const std::shared_ptr<eeg_interfaces::msg::Sample> msg) -> void {
      /* Update the maximum time between two consecutive samples. */
      auto now = this->now();

      if (last_raw_sample_time.nanoseconds() != 0) {
        auto time_diff = (now - last_raw_sample_time).seconds();
        if (num_of_raw_eeg_samples > 0 && time_diff > max_interval_between_raw_samples) {
          max_interval_between_raw_samples = time_diff;
        }
      }
      last_raw_sample_time = now;

      /* Update the number of samples. */
      num_of_raw_eeg_samples++;
    };

    eeg_enriched_subscriber = this->create_subscription<eeg_interfaces::msg::Sample>(
      EEG_ENRICHED_TOPIC,
      10,
      eeg_enriched_subscriber_callback);

    /* Subscriber for preprocessed EEG. */
    auto eeg_preprocessed_subscriber_callback = [this](const std::shared_ptr<eeg_interfaces::msg::Sample> msg) -> void {
      /* Update the maximum time between two consecutive samples. */
      auto now = this->now();

      if (last_preprocessed_sample_time.nanoseconds() != 0) {
        auto time_diff = (now - last_preprocessed_sample_time).seconds();
        if (num_of_preprocessed_eeg_samples > 0 && time_diff > max_interval_between_preprocessed_samples) {
          max_interval_between_preprocessed_samples = time_diff;
        }
      }
      last_preprocessed_sample_time = now;

      /* Update the number of samples. */
      num_of_preprocessed_eeg_samples++;

      /* Update the preprocessing durations. */
      preprocessor_durations.push_back(msg->preprocessor_duration);
    };

    eeg_preprocessed_subscriber = this->create_subscription<eeg_interfaces::msg::Sample>(
      EEG_PREPROCESSED_TOPIC,
      10,
      eeg_preprocessed_subscriber_callback);

    /* Publisher for statistics. */
    eeg_statistics_publisher = this->create_publisher<eeg_interfaces::msg::EegStatistics>(
      EEG_STATISTICS_TOPIC,
      10);

    /* Timer for computing statistics for each second. */
    timer = this->create_wall_timer(1000ms, std::bind(&EegMonitor::timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "Started timer");
  }

  void timer_callback() {
    auto start_time = std::chrono::steady_clock::now();

    double max_duration = 0.0;
    double q95_duration = 0.0;
    double median_duration = 0.0;

    if (!preprocessor_durations.empty()) {
      /* Maximum */
      max_duration = *std::max_element(preprocessor_durations.begin(), preprocessor_durations.end());

      /* 95% Percentile */
      size_t q95_index = static_cast<size_t>(0.95 * preprocessor_durations.size());

      std::nth_element(preprocessor_durations.begin(), preprocessor_durations.begin() + q95_index, preprocessor_durations.end());
      q95_duration = preprocessor_durations[q95_index];

      /* Median */
      size_t middle_index = preprocessor_durations.size() / 2;

      std::nth_element(preprocessor_durations.begin(), preprocessor_durations.begin() + middle_index, preprocessor_durations.end());
      median_duration = preprocessor_durations[middle_index];

      if(preprocessor_durations.size() % 2 == 0) {
        std::nth_element(preprocessor_durations.begin(), preprocessor_durations.begin() + middle_index - 1, preprocessor_durations.end());
        median_duration = (median_duration + preprocessor_durations[middle_index - 1]) / 2;
      }
    }

    /* Publish the statistics. */
    auto msg = eeg_interfaces::msg::EegStatistics();

    msg.num_of_raw_samples = num_of_raw_eeg_samples;
    msg.max_interval_between_raw_samples = max_interval_between_raw_samples;

    msg.num_of_preprocessed_samples = num_of_preprocessed_eeg_samples;
    msg.max_interval_between_preprocessed_samples = max_interval_between_preprocessed_samples;

    msg.preprocessor_duration_max = max_duration;
    msg.preprocessor_duration_q95 = q95_duration;
    msg.preprocessor_duration_median = median_duration;

    eeg_statistics_publisher->publish(msg);

    /* Reset counters. */
    num_of_raw_eeg_samples = 0;
    num_of_preprocessed_eeg_samples = 0;

    max_interval_between_raw_samples = 0;
    max_interval_between_preprocessed_samples = 0;

    preprocessor_durations.clear();

    /* Log the duration of the callback. */
    auto end_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

    RCLCPP_DEBUG(this->get_logger(), "Timer callback duration: %lld us", duration.count());
  }


private:
  rclcpp::Subscription<eeg_interfaces::msg::Sample>::SharedPtr eeg_enriched_subscriber;
  rclcpp::Subscription<eeg_interfaces::msg::Sample>::SharedPtr eeg_preprocessed_subscriber;
  rclcpp::Publisher<eeg_interfaces::msg::EegStatistics>::SharedPtr eeg_statistics_publisher;

  rclcpp::TimerBase::SharedPtr timer;

  uint16_t num_of_raw_eeg_samples;
  double_t max_interval_between_raw_samples = 0.0;
  rclcpp::Time last_raw_sample_time;

  uint16_t num_of_preprocessed_eeg_samples;
  double_t max_interval_between_preprocessed_samples = 0.0;
  rclcpp::Time last_preprocessed_sample_time;

  std::vector<double_t> preprocessor_durations;
};


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<EegMonitor>();

  rclcpp::executors::StaticSingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
