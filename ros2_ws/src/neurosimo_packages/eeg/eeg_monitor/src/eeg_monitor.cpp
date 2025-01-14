#include <algorithm>
#include <chrono>
#include <cstdio>
#include <functional>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "eeg_msgs/msg/sample.hpp"
#include "eeg_msgs/msg/preprocessed_sample.hpp"
#include "eeg_msgs/msg/eeg_statistics.hpp"

using namespace std::chrono_literals;

const std::string EEG_RAW_TOPIC = "/eeg/raw";
const std::string EEG_PREPROCESSED_TOPIC = "/eeg/preprocessed";
const std::string EEG_STATISTICS_TOPIC = "/eeg/statistics";

const double_t UNSET_TIME = std::numeric_limits<double_t>::quiet_NaN();

class EegMonitor : public rclcpp::Node {

public:
  EegMonitor() : Node("eeg_monitor") {
    num_of_raw_eeg_samples = 0;

    /* Subscriber for raw EEG. */
    auto eeg_raw_subscriber_callback = [this](const std::shared_ptr<eeg_msgs::msg::Sample> msg) -> void {
      /* Update the maximum time between two consecutive samples. */
      auto now = this->now();

      if (last_raw_sample_time.nanoseconds() != 0) {
        auto time_diff = (now - last_raw_sample_time).seconds();
        if (num_of_raw_eeg_samples > 0 && time_diff > max_time_between_raw_samples) {
          max_time_between_raw_samples = time_diff;
        }
      }
      last_raw_sample_time = now;

      /* Update the number of samples. */
      num_of_raw_eeg_samples++;
    };

    eeg_raw_subscriber = this->create_subscription<eeg_msgs::msg::Sample>(
      EEG_RAW_TOPIC,
      10,
      eeg_raw_subscriber_callback);

    /* Subscriber for preprocessed EEG. */
    auto eeg_preprocessed_subscriber_callback = [this](const std::shared_ptr<eeg_msgs::msg::PreprocessedSample> msg) -> void {
      /* Update the maximum time between two consecutive samples. */
      auto now = this->now();

      if (last_preprocessed_sample_time.nanoseconds() != 0) {
        auto time_diff = (now - last_preprocessed_sample_time).seconds();
        if (num_of_preprocessed_eeg_samples > 0 && time_diff > max_time_between_preprocessed_samples) {
          max_time_between_preprocessed_samples = time_diff;
        }
      }
      last_preprocessed_sample_time = now;

      /* Update the number of samples. */
      num_of_preprocessed_eeg_samples++;

      /* Update the processing times. */
      processing_times.push_back(msg->metadata.processing_time);
    };

    eeg_preprocessed_subscriber = this->create_subscription<eeg_msgs::msg::PreprocessedSample>(
      EEG_PREPROCESSED_TOPIC,
      10,
      eeg_preprocessed_subscriber_callback);

    /* Publisher for statistics. */
    eeg_statistics_publisher = this->create_publisher<eeg_msgs::msg::EegStatistics>(
      EEG_STATISTICS_TOPIC,
      10);

    /* Timer for computing statistics for each second. */
    timer = this->create_wall_timer(1000ms, std::bind(&EegMonitor::timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "Started timer");
  }

  void timer_callback() {
    auto start_time = std::chrono::steady_clock::now();

    double max_time = 0.0;
    double q95_time = 0.0;
    double median_time = 0.0;

    if (!processing_times.empty()) {
      /* Maximum */
      max_time = *std::max_element(processing_times.begin(), processing_times.end());

      /* 95% Percentile */
      size_t q95_index = static_cast<size_t>(0.95 * processing_times.size());

      std::nth_element(processing_times.begin(), processing_times.begin() + q95_index, processing_times.end());
      q95_time = processing_times[q95_index];

      /* Median */
      size_t middle_index = processing_times.size() / 2;

      std::nth_element(processing_times.begin(), processing_times.begin() + middle_index, processing_times.end());
      median_time = processing_times[middle_index];

      if(processing_times.size() % 2 == 0) {
        std::nth_element(processing_times.begin(), processing_times.begin() + middle_index - 1, processing_times.end());
        median_time = (median_time + processing_times[middle_index - 1]) / 2;
      }
    }

    /* Publish the statistics. */
    auto msg = eeg_msgs::msg::EegStatistics();

    msg.num_of_raw_samples = num_of_raw_eeg_samples;
    msg.max_time_between_raw_samples = max_time_between_raw_samples;

    msg.num_of_preprocessed_samples = num_of_preprocessed_eeg_samples;
    msg.max_time_between_preprocessed_samples = max_time_between_preprocessed_samples;

    msg.preprocessing_time_max = max_time;
    msg.preprocessing_time_q95 = q95_time;
    msg.preprocessing_time_median = median_time;

    eeg_statistics_publisher->publish(msg);

    /* Reset counters. */
    num_of_raw_eeg_samples = 0;
    num_of_preprocessed_eeg_samples = 0;

    max_time_between_raw_samples = 0;
    max_time_between_preprocessed_samples = 0;

    processing_times.clear();

    /* Log the duration of the callback. */
    auto end_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

    RCLCPP_DEBUG(this->get_logger(), "Timer callback duration: %lld us", duration.count());
  }


private:
  rclcpp::Subscription<eeg_msgs::msg::Sample>::SharedPtr eeg_raw_subscriber;
  rclcpp::Subscription<eeg_msgs::msg::PreprocessedSample>::SharedPtr eeg_preprocessed_subscriber;
  rclcpp::Publisher<eeg_msgs::msg::EegStatistics>::SharedPtr eeg_statistics_publisher;

  rclcpp::TimerBase::SharedPtr timer;

  uint16_t num_of_raw_eeg_samples;
  double_t max_time_between_raw_samples = 0.0;
  rclcpp::Time last_raw_sample_time;

  uint16_t num_of_preprocessed_eeg_samples;
  double_t max_time_between_preprocessed_samples = 0.0;
  rclcpp::Time last_preprocessed_sample_time;

  std::vector<double_t> processing_times;
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
