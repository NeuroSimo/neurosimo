#include <chrono>
#include <functional>

#define DEFAULT_FREQUENCY_VALUE 0.5 // Hz
#define NUMBER_OF_CHANNELS 62
#define RANDOM_MAX 4000.0

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "eeg_interfaces/msg/sample.hpp"

using namespace std::chrono_literals;


class MockEegBridge : public rclcpp::Node {

  public:
    MockEegBridge() : Node("mock_eeg_bridge") {

        static const rmw_qos_profile_t qos_profile = {
            RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            1,
            RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            RMW_QOS_DEADLINE_DEFAULT,
            RMW_QOS_LIFESPAN_DEFAULT,
            RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
            RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
            false
        };

        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth), qos_profile);

        publisher_data_ = this->create_publisher<eeg_interfaces::msg::Sample>("/eeg/raw", 10);
        publisher_streaming_ = this->create_publisher<std_msgs::msg::Bool>("/eeg/is_streaming", qos);

        this->declare_parameter<float>("sampling_frequency", DEFAULT_FREQUENCY_VALUE);
        this->get_parameter("sampling_frequency", sampling_frequency_);

        this->sampling_interval_ = 1.0 / this->sampling_frequency_;

        /* TODO: Setting up the timer could be cleaned up to use sampling_interval calculated above.
         */
        auto sampling_interval_ms_int = int(round(1000 / this->sampling_frequency_));
        auto sampling_interval_ms = std::chrono::milliseconds(sampling_interval_ms_int);

        this->time_ = 0;

        timer_ = this->create_wall_timer(sampling_interval_ms, std::bind(&MockEegBridge::timer_callback, this));
    }

    void timer_callback() {

        auto message = eeg_interfaces::msg::Sample();

        for (int channel = 1; channel <= NUMBER_OF_CHANNELS; channel++) {

            double result = static_cast <double> (rand()) / (static_cast <double> (RAND_MAX/(2 * RANDOM_MAX)));

            if (result > RANDOM_MAX) {
                result -= 2 * RANDOM_MAX;
            }

            message.eeg.push_back(result);
            RCLCPP_INFO(this->get_logger(), "EEG channel: %d, Result: %f", channel, result);
        }
        message.time = this->time_;
        this->time_ += this->sampling_interval_;

        /* Set the system time when the sample was published. */
        auto now = std::chrono::high_resolution_clock::now();
        uint64_t system_time_data_source_published = std::chrono::duration_cast<std::chrono::nanoseconds>(
          now.time_since_epoch()).count();

        message.system_time_data_source_published = system_time_data_source_published;

        this->publisher_data_->publish(message);

        auto stream_msg = std_msgs::msg::Bool();
        stream_msg.data = true;
        this->publisher_streaming_->publish(stream_msg);
    }

  private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<eeg_interfaces::msg::Sample>::SharedPtr publisher_data_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_streaming_;
    float sampling_frequency_;
    double_t sampling_interval_;
    double_t time_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MockEegBridge>());
  rclcpp::shutdown();
  return 0;
}
