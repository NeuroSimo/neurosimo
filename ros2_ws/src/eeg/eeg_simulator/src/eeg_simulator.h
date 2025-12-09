#ifndef EEG_SIMULATOR_H
#define EEG_SIMULATOR_H

#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"

#include "eeg_msgs/msg/sample.hpp"
#include "eeg_msgs/msg/eeg_info.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"

#include "project_interfaces/msg/dataset.hpp"
#include "project_interfaces/msg/dataset_list.hpp"
#include "project_interfaces/srv/set_dataset.hpp"
#include "project_interfaces/srv/set_start_time.hpp"

#include "std_srvs/srv/set_bool.hpp"

#include "system_interfaces/msg/healthcheck.hpp"
#include "system_interfaces/msg/healthcheck_status.hpp"

#include "system_interfaces/msg/session.hpp"
#include "system_interfaces/msg/session_state.hpp"
#include "system_interfaces/srv/stop_session.hpp"


enum class EegSimulatorState {
  READY,
  LOADING,
  ERROR_LOADING
  // TODO: Maybe add STREAMING here. Unsure if it is somewhat orthogonal to READY/LOADING or not,
  //   hence keep it separate for now.
};

const double_t UNSET_TIME = std::numeric_limits<double_t>::quiet_NaN();
const std::string UNSET_STRING = "";

class EegSimulator : public rclcpp::Node {
public:
  EegSimulator();
  ~EegSimulator();

private:
  void publish_healthcheck();
  void handle_eeg_bridge_healthcheck(const std::shared_ptr<system_interfaces::msg::Healthcheck> msg);

  std::tuple<bool, int, double, bool> get_dataset_info(const std::string& data_file_path);
  std::vector<project_interfaces::msg::Dataset> list_datasets(const std::string& path);
  void update_dataset_list();
  void handle_set_active_project(const std::shared_ptr<std_msgs::msg::String> msg);

  bool set_dataset(std::string filename);
  void handle_set_dataset(
      const std::shared_ptr<project_interfaces::srv::SetDataset::Request> request,
      std::shared_ptr<project_interfaces::srv::SetDataset::Response> response);

  void set_playback(bool playback);
  void handle_set_playback(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  void handle_set_loop(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  void handle_set_start_time(
      const std::shared_ptr<project_interfaces::srv::SetStartTime::Request> request,
      std::shared_ptr<project_interfaces::srv::SetStartTime::Response> response);

  void handle_session(const std::shared_ptr<system_interfaces::msg::Session> msg);

  void initialize_streaming();

  /* Publish a single sample at the given index. Returns true if the sample was published successfully. */
  bool publish_single_sample(size_t sample_index);

  /* Publish samples from current_index until (but not including) the first sample that is after until_time.
     Returns true if the samples were published successfully. */
  bool publish_until(double_t until_time);

  void read_next_event();

  void update_inotify_watch();
  void inotify_timer_callback();

  std::unordered_map<std::string, project_interfaces::msg::Dataset> dataset_map;
  std::string default_dataset_json;

  project_interfaces::msg::Dataset dataset;

  EegSimulatorState eeg_simulator_state = EegSimulatorState::READY;

  bool eeg_bridge_available = false;

  bool playback = false;
  bool loop = false;
  double_t play_dataset_from = 0.0;

  size_t current_index = 0;

  bool session_started = false;
  bool events_left = false;

  bool is_streaming = false;
  std::string error_message = UNSET_STRING;

  std::mutex dataset_mutex;

  double_t session_time;

  double_t sampling_period;

  double_t next_event_time;
  std::string next_event_type;

  uint16_t sampling_frequency;
  uint8_t num_of_eeg_channels;
  uint8_t num_of_emg_channels;
  uint8_t total_channels;

  std::ifstream data_file;

  std::vector<std::vector<double_t>> dataset_buffer;
  size_t current_sample_index = 0;
  
  double_t latest_sample_time = 0.0;
  double_t time_offset = 0.0;

  struct Event {
    double_t time;
    std::string type;
  };
  std::vector<Event> events;
  size_t current_event_index = 0;

  std::string active_project;
  std::string data_directory;

  std::string current_data_file_path = UNSET_STRING;

  rclcpp::CallbackGroup::SharedPtr callback_group;

  rclcpp::Subscription<system_interfaces::msg::Healthcheck>::SharedPtr eeg_bridge_healthcheck_subscriber;
  rclcpp::Publisher<system_interfaces::msg::Healthcheck>::SharedPtr healthcheck_publisher;
  rclcpp::TimerBase::SharedPtr healthcheck_publisher_timer;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr active_project_subscriber;
  rclcpp::Publisher<project_interfaces::msg::DatasetList>::SharedPtr dataset_list_publisher;

  rclcpp::Service<project_interfaces::srv::SetDataset>::SharedPtr set_dataset_service;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr dataset_publisher;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_playback_service;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr playback_publisher;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_loop_service;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr loop_publisher;

  rclcpp::Service<project_interfaces::srv::SetStartTime>::SharedPtr start_time_service;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr start_time_publisher;

  rclcpp::Publisher<eeg_msgs::msg::Sample>::SharedPtr eeg_publisher;
  rclcpp::Publisher<eeg_msgs::msg::EegInfo>::SharedPtr eeg_info_publisher;

  rclcpp::Subscription<system_interfaces::msg::Session>::SharedPtr session_subscriber;
  rclcpp::Client<system_interfaces::srv::StopSession>::SharedPtr stop_session_client;

  /* Inotify variables */
  rclcpp::TimerBase::SharedPtr inotify_timer;
  int inotify_descriptor;
  int watch_descriptor;
  char inotify_buffer[1024];

  /* When determining if samples have been dropped by comparing the timestamps of two consecutive
     samples, allow some tolerance to account for finite precision of floating point numbers. */
  static constexpr double_t TOLERANCE_S = 2 * pow(10, -5);
};

#endif //EEG_SIMULATOR_H
