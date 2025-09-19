#ifndef DECIDER_WRAPPER_H
#define DECIDER_WRAPPER_H

#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/embed.h>
#include <pybind11/numpy.h>

#include "rclcpp/rclcpp.hpp"

#include <diagnostic_msgs/msg/key_value.hpp>

#include "eeg_msgs/msg/sample.hpp"
#include "eeg_msgs/msg/preprocessed_sample.hpp"

#include "mtms_trial_interfaces/msg/trial.hpp"
#include "targeting_msgs/msg/electric_target.hpp"

#include "pipeline_interfaces/msg/sensory_stimulus.hpp"
#include "pipeline_interfaces/msg/timed_trigger.hpp"

#include "std_msgs/msg/string.hpp"

#include "ring_buffer.h"

namespace py = pybind11;

const std::string bold_on = "\033[1m";
const std::string bold_off = "\033[0m";

enum class WrapperState {
  UNINITIALIZED,
  READY,
  ERROR
};

class DeciderWrapper {
public:
  DeciderWrapper(rclcpp::Logger& logger);
  ~DeciderWrapper();

  void remove_modules(const std::string& base_directory);
  void update_internal_imports(const std::string& base_directory);

  void initialize_module(
      const std::string& project_directory,
      const std::string& module_directory,
      const std::string& module_name,
      const size_t eeg_data_size,
      const size_t emg_data_size,
      const uint16_t sampling_frequency,
      std::vector<pipeline_interfaces::msg::SensoryStimulus>& sensory_stimuli,
      std::priority_queue<std::pair<double, std::string>,
                         std::vector<std::pair<double, std::string>>,
                         std::greater<std::pair<double, std::string>>>& event_queue,
      std::mutex& event_queue_mutex);

  void reset_module_state();

  void warm_up();

  std::vector<std::vector<targeting_msgs::msg::ElectricTarget>> get_targets();

  bool parse_sensory_stimulus_dict(
    const py::dict& py_sensory_stimulus,
    pipeline_interfaces::msg::SensoryStimulus& out_msg);
  std::tuple<bool, std::shared_ptr<mtms_trial_interfaces::msg::Trial>, std::shared_ptr<pipeline_interfaces::msg::TimedTrigger>, std::string> process(
    std::vector<pipeline_interfaces::msg::SensoryStimulus>& sensory_stimuli,
    const RingBuffer<std::shared_ptr<eeg_msgs::msg::PreprocessedSample>>& buffer,
    double_t sample_time,
    bool ready_for_trial,
    bool is_trigger,
    bool has_event,
    std::string event_type,
    std::priority_queue<std::pair<double, std::string>,
                       std::vector<std::pair<double, std::string>>,
                       std::greater<std::pair<double, std::string>>>& event_queue,
    std::mutex& event_queue_mutex,
    bool is_coil_at_target);

  WrapperState get_state() const;
  std::vector<std::string> get_internal_imports() const;

  std::size_t get_buffer_size() const;
  uint16_t get_processing_interval_in_samples() const;
  bool is_processing_interval_enabled() const;
  bool is_process_on_trigger_enabled() const;

  void setup_custom_print();

  /* log and log_throttle are exposed to Python, defined in cpp_bindings.cpp. */
  static void log(const std::string& message);
  static void log_throttle(const std::string& message, const double_t period);

  /* Get buffered logs and clear the buffer */
  std::vector<std::string> get_and_clear_logs();

private:
  /* XXX: Have a static ROS2 logger to expose it more easily to the Python side (see cpp_bindings.cpp). */
  static rclcpp::Logger* logger_ptr;

  /* Buffer for Python logs - static to be accessible from static log functions */
  static std::vector<std::string> log_buffer;
  static std::mutex log_buffer_mutex;

  WrapperState state;

  std::unique_ptr<py::module> decider_module;
  std::unique_ptr<py::object> decider_instance;

  std::unique_ptr<py::scoped_interpreter> interpreter;

  std::unique_ptr<py::array_t<double>> py_timestamps;
  std::unique_ptr<py::array_t<bool>> py_valid;
  std::unique_ptr<py::array_t<double>> py_eeg_data;
  std::unique_ptr<py::array_t<double>> py_emg_data;

  std::unordered_map<std::string, std::chrono::steady_clock::time_point> last_log_time;

  int earliest_sample;
  int latest_sample;
  uint16_t sampling_frequency;
  uint16_t processing_interval_in_samples = 0;
  bool process_on_trigger = false;

  std::vector<std::string> internal_imports;

  std::size_t buffer_size = 0;
  std::size_t eeg_data_size;
  std::size_t emg_data_size;

  bool process_sensory_stimuli_list(
    const py::list& py_sensory_stimuli,
    std::vector<pipeline_interfaces::msg::SensoryStimulus>& sensory_stimuli);
};

#endif
