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

#include "pipeline_interfaces/msg/coil_target.hpp"

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

enum class LogLevel : uint8_t {
  INFO = 0,
  WARNING = 1,
  ERROR = 2
};

struct LogEntry {
  std::string message;
  LogLevel level;
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

  bool parse_sensory_stimulus_dict(
    const py::dict& py_sensory_stimulus,
    pipeline_interfaces::msg::SensoryStimulus& out_msg);
  std::tuple<bool, std::shared_ptr<pipeline_interfaces::msg::TimedTrigger>, std::string> process(
    std::vector<pipeline_interfaces::msg::SensoryStimulus>& sensory_stimuli,
    const RingBuffer<std::shared_ptr<eeg_msgs::msg::Sample>>& buffer,
    double_t sample_window_base_time,
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
  double get_periodic_processing_interval() const;
  double get_first_periodic_processing_at() const;
  bool is_processing_interval_enabled() const;
  int get_look_ahead_samples() const;
  int get_look_ahead_samples_for_event(const std::string& event_type) const;
  double get_pulse_lockout_duration() const;

  void setup_custom_print();

  /* log and log_throttle are exposed to Python, defined in cpp_bindings.cpp. */
  static void log(const std::string& message);
  static void log_throttle(const std::string& message, const double_t period);

  /* Get buffered logs and clear the buffer */
  std::vector<LogEntry> get_and_clear_logs();
  void log_section_header(const std::string& title);

private:
  /* XXX: Have a static ROS2 logger to expose it more easily to the Python side (see cpp_bindings.cpp). */
  static rclcpp::Logger* logger_ptr;

  /* Buffer for Python logs - static to be accessible from static log functions */
  static std::vector<LogEntry> log_buffer;
  static std::mutex log_buffer_mutex;

  WrapperState state;

  std::unique_ptr<py::module> decider_module;
  std::unique_ptr<py::object> decider_instance;

  std::unique_ptr<py::scoped_interpreter> interpreter;

  /* Map of event type to processor function */
  std::unordered_map<std::string, py::object> event_processors;

  /* Map of event type to custom sample window (optional) */
  std::unordered_map<std::string, std::pair<int, int>> event_sample_windows;

  /* Preallocated numpy arrays for default sample window */
  std::unique_ptr<py::array_t<double>> py_time_offsets;
  std::unique_ptr<py::array_t<bool>> py_valid;
  std::unique_ptr<py::array_t<double>> py_eeg_data;
  std::unique_ptr<py::array_t<double>> py_emg_data;

  /* Preallocated numpy arrays for custom event windows */
  struct EventArrays {
    std::unique_ptr<py::array_t<double>> time_offsets;
    std::unique_ptr<py::array_t<bool>> valid;
    std::unique_ptr<py::array_t<double>> eeg_data;
    std::unique_ptr<py::array_t<double>> emg_data;
    size_t buffer_size;
    int reference_index;
  };
  std::unordered_map<std::string, EventArrays> event_arrays;

  std::unordered_map<std::string, std::chrono::steady_clock::time_point> last_log_time;

  int look_back_samples;
  int look_ahead_samples;
  uint16_t sampling_frequency;
  bool periodic_processing_enabled = false;
  double periodic_processing_interval = 0.0;
  double first_periodic_processing_at = 0.0;
  double pulse_lockout_duration = 0.0;
  
  /* Maximum window covering all processors */
  int max_look_back_samples;
  int max_look_ahead_samples;

  std::vector<std::string> internal_imports;

  std::size_t buffer_size = 0;
  std::size_t eeg_data_size;
  std::size_t emg_data_size;

  bool process_sensory_stimuli_list(
    const py::list& py_sensory_stimuli,
    std::vector<pipeline_interfaces::msg::SensoryStimulus>& sensory_stimuli);

  void fill_arrays_from_buffer(
    const RingBuffer<std::shared_ptr<eeg_msgs::msg::Sample>>& buffer,
    double_t sample_window_base_time,
    py::array_t<double>& timestamps,
    py::array_t<bool>& valid,
    py::array_t<double>& eeg_data,
    py::array_t<double>& emg_data,
    size_t start_offset,
    size_t num_samples);
};

#endif
