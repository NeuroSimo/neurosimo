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

#include "eeg_interfaces/msg/sample.hpp"

#include "pipeline_interfaces/msg/coil_target.hpp"

#include "pipeline_interfaces/msg/sensory_stimulus.hpp"
#include "pipeline_interfaces/msg/timed_trigger.hpp"

#include "std_msgs/msg/string.hpp"

#include "ring_buffer.h"
#include "log_ipc_server.h"

namespace py = pybind11;

const std::string bold_on = "\033[1m";
const std::string bold_off = "\033[0m";

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

  bool initialize_module(
      const std::string& project_directory,
      const std::string& module_directory,
      const std::string& module_name,
      const std::string& subject_id,
      const size_t eeg_size,
      const size_t emg_size,
      const uint16_t sampling_frequency,
      std::vector<pipeline_interfaces::msg::SensoryStimulus>& sensory_stimuli,
      std::priority_queue<double, std::vector<double>, std::greater<double>>& event_queue);

  bool reset_module_state();

  bool warm_up();

  bool parse_sensory_stimulus_dict(
    const py::dict& py_sensory_stimulus,
    pipeline_interfaces::msg::SensoryStimulus& out_msg);
  std::tuple<bool, std::shared_ptr<pipeline_interfaces::msg::TimedTrigger>, std::string> process(
    std::vector<pipeline_interfaces::msg::SensoryStimulus>& sensory_stimuli,
    const RingBuffer<std::shared_ptr<eeg_interfaces::msg::Sample>>& buffer,
    double_t sample_window_base_time,
    bool pulse_trigger,
    bool has_event,
    std::priority_queue<double, std::vector<double>, std::greater<double>>& event_queue,
    bool is_coil_at_target);

  std::size_t get_buffer_size() const;
  double get_periodic_processing_interval() const;
  double get_first_periodic_processing_at() const;
  bool is_processing_interval_enabled() const;
  int get_look_ahead_samples() const;
  int get_look_ahead_samples_for_pulse() const;
  int get_look_ahead_samples_for_event() const;
  double get_pulse_lockout_duration() const;

  void setup_custom_print();

  /* log is exposed to Python, defined in cpp_bindings.cpp. */
  static void log(const std::string& message);

  /* Get buffered logs and clear the buffer */
  std::vector<LogEntry> get_and_clear_logs();

  /* Drain any pending log messages. Call at session end. */
  void drain_logs();

  /* Destroy the Python decider instance, triggering __del__. */
  void destroy_instance();

  void log_section_header(const std::string& title);

private:
  /* XXX: Have a static ROS2 logger to expose it more easily to the Python side (see cpp_bindings.cpp). */
  static rclcpp::Logger* logger_ptr;

  /* Buffer for Python logs - static to be accessible from static log functions */
  static std::vector<LogEntry> log_buffer;
  static std::mutex log_buffer_mutex;

  /* IPC server for receiving logs from multiprocessing workers */
  std::unique_ptr<LogIpcServer> log_server;

  std::unique_ptr<py::module> decider_module;
  std::unique_ptr<py::object> decider_instance;

  std::unique_ptr<py::scoped_interpreter> interpreter;

  /* Processor functions */
  py::object pulse_processor;
  py::object event_processor;

  /* Preallocated numpy arrays for default window */
  std::unique_ptr<py::array_t<double>> py_time_offsets;
  std::unique_ptr<py::array_t<double>> py_eeg;
  std::unique_ptr<py::array_t<double>> py_emg;
  
  /* Preallocated numpy arrays for pulse processor (if custom window) */
  std::unique_ptr<py::array_t<double>> pulse_time_offsets;
  std::unique_ptr<py::array_t<double>> pulse_eeg;
  std::unique_ptr<py::array_t<double>> pulse_emg;
  
  /* Preallocated numpy arrays for event processor (if custom window) */
  std::unique_ptr<py::array_t<double>> event_time_offsets;
  std::unique_ptr<py::array_t<double>> event_eeg;
  std::unique_ptr<py::array_t<double>> event_emg;

  std::unordered_map<std::string, std::chrono::steady_clock::time_point> last_log_time;

  int look_back_samples;
  int look_ahead_samples;
  uint16_t sampling_frequency;
  bool periodic_processing_enabled = false;
  double periodic_processing_interval = 0.0;
  double first_periodic_processing_at = 0.0;
  double pulse_lockout_duration = 0.0;
  
  /* Custom window parameters for pulse processor */
  bool has_custom_pulse_window = false;
  int pulse_look_back_samples = 0;
  int pulse_look_ahead_samples = 0;
  
  /* Custom window parameters for event processor */
  bool has_custom_event_window = false;
  int event_look_back_samples = 0;
  int event_look_ahead_samples = 0;

  std::size_t buffer_size = 0;
  std::size_t eeg_size;
  std::size_t emg_size;

  bool process_sensory_stimuli_list(
    const py::list& py_sensory_stimuli,
    std::vector<pipeline_interfaces::msg::SensoryStimulus>& sensory_stimuli);

  void fill_arrays_from_buffer(
    const RingBuffer<std::shared_ptr<eeg_interfaces::msg::Sample>>& buffer,
    double_t sample_window_base_time,
    py::array_t<double>& timestamps,
    py::array_t<double>& eeg,
    py::array_t<double>& emg,
    size_t start_offset,
    size_t num_samples);
};

#endif
