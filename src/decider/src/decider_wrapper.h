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

#include "neurosimo_eeg_interfaces/msg/sample.hpp"

#include "shared_stimulation_interfaces/msg/coil_target.hpp"
#include "shared_stimulation_interfaces/msg/targeted_pulse.hpp"

#include "neurosimo_pipeline_interfaces/msg/sensory_stimulus.hpp"
#include "neurosimo_pipeline_interfaces/msg/log_message.hpp"

#include "std_msgs/msg/string.hpp"

#include "ring_buffer.h"
#include "log_ipc_server.h"

namespace py = pybind11;

const std::string bold_on = "\033[1m";
const std::string bold_off = "\033[0m";

enum class ProcessingReason {
  Pulse,
  Event,
  Periodic
};

enum class LogLevel : uint8_t {
  INFO = 0,
  WARNING = 1,
  ERROR = 2
};

struct LogEntry {
  std::string message;
  LogLevel level;
  uint8_t processing_path;
};

class DeciderWrapper {
public:
  DeciderWrapper(rclcpp::Logger& logger);
  ~DeciderWrapper();

  bool initialize_module(
      const std::string& project_directory,
      const std::string& module_directory,
      const std::string& module_name,
      const std::string& subject_id,
      const size_t eeg_size,
      const size_t emg_size,
      const uint16_t sampling_frequency,
      std::vector<neurosimo_pipeline_interfaces::msg::SensoryStimulus>& sensory_stimuli,
      std::priority_queue<double, std::vector<double>, std::greater<double>>& event_queue);

  bool warm_up();

  bool parse_sensory_stimulus_dict(
    const py::dict& py_sensory_stimulus,
    neurosimo_pipeline_interfaces::msg::SensoryStimulus& out_msg);

  std::tuple<
    bool,
    std::shared_ptr<double_t>,
    std::string,
    std::vector<shared_stimulation_interfaces::msg::TargetedPulse>> process(
    std::vector<neurosimo_pipeline_interfaces::msg::SensoryStimulus>& sensory_stimuli,
    const RingBuffer<std::shared_ptr<neurosimo_eeg_interfaces::msg::Sample>>& buffer,
    double_t sample_window_base_time,
    ProcessingReason processing_reason,
    std::priority_queue<double, std::vector<double>, std::greater<double>>& event_queue,
    bool is_coil_at_target,
    const std::string& stage_name);

  std::size_t get_envelope_buffer_size() const;
  double get_periodic_processing_interval() const;
  double get_first_periodic_processing_at() const;
  bool is_processing_interval_enabled() const;
  int get_periodic_look_ahead_samples() const;
  int get_pulse_look_ahead_samples() const;
  int get_event_look_ahead_samples() const;
  double get_pulse_lockout_duration() const;

  void setup_custom_print();

  /* log is exposed to Python, defined in cpp_bindings.cpp. */
  static void log(const std::string& message);

  /* Get buffered logs and clear the buffer */
  std::vector<LogEntry> get_and_clear_logs();

  /* Drain any pending log messages. Call at session end. */
  void drain_logs();

  /* Set the current processing path for log messages */
  static void set_current_processing_path(uint8_t processing_path);

  /* Handle incoming IPC log messages */
  void handle_ipc_log_message(std::string&& msg);

  /* Destroy the Python decider instance, triggering __del__. */
  void destroy_instance();

  void log_section_header(const std::string& title);

private:
  void log_error(const std::string& message);
  /* XXX: Have a static ROS2 logger to expose it more easily to the Python side (see cpp_bindings.cpp). */
  static rclcpp::Logger* logger_ptr;

  /* Buffer for Python logs - static to be accessible from static log functions */
  static std::vector<LogEntry> log_buffer;

  /* Current processing path for log messages */
  static uint8_t current_processing_path;

  /* IPC server for receiving logs from multiprocessing workers */
  std::unique_ptr<LogIpcServer> log_server;

  std::unique_ptr<py::module> decider_module;
  std::unique_ptr<py::object> decider_instance;

  std::unique_ptr<py::scoped_interpreter> interpreter;

  /* Processor functions */
  py::object pulse_processor;
  py::object event_processor;

  /* Preallocated numpy arrays for periodic processor */
  std::unique_ptr<py::array_t<double>> periodic_time_offsets;
  std::unique_ptr<py::array_t<double>> periodic_eeg;
  std::unique_ptr<py::array_t<double>> periodic_emg;
  
  /* Preallocated numpy arrays for pulse processor */
  std::unique_ptr<py::array_t<double>> pulse_time_offsets;
  std::unique_ptr<py::array_t<double>> pulse_eeg;
  std::unique_ptr<py::array_t<double>> pulse_emg;
  
  /* Preallocated numpy arrays for event processor */
  std::unique_ptr<py::array_t<double>> event_time_offsets;
  std::unique_ptr<py::array_t<double>> event_eeg;
  std::unique_ptr<py::array_t<double>> event_emg;

  std::unordered_map<std::string, std::chrono::steady_clock::time_point> last_log_time;

  uint16_t sampling_frequency = 0;
  bool periodic_processing_enabled = false;
  double periodic_processing_interval = 0.0;
  double first_periodic_processing_at = 0.0;
  double pulse_lockout_duration = 0.0;
  uint16_t warm_up_rounds = 0;

  /* Window parameters for periodic processing */
  int periodic_sample_window_start = 0;
  int periodic_sample_window_end = 0;
  std::size_t periodic_window_start_offset_in_envelope = 0;

  /* Window parameters for pulse processor */
  bool has_custom_pulse_window = false;
  int pulse_sample_window_start = 0;
  int pulse_sample_window_end = 0;
  std::size_t pulse_window_start_offset_in_envelope = 0;

  /* Window parameters for event processor */
  bool has_custom_event_window = false;
  int event_sample_window_start = 0;
  int event_sample_window_end = 0;
  std::size_t event_window_start_offset_in_envelope = 0;

  std::size_t envelope_buffer_size = 0;
  std::size_t eeg_size = 0;
  std::size_t emg_size = 0;

  bool process_sensory_stimuli_list(
    const py::list& py_sensory_stimuli,
    std::vector<neurosimo_pipeline_interfaces::msg::SensoryStimulus>& sensory_stimuli);

  bool parse_targeted_pulse_dict(
    const py::dict& py_targeted_pulse,
    shared_stimulation_interfaces::msg::TargetedPulse& out_msg);

  bool process_targeted_pulses_list(
    const py::list& py_targeted_pulses,
    std::vector<shared_stimulation_interfaces::msg::TargetedPulse>& targeted_pulses);

  void fill_arrays_from_buffer(
    const RingBuffer<std::shared_ptr<neurosimo_eeg_interfaces::msg::Sample>>& buffer,
    double_t sample_window_base_time,
    py::array_t<double>& timestamps,
    py::array_t<double>& eeg,
    py::array_t<double>& emg,
    size_t start_offset,
    size_t num_samples);
};

#endif
