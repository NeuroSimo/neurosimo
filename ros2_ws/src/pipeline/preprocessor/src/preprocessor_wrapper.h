#ifndef PREPROCESSOR_WRAPPER_H
#define PREPROCESSOR_WRAPPER_H

#include <pybind11/pybind11.h>
#include <pybind11/embed.h>
#include <pybind11/numpy.h>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "eeg_interfaces/msg/sample.hpp"
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

class PreprocessorWrapper {
public:
  PreprocessorWrapper(rclcpp::Logger& logger);
  ~PreprocessorWrapper();

  void initialize_module(
      const std::string& directory,
      const std::string& module_name,
      const size_t eeg_size,
      const size_t emg_size,
      const uint16_t sampling_frequency);

  void reset_module_state();

  bool process(
      eeg_interfaces::msg::Sample& output_sample,
      const RingBuffer<std::shared_ptr<eeg_interfaces::msg::Sample>>& buffer,
      double_t sample_window_base_time,
      bool pulse_given);

  WrapperState get_state() const;
  std::size_t get_buffer_size() const;
  int get_look_ahead_samples() const;

  /* Exposed to Python, defined in cpp_bindings.cpp. */
  static void log(const std::string& message);

  /* Exposed to Python, defined in cpp_bindings.cpp. */
  static void log_throttle(const std::string& message, const double_t period);

  /* Get buffered logs and clear the buffer */
  std::vector<LogEntry> get_and_clear_logs();

private:
  void setup_custom_print();
  /* XXX: Have a static ROS2 logger to expose it more easily to the Python side (see cpp_bindings.cpp). */
  static rclcpp::Logger* logger_ptr;

  /* Buffer for Python logs - static to be accessible from static log functions */
  static std::vector<LogEntry> log_buffer;
  static std::mutex log_buffer_mutex;

  WrapperState state;

  std::unique_ptr<py::module> preprocessor_module;
  std::unique_ptr<py::object> preprocessor_instance;

  std::unique_ptr<py::scoped_interpreter> guard;

  std::unique_ptr<py::array_t<double>> py_time_offsets;
  std::unique_ptr<py::array_t<double>> py_eeg;
  std::unique_ptr<py::array_t<double>> py_emg;

  int look_back_samples;
  int look_ahead_samples;
  uint16_t sampling_frequency;

  std::size_t buffer_size = 0;
  std::size_t eeg_size;
  std::size_t emg_size;
};

#endif
