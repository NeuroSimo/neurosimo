#ifndef PREPROCESSOR_WRAPPER_H
#define PREPROCESSOR_WRAPPER_H

#include <pybind11/pybind11.h>
#include <pybind11/embed.h>
#include <pybind11/numpy.h>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "eeg_msgs/msg/sample.hpp"
#include "eeg_msgs/msg/preprocessed_sample.hpp"
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

class PreprocessorWrapper {
public:
  PreprocessorWrapper(rclcpp::Logger& logger);
  ~PreprocessorWrapper();

  void initialize_module(
      const std::string& directory,
      const std::string& module_name,
      const size_t eeg_data_size,
      const size_t emg_data_size,
      const uint16_t sampling_frequency);

  void reset_module_state();

  bool process(
      eeg_msgs::msg::PreprocessedSample& output_sample,
      const RingBuffer<std::shared_ptr<eeg_msgs::msg::Sample>>& buffer,
      double_t sample_time,
      bool pulse_given);

  WrapperState get_state() const;
  std::size_t get_buffer_size() const;

  /* Exposed to Python, defined in cpp_bindings.cpp. */
  static void log(const std::string& message);

  /* Exposed to Python, defined in cpp_bindings.cpp. */
  static void log_throttle(const std::string& message, const double_t period);

private:
  /* XXX: Have a static ROS2 logger to expose it more easily to the Python side (see cpp_bindings.cpp). */
  static rclcpp::Logger* logger_ptr;

  WrapperState state;

  std::unique_ptr<py::module> preprocessor_module;
  std::unique_ptr<py::object> preprocessor_instance;

  std::unique_ptr<py::scoped_interpreter> guard;

  std::unique_ptr<py::array_t<double>> py_timestamps;
  std::unique_ptr<py::array_t<double>> py_eeg_data;
  std::unique_ptr<py::array_t<double>> py_emg_data;

  int earliest_sample;
  int latest_sample;
  uint16_t sampling_frequency;

  std::size_t buffer_size = 0;
  std::size_t eeg_data_size;
  std::size_t emg_data_size;
};

#endif
