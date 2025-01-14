#include <pybind11/pybind11.h>
#include <pybind11/embed.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include "preprocessor_wrapper.h"

namespace py = pybind11;

void PreprocessorWrapper::log(const std::string& message) {
  RCLCPP_INFO(*logger_ptr, "[Python]: %s", message.c_str());
}

void PreprocessorWrapper::log_throttle(const std::string& message, const double_t period) {
  static double_t last_log_time = 0.0;

  double_t current_time = rclcpp::Clock().now().seconds();
  if (current_time - last_log_time < period) {
    return;
  }
  RCLCPP_INFO(*logger_ptr, "[Python, throttled]: %s", message.c_str());

  last_log_time = current_time;
}

PYBIND11_MODULE(cpp_bindings, m) {
    m.def("log", &PreprocessorWrapper::log);
    m.def("log_throttle", &PreprocessorWrapper::log_throttle);
}
