#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "decider_wrapper.h"

namespace py = pybind11;

void DeciderWrapper::log(const std::string& message) {
  std::lock_guard<std::mutex> lock(log_buffer_mutex);
  log_buffer.push_back({message, LogLevel::INFO});
}

void DeciderWrapper::log_throttle(const std::string& message, const double_t period) {
  static std::unordered_map<std::string, double_t> last_log_times;

  const double_t current_time = rclcpp::Clock().now().seconds();

  auto it = last_log_times.find(message);
  if (it != last_log_times.end() && current_time - it->second < period) {
    return;
  }

  std::lock_guard<std::mutex> lock(log_buffer_mutex);
  log_buffer.push_back({"[Throttled] " + message, LogLevel::INFO});

  last_log_times[message] = current_time;
}

PYBIND11_MODULE(cpp_bindings, m) {
  m.def("log", &DeciderWrapper::log, py::arg("message"));
  m.def("log_throttle", &DeciderWrapper::log_throttle,
        py::arg("message"), py::arg("period") = 1.0);
}
