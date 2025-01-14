#ifndef presenter_WRAPPER_H
#define presenter_WRAPPER_H

#include <pybind11/pybind11.h>
#include <pybind11/embed.h>
#include <pybind11/numpy.h>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "pipeline_interfaces/msg/sensory_stimulus.hpp"

namespace py = pybind11;

class PresenterWrapper {
public:
  PresenterWrapper(rclcpp::Logger& logger);

  void initialize_module(
      const std::string& directory,
      const std::string& module_name);

  void reset_module_state();

  bool process(pipeline_interfaces::msg::SensoryStimulus& msg);

  bool is_initialized() const;
  bool error_occurred() const;

  /* Exposed to Python, defined in cpp_bindings.cpp. */
  static void log(const std::string& message);

  /* Exposed to Python, defined in cpp_bindings.cpp. */
  static void log_throttle(const std::string& message, const double_t period);

private:
  /* XXX: Have a static ROS2 logger to expose it more easily to the Python side (see cpp_bindings.cpp). */
  static rclcpp::Logger* logger_ptr;

  bool _is_initialized;
  bool _error_occurred;

  std::unique_ptr<py::module> presenter_module;
  std::unique_ptr<py::object> presenter_instance;

  std::unique_ptr<py::scoped_interpreter> guard;
};

#endif
