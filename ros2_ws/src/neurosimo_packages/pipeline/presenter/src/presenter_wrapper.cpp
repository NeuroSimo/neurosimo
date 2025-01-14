#include <pybind11/pybind11.h>
#include <pybind11/embed.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include "presenter_wrapper.h"

namespace py = pybind11;

PresenterWrapper::PresenterWrapper(rclcpp::Logger& logger) {
  logger_ptr = &logger;
  _is_initialized = false;
  guard = std::make_unique<py::scoped_interpreter>();
}

void PresenterWrapper::initialize_module(
    const std::string& directory,
    const std::string& module_name) {

  /* If we have an existing presenter instance, release it which will call the destructor */
  presenter_instance = nullptr;
  presenter_module = nullptr;

  /* Set the sys.path to include the directory of the module. */
  py::module sys_module = py::module::import("sys");
  py::list sys_path = sys_module.attr("path");
  sys_path.append(directory);

  /* Remove the module from sys.modules if it exists, to ensure it is reloaded. */
  py::dict sys_modules = sys_module.attr("modules");
  if (sys_modules.contains(module_name.c_str())) {
    sys_modules.attr("__delitem__")(module_name.c_str());
  }

  /* Import the module and initialize the presenter instance. */
  try {
    auto imported_module = py::module::import(module_name.c_str());
    presenter_module = std::make_unique<py::module>(imported_module);
    auto instance = presenter_module->attr("Presenter")();
    presenter_instance = std::make_unique<py::object>(instance);

  } catch(const py::error_already_set& e) {
    RCLCPP_ERROR(*logger_ptr, "Python error: %s", e.what());
    this->_is_initialized = false;
    return;

  } catch(const std::exception& e) {
    RCLCPP_ERROR(*logger_ptr, "C++ error: %s", e.what());
    this->_is_initialized = false;
    return;
  }

  RCLCPP_INFO(*logger_ptr, "Presenter set to: %s.", module_name.c_str());

  this->_is_initialized = true;
  this->_error_occurred = false;
}

void PresenterWrapper::reset_module_state() {
  presenter_module = nullptr;
  presenter_instance = nullptr;

  this->_is_initialized = false;
  this->_error_occurred = false;
}

bool PresenterWrapper::is_initialized() const {
  return this->_is_initialized;
}

bool PresenterWrapper::error_occurred() const {
  return this->_error_occurred;
}

bool PresenterWrapper::process(pipeline_interfaces::msg::SensoryStimulus& msg) {
  auto state = msg.state;
  auto parameter = msg.parameter;
  auto duration = msg.duration;

  /* Call the Python function. */
  py::object py_result;
  try {
    py_result = presenter_instance->attr("process")(state, parameter, duration);

  } catch(const py::error_already_set& e) {
    RCLCPP_ERROR(*logger_ptr, "Python error: %s", e.what());
    this->_error_occurred = true;
    return false;

  } catch(const std::exception& e) {
    RCLCPP_ERROR(*logger_ptr, "C++ error: %s", e.what());
    this->_error_occurred = true;
    return false;
  }

  /* Validate the return value of the Python function call. */
  if (!py::isinstance<py::bool_>(py_result)) {
    RCLCPP_ERROR(*logger_ptr, "Python module should return a boolean.");
    this->_error_occurred = true;
    return false;
  }

  bool result = py_result.cast<bool>();
  return result;
}

rclcpp::Logger* PresenterWrapper::logger_ptr = nullptr;
