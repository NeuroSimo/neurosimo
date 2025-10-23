#include <pybind11/pybind11.h>
#include <pybind11/embed.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include "presenter_wrapper.h"

namespace py = pybind11;

PresenterWrapper::PresenterWrapper(rclcpp::Logger& logger) {
  logger_ptr = &logger;
  _is_initialized = false;

  /* Initialize the Python interpreter. */
  interpreter = std::make_unique<py::scoped_interpreter>();
  setup_custom_print();
}

void PresenterWrapper::setup_custom_print() {
    py::exec(R"(
import builtins
import cpp_bindings

def custom_print(*args, sep=' ', end='\n', file=None, flush=False):
    output = sep.join(map(str, args))
    cpp_bindings.log(output)
    if file is not None:
        file.write(output + end)
        if flush:
            file.flush()

def print_throttle(*args, period=1.0, sep=' ', end='\n', file=None, flush=False):
    assert period > 0.0, 'The period must be greater than zero.'
    output = sep.join(map(str, args))
    cpp_bindings.log_throttle(output, period)
    if file is not None:
        file.write(output + end)
        if flush:
            file.flush()

builtins.print = custom_print
builtins.print_throttle = print_throttle
    )", py::globals());
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
  // Extract fields
  std::string type = msg.type;

  // Build a py::dict for parameters, parsing numbers when possible
  py::dict py_params;
  for (const auto &kv : msg.parameters) {
    const std::string &valstr = kv.value;
    py::object pyval;

    try {
      if (valstr.find_first_of(".eE") == std::string::npos) {
        int i = std::stoi(valstr);
        pyval = py::int_(i);
      } else {
        double d = std::stod(valstr);
        pyval = py::float_(d);
      }
    }
    catch (...) {
      pyval = py::str(valstr);
    }

    py_params[py::str(kv.key)] = pyval;
  }

  // Call Python: process(type, parameters_dict)
  py::object py_result;
  try {
    py_result = presenter_instance
                  ->attr("process")(type, py_params);
  }
  catch (const py::error_already_set &e) {
    RCLCPP_ERROR(*logger_ptr,
      "Python error in presenter.process: %s", e.what());
    this->_error_occurred = true;
    return false;
  }
  catch (const std::exception &e) {
    RCLCPP_ERROR(*logger_ptr,
      "C++ exception in presenter.process: %s", e.what());
    this->_error_occurred = true;
    return false;
  }

  // Validate return type
  if (!py::isinstance<py::bool_>(py_result)) {
    // convert the Python type object to a C++ string
    std::string got_type = py::str(py_result.get_type());
    RCLCPP_ERROR(*logger_ptr,
      "presenter.process must return a bool, got %s",
      got_type.c_str());
    this->_error_occurred = true;
    return false;
  }

  return py_result.cast<bool>();
}

std::vector<LogEntry> PresenterWrapper::get_and_clear_logs() {
  std::lock_guard<std::mutex> lock(log_buffer_mutex);
  std::vector<LogEntry> logs = std::move(log_buffer);
  log_buffer.clear();
  return logs;
}

rclcpp::Logger* PresenterWrapper::logger_ptr = nullptr;
std::vector<LogEntry> PresenterWrapper::log_buffer;
std::mutex PresenterWrapper::log_buffer_mutex;
