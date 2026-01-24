#include <pybind11/pybind11.h>
#include <pybind11/embed.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include "presenter_wrapper.h"

namespace py = pybind11;

PresenterWrapper::PresenterWrapper(rclcpp::Logger& logger) {
  logger_ptr = &logger;

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

bool PresenterWrapper::initialize_module(
    const std::string& directory,
    const std::string& module_name,
    const std::string& subject_id) {

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
    auto instance = presenter_module->attr("Presenter")(subject_id);
    presenter_instance = std::make_unique<py::object>(instance);

  } catch(const py::error_already_set& e) {
    std::string error_msg = std::string("Python error: ") + e.what();
    RCLCPP_ERROR(*logger_ptr, "%s", error_msg.c_str());
    
    // Add error to log buffer so it can be published to UI
    {
      std::lock_guard<std::mutex> lock(log_buffer_mutex);
      log_buffer.push_back({error_msg, LogLevel::ERROR});
    }
    return false;

  } catch(const std::exception& e) {
    std::string error_msg = std::string("C++ error: ") + e.what();
    RCLCPP_ERROR(*logger_ptr, "%s", error_msg.c_str());
    
    // Add error to log buffer so it can be published to UI
    {
      std::lock_guard<std::mutex> lock(log_buffer_mutex);
      log_buffer.push_back({error_msg, LogLevel::ERROR});
    }
    return false;
  }

  /* Check that the Python module has a get_configuration method (mandatory). */
  if (!py::hasattr(*presenter_instance, "get_configuration")) {
    RCLCPP_ERROR(*logger_ptr, "Presenter module must implement 'get_configuration' method.");
    return false;
  }

  /* Extract the configuration from presenter_instance. */
  py::dict config = presenter_instance->attr("get_configuration")().cast<py::dict>();

  /* Validate that only allowed keys are present */
  std::vector<std::string> allowed_keys = {"stimulus_processors"};
  for (const auto& item : config) {
    std::string key = py::str(item.first).cast<std::string>();
    if (std::find(allowed_keys.begin(), allowed_keys.end(), key) == allowed_keys.end()) {
      RCLCPP_ERROR(*logger_ptr, "Unexpected key '%s' in configuration dictionary. Only 'stimulus_processors' is allowed.", key.c_str());
      return false;
    }
  }

  /* Check that the configuration contains a 'stimulus_processors' key. */
  if (!config.contains("stimulus_processors")) {
    RCLCPP_ERROR(*logger_ptr, "Configuration must contain 'stimulus_processors' key.");
    return false;
  }

  /* Check that the 'stimulus_processors' value is a dictionary. */
  if (!py::isinstance<py::dict>(config["stimulus_processors"])) {
    RCLCPP_ERROR(*logger_ptr, "stimulus_processors must be a dictionary.");
    return false;
  }

  /* Extract stimulus_processors. */
  py::dict processors = config["stimulus_processors"].cast<py::dict>();
  this->stimulus_processors.clear();
  
  for (const auto& item : processors) {
    std::string stimulus_type = py::str(item.first).cast<std::string>();
    py::object processor = item.second.cast<py::object>();
    
    /* Verify that the processor is callable */
    if (!py::hasattr(processor, "__call__")) {
      RCLCPP_ERROR(*logger_ptr, "Stimulus processor for '%s' is not callable.", stimulus_type.c_str());
      return false;
    }
    
    this->stimulus_processors[stimulus_type] = processor;
    RCLCPP_DEBUG(*logger_ptr, "Registered stimulus processor for: %s", stimulus_type.c_str());
  }

  RCLCPP_INFO(*logger_ptr, "Presenter set to: %s.", module_name.c_str());

  return true;
}

bool PresenterWrapper::reset_module_state() {
  presenter_module = nullptr;
  presenter_instance = nullptr;
  stimulus_processors.clear();

  return true;
}

bool PresenterWrapper::process(pipeline_interfaces::msg::SensoryStimulus& msg) {
  // Extract fields
  std::string type = msg.type;

  // Look up the processor for this stimulus type
  auto processor_it = stimulus_processors.find(type);
  if (processor_it == stimulus_processors.end()) {
    std::string error_msg = std::string("No stimulus processor registered for type: ") + type;
    RCLCPP_ERROR(*logger_ptr, "%s", error_msg.c_str());
    
    // Add error to log buffer so it can be published to UI
    {
      std::lock_guard<std::mutex> lock(log_buffer_mutex);
      log_buffer.push_back({error_msg, LogLevel::ERROR});
    }
    return false;
  }

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

  // Call Python processor with parameters
  py::object py_result;
  try {
    py_result = processor_it->second(py_params);
  }
  catch (const py::error_already_set &e) {
    std::string error_msg = std::string("Python error in stimulus processor '") + type + "': " + e.what();
    RCLCPP_ERROR(*logger_ptr, "%s", error_msg.c_str());
    
    // Add error to log buffer so it can be published to UI
    {
      std::lock_guard<std::mutex> lock(log_buffer_mutex);
      log_buffer.push_back({error_msg, LogLevel::ERROR});
    }
    return false;
  }
  catch (const std::exception &e) {
    std::string error_msg = std::string("C++ exception in stimulus processor '") + type + "': " + e.what();
    RCLCPP_ERROR(*logger_ptr, "%s", error_msg.c_str());
    
    // Add error to log buffer so it can be published to UI
    {
      std::lock_guard<std::mutex> lock(log_buffer_mutex);
      log_buffer.push_back({error_msg, LogLevel::ERROR});
    }
    return false;
  }

  // Validate return type
  if (!py::isinstance<py::bool_>(py_result)) {
    // convert the Python type object to a C++ string
    std::string got_type = py::str(py_result.get_type());
    RCLCPP_ERROR(*logger_ptr,
      "Stimulus processor for '%s' must return a bool, got %s",
      type.c_str(), got_type.c_str());

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
