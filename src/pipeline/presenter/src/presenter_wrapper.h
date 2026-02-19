#ifndef presenter_WRAPPER_H
#define presenter_WRAPPER_H

#include <pybind11/pybind11.h>
#include <pybind11/embed.h>
#include <pybind11/numpy.h>
#include <memory>
#include <string>
#include <map>

#include "rclcpp/rclcpp.hpp"

#include "pipeline_interfaces/msg/sensory_stimulus.hpp"
#include "log_ipc_server.h"

namespace py = pybind11;

enum class LogLevel : uint8_t {
  INFO = 0,
  WARNING = 1,
  ERROR = 2
};

struct LogEntry {
  std::string message;
  LogLevel level;
};

class PresenterWrapper {
public:
  PresenterWrapper(rclcpp::Logger& logger);
  ~PresenterWrapper();

  void setup_custom_print();

  bool initialize_module(
      const std::string& directory,
      const std::string& module_name,
      const std::string& subject_id);

  bool reset_module_state();

  bool process(pipeline_interfaces::msg::SensoryStimulus& msg);

  /* Exposed to Python, defined in cpp_bindings.cpp. */
  static void log(const std::string& message);

  /* Get buffered logs and clear the buffer */
  std::vector<LogEntry> get_and_clear_logs();

  /* Drain any pending log messages. Call at session end. */
  void drain_logs();

private:
  /* XXX: Have a static ROS2 logger to expose it more easily to the Python side (see cpp_bindings.cpp). */
  static rclcpp::Logger* logger_ptr;

  /* Buffer for Python logs - static to be accessible from static log functions */
  static std::vector<LogEntry> log_buffer;
  static std::mutex log_buffer_mutex;

  /* IPC server for receiving logs from multiprocessing workers */
  std::unique_ptr<LogIpcServer> log_server;

  std::unique_ptr<py::module> presenter_module;
  std::unique_ptr<py::object> presenter_instance;

  std::unique_ptr<py::scoped_interpreter> interpreter;
  
  /* Map from stimulus type to Python processor callable */
  std::map<std::string, py::object> stimulus_processors;
};

#endif
