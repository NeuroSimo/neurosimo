#include <pybind11/pybind11.h>
#include <pybind11/embed.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <algorithm>
#include <cmath>

#include "preprocessor_wrapper.h"
#include <eeg_interfaces/msg/sample.hpp>

namespace py = pybind11;

PreprocessorWrapper::PreprocessorWrapper(rclcpp::Logger& logger) {
  logger_ptr = &logger;

  /* Start IPC log server first, so cpp_bindings.log() can forward immediately. */
  log_server = std::make_unique<LogIpcServer>([](std::string&& msg) {
    std::lock_guard<std::mutex> lock(log_buffer_mutex);
    log_buffer.push_back({std::move(msg), LogLevel::INFO});
  });
  if (!log_server->start()) {
    RCLCPP_WARN(*logger_ptr, "Failed to start log IPC server; worker logs may be lost.");
  }

  interpreter = std::make_unique<py::scoped_interpreter>();
  setup_custom_print();
}

void PreprocessorWrapper::setup_custom_print() {
    py::exec(R"(
import builtins
import multiprocessing
import cpp_bindings

# Force fork so workers inherit this print patch (Linux-only).
try:
    multiprocessing.set_start_method("fork", force=True)
except RuntimeError:
    pass

def _print(*args, sep=' ', end='\n', file=None, flush=False):
    msg = sep.join(map(str, args)) + end
    cpp_bindings.log(msg)

builtins.print = _print
    )", py::globals());
}

void PreprocessorWrapper::log_error(const std::string& message) {
  RCLCPP_ERROR(*logger_ptr, "%s", message.c_str());
  log_buffer.push_back({message, LogLevel::ERROR});
}

bool PreprocessorWrapper::initialize_module(
    const std::string& directory,
    const std::string& module_name,
    const std::string& subject_id,
    const size_t eeg_size,
    const size_t emg_size,
    const uint16_t sampling_frequency) {

  this->sampling_frequency = sampling_frequency;
  if (this->sampling_frequency == 0) {
    log_error("Sampling frequency must be greater than zero to interpret sample_window expressed in seconds.");
    return false;
  }

  /* Set the sys.path to include the directory of the module. */
  py::module sys_module = py::module::import("sys");
  py::list sys_path = sys_module.attr("path");
  sys_path.append(directory);

  /* Import the module and initialize the Preprocessor instance. */
  try {
    auto imported_module = py::module::import(module_name.c_str());
    preprocessor_module = std::make_unique<py::module>(imported_module);
    auto instance = preprocessor_module->attr("Preprocessor")(subject_id, eeg_size, emg_size, sampling_frequency);
    preprocessor_instance = std::make_unique<py::object>(instance);

  } catch(const py::error_already_set& e) {
    std::string error_msg = std::string("Python error: ") + e.what();
    log_error(error_msg);
    return false;

  } catch(const std::exception& e) {
    std::string error_msg = std::string("C++ error: ") + e.what();
    log_error(error_msg);
    return false;
  }

  /* Helper to convert seconds to sample counts. */
  const auto to_samples = [this](double seconds) -> int {
    return static_cast<int>(std::ceil(seconds * static_cast<double>(this->sampling_frequency)));
  };

  double sample_window_start_seconds = 0.0;
  double sample_window_end_seconds = 0.0;
  py::dict config;

  /* Extract the configuration from preprocessor_instance. */
  if (!py::hasattr(*preprocessor_instance, "get_configuration")) {
    log_error("get_configuration method not found in the Preprocessor instance.");
    return false;
  }

  try {
    config = preprocessor_instance->attr("get_configuration")().cast<py::dict>();

  } catch(const py::error_already_set& e) {
    std::string error_msg = std::string("Python error during get_configuration call: ") + e.what();
    log_error(error_msg);
    return false;

  } catch(const std::exception& e) {
    std::string error_msg = std::string("C++ error during get_configuration call: ") + e.what();
    log_error(error_msg);
    return false;
  }

  /* Extract sample_window. */
  if (!config.contains("sample_window")) {
    log_error("'sample_window' key not found in configuration dictionary.");
    return false;
  }

  py::list sample_window = config["sample_window"].cast<py::list>();

  if (sample_window.size() == 2) {
    sample_window_start_seconds = sample_window[0].cast<double>();
    sample_window_end_seconds = sample_window[1].cast<double>();
    if (sample_window_start_seconds > sample_window_end_seconds) {
      std::string error_msg = "Invalid sample_window: start (" +
                              std::to_string(sample_window_start_seconds) +
                              " s) must be <= end (" +
                              std::to_string(sample_window_end_seconds) + " s).";
      log_error(error_msg);
      return false;
    }

    this->sample_window_start = to_samples(sample_window_start_seconds);
    this->sample_window_end = to_samples(sample_window_end_seconds);
  } else {
    log_error("'sample_window' value in configuration is of incorrect length (should be two elements).");
    return false;
  }

  /* Similar to decider_wrapper.cpp, the logic below works but is too complex just to cover the case where the sample window is fully in the past. It should be simplified. */

  /* Negative look-ahead is treated as zero by scheduler logic, so the envelope must
     include samples up to max(sample_window_end, 0). */
  const int effective_look_ahead = std::max(this->sample_window_end, 0);
  this->envelope_buffer_size =
      static_cast<std::size_t>(effective_look_ahead - this->sample_window_start + 1);

  const int oldest_offset_in_buffer =
      effective_look_ahead - static_cast<int>(this->envelope_buffer_size) + 1;
  this->sample_window_start_offset_in_envelope =
      static_cast<std::size_t>(this->sample_window_start - oldest_offset_in_buffer);

  /* Initialize numpy arrays. */
  py_time_offsets = std::make_unique<py::array_t<double>>(envelope_buffer_size);

  std::vector<size_t> eeg_shape = {envelope_buffer_size, eeg_size};
  py_eeg = std::make_unique<py::array_t<double>>(eeg_shape);

  std::vector<size_t> emg_shape = {envelope_buffer_size, emg_size};
  py_emg = std::make_unique<py::array_t<double>>(emg_shape);

  this->eeg_size = eeg_size;
  this->emg_size = emg_size;

  /* Log the configuration. */
  RCLCPP_INFO(*logger_ptr, "Configuration:");
  RCLCPP_INFO(*logger_ptr, " ");
  RCLCPP_INFO(*logger_ptr, "  - Sample window: %s[%.3f s, %.3f s]%s",
              bold_on.c_str(),
              sample_window_start_seconds,
              sample_window_end_seconds,
              bold_off.c_str());
  RCLCPP_INFO(*logger_ptr, " ");

  return true;
}

void PreprocessorWrapper::destroy_instance() {
  /* Setting to nullptr decrements the Python refcount; in CPython this triggers __del__ synchronously. */
  preprocessor_instance = nullptr;
}

PreprocessorWrapper::~PreprocessorWrapper() {
  if (log_server) {
    log_server->stop();
  }

  /* XXX: Let the interpreter leak - process is exiting anyway. Without this, the
          process will segfault on exit. Maybe there is a better way to do this? */
  interpreter.release();
}

std::size_t PreprocessorWrapper::get_buffer_size() const {
  return this->envelope_buffer_size;
}

int PreprocessorWrapper::get_look_ahead_samples() const {
  /* For a sample window like [-10, 5], sample_window_end is 5, which represents
     the number of samples we need to look ahead from the triggering sample. */
  return std::max(this->sample_window_end, 0);
}

bool PreprocessorWrapper::process(
    eeg_interfaces::msg::Sample& output_sample,
    const RingBuffer<std::shared_ptr<eeg_interfaces::msg::Sample>>& buffer,
    double_t reference_time,
    bool pulse_given) {

  int reference_index = -this->sample_window_start;
  std::size_t num_samples = this->envelope_buffer_size;

  fill_arrays_from_buffer(
    buffer,
    reference_time,
    *py_time_offsets,
    *py_eeg,
    *py_emg,
    this->sample_window_start_offset_in_envelope,
    num_samples);

  /* Call the Python function. */
  py::object result;
  try {
    result = preprocessor_instance->attr("process")(reference_time, reference_index, *py_time_offsets, *py_eeg, *py_emg, pulse_given);

  } catch(const py::error_already_set& e) {
    std::string error_msg = std::string("Python error: ") + e.what();
    log_error(error_msg);
    return false;

  } catch(const std::exception& e) {
    std::string error_msg = std::string("C++ error: ") + e.what();
    log_error(error_msg);
    return false;
  }

  /* Validate the return value of the Python function call. */
  if (!py::isinstance<py::dict>(result)) {
    log_error("Python module should return a dictionary.");
    return false;
  }

  py::dict dict_result = result.cast<py::dict>();

  if (!dict_result.contains("eeg_sample")) {
    log_error("Python module should return a dictionary with the field: eeg_sample.");
    return false;
  }

  if (!dict_result.contains("emg_sample")) {
    log_error("Python module should return a dictionary with the field: emg_sample.");
    return false;
  }

  if (!dict_result.contains("valid")) {
    log_error("Python module should return a dictionary with the field: valid.");
    return false;
  }

  /* Convert the Python dictionary to a ROS message. */
  output_sample.eeg = dict_result["eeg_sample"].cast<std::vector<double>>();
  output_sample.emg = dict_result["emg_sample"].cast<std::vector<double>>();
  output_sample.valid = dict_result["valid"].cast<bool>();

  output_sample.time = reference_time;

  return true;
}

void PreprocessorWrapper::fill_arrays_from_buffer(
    const RingBuffer<std::shared_ptr<eeg_interfaces::msg::Sample>>& buffer,
    double_t reference_time,
    py::array_t<double>& time_offsets,
    py::array_t<double>& eeg,
    py::array_t<double>& emg,
    std::size_t start_offset,
    std::size_t num_samples) {

  auto time_offsets_ptr = time_offsets.mutable_data();
  auto eeg_ptr = eeg.mutable_data();
  auto emg_ptr = emg.mutable_data();

  std::size_t ring_idx = 0;
  std::size_t out_idx = 0;
  buffer.process_elements([&](const auto& sample_ptr) {
    if (ring_idx >= start_offset && out_idx < num_samples) {
      const auto& sample = *sample_ptr;
      time_offsets_ptr[out_idx] = sample.time - reference_time;
      std::memcpy(eeg_ptr + out_idx * eeg_size, sample.eeg.data(), eeg_size * sizeof(double));
      std::memcpy(emg_ptr + out_idx * emg_size, sample.emg.data(), emg_size * sizeof(double));
      out_idx++;
    }
    ring_idx++;
  });
}

std::vector<LogEntry> PreprocessorWrapper::get_and_clear_logs() {
  std::lock_guard<std::mutex> lock(log_buffer_mutex);
  std::vector<LogEntry> logs = std::move(log_buffer);
  log_buffer.clear();
  return logs;
}

void PreprocessorWrapper::drain_logs() {
  if (log_server) {
    log_server->drain();
  }
}

rclcpp::Logger* PreprocessorWrapper::logger_ptr = nullptr;
std::vector<LogEntry> PreprocessorWrapper::log_buffer;
std::mutex PreprocessorWrapper::log_buffer_mutex;
