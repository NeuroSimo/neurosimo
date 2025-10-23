#include <pybind11/pybind11.h>
#include <pybind11/embed.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include "preprocessor_wrapper.h"
#include <eeg_msgs/msg/sample.hpp>

namespace py = pybind11;

PreprocessorWrapper::PreprocessorWrapper(rclcpp::Logger& logger) {
  logger_ptr = &logger;
  state = WrapperState::UNINITIALIZED;
  guard = std::make_unique<py::scoped_interpreter>();
}

void PreprocessorWrapper::initialize_module(
    const std::string& directory,
    const std::string& module_name,
    const size_t eeg_data_size,
    const size_t emg_data_size,
    const uint16_t sampling_frequency) {

  this->sampling_frequency = sampling_frequency;

  /* If we have an existing preprocessor instance, release it which will call the destructor. */
  preprocessor_instance = nullptr;
  preprocessor_module = nullptr;

  /* Set the sys.path to include the directory of the module. */
  py::module sys_module = py::module::import("sys");
  py::list sys_path = sys_module.attr("path");
  sys_path.append(directory);

  /* Remove the module from sys.modules if it exists, to ensure it is reloaded. */
  py::dict sys_modules = sys_module.attr("modules");
  if (sys_modules.contains(module_name.c_str())) {
    sys_modules.attr("__delitem__")(module_name.c_str());
  }

  /* Import the module and initialize the Preprocessor instance. */

  try {
    auto imported_module = py::module::import(module_name.c_str());
    preprocessor_module = std::make_unique<py::module>(imported_module);
    auto instance = preprocessor_module->attr("Preprocessor")(eeg_data_size, emg_data_size, sampling_frequency);
    preprocessor_instance = std::make_unique<py::object>(instance);

  } catch(const py::error_already_set& e) {
    RCLCPP_ERROR(*logger_ptr, "Python error: %s", e.what());
    state = WrapperState::ERROR;
    return;

  } catch(const std::exception& e) {
    RCLCPP_ERROR(*logger_ptr, "C++ error: %s", e.what());
    state = WrapperState::ERROR;
    return;
  }

  /* Extract the sample_window from preprocessor_instance. */
  if (py::hasattr(*preprocessor_instance, "sample_window")) {
    py::list sample_window = preprocessor_instance->attr("sample_window").cast<py::list>();
    if (sample_window.size() == 2) {
      this->earliest_sample = sample_window[0].cast<int>();
      this->latest_sample = sample_window[1].cast<int>();

      this->buffer_size = this->latest_sample - this->earliest_sample + 1;
    } else {
      RCLCPP_WARN(*logger_ptr, "sample_window class attribute is of incorrect length (should be two elements).");
    }
  } else {
    RCLCPP_WARN(*logger_ptr, "sample_window class attribute not defined by the preprocessor.");
  }

  /* Initialize numpy arrays. */
  py_timestamps = std::make_unique<py::array_t<double>>(buffer_size);

  std::vector<size_t> eeg_data_shape = {buffer_size, eeg_data_size};
  py_eeg_data = std::make_unique<py::array_t<double>>(eeg_data_shape);

  std::vector<size_t> emg_data_shape = {buffer_size, emg_data_size};
  py_emg_data = std::make_unique<py::array_t<double>>(emg_data_shape);

  this->eeg_data_size = eeg_data_size;
  this->emg_data_size = emg_data_size;

  state = WrapperState::READY;

  /* Log the configuration. */
  RCLCPP_INFO(*logger_ptr, "Configuration:");
  RCLCPP_INFO(*logger_ptr, " ");
  RCLCPP_INFO(*logger_ptr, "  - Sample window: %s[%d, %d]%s", bold_on.c_str(), this->earliest_sample, this->latest_sample, bold_off.c_str());
  RCLCPP_INFO(*logger_ptr, " ");
}

void PreprocessorWrapper::reset_module_state() {
  preprocessor_module = nullptr;
  preprocessor_instance = nullptr;

  py_timestamps.reset();
  py_eeg_data.reset();
  py_emg_data.reset();

  state = WrapperState::UNINITIALIZED;

  RCLCPP_INFO(*logger_ptr, "Preprocessor reset.");
}

PreprocessorWrapper::~PreprocessorWrapper() {
  py_timestamps.reset();
  py_eeg_data.reset();
  py_emg_data.reset();
}

WrapperState PreprocessorWrapper::get_state() const {
  return this->state;
}

std::size_t PreprocessorWrapper::get_buffer_size() const {
  return this->buffer_size;
}

bool PreprocessorWrapper::process(
    eeg_msgs::msg::PreprocessedSample& output_sample,
    const RingBuffer<std::shared_ptr<eeg_msgs::msg::Sample>>& buffer,
    double_t sample_time,
    bool pulse_given) {

  /* TODO: The logic below, as well as the difference in semantics between "sample time" and "current time", needs to
     be documented somewhere more thoroughly. */

  /* An example: If the sample window is set to [-2, 2], the earliest sample will be -2, and the current sample,
     i.e., the sample corresponding to 0, will have the index 2, hence the calculation below. */
  int current_sample_index = -this->earliest_sample;

  /* An example: If sample time is 5.0 and the sampling frequency is 5 kHz, and we have a sample window [-2, 2]
     (the latest sample being 2), the sample 2 in the sample window corresponds to the time 5.0. Hence, sample 0
     corresponds to the time 5.0 s - 2 / (5000 Hz) = 4.9996 s. */
  double_t current_time = sample_time - (double)this->latest_sample / this->sampling_frequency;

  /* Fill the numpy arrays. */
  auto timestamps_ptr = py_timestamps->mutable_data();
  auto eeg_data_ptr = py_eeg_data->mutable_data();
  auto emg_data_ptr = py_emg_data->mutable_data();

  buffer.process_elements([&](const auto& sample_ptr) {
    const auto& sample = *sample_ptr;

    *timestamps_ptr++ = sample.time - current_time;
    std::memcpy(eeg_data_ptr, sample.eeg_data.data(), eeg_data_size * sizeof(double));
    eeg_data_ptr += eeg_data_size;
    std::memcpy(emg_data_ptr, sample.emg_data.data(), emg_data_size * sizeof(double));
    emg_data_ptr += emg_data_size;
  });

  /* Call the Python function. */
  py::object result;
  try {
    result = preprocessor_instance->attr("process")(*py_timestamps, *py_eeg_data, *py_emg_data, current_sample_index, pulse_given);

  } catch(const py::error_already_set& e) {
    RCLCPP_ERROR(*logger_ptr, "Python error: %s", e.what());
    state = WrapperState::ERROR;
    return false;

  } catch(const std::exception& e) {
    RCLCPP_ERROR(*logger_ptr, "C++ error: %s", e.what());
    state = WrapperState::ERROR;
    return false;
  }

  /* Validate the return value of the Python function call. */
  if (!py::isinstance<py::dict>(result)) {
    RCLCPP_ERROR(*logger_ptr, "Python module should return a dictionary.");
    state = WrapperState::ERROR;
    return false;
  }

  py::dict dict_result = result.cast<py::dict>();

  if (!dict_result.contains("eeg_sample")) {
    RCLCPP_ERROR(*logger_ptr, "Python module should return a dictionary with the field: eeg_sample.");
    state = WrapperState::ERROR;
    return false;
  }

  if (!dict_result.contains("emg_sample")) {
    RCLCPP_ERROR(*logger_ptr, "Python module should return a dictionary with the field: emg_sample.");
    state = WrapperState::ERROR;
    return false;
  }

  if (!dict_result.contains("valid")) {
    RCLCPP_ERROR(*logger_ptr, "Python module should return a dictionary with the field: valid.");
    state = WrapperState::ERROR;
    return false;
  }

  /* Convert the Python dictionary to a ROS message. */
  output_sample.eeg_data = dict_result["eeg_sample"].cast<std::vector<double>>();
  output_sample.emg_data = dict_result["emg_sample"].cast<std::vector<double>>();
  output_sample.valid = dict_result["valid"].cast<bool>();

  output_sample.time = current_time;

  return true;
}

std::vector<LogEntry> PreprocessorWrapper::get_and_clear_logs() {
  std::lock_guard<std::mutex> lock(log_buffer_mutex);
  std::vector<LogEntry> logs = std::move(log_buffer);
  log_buffer.clear();
  return logs;
}

rclcpp::Logger* PreprocessorWrapper::logger_ptr = nullptr;
std::vector<LogEntry> PreprocessorWrapper::log_buffer;
std::mutex PreprocessorWrapper::log_buffer_mutex;
