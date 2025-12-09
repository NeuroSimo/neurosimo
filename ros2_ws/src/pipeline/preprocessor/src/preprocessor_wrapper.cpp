#include <pybind11/pybind11.h>
#include <pybind11/embed.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include "preprocessor_wrapper.h"
#include <eeg_interfaces/msg/sample.hpp>

namespace py = pybind11;

PreprocessorWrapper::PreprocessorWrapper(rclcpp::Logger& logger) {
  logger_ptr = &logger;
  state = WrapperState::UNINITIALIZED;
  guard = std::make_unique<py::scoped_interpreter>();
  setup_custom_print();
}

void PreprocessorWrapper::setup_custom_print() {
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

void PreprocessorWrapper::initialize_module(
    const std::string& directory,
    const std::string& module_name,
    const size_t eeg_size,
    const size_t emg_size,
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
    auto instance = preprocessor_module->attr("Preprocessor")(eeg_size, emg_size, sampling_frequency);
    preprocessor_instance = std::make_unique<py::object>(instance);

  } catch(const py::error_already_set& e) {
    std::string error_msg = std::string("Python error: ") + e.what();
    RCLCPP_ERROR(*logger_ptr, "%s", error_msg.c_str());
    
    // Add error to log buffer so it can be published to UI
    {
      std::lock_guard<std::mutex> lock(log_buffer_mutex);
      log_buffer.push_back({error_msg, LogLevel::ERROR});
    }
    
    state = WrapperState::ERROR;
    return;

  } catch(const std::exception& e) {
    std::string error_msg = std::string("C++ error: ") + e.what();
    RCLCPP_ERROR(*logger_ptr, "%s", error_msg.c_str());
    
    // Add error to log buffer so it can be published to UI
    {
      std::lock_guard<std::mutex> lock(log_buffer_mutex);
      log_buffer.push_back({error_msg, LogLevel::ERROR});
    }
    
    state = WrapperState::ERROR;
    return;
  }

  /* Extract the sample_window from preprocessor_instance. */
  if (py::hasattr(*preprocessor_instance, "sample_window")) {
    py::list sample_window = preprocessor_instance->attr("sample_window").cast<py::list>();
    if (sample_window.size() == 2) {
      int earliest_sample_index = sample_window[0].cast<int>();
      int latest_sample_index = sample_window[1].cast<int>();
      
      /* Convert from indices to positive sample counts.
         For window [-10, 5]: look_back_samples = 10, look_ahead_samples = 5 */
      this->look_back_samples = -earliest_sample_index;
      this->look_ahead_samples = latest_sample_index;
      
      this->buffer_size = this->look_back_samples + this->look_ahead_samples + 1;
    } else {
      RCLCPP_WARN(*logger_ptr, "sample_window class attribute is of incorrect length (should be two elements).");
    }
  } else {
    RCLCPP_WARN(*logger_ptr, "sample_window class attribute not defined by the preprocessor.");
  }

  /* Initialize numpy arrays. */
  py_time_offsets = std::make_unique<py::array_t<double>>(buffer_size);

  std::vector<size_t> eeg_shape = {buffer_size, eeg_size};
  py_eeg = std::make_unique<py::array_t<double>>(eeg_shape);

  std::vector<size_t> emg_shape = {buffer_size, emg_size};
  py_emg = std::make_unique<py::array_t<double>>(emg_shape);

  this->eeg_size = eeg_size;
  this->emg_size = emg_size;

  state = WrapperState::READY;

  /* Log the configuration. */
  RCLCPP_INFO(*logger_ptr, "Configuration:");
  RCLCPP_INFO(*logger_ptr, " ");
  RCLCPP_INFO(*logger_ptr, "  - Sample window: %s[%d, %d]%s", bold_on.c_str(), -this->look_back_samples, this->look_ahead_samples, bold_off.c_str());
  RCLCPP_INFO(*logger_ptr, " ");
}

void PreprocessorWrapper::reset_module_state() {
  preprocessor_module = nullptr;
  preprocessor_instance = nullptr;

  py_time_offsets.reset();
  py_eeg.reset();
  py_emg.reset();

  state = WrapperState::UNINITIALIZED;

  RCLCPP_INFO(*logger_ptr, "Preprocessor reset.");
}

PreprocessorWrapper::~PreprocessorWrapper() {
  py_time_offsets.reset();
  py_eeg.reset();
  py_emg.reset();
}

WrapperState PreprocessorWrapper::get_state() const {
  return this->state;
}

std::size_t PreprocessorWrapper::get_buffer_size() const {
  return this->buffer_size;
}

int PreprocessorWrapper::get_look_ahead_samples() const {
  /* For a sample window like [-10, 5], look_ahead_samples is 5, which represents
     the number of samples we need to look ahead from the triggering sample. */
  return this->look_ahead_samples;
}

bool PreprocessorWrapper::process(
    eeg_interfaces::msg::Sample& output_sample,
    const RingBuffer<std::shared_ptr<eeg_interfaces::msg::Sample>>& buffer,
    double_t reference_time,
    bool pulse_given) {

  /* An example: If the sample window is set to [-2, 5], look_back_samples = 2, and the sample window base index
     (the index of sample 0 in the buffer) is 2. */
  int reference_index = this->look_back_samples;

  /* Fill the numpy arrays. */
  auto time_offsets_ptr = py_time_offsets->mutable_data();
  auto eeg_ptr = py_eeg->mutable_data();
  auto emg_ptr = py_emg->mutable_data();

  buffer.process_elements([&](const auto& sample_ptr) {
    const auto& sample = *sample_ptr;

    *time_offsets_ptr++ = sample.time - reference_time;
    std::memcpy(eeg_ptr, sample.eeg.data(), eeg_size * sizeof(double));
    eeg_ptr += eeg_size;
    std::memcpy(emg_ptr, sample.emg.data(), emg_size * sizeof(double));
    emg_ptr += emg_size;
  });

  /* Call the Python function. */
  py::object result;
  try {
    result = preprocessor_instance->attr("process")(reference_time, reference_index, *py_time_offsets, *py_eeg, *py_emg, pulse_given);

  } catch(const py::error_already_set& e) {
    std::string error_msg = std::string("Python error: ") + e.what();
    RCLCPP_ERROR(*logger_ptr, "%s", error_msg.c_str());
    
    // Add error to log buffer so it can be published to UI
    {
      std::lock_guard<std::mutex> lock(log_buffer_mutex);
      log_buffer.push_back({error_msg, LogLevel::ERROR});
    }
    
    state = WrapperState::ERROR;
    return false;

  } catch(const std::exception& e) {
    std::string error_msg = std::string("C++ error: ") + e.what();
    RCLCPP_ERROR(*logger_ptr, "%s", error_msg.c_str());
    
    // Add error to log buffer so it can be published to UI
    {
      std::lock_guard<std::mutex> lock(log_buffer_mutex);
      log_buffer.push_back({error_msg, LogLevel::ERROR});
    }
    
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
  output_sample.eeg = dict_result["eeg_sample"].cast<std::vector<double>>();
  output_sample.emg = dict_result["emg_sample"].cast<std::vector<double>>();
  output_sample.valid = dict_result["valid"].cast<bool>();

  output_sample.time = reference_time;

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
