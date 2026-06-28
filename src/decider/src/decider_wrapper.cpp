#include <pybind11/pybind11.h>
#include <pybind11/embed.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <rcpputils/filesystem_helper.hpp>
#include <cmath>
#include <chrono>
#include <cstdlib>
#include <filesystem>

#include "decider_wrapper.h"
#include <neurosimo_eeg_interfaces/msg/sample.hpp>

namespace py = pybind11;

namespace {

template<typename T>
void rotate_if_referenced(
    std::unique_ptr<py::array_t<T>>& arr,
    const std::vector<size_t>& shape) {
  if (Py_REFCNT(arr->ptr()) > 1) {
    arr = std::make_unique<py::array_t<T>>(shape);
  }
}

void rotate_sample_buffers_if_referenced(
    std::unique_ptr<py::array_t<double>>& time_offsets,
    std::unique_ptr<py::array_t<double>>& eeg,
    std::unique_ptr<py::array_t<double>>& emg,
    size_t num_samples,
    size_t eeg_channels,
    size_t emg_channels) {
  rotate_if_referenced(time_offsets, {num_samples});
  rotate_if_referenced(eeg, {num_samples, eeg_channels});
  rotate_if_referenced(emg, {num_samples, emg_channels});
}

py::module load_python_file_module(
    const std::filesystem::path& module_directory,
    const std::string& module_name,
    const std::string& loaded_module_suffix = "_wrapper") {
  const std::string loaded_module_name = module_name + loaded_module_suffix;
  const std::filesystem::path module_path = module_directory / (module_name + ".py");

  py::module importlib_util = py::module::import("importlib.util");
  py::object spec = importlib_util.attr("spec_from_file_location")(
    loaded_module_name,
    module_path.string());
  if (spec.is_none()) {
    throw std::runtime_error(
      "Failed to create module spec for '" + loaded_module_name +
      "' from path '" + module_path.string() + "'.");
  }

  py::object module = importlib_util.attr("module_from_spec")(spec);

  py::module sys_module = py::module::import("sys");
  py::dict sys_modules = sys_module.attr("modules");
  sys_modules[py::str(loaded_module_name)] = module;

  py::object loader = spec.attr("loader");
  if (loader.is_none()) {
    throw std::runtime_error(
      "Import spec for '" + loaded_module_name +
      "' has no loader (path: '" + module_path.string() + "').");
  }

  loader.attr("exec_module")(module);
  return module.cast<py::module>();
}

}  // namespace

DeciderWrapper::DeciderWrapper(rclcpp::Logger& logger) {
  logger_ptr = &logger;

  /* Initialize processing path to undetermined */
  current_processing_path = neurosimo_pipeline_interfaces::msg::LogMessage::PROCESSING_PATH_UNDETERMINED;

  /* Start IPC log server first, so cpp_bindings.log() can forward immediately. */
  log_server = std::make_unique<LogIpcServer>(
    std::bind(&DeciderWrapper::handle_ipc_log_message, this, std::placeholders::_1)
  );
  if (!log_server->start()) {
    RCLCPP_WARN(*logger_ptr, "Failed to start log IPC server; worker logs may be lost.");
  }

  /* Initialize the Python interpreter. */
  interpreter = std::make_unique<py::scoped_interpreter>();
  setup_custom_print();
}

void DeciderWrapper::setup_custom_print() {
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

void DeciderWrapper::log_section_header(const std::string& title) {
  std::wstring underline_str(title.size(), L'–');
  RCLCPP_INFO(*logger_ptr, "%s%s%s", bold_on.c_str(), title.c_str(), bold_off.c_str());
  RCLCPP_INFO(*logger_ptr, "%s%ls%s", bold_on.c_str(), underline_str.c_str(), bold_off.c_str());
  RCLCPP_INFO(*logger_ptr, " ");
}

void DeciderWrapper::log_error(const std::string& message) {
  RCLCPP_ERROR(*logger_ptr, "%s", message.c_str());
  log_buffer.push_back({message, LogLevel::ERROR, current_processing_path});
}

bool DeciderWrapper::initialize_module(
    const std::string& project_directory,
    const std::string& module_directory,
    const std::string& module_name,
    int32_t subject_id,
    const size_t eeg_size,
    const size_t emg_size,
    const uint16_t sampling_frequency,
    std::vector<neurosimo_pipeline_interfaces::msg::SensoryStimulus>& sensory_stimuli,
    std::priority_queue<double, std::vector<double>, std::greater<double>>& event_queue) {

  this->sampling_frequency = sampling_frequency;
  if (this->sampling_frequency == 0) {
    log_error("Sampling frequency must be greater than zero to interpret sample_window expressed in seconds.");
    return false;
  }

  /* Set up the Python environment. */
  py::module sys_module = py::module::import("sys");
  py::list sys_path = sys_module.attr("path");
  sys_path.append(module_directory);

  /* Import the module from the exact file path.
     This avoids accidentally importing a package with the same name
     (e.g. both 'prime.py' and 'prime/' existing side-by-side). */
  try {
    auto imported_module =
      load_python_file_module(std::filesystem::path(module_directory), module_name);
    decider_module = std::make_unique<py::module>(imported_module);

  } catch (const py::error_already_set &e) {
    std::string error_msg = std::string("Python import error: ") + e.what();
    log_error(error_msg);
    return false;

  } catch (const std::exception &e) {
    std::string error_msg = std::string("C++ error during import: ") + e.what();
    log_error(error_msg);
    return false;
  }

  /* Initialize Decider instance. */
  try {
    auto instance = decider_module->attr("Decider")(subject_id, eeg_size, emg_size, sampling_frequency);
    decider_instance = std::make_unique<py::object>(instance);

  } catch (const py::error_already_set &e) {
    std::string error_msg = std::string("Python initialization error: ") + e.what();
    log_error(error_msg);
    return false;

  } catch (const std::exception &e) {
    std::string error_msg = std::string("C++ error during initialization: ") + e.what();
    log_error(error_msg);
    return false;
  }

  /* Extract the configuration from decider_instance. */
  if (!py::hasattr(*decider_instance, "get_configuration")) {
    log_error("get_configuration method not found in the Decider instance.");
    return false;
  }
  py::dict config;

  /* Extract the configuration from decider_instance. */
  try {
    config = decider_instance->attr("get_configuration")().cast<py::dict>();

  } catch(const py::error_already_set& e) {
    std::string error_msg = std::string("Python error during get_configuration call: ") + e.what();
    log_error(error_msg);
    return false;

  } catch(const std::exception& e) {
    std::string error_msg = std::string("C++ error during get_configuration call: ") + e.what();
    log_error(error_msg);
    return false;
  }

  /* Helper lambdas to convert seconds to sample counts. */
  const auto to_samples = [this](double seconds) -> int {
    return static_cast<int>(std::ceil(seconds * static_cast<double>(this->sampling_frequency)));
  };

  /* Validate that only allowed keys are present */
  std::vector<std::string> allowed_keys = {"sample_window", "warm_up_rounds", "predefined_sensory_stimuli", "periodic_processing_interval", "predefined_events", "pulse_sample_window", "event_sample_window"};
  for (const auto& item : config) {
    std::string key = py::str(item.first).cast<std::string>();
    if (std::find(allowed_keys.begin(), allowed_keys.end(), key) == allowed_keys.end()) {
      log_error("Unexpected key '" + key + "' in configuration dictionary. Only 'sample_window', 'warm_up_rounds', 'predefined_sensory_stimuli', 'periodic_processing_interval', 'predefined_events', 'pulse_sample_window', and 'event_sample_window' are allowed.");
      return false;
    }
  }

  /* Extract predefined_sensory_stimuli (optional). */
  if (config.contains("predefined_sensory_stimuli")) {
    if (!py::isinstance<py::list>(config["predefined_sensory_stimuli"])) {
      log_error("predefined_sensory_stimuli must be a list.");
      return false;
    }

    py::list py_sensory_stimuli = config["predefined_sensory_stimuli"].cast<py::list>();
    if (!process_sensory_stimuli_list(py_sensory_stimuli, sensory_stimuli)) {
      return false;
    }
  }

  /* Extract periodic_processing_interval (optional, defaults to 0.1). */
  if (config.contains("periodic_processing_interval")) {
    try {
      this->periodic_processing_interval = config["periodic_processing_interval"].cast<double>();
      if (this->periodic_processing_interval <= 0.0) {
        log_error("periodic_processing_interval must be a positive number (got " + std::to_string(this->periodic_processing_interval) + ").");
        return false;
      }
    } catch (const py::cast_error& e) {
      log_error(std::string("periodic_processing_interval must be a number: ") + e.what());
      return false;
    }
  } else {
    this->periodic_processing_interval = 0.1;
  }

  /* Extract predefined_events (optional). */
  if (config.contains("predefined_events")) {
    py::list events = config["predefined_events"].cast<py::list>();
    for (const auto& event : events) {
      double event_time = event.cast<double>();
      event_queue.push(event_time);
    }
  }

  /* Extract warm_up_rounds (optional, defaults to 0). */
  if (config.contains("warm_up_rounds")) {
    try {
      this->warm_up_rounds = config["warm_up_rounds"].cast<int>();
      if (this->warm_up_rounds < 0) {
        log_error("warm_up_rounds must be non-negative.");
        return false;
      }
    } catch (const py::cast_error& e) {
      log_error(std::string("warm_up_rounds must be an integer: ") + e.what());
      return false;
    }
  } else {
    this->warm_up_rounds = 0;
  }

  /* Extract sample_window. */
  if (!config.contains("sample_window")) {
    log_error("'sample_window' key not found in configuration dictionary.");
    return false;
  }

  double default_sample_window_start_seconds = 0.0;
  double default_sample_window_end_seconds = 0.0;

  py::list sample_window = config["sample_window"].cast<py::list>();
  if (sample_window.size() == 2) {
    default_sample_window_start_seconds = sample_window[0].cast<double>();
    default_sample_window_end_seconds = sample_window[1].cast<double>();
    if (default_sample_window_start_seconds > default_sample_window_end_seconds) {
      log_error(
        "Invalid sample_window: start (" + std::to_string(default_sample_window_start_seconds) +
        " s) must be <= end (" + std::to_string(default_sample_window_end_seconds) + " s).");
      return false;
    }

    /* Convert seconds to sample counts. */
    this->periodic_sample_window_start = to_samples(default_sample_window_start_seconds);
    this->periodic_sample_window_end = to_samples(default_sample_window_end_seconds);
  } else {
    log_error("'sample_window' value in configuration is of incorrect length (should be two elements).");
    return false;
  }

  /* Detect process_pulse method by convention. */
  double pulse_sample_window_start_seconds = 0.0;
  double pulse_sample_window_end_seconds = 0.0;

  if (py::hasattr(*decider_instance, "process_pulse")) {
    this->has_pulse_processor_ = true;

    /* Extract optional pulse_sample_window (flat key). */
    if (config.contains("pulse_sample_window")) {
      py::list sample_window = config["pulse_sample_window"].cast<py::list>();
      if (sample_window.size() != 2) {
        log_error("pulse_sample_window must have 2 elements.");
        return false;
      }
      pulse_sample_window_start_seconds = sample_window[0].cast<double>();
      pulse_sample_window_end_seconds = sample_window[1].cast<double>();
      if (pulse_sample_window_start_seconds > pulse_sample_window_end_seconds) {
        log_error(
          "Invalid pulse_sample_window: start (" + std::to_string(pulse_sample_window_start_seconds) +
          " s) must be <= end (" + std::to_string(pulse_sample_window_end_seconds) + " s).");
        return false;
      }

      this->pulse_sample_window_start = to_samples(pulse_sample_window_start_seconds);
      this->pulse_sample_window_end = to_samples(pulse_sample_window_end_seconds);
    } else {
      /* Use the same sample window as periodic processing. */
      this->pulse_sample_window_start = this->periodic_sample_window_start;
      this->pulse_sample_window_end = this->periodic_sample_window_end;

      pulse_sample_window_start_seconds = default_sample_window_start_seconds;
      pulse_sample_window_end_seconds = default_sample_window_end_seconds;
    }
    RCLCPP_DEBUG(*logger_ptr, "Registered pulse processor (process_pulse)");
  }

  /* Detect process_event method by convention. */
  double event_sample_window_start_seconds = 0.0;
  double event_sample_window_end_seconds = 0.0;

  if (py::hasattr(*decider_instance, "process_event")) {
    this->has_event_processor_ = true;

    /* Extract optional event_sample_window (flat key). */
    if (config.contains("event_sample_window")) {
      py::list sample_window = config["event_sample_window"].cast<py::list>();
      if (sample_window.size() != 2) {
        log_error("event_sample_window must have 2 elements.");
        return false;
      }
      event_sample_window_start_seconds = sample_window[0].cast<double>();
      event_sample_window_end_seconds = sample_window[1].cast<double>();
      if (event_sample_window_start_seconds > event_sample_window_end_seconds) {
        log_error(
          "Invalid event_sample_window: start (" + std::to_string(event_sample_window_start_seconds) +
          " s) must be <= end (" + std::to_string(event_sample_window_end_seconds) + " s).");
        return false;
      }

      this->event_sample_window_start = to_samples(event_sample_window_start_seconds);
      this->event_sample_window_end = to_samples(event_sample_window_end_seconds);
    } else {
      /* Use the same sample window as periodic processing. */
      this->event_sample_window_start = this->periodic_sample_window_start;
      this->event_sample_window_end = this->periodic_sample_window_end;

      event_sample_window_start_seconds = default_sample_window_start_seconds;
      event_sample_window_end_seconds = default_sample_window_end_seconds;
    }
    RCLCPP_DEBUG(*logger_ptr, "Registered event processor (process_event)");
  }

  /* Detect prepare_trial method by convention. */
  if (py::hasattr(*decider_instance, "prepare_trial")) {
    this->has_prepare_trial_processor_ = true;
    RCLCPP_DEBUG(*logger_ptr, "Registered trial preparer (prepare_trial)");
  }

  /* Detect process_task method by convention. */
  if (py::hasattr(*decider_instance, "process_task")) {
    this->has_task_processor_ = true;
    RCLCPP_DEBUG(*logger_ptr, "Registered task processor (process_task)");
  }

  /* TODO: The logic below works but is too complex just to cover the case where the sample window is fully in the past. It should be simplified. */

  /* Calculate maximum buffer size needed to cover all windows.
     Negative look-ahead is treated as zero in runtime scheduling, so the envelope must
     be sized using "effective" look-ahead values (max(window_end, 0)). */
  const auto effective_look_ahead = [](int window_end) -> int {
    return std::max(window_end, 0);
  };

  const int periodic_effective_look_ahead = effective_look_ahead(this->periodic_sample_window_end);
  const int pulse_effective_look_ahead = effective_look_ahead(this->pulse_sample_window_end);
  const int event_effective_look_ahead = effective_look_ahead(this->event_sample_window_end);

  int min_start = this->periodic_sample_window_start;
  min_start = std::min(min_start, this->pulse_sample_window_start);
  min_start = std::min(min_start, this->event_sample_window_start);

  int max_effective_look_ahead = periodic_effective_look_ahead;
  max_effective_look_ahead = std::max(max_effective_look_ahead, pulse_effective_look_ahead);
  max_effective_look_ahead = std::max(max_effective_look_ahead, event_effective_look_ahead);

  this->envelope_buffer_size = static_cast<std::size_t>(max_effective_look_ahead - min_start + 1);

  /* Store each window's start index in the envelope.
     The newest sample in the buffer corresponds to effective look-ahead (>= 0), not
     necessarily to window_end when the full window lies in the past. */
  const auto compute_window_start_offset = [this](int window_start, int effective_look_ahead_for_reason) -> std::size_t {
    const int oldest_offset_in_buffer = effective_look_ahead_for_reason - static_cast<int>(this->envelope_buffer_size) + 1;
    return static_cast<std::size_t>(window_start - oldest_offset_in_buffer);
  };

  this->periodic_window_start_offset_in_envelope =
      compute_window_start_offset(this->periodic_sample_window_start, periodic_effective_look_ahead);
  this->pulse_window_start_offset_in_envelope =
      compute_window_start_offset(this->pulse_sample_window_start, pulse_effective_look_ahead);
  this->event_window_start_offset_in_envelope =
      compute_window_start_offset(this->event_sample_window_start, event_effective_look_ahead);
  
  RCLCPP_DEBUG(*logger_ptr, "Maximum envelope: start=%d, latest=%d, buffer size: %zu",
               min_start, max_effective_look_ahead, this->envelope_buffer_size);

  this->eeg_size = eeg_size;
  this->emg_size = emg_size;

  /* Initialize numpy arrays for default window. */
  size_t periodic_buffer_size = this->periodic_sample_window_end - this->periodic_sample_window_start + 1;
  periodic_time_offsets = std::make_unique<py::array_t<double>>(periodic_buffer_size);

  std::vector<size_t> eeg_shape = {periodic_buffer_size, eeg_size};
  periodic_eeg = std::make_unique<py::array_t<double>>(eeg_shape);

  std::vector<size_t> emg_shape = {periodic_buffer_size, emg_size};
  periodic_emg = std::make_unique<py::array_t<double>>(emg_shape);
  
  /* Initialize numpy arrays for pulse processor. */
  size_t pulse_buffer_size = this->pulse_sample_window_end - this->pulse_sample_window_start + 1;
  pulse_time_offsets = std::make_unique<py::array_t<double>>(pulse_buffer_size);

  std::vector<size_t> pulse_eeg_shape = {pulse_buffer_size, eeg_size};
  pulse_eeg = std::make_unique<py::array_t<double>>(pulse_eeg_shape);

  std::vector<size_t> pulse_emg_shape = {pulse_buffer_size, emg_size};
  pulse_emg = std::make_unique<py::array_t<double>>(pulse_emg_shape);
  
  /* Initialize numpy arrays for event processor. */
  size_t event_buffer_size = this->event_sample_window_end - this->event_sample_window_start + 1;
  event_time_offsets = std::make_unique<py::array_t<double>>(event_buffer_size);

  std::vector<size_t> event_eeg_shape = {event_buffer_size, eeg_size};
  event_eeg = std::make_unique<py::array_t<double>>(event_eeg_shape);

  std::vector<size_t> event_emg_shape = {event_buffer_size, emg_size};
  event_emg = std::make_unique<py::array_t<double>>(event_emg_shape);

  /* Log the configuration. */
  RCLCPP_INFO(*logger_ptr, "Configuration:");
  RCLCPP_INFO(*logger_ptr, " ");
  RCLCPP_INFO(*logger_ptr, "  - Periodic sample window: %s[%.3f s, %.3f s]%s",
              bold_on.c_str(),
              default_sample_window_start_seconds,
              default_sample_window_end_seconds,
              bold_off.c_str());

  RCLCPP_INFO(*logger_ptr, "  - Periodic processing interval: %s%.3f%s (s)", bold_on.c_str(), this->periodic_processing_interval, bold_off.c_str());
            
  if (!this->has_pulse_processor_) {
    RCLCPP_INFO(*logger_ptr, "  - Pulse processor: %sDisabled%s (no process_pulse method)", bold_on.c_str(), bold_off.c_str());
  } else {
    RCLCPP_INFO(*logger_ptr, "  - Pulse processor: %sEnabled%s", bold_on.c_str(), bold_off.c_str());
    RCLCPP_INFO(*logger_ptr, "    - Pulse processor window: %s[%.3f s, %.3f s]%s",
                bold_on.c_str(), pulse_sample_window_start_seconds, pulse_sample_window_end_seconds, bold_off.c_str());
  }

  if (!this->has_event_processor_) {
    RCLCPP_INFO(*logger_ptr, "  - Event processor: %sDisabled%s (no process_event method)", bold_on.c_str(), bold_off.c_str());
  } else {
    RCLCPP_INFO(*logger_ptr, "  - Event processor: %sEnabled%s", bold_on.c_str(), bold_off.c_str());
    RCLCPP_INFO(*logger_ptr, "    - Event processor window: %s[%.3f s, %.3f s]%s",
                bold_on.c_str(), event_sample_window_start_seconds, event_sample_window_end_seconds, bold_off.c_str());
  }

  if (!this->has_prepare_trial_processor_) {
    RCLCPP_INFO(*logger_ptr, "  - Trial preparer: %sDisabled%s (no prepare_trial method)", bold_on.c_str(), bold_off.c_str());
  } else {
    RCLCPP_INFO(*logger_ptr, "  - Trial preparer: %sEnabled%s", bold_on.c_str(), bold_off.c_str());
  }

  if (!this->has_task_processor_) {
    RCLCPP_INFO(*logger_ptr, "  - Task processor: %sDisabled%s (no process_task method)", bold_on.c_str(), bold_off.c_str());
  } else {
    RCLCPP_INFO(*logger_ptr, "  - Task processor: %sEnabled%s", bold_on.c_str(), bold_off.c_str());
  }

  RCLCPP_INFO(*logger_ptr, " ");

  return true;
}

void DeciderWrapper::destroy_instance() {
  decider_instance = nullptr;
}

bool DeciderWrapper::warm_up() {
  if (!decider_instance) {
    RCLCPP_WARN(*logger_ptr, "Cannot warm up: decider instance not available");
    return false;
  }

  log_section_header("Warm-up");

  if (this->warm_up_rounds == 0) {
    RCLCPP_INFO(*logger_ptr, "Warm-up disabled (warm_up_rounds = %d)", this->warm_up_rounds);
    RCLCPP_INFO(*logger_ptr, " ");
    log_section_header("Operation");

    // Return true to indicate that warm-up was successful (even though it was disabled).
    return true;
  }

  RCLCPP_INFO(*logger_ptr, "Starting %d warm-up rounds...", this->warm_up_rounds);

  // Initialize RNG with constant seed for reproducible warm-up data
  std::srand(12345);

  // Get pointers to numpy arrays
  auto time_offsets_ptr = periodic_time_offsets->mutable_data();
  auto eeg_ptr = periodic_eeg->mutable_data();
  auto emg_ptr = periodic_emg->mutable_data();

  // Dummy parameters for process method (constant across all rounds)
  double_t dummy_reference_time = 0.0;
  int dummy_reference_index = -this->periodic_sample_window_start;
  bool dummy_is_coil_at_target = true;

  size_t periodic_buffer_size = this->periodic_sample_window_end - this->periodic_sample_window_start + 1;

  // Perform warm-up rounds with fresh random data for each round
  for (int round = 0; round < this->warm_up_rounds; round++) {
    // Generate fresh random data for this round
    for (size_t i = 0; i < periodic_buffer_size; i++) {
      time_offsets_ptr[i] = -static_cast<double>(periodic_buffer_size - 1 - i) / sampling_frequency;

      // Fill EEG data with small random values
      for (size_t j = 0; j < eeg_size; j++) {
        eeg_ptr[i * eeg_size + j] = (static_cast<double>(std::rand()) / RAND_MAX - 0.5) * 1e-6;
      }
      
      // Fill EMG data with small random values
      for (size_t j = 0; j < emg_size; j++) {
        emg_ptr[i * emg_size + j] = (static_cast<double>(std::rand()) / RAND_MAX - 0.5) * 1e-6;
      }
    }

    try {
      auto start_time = std::chrono::high_resolution_clock::now();
      
      py::object py_result = decider_instance->attr("process_periodic")(
        dummy_reference_time,
        dummy_reference_index,
        *periodic_time_offsets,
        *periodic_eeg,
        *periodic_emg,
        dummy_is_coil_at_target,
        "",
        (uint64_t)0,  /* trial_in_stage */
        true  /* is_warm_up */
      );

      rotate_sample_buffers_if_referenced(
        periodic_time_offsets, periodic_eeg, periodic_emg,
        periodic_buffer_size, eeg_size, emg_size);
      
      auto end_time = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration<double, std::milli>(end_time - start_time).count();
      
      RCLCPP_INFO(*logger_ptr, "  Round %d/%d: %.2f ms", 
                    round + 1, this->warm_up_rounds, duration);
      
    } catch (const py::error_already_set& e) {
      std::string error_msg = std::string("Python error: ") + e.what();
      RCLCPP_WARN(*logger_ptr, "  Round %d/%d: FAILED (%s)", round + 1, this->warm_up_rounds, error_msg.c_str());

      // Add error to log buffer so it can be published to UI
      log_buffer.push_back({error_msg, LogLevel::ERROR, current_processing_path});

      return false;
    } catch (const std::exception& e) {
      std::string error_msg = std::string("C++ error: ") + e.what();
      RCLCPP_WARN(*logger_ptr, "  Round %d/%d: FAILED (%s)", round + 1, this->warm_up_rounds, error_msg.c_str());

      // Add error to log buffer so it can be published to UI
      log_buffer.push_back({error_msg, LogLevel::ERROR, current_processing_path});

      return false;
    }
  }

  RCLCPP_INFO(*logger_ptr, " ");
  RCLCPP_INFO(*logger_ptr, "Warm-up completed successfully (%d rounds)", this->warm_up_rounds);
  RCLCPP_INFO(*logger_ptr, " ");
  
  // Drain IPC buffer and clear any logs accumulated during warm-up to prevent them from being published
  drain_logs();
  get_and_clear_logs();
  
  log_section_header("Operation");

  return true;
}

DeciderWrapper::~DeciderWrapper() {
  if (log_server) {
    log_server->stop();
  }

  /* XXX: Let the interpreter leak - process is exiting anyway. Without this, the
          process will segfault on exit. Maybe there is a better way to do this? */
  interpreter.release();
}

std::vector<LogEntry> DeciderWrapper::get_and_clear_logs() {
  std::vector<LogEntry> logs = std::move(log_buffer);
  log_buffer.clear();
  return logs;
}

void DeciderWrapper::drain_logs() {
  if (log_server) {
    log_server->drain();
  }
}

void DeciderWrapper::set_current_processing_path(uint8_t processing_path) {
  current_processing_path = processing_path;
}

void DeciderWrapper::handle_ipc_log_message(std::string&& msg) {
  log_buffer.push_back({std::move(msg), LogLevel::INFO, current_processing_path});
}

std::size_t DeciderWrapper::get_envelope_buffer_size() const {
  return this->envelope_buffer_size;
}

double DeciderWrapper::get_periodic_processing_interval() const {
  return this->periodic_processing_interval;
}

int DeciderWrapper::get_periodic_look_ahead_samples() const {
  return std::max(this->periodic_sample_window_end, 0);
}

int DeciderWrapper::get_pulse_look_ahead_samples() const {
  /* If no pulse processor is defined, pulses should be processed immediately. */
  if (!this->has_pulse_processor_) {
    return 0;
  }
  return std::max(this->pulse_sample_window_end, 0);
}

int DeciderWrapper::get_event_look_ahead_samples() const {
  return std::max(this->event_sample_window_end, 0);
}

bool DeciderWrapper::parse_sensory_stimulus_dict(
  const py::dict& py_sensory_stimulus,
  neurosimo_pipeline_interfaces::msg::SensoryStimulus& out_msg) {

  static const std::vector<std::string> required = {
    "time",
    "type",
    "parameters"
  };

  // 1) Check presence
  for (const auto& field : required) {
    if (!py_sensory_stimulus.contains(field)) {
      log_error("sensory_stimulus missing field '" + field + "'");
      return false;
    }
  }

  // 2) Type‐check & cast with per-field error reporting
  try {
    out_msg.time = py_sensory_stimulus["time"].cast<double>();
  } catch (const py::cast_error& e) {
    log_error(std::string("'time' must be a float or int: ") + e.what());
    return false;
  }

  try {
    out_msg.type = py_sensory_stimulus["type"].cast<std::string>();
  } catch (const py::cast_error& e) {
    log_error(std::string("'type' must be a string: ") + e.what());
    return false;
  }

  py::dict params;
  try {
    params = py_sensory_stimulus["parameters"].cast<py::dict>();
  } catch (const py::cast_error& e) {
    log_error(std::string("'parameters' must be a dictionary: ") + e.what());
    return false;
  }

  // 3) Iterate and fill diagnostic_msgs::msg::KeyValue[]
  out_msg.parameters.clear();
  for (const auto& item : params) {
    std::string key;
    try {
      key = item.first.cast<std::string>();
    } catch (const py::cast_error& e) {
      log_error(std::string("parameter key is not a string: ") + e.what());
      return false;
    }

    std::string val;
    try {
      val = py::str(item.second);  // Serializes any Python value
    } catch (const std::exception& e) {
      log_error(std::string("parameter value could not be serialized to string: ") + e.what());
      return false;
    }

    diagnostic_msgs::msg::KeyValue kv;
    kv.key = std::move(key);
    kv.value = std::move(val);
    out_msg.parameters.push_back(std::move(kv));
  }

  return true;
}

bool DeciderWrapper::process_sensory_stimuli_list(
    const py::list& py_sensory_stimuli,
    std::vector<neurosimo_pipeline_interfaces::msg::SensoryStimulus>& sensory_stimuli) {

  if (py_sensory_stimuli.size() > 0) {
    for (const auto& py_sensory_stimulus : py_sensory_stimuli) {
      if (!py::isinstance<py::dict>(py_sensory_stimulus)) {
        log_error("sensory_stimuli must be a list of dictionaries.");
        return false;
      }
      py::dict py_sensory_stimulus_dict = py_sensory_stimulus.cast<py::dict>();
      neurosimo_pipeline_interfaces::msg::SensoryStimulus msg;
      if (parse_sensory_stimulus_dict(py_sensory_stimulus_dict, msg)) {
        sensory_stimuli.push_back(msg);
      } else {
        log_error("Failed to parse sensory_stimuli dictionary.");
        return false;
      }
    }
  }
  return true;
}

bool DeciderWrapper::parse_targeted_pulse_dict(
  const py::dict& py_targeted_pulse,
  shared_stimulation_interfaces::msg::TargetedPulse& out_msg) {

  static const std::vector<std::string> required = {
    "time_offset",
    "displacement_x",
    "displacement_y",
    "rotation_angle",
    "intensity"
  };

  for (const auto& field : required) {
    if (!py_targeted_pulse.contains(field)) {
      log_error("targeted_pulse missing field '" + field + "'");
      return false;
    }
  }

  try {
    out_msg.time_offset = py_targeted_pulse["time_offset"].cast<double>();
    out_msg.displacement_x = py_targeted_pulse["displacement_x"].cast<double>();
    out_msg.displacement_y = py_targeted_pulse["displacement_y"].cast<double>();
    out_msg.rotation_angle = py_targeted_pulse["rotation_angle"].cast<double>();
    out_msg.intensity = py_targeted_pulse["intensity"].cast<double>();
  } catch (const py::cast_error& e) {
    log_error(std::string("Invalid targeted_pulse field type: ") + e.what());
    return false;
  }

  return true;
}

bool DeciderWrapper::process_targeted_pulses_list(
  const py::list& py_targeted_pulses,
  std::vector<shared_stimulation_interfaces::msg::TargetedPulse>& targeted_pulses) {

  for (const auto& py_targeted_pulse : py_targeted_pulses) {
    if (!py::isinstance<py::dict>(py_targeted_pulse)) {
      log_error("targeted_pulses must be a list of dictionaries.");
      return false;
    }

    shared_stimulation_interfaces::msg::TargetedPulse msg;
    if (!parse_targeted_pulse_dict(py_targeted_pulse.cast<py::dict>(), msg)) {
      log_error("Failed to parse targeted_pulse dictionary.");
      return false;
    }
    targeted_pulses.push_back(msg);
  }

  return true;
}

void DeciderWrapper::fill_arrays_from_buffer(
    const RingBuffer<std::shared_ptr<neurosimo_eeg_interfaces::msg::Sample>>& buffer,
    double_t reference_time,
    py::array_t<double>& time_offsets,
    py::array_t<double>& eeg,
    py::array_t<double>& emg,
    size_t start_offset,
    size_t num_samples) {

  auto time_offsets_ptr = time_offsets.mutable_data();
  auto eeg_ptr = eeg.mutable_data();
  auto emg_ptr = emg.mutable_data();

  size_t ring_idx = 0;
  size_t out_idx = 0;
  buffer.process_elements([&](const auto& sample_ptr) {
    /* Check if this sample is in the range we want */
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

/* Shared result parser used by all processing entry points. */
ProcessResult DeciderWrapper::parse_result_dict(
    const py::object& py_result,
    bool allow_stimulation_request,
    std::vector<neurosimo_pipeline_interfaces::msg::SensoryStimulus>& sensory_stimuli,
    std::priority_queue<double, std::vector<double>, std::greater<double>>& event_queue) {

  /* If the return value is None, return early but mark it as successful. */
  if (py_result.is_none()) {
    return ProcessResult::success_empty();
  }

  /* If the return value is not None, ensure that it is a dictionary. */
  if (!py::isinstance<py::dict>(py_result)) {
    log_error("Python module should return a dictionary or None.");
    return ProcessResult::failure();
  }

  py::dict dict_result = py_result.cast<py::dict>();

  /* Validate that only allowed keys are present */
  std::vector<std::string> allowed_keys = {
    "trigger_offset",
    "targeted_pulses",
    "sensory_stimuli",
    "events",
    "coil_target",
    "invalid_trial"
  };
  for (const auto& item : dict_result) {
    std::string key = py::str(item.first).cast<std::string>();
    if (std::find(allowed_keys.begin(), allowed_keys.end(), key) == allowed_keys.end()) {
      log_error("Unexpected key '" + key + "' in return value, only 'trigger_offset', 'targeted_pulses', 'sensory_stimuli', 'events', 'coil_target', and 'invalid_trial' are allowed.");
      return ProcessResult::failure();
    }
  }

  ProcessResult result;
  result.success = true;

  if (dict_result.contains("coil_target")) {
    result.coil_target = dict_result["coil_target"].cast<std::string>();
  }

  if (dict_result.contains("trigger_offset")) {
    if (!allow_stimulation_request) {
      log_error("Timed trigger requests are not allowed for pulse or event processing.");
      return ProcessResult::failure();
    }
    result.trigger_offset = std::make_shared<double_t>(dict_result["trigger_offset"].cast<double_t>());
  }

  if (dict_result.contains("targeted_pulses")) {
    if (!allow_stimulation_request) {
      log_error("Targeted pulse requests are not allowed for pulse or event processing.");
      return ProcessResult::failure();
    }

    if (!py::isinstance<py::list>(dict_result["targeted_pulses"])) {
      log_error("targeted_pulses must be a list.");
      return ProcessResult::failure();
    }

    py::list py_targeted_pulses = dict_result["targeted_pulses"].cast<py::list>();
    if (!process_targeted_pulses_list(py_targeted_pulses, result.targeted_pulses)) {
      return ProcessResult::failure();
    }
  }

  if (result.trigger_offset != nullptr && !result.targeted_pulses.empty()) {
    log_error("Return value cannot include both 'trigger_offset' and 'targeted_pulses'.");
    return ProcessResult::failure();
  }

  if (dict_result.contains("sensory_stimuli")) {
    if (!py::isinstance<py::list>(dict_result["sensory_stimuli"])) {
      log_error("sensory_stimuli must be a list.");
      return ProcessResult::failure();
    }

    py::list py_sensory_stimuli = dict_result["sensory_stimuli"].cast<py::list>();
    if (!process_sensory_stimuli_list(py_sensory_stimuli, sensory_stimuli)) {
      return ProcessResult::failure();
    }
  }

  if (dict_result.contains("events")) {
    if (!py::isinstance<py::list>(dict_result["events"])) {
      log_error("events must be a list.");
      return ProcessResult::failure();
    }

    py::list events = dict_result["events"].cast<py::list>();
    for (const auto& event : events) {
      double event_time = event.cast<double>();
      event_queue.push(event_time);
    }
  }

  if (dict_result.contains("invalid_trial")) {
    result.invalid_trial = dict_result["invalid_trial"].cast<bool>();
  }

  return result;
}

ProcessResult DeciderWrapper::process_periodic(
    std::vector<neurosimo_pipeline_interfaces::msg::SensoryStimulus>& sensory_stimuli,
    const RingBuffer<std::shared_ptr<neurosimo_eeg_interfaces::msg::Sample>>& buffer,
    double_t reference_time,
    std::priority_queue<double, std::vector<double>, std::greater<double>>& event_queue,
    bool is_coil_at_target,
    const std::string& stage_name,
    uint64_t trial_in_stage) {

  int reference_index = -this->periodic_sample_window_start;
  size_t num_samples = this->periodic_sample_window_end - this->periodic_sample_window_start + 1;

  fill_arrays_from_buffer(buffer, reference_time,
    *periodic_time_offsets, *periodic_eeg, *periodic_emg,
    this->periodic_window_start_offset_in_envelope, num_samples);

  py::object py_result;
  try {
    set_current_processing_path(neurosimo_pipeline_interfaces::msg::LogMessage::PROCESSING_PATH_PERIODIC);
    py_result = decider_instance->attr("process_periodic")(
      reference_time, reference_index, *periodic_time_offsets, *periodic_eeg, *periodic_emg,
      is_coil_at_target, stage_name, trial_in_stage, false);

  } catch(const py::error_already_set& e) {
    std::string error_msg = std::string("Python error: ") + e.what();
    log_error(error_msg);
    log_buffer.push_back({error_msg, LogLevel::ERROR, current_processing_path});
    return ProcessResult::failure();

  } catch(const std::exception& e) {
    std::string error_msg = std::string("C++ error: ") + e.what();
    log_error(error_msg);
    log_buffer.push_back({error_msg, LogLevel::ERROR, current_processing_path});
    return ProcessResult::failure();
  }

  rotate_sample_buffers_if_referenced(
    periodic_time_offsets, periodic_eeg, periodic_emg,
    num_samples, eeg_size, emg_size);
  return parse_result_dict(py_result, true, sensory_stimuli, event_queue);
}

ProcessResult DeciderWrapper::process_pulse(
    std::vector<neurosimo_pipeline_interfaces::msg::SensoryStimulus>& sensory_stimuli,
    const RingBuffer<std::shared_ptr<neurosimo_eeg_interfaces::msg::Sample>>& buffer,
    double_t reference_time,
    std::priority_queue<double, std::vector<double>, std::greater<double>>& event_queue,
    bool is_coil_at_target,
    const std::string& stage_name,
    uint64_t trial_in_stage) {

  if (!has_pulse_processor_) {
    return ProcessResult::success_empty();
  }

  int reference_index = -this->pulse_sample_window_start;
  size_t num_samples = this->pulse_sample_window_end - this->pulse_sample_window_start + 1;

  fill_arrays_from_buffer(buffer, reference_time,
    *pulse_time_offsets, *pulse_eeg, *pulse_emg,
    this->pulse_window_start_offset_in_envelope, num_samples);

  py::object py_result;
  try {
    set_current_processing_path(neurosimo_pipeline_interfaces::msg::LogMessage::PROCESSING_PATH_PULSE);
    py_result = decider_instance->attr("process_pulse")(
      reference_time, reference_index, *pulse_time_offsets, *pulse_eeg, *pulse_emg,
      is_coil_at_target, stage_name, trial_in_stage);

  } catch(const py::error_already_set& e) {
    std::string error_msg = std::string("Python error: ") + e.what();
    log_error(error_msg);
    log_buffer.push_back({error_msg, LogLevel::ERROR, current_processing_path});
    return ProcessResult::failure();

  } catch(const std::exception& e) {
    std::string error_msg = std::string("C++ error: ") + e.what();
    log_error(error_msg);
    log_buffer.push_back({error_msg, LogLevel::ERROR, current_processing_path});
    return ProcessResult::failure();
  }

  rotate_sample_buffers_if_referenced(
    pulse_time_offsets, pulse_eeg, pulse_emg,
    num_samples, eeg_size, emg_size);
  return parse_result_dict(py_result, false, sensory_stimuli, event_queue);
}

ProcessResult DeciderWrapper::process_event(
    std::vector<neurosimo_pipeline_interfaces::msg::SensoryStimulus>& sensory_stimuli,
    const RingBuffer<std::shared_ptr<neurosimo_eeg_interfaces::msg::Sample>>& buffer,
    double_t reference_time,
    std::priority_queue<double, std::vector<double>, std::greater<double>>& event_queue,
    bool is_coil_at_target,
    const std::string& stage_name,
    uint64_t trial_in_stage) {

  if (!has_event_processor_) {
    return ProcessResult::success_empty();
  }

  int reference_index = -this->event_sample_window_start;
  size_t num_samples = this->event_sample_window_end - this->event_sample_window_start + 1;

  fill_arrays_from_buffer(buffer, reference_time,
    *event_time_offsets, *event_eeg, *event_emg,
    this->event_window_start_offset_in_envelope, num_samples);

  py::object py_result;
  try {
    set_current_processing_path(neurosimo_pipeline_interfaces::msg::LogMessage::PROCESSING_PATH_EVENT);
    py_result = decider_instance->attr("process_event")(
      reference_time, reference_index, *event_time_offsets, *event_eeg, *event_emg,
      is_coil_at_target, stage_name, trial_in_stage);

  } catch(const py::error_already_set& e) {
    std::string error_msg = std::string("Python error: ") + e.what();
    log_error(error_msg);
    log_buffer.push_back({error_msg, LogLevel::ERROR, current_processing_path});
    return ProcessResult::failure();

  } catch(const std::exception& e) {
    std::string error_msg = std::string("C++ error: ") + e.what();
    log_error(error_msg);
    log_buffer.push_back({error_msg, LogLevel::ERROR, current_processing_path});
    return ProcessResult::failure();
  }

  rotate_sample_buffers_if_referenced(
    event_time_offsets, event_eeg, event_emg,
    num_samples, eeg_size, emg_size);
  return parse_result_dict(py_result, false, sensory_stimuli, event_queue);
}

bool DeciderWrapper::has_pulse_processor() const {
  return this->has_pulse_processor_;
}

bool DeciderWrapper::has_event_processor() const {
  return this->has_event_processor_;
}

PrepareTrialResult DeciderWrapper::prepare_trial(
    const std::string& stage_name,
    uint64_t trial_in_stage) {

  if (!has_prepare_trial_processor_) {
    return PrepareTrialResult::success_empty();
  }

  py::object py_result;
  try {
    set_current_processing_path(neurosimo_pipeline_interfaces::msg::LogMessage::PROCESSING_PATH_PREPARE_TRIAL);
    py_result = decider_instance->attr("prepare_trial")(
      stage_name, trial_in_stage);

  } catch(const py::error_already_set& e) {
    std::string error_msg = std::string("Python error in prepare_trial: ") + e.what();
    log_error(error_msg);
    log_buffer.push_back({error_msg, LogLevel::ERROR, current_processing_path});
    return PrepareTrialResult::failure();

  } catch(const std::exception& e) {
    std::string error_msg = std::string("C++ error in prepare_trial: ") + e.what();
    log_error(error_msg);
    log_buffer.push_back({error_msg, LogLevel::ERROR, current_processing_path});
    return PrepareTrialResult::failure();
  }

  /* If the return value is None, no trigger offset — proceed with periodic processing. */
  if (py_result.is_none()) {
    return PrepareTrialResult::success_empty();
  }

  /* If a dict is returned, look for trigger_offset. */
  if (py::isinstance<py::dict>(py_result)) {
    py::dict dict_result = py_result.cast<py::dict>();

    PrepareTrialResult result;
    result.success = true;

    if (dict_result.contains("trigger_offset")) {
      result.trigger_offset = std::make_shared<double_t>(dict_result["trigger_offset"].cast<double_t>());
    }

    return result;
  }

  log_error("prepare_trial must return None or a dictionary.");
  return PrepareTrialResult::failure();
}

bool DeciderWrapper::has_prepare_trial_processor() const {
  return this->has_prepare_trial_processor_;
}

bool DeciderWrapper::process_task(const std::string& task_name) {
  if (!has_task_processor_) {
    log_error("process_task called but Python Decider has no process_task method.");
    return false;
  }

  try {
    set_current_processing_path(neurosimo_pipeline_interfaces::msg::LogMessage::PROCESSING_PATH_TASK);
    decider_instance->attr("process_task")(task_name);

  } catch(const py::error_already_set& e) {
    std::string error_msg = std::string("Python error in process_task: ") + e.what();
    log_error(error_msg);
    log_buffer.push_back({error_msg, LogLevel::ERROR, current_processing_path});
    return false;

  } catch(const std::exception& e) {
    std::string error_msg = std::string("C++ error in process_task: ") + e.what();
    log_error(error_msg);
    log_buffer.push_back({error_msg, LogLevel::ERROR, current_processing_path});
    return false;
  }

  return true;
}

bool DeciderWrapper::has_task_processor() const {
  return this->has_task_processor_;
}

rclcpp::Logger* DeciderWrapper::logger_ptr = nullptr;
std::vector<LogEntry> DeciderWrapper::log_buffer;
uint8_t DeciderWrapper::current_processing_path;
