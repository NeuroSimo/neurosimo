#include <pybind11/pybind11.h>
#include <pybind11/embed.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <rcpputils/filesystem_helper.hpp>
#include <cmath>
#include <chrono>
#include <cstdlib>

#include "decider_wrapper.h"
#include <eeg_interfaces/msg/sample.hpp>

namespace py = pybind11;

DeciderWrapper::DeciderWrapper(rclcpp::Logger& logger) {
  logger_ptr = &logger;

  /* Initialize processing path to undetermined */
  current_processing_path = pipeline_interfaces::msg::LogMessage::PROCESSING_PATH_UNDETERMINED;

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

bool DeciderWrapper::initialize_module(
    const std::string& project_directory,
    const std::string& module_directory,
    const std::string& module_name,
    const std::string& subject_id,
    const size_t eeg_size,
    const size_t emg_size,
    const uint16_t sampling_frequency,
    std::vector<pipeline_interfaces::msg::SensoryStimulus>& sensory_stimuli,
    std::priority_queue<double, std::vector<double>, std::greater<double>>& event_queue) {

  this->sampling_frequency = sampling_frequency;
  if (this->sampling_frequency == 0) {
    RCLCPP_ERROR(*logger_ptr, "Sampling frequency must be greater than zero to interpret sample_window expressed in seconds.");
    return false;
  }

  /* Set up the Python environment. */
  py::module sys_module = py::module::import("sys");
  py::list sys_path = sys_module.attr("path");
  sys_path.append(module_directory);

  /* Import the module. */
  try {
    auto imported_module = py::module::import(module_name.c_str());
    decider_module = std::make_unique<py::module>(imported_module);

  } catch (const py::error_already_set &e) {
    std::string error_msg = std::string("Python import error: ") + e.what();
    RCLCPP_ERROR(*logger_ptr, "%s", error_msg.c_str());

    // Add error to log buffer so it can be published to UI
    log_buffer.push_back({error_msg, LogLevel::ERROR, current_processing_path});
    return false;

  } catch (const std::exception &e) {
    std::string error_msg = std::string("C++ error during import: ") + e.what();
    RCLCPP_ERROR(*logger_ptr, "%s", error_msg.c_str());

    // Add error to log buffer so it can be published to UI
    log_buffer.push_back({error_msg, LogLevel::ERROR, current_processing_path});

    return false;
  }

  /* Initialize Decider instance. */
  try {
    auto instance = decider_module->attr("Decider")(subject_id, eeg_size, emg_size, sampling_frequency);
    decider_instance = std::make_unique<py::object>(instance);

  } catch (const py::error_already_set &e) {
    std::string error_msg = std::string("Python initialization error: ") + e.what();
    RCLCPP_ERROR(*logger_ptr, "%s", error_msg.c_str());

    // Add error to log buffer so it can be published to UI
    log_buffer.push_back({error_msg, LogLevel::ERROR, current_processing_path});
    return false;

  } catch (const std::exception &e) {
    std::string error_msg = std::string("C++ error during initialization: ") + e.what();
    RCLCPP_ERROR(*logger_ptr, "%s", error_msg.c_str());

    // Add error to log buffer so it can be published to UI
    log_buffer.push_back({error_msg, LogLevel::ERROR, current_processing_path});
    return false;
  }

  /* Extract the configuration from decider_instance. */
  if (!py::hasattr(*decider_instance, "get_configuration")) {
    RCLCPP_ERROR(*logger_ptr, "get_configuration method not found in the Decider instance.");
    return false;
  }

  /* Extract the configuration from decider_instance. */
  py::dict config = decider_instance->attr("get_configuration")().cast<py::dict>();

  /* Helper lambdas to convert seconds to sample counts. */
  const auto to_samples = [this](double seconds) -> int {
    return static_cast<int>(std::ceil(seconds * static_cast<double>(this->sampling_frequency)));
  };

  /* Validate that only allowed keys are present */
  std::vector<std::string> allowed_keys = {"sample_window", "warm_up_rounds", "predefined_sensory_stimuli", "periodic_processing_enabled", "periodic_processing_interval", "first_periodic_processing_at", "predefined_events", "pulse_lockout_duration", "pulse_processor", "event_processor"};
  for (const auto& item : config) {
    std::string key = py::str(item.first).cast<std::string>();
    if (std::find(allowed_keys.begin(), allowed_keys.end(), key) == allowed_keys.end()) {
      RCLCPP_ERROR(*logger_ptr, "Unexpected key '%s' in configuration dictionary. Only 'sample_window', 'warm_up_rounds', 'predefined_sensory_stimuli', 'periodic_processing_enabled', 'periodic_processing_interval', 'first_periodic_processing_at', 'predefined_events', 'pulse_lockout_duration', 'pulse_processor', and 'event_processor' are allowed.", key.c_str());
      return false;
    }
  }

  /* Extract predefined_sensory_stimuli (optional). */
  if (config.contains("predefined_sensory_stimuli")) {
    if (!py::isinstance<py::list>(config["predefined_sensory_stimuli"])) {
      RCLCPP_ERROR(*logger_ptr, "predefined_sensory_stimuli must be a list.");
      return false;
    }

    py::list py_sensory_stimuli = config["predefined_sensory_stimuli"].cast<py::list>();
    if (!process_sensory_stimuli_list(py_sensory_stimuli, sensory_stimuli)) {
      return false;
    }
  }

  /* Extract periodic_processing_enabled (mandatory). */
  if (!config.contains("periodic_processing_enabled")) {
    RCLCPP_ERROR(*logger_ptr, "'periodic_processing_enabled' key not found in configuration dictionary.");
    return false;
  }

  try {
    this->periodic_processing_enabled = config["periodic_processing_enabled"].cast<bool>();
  } catch (const py::cast_error& e) {
    RCLCPP_ERROR(*logger_ptr, "periodic_processing_enabled must be a boolean: %s", e.what());
    return false;
  }

  /* Extract periodic_processing_interval. */
  if (config.contains("periodic_processing_interval")) {
    try {
      this->periodic_processing_interval = config["periodic_processing_interval"].cast<double>();
      if (this->periodic_processing_interval <= 0.0) {
        RCLCPP_ERROR(*logger_ptr, "periodic_processing_interval must be a positive number (got %.3f).", this->periodic_processing_interval);
        return false;
      }
    } catch (const py::cast_error& e) {
      RCLCPP_ERROR(*logger_ptr, "periodic_processing_interval must be a number: %s", e.what());
      return false;
    }
  } else {
    this->periodic_processing_interval = 0.0;
  }

  /* Check that if periodic processing is enabled, the periodic_processing_interval is provided. */
  if (this->periodic_processing_enabled && !config.contains("periodic_processing_interval")) {
    RCLCPP_ERROR(*logger_ptr, "periodic_processing_enabled is true but 'periodic_processing_interval' is not provided in configuration dictionary.");
    return false;
  }

  /* Extract first_periodic_processing_at (in seconds), defaulting to periodic_processing_interval. */
  if (config.contains("first_periodic_processing_at")) {
    try {
      this->first_periodic_processing_at = config["first_periodic_processing_at"].cast<double>();
      if (this->first_periodic_processing_at < 0.0) {
        RCLCPP_ERROR(*logger_ptr, "first_periodic_processing_at must be non-negative.");
        return false;
      }
    } catch (const py::cast_error& e) {
      RCLCPP_ERROR(*logger_ptr, "first_periodic_processing_at must be a number: %s", e.what());
      return false;
    }
  } else {
    /* Default to same as periodic_processing_interval. */
    this->first_periodic_processing_at = this->periodic_processing_interval;
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
        RCLCPP_ERROR(*logger_ptr, "warm_up_rounds must be non-negative.");
        return false;
      }
    } catch (const py::cast_error& e) {
      RCLCPP_ERROR(*logger_ptr, "warm_up_rounds must be an integer: %s", e.what());
      return false;
    }
  } else {
    this->warm_up_rounds = 0;
  }

  /* Extract pulse_lockout_duration (optional, defaults to 0.0). */
  if (config.contains("pulse_lockout_duration")) {
    try {
      this->pulse_lockout_duration = config["pulse_lockout_duration"].cast<double>();
      if (this->pulse_lockout_duration < 0.0) {
        RCLCPP_ERROR(*logger_ptr, "pulse_lockout_duration must be non-negative.");
        return false;
      }
    } catch (const py::cast_error& e) {
      RCLCPP_ERROR(*logger_ptr, "pulse_lockout_duration must be a number: %s", e.what());
      return false;
    }
  } else {
    this->pulse_lockout_duration = 0.0;
  }

  /* Extract sample_window. */
  if (!config.contains("sample_window")) {
    RCLCPP_ERROR(*logger_ptr, "'sample_window' key not found in configuration dictionary.");
    return false;
  }

  double default_sample_window_start_seconds = 0.0;
  double default_sample_window_end_seconds = 0.0;

  py::list sample_window = config["sample_window"].cast<py::list>();
  if (sample_window.size() == 2) {
    default_sample_window_start_seconds = sample_window[0].cast<double>();
    default_sample_window_end_seconds = sample_window[1].cast<double>();
    if (default_sample_window_start_seconds > default_sample_window_end_seconds) {
      RCLCPP_ERROR(*logger_ptr,
                   "Invalid sample_window: start (%.3f s) must be <= end (%.3f s).",
                   default_sample_window_start_seconds,
                   default_sample_window_end_seconds);
      return false;
    }

    /* Convert seconds to sample counts. */
    this->periodic_sample_window_start = to_samples(default_sample_window_start_seconds);
    this->periodic_sample_window_end = to_samples(default_sample_window_end_seconds);
  } else {
    RCLCPP_ERROR(*logger_ptr, "'sample_window' value in configuration is of incorrect length (should be two elements).");
    return false;
  }

  /* Extract pulse_processor (optional). */
  double pulse_sample_window_start_seconds = 0.0;
  double pulse_sample_window_end_seconds = 0.0;

  if (config.contains("pulse_processor")) {
    py::object value = config["pulse_processor"].cast<py::object>();
    
    /* Check if value is a dict (with 'processor' and optional 'sample_window') or a callable */
    if (py::isinstance<py::dict>(value)) {
      py::dict processor_config = value.cast<py::dict>();
      
      /* Extract processor */
      if (!processor_config.contains("processor")) {
        RCLCPP_ERROR(*logger_ptr, "pulse_processor config must contain 'processor' key.");
        return false;
      }
      this->pulse_processor = processor_config["processor"].cast<py::object>();
      
      /* Extract optional sample_window */
      if (processor_config.contains("sample_window")) {
        py::list sample_window = processor_config["sample_window"].cast<py::list>();
        if (sample_window.size() != 2) {
          RCLCPP_ERROR(*logger_ptr, "sample_window for pulse_processor must have 2 elements.");
          return false;
        }
        pulse_sample_window_start_seconds = sample_window[0].cast<double>();
        pulse_sample_window_end_seconds = sample_window[1].cast<double>();
        if (pulse_sample_window_start_seconds > pulse_sample_window_end_seconds) {
          RCLCPP_ERROR(*logger_ptr,
                       "Invalid sample_window for pulse_processor: start (%.3f s) must be <= end (%.3f s).",
                       pulse_sample_window_start_seconds,
                       pulse_sample_window_end_seconds);
          return false;
        }

        this->pulse_sample_window_start = to_samples(pulse_sample_window_start_seconds);
        this->pulse_sample_window_end = to_samples(pulse_sample_window_end_seconds);
      }
    } else {
      /* Simple format: value is the processor directly */
      this->pulse_processor = value;

      /* Use the same sample window as periodic processing. */
      this->pulse_sample_window_start = this->periodic_sample_window_start;
      this->pulse_sample_window_end = this->periodic_sample_window_end;

      pulse_sample_window_start_seconds = default_sample_window_start_seconds;
      pulse_sample_window_end_seconds = default_sample_window_end_seconds;
    }
    
    if (!py::hasattr(this->pulse_processor, "__call__")) {
      RCLCPP_ERROR(*logger_ptr, "pulse_processor is not callable.");
      return false;
    }
    RCLCPP_DEBUG(*logger_ptr, "Registered pulse processor");
  }

  /* Extract event_processor (optional). */
  double event_sample_window_start_seconds = 0.0;
  double event_sample_window_end_seconds = 0.0;

  if (config.contains("event_processor")) {
    py::object value = config["event_processor"].cast<py::object>();
    
    /* Check if value is a dict (with 'processor' and optional 'sample_window') or a callable */
    if (py::isinstance<py::dict>(value)) {
      py::dict processor_config = value.cast<py::dict>();
      
      /* Extract processor */
      if (!processor_config.contains("processor")) {
        RCLCPP_ERROR(*logger_ptr, "event_processor config must contain 'processor' key.");
        return false;
      }
      this->event_processor = processor_config["processor"].cast<py::object>();
      
      /* Extract optional sample_window */
      if (processor_config.contains("sample_window")) {
        py::list sample_window = processor_config["sample_window"].cast<py::list>();
        if (sample_window.size() != 2) {
          RCLCPP_ERROR(*logger_ptr, "sample_window for event_processor must have 2 elements.");
          return false;
        }
        event_sample_window_start_seconds = sample_window[0].cast<double>();
        event_sample_window_end_seconds = sample_window[1].cast<double>();
        if (event_sample_window_start_seconds > event_sample_window_end_seconds) {
          RCLCPP_ERROR(*logger_ptr,
                       "Invalid sample_window for event_processor: start (%.3f s) must be <= end (%.3f s).",
                       event_sample_window_start_seconds,
                       event_sample_window_end_seconds);
          return false;
        }

        this->event_sample_window_start = to_samples(event_sample_window_start_seconds);
        this->event_sample_window_end = to_samples(event_sample_window_end_seconds);
      }
    } else {
      /* Simple format: value is the processor directly */
      this->event_processor = value;

      /* Use the same sample window as periodic processing. */
      this->event_sample_window_start = this->periodic_sample_window_start;
      this->event_sample_window_end = this->periodic_sample_window_end;

      event_sample_window_start_seconds = default_sample_window_start_seconds;
      event_sample_window_end_seconds = default_sample_window_end_seconds;
    }
    
    if (!py::hasattr(this->event_processor, "__call__")) {
      RCLCPP_ERROR(*logger_ptr, "event_processor is not callable.");
      return false;
    }
    RCLCPP_DEBUG(*logger_ptr, "Registered event processor");
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

  if (!this->periodic_processing_enabled) {
    RCLCPP_INFO(*logger_ptr, "  - Periodic processing: %sDisabled%s", bold_on.c_str(), bold_off.c_str());
  } else {
    RCLCPP_INFO(*logger_ptr, "  - Periodic processing: %sEnabled%s (interval: %.3f s)", bold_on.c_str(), bold_off.c_str(), this->periodic_processing_interval);
  }
            
  if (this->pulse_processor.is_none()) {
    RCLCPP_INFO(*logger_ptr, "  - Pulse processor: %sDisabled%s", bold_on.c_str(), bold_off.c_str());
  } else {
    RCLCPP_INFO(*logger_ptr, "  - Pulse processor: %sEnabled%s", bold_on.c_str(), bold_off.c_str());
    RCLCPP_INFO(*logger_ptr, "    - Pulse processor window: %s[%.3f s, %.3f s]%s",
                bold_on.c_str(), pulse_sample_window_start_seconds, pulse_sample_window_end_seconds, bold_off.c_str());
  }

  if (this->event_processor.is_none()) {
    RCLCPP_INFO(*logger_ptr, "  - Event processor: %sDisabled%s", bold_on.c_str(), bold_off.c_str());
  } else {
    RCLCPP_INFO(*logger_ptr, "  - Event processor: %sEnabled%s", bold_on.c_str(), bold_off.c_str());
    RCLCPP_INFO(*logger_ptr, "    - Event processor window: %s[%.3f s, %.3f s]%s",
                bold_on.c_str(), event_sample_window_start_seconds, event_sample_window_end_seconds, bold_off.c_str());
  }

  if (this->pulse_lockout_duration > 0.0) {
    RCLCPP_INFO(*logger_ptr, "  - Pulse lockout duration: %s%.1f%s (s)", bold_on.c_str(), this->pulse_lockout_duration, bold_off.c_str());
  }
  RCLCPP_INFO(*logger_ptr, " ");

  return true;
}

void DeciderWrapper::destroy_instance() {
  /* Initially, there are a maximum of three references to the decider instance:
     - the decider_instance
     - the pulse_processor
     - the event_processor

    Release these references one by one; once the reference count reaches 0, __del__ will be
    triggered synchronously. */
  pulse_processor = py::none();
  event_processor = py::none();
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
        true  /* is_warm_up */
      );
      
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

double DeciderWrapper::get_first_periodic_processing_at() const {
  return this->first_periodic_processing_at;
}

bool DeciderWrapper::is_processing_interval_enabled() const {
  return this->periodic_processing_enabled;
}

int DeciderWrapper::get_periodic_look_ahead_samples() const {
  return std::max(this->periodic_sample_window_end, 0);
}

int DeciderWrapper::get_pulse_look_ahead_samples() const {
  /* If no pulse processor is configured, pulses should be processed immediately. */
  if (this->pulse_processor.is_none()) {
    return 0;
  }
  return std::max(this->pulse_sample_window_end, 0);
}

int DeciderWrapper::get_event_look_ahead_samples() const {
  return std::max(this->event_sample_window_end, 0);
}

double DeciderWrapper::get_pulse_lockout_duration() const {
  return this->pulse_lockout_duration;
}

bool DeciderWrapper::parse_sensory_stimulus_dict(
  const py::dict& py_sensory_stimulus,
  pipeline_interfaces::msg::SensoryStimulus& out_msg) {

  static const std::vector<std::string> required = {
    "time",
    "type",
    "parameters"
  };

  // 1) Check presence
  for (const auto& field : required) {
    if (!py_sensory_stimulus.contains(field)) {
      RCLCPP_ERROR(*logger_ptr,
        "sensory_stimulus missing field '%s'", field.c_str());
      return false;
    }
  }

  // 2) Type‐check & cast with per-field error reporting
  try {
    out_msg.time = py_sensory_stimulus["time"].cast<double>();
  } catch (const py::cast_error& e) {
    RCLCPP_ERROR(*logger_ptr, "'time' must be a float or int: %s", e.what());
    return false;
  }

  try {
    out_msg.type = py_sensory_stimulus["type"].cast<std::string>();
  } catch (const py::cast_error& e) {
    RCLCPP_ERROR(*logger_ptr, "'type' must be a string: %s", e.what());
    return false;
  }

  py::dict params;
  try {
    params = py_sensory_stimulus["parameters"].cast<py::dict>();
  } catch (const py::cast_error& e) {
    RCLCPP_ERROR(*logger_ptr, "'parameters' must be a dictionary: %s", e.what());
    return false;
  }

  // 3) Iterate and fill diagnostic_msgs::msg::KeyValue[]
  out_msg.parameters.clear();
  for (const auto& item : params) {
    std::string key;
    try {
      key = item.first.cast<std::string>();
    } catch (const py::cast_error& e) {
      RCLCPP_ERROR(*logger_ptr, "parameter key is not a string: %s", e.what());
      return false;
    }

    std::string val;
    try {
      val = py::str(item.second);  // Serializes any Python value
    } catch (const std::exception& e) {
      RCLCPP_ERROR(*logger_ptr, "parameter value could not be serialized to string: %s", e.what());
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
    std::vector<pipeline_interfaces::msg::SensoryStimulus>& sensory_stimuli) {

  if (py_sensory_stimuli.size() > 0) {
    for (const auto& py_sensory_stimulus : py_sensory_stimuli) {
      if (!py::isinstance<py::dict>(py_sensory_stimulus)) {
        RCLCPP_ERROR(*logger_ptr, "sensory_stimuli must be a list of dictionaries.");
        return false;
      }
      py::dict py_sensory_stimulus_dict = py_sensory_stimulus.cast<py::dict>();
      pipeline_interfaces::msg::SensoryStimulus msg;
      if (parse_sensory_stimulus_dict(py_sensory_stimulus_dict, msg)) {
        sensory_stimuli.push_back(msg);
      } else {
        RCLCPP_ERROR(*logger_ptr, "Failed to parse sensory_stimuli dictionary.");
        return false;
      }
    }
  }
  return true;
}

void DeciderWrapper::fill_arrays_from_buffer(
    const RingBuffer<std::shared_ptr<eeg_interfaces::msg::Sample>>& buffer,
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

/* TODO: Use struct for the return value. */
std::tuple<bool, std::shared_ptr<pipeline_interfaces::msg::TimedTrigger>, std::string> DeciderWrapper::process(
    std::vector<pipeline_interfaces::msg::SensoryStimulus>& sensory_stimuli,
    const RingBuffer<std::shared_ptr<eeg_interfaces::msg::Sample>>& buffer,
    double_t reference_time,
    ProcessingReason processing_reason,
    std::priority_queue<double, std::vector<double>, std::greater<double>>& event_queue,
    bool is_coil_at_target) {

  std::shared_ptr<pipeline_interfaces::msg::TimedTrigger> timed_trigger = nullptr;
  std::string coil_target;

  /* Determine which arrays to use and their parameters. */
  py::array_t<double>* py_time_offsets;
  py::array_t<double>* py_eeg;
  py::array_t<double>* py_emg;
  int reference_index;
  size_t start_offset;
  size_t num_samples;

  if (processing_reason == ProcessingReason::Periodic) {
    py_time_offsets = periodic_time_offsets.get();
    py_eeg = periodic_eeg.get();
    py_emg = periodic_emg.get();
    reference_index = -this->periodic_sample_window_start;
    start_offset = this->periodic_window_start_offset_in_envelope;
    num_samples = this->periodic_sample_window_end - this->periodic_sample_window_start + 1;

  } else if (processing_reason == ProcessingReason::Pulse) {
    py_time_offsets = pulse_time_offsets.get();
    py_eeg = pulse_eeg.get();
    py_emg = pulse_emg.get();
    reference_index = -this->pulse_sample_window_start;
    start_offset = this->pulse_window_start_offset_in_envelope;
    num_samples = this->pulse_sample_window_end - this->pulse_sample_window_start + 1;

  } else if (processing_reason == ProcessingReason::Event) {
    py_time_offsets = event_time_offsets.get();
    py_eeg = event_eeg.get();
    py_emg = event_emg.get();
    reference_index = -this->event_sample_window_start;
    start_offset = this->event_window_start_offset_in_envelope;
    num_samples = this->event_sample_window_end - this->event_sample_window_start + 1;

  } else {
    RCLCPP_ERROR(*logger_ptr, "Invalid processing reason: %d", static_cast<int>(processing_reason));
    return {false, timed_trigger, coil_target};
  }

  /* Fill the selected arrays from the reason-specific window position in the envelope. */
  fill_arrays_from_buffer(buffer, reference_time, *py_time_offsets, *py_eeg, *py_emg, start_offset, num_samples);

  /* Call the appropriate Python function. */
  py::object py_result;
  try {
    switch (processing_reason) {
      case ProcessingReason::Periodic:
        set_current_processing_path(pipeline_interfaces::msg::LogMessage::PROCESSING_PATH_PERIODIC);
        /* Call periodic processor. */
        py_result = decider_instance->attr("process_periodic")(reference_time, reference_index, *py_time_offsets, *py_eeg, *py_emg, is_coil_at_target, false);
        break;

      case ProcessingReason::Pulse:
        set_current_processing_path(pipeline_interfaces::msg::LogMessage::PROCESSING_PATH_PULSE);
        /* Call pulse processor if registered. */
        if (pulse_processor.is_none()) {
          py_result = py::none();
          break;
        }
        py_result = pulse_processor(reference_time, reference_index, *py_time_offsets, *py_eeg, *py_emg, is_coil_at_target);
        break;

      case ProcessingReason::Event:
        set_current_processing_path(pipeline_interfaces::msg::LogMessage::PROCESSING_PATH_EVENT);
        /* Call event processor if registered. */
        if (event_processor.is_none()) {
          py_result = py::none();
          break;
        }
        py_result = event_processor(reference_time, reference_index, *py_time_offsets, *py_eeg, *py_emg, is_coil_at_target);
        break;

      default:
        RCLCPP_ERROR(*logger_ptr, "Invalid processing type: %d", static_cast<int>(processing_reason));
        py_result = py::none();
        break;
    }

  } catch(const py::error_already_set& e) {
    std::string error_msg = std::string("Python error: ") + e.what();
    RCLCPP_ERROR(*logger_ptr, "%s", error_msg.c_str());

    // Add error to log buffer so it can be published to UI
    log_buffer.push_back({error_msg, LogLevel::ERROR, current_processing_path});

    return {false, timed_trigger, coil_target};

  } catch(const std::exception& e) {
    std::string error_msg = std::string("C++ error: ") + e.what();
    RCLCPP_ERROR(*logger_ptr, "%s", error_msg.c_str());

    // Add error to log buffer so it can be published to UI
    log_buffer.push_back({error_msg, LogLevel::ERROR, current_processing_path});

    return {false, timed_trigger, coil_target};
  }

  /* If the return value is None, return early but mark it as successful. */
  if (py_result.is_none()) {
    return {true, timed_trigger, coil_target};
  }

  /* If the return value is not None, ensure that it is a dictionary. */
  if (!py::isinstance<py::dict>(py_result)) {
    RCLCPP_ERROR(*logger_ptr, "Python module should return a dictionary.");
    return {false, timed_trigger, coil_target};
  }

  /* Extract the dictionary from the result first */
  py::dict dict_result = py_result.cast<py::dict>();

  /* Validate that only allowed keys are present */
  std::vector<std::string> allowed_keys = {"timed_trigger", "sensory_stimuli", "events", "coil_target"};
  for (const auto& item : dict_result) {
    std::string key = py::str(item.first).cast<std::string>();
    if (std::find(allowed_keys.begin(), allowed_keys.end(), key) == allowed_keys.end()) {
      RCLCPP_ERROR(*logger_ptr, "Unexpected key '%s' in return value, only 'timed_trigger', 'sensory_stimuli', 'events', and 'coil_target' are allowed.", key.c_str());
      return {false, timed_trigger, coil_target};
    }
  }

  if (dict_result.contains("coil_target")) {
    coil_target = dict_result["coil_target"].cast<std::string>();
  }

  if (dict_result.contains("timed_trigger")) {
    if (processing_reason == ProcessingReason::Pulse || processing_reason == ProcessingReason::Event) {
      RCLCPP_ERROR(*logger_ptr, "Timed trigger requests are not allowed for pulse or event processing.");
      return {false, timed_trigger, coil_target};
    }
    timed_trigger = std::make_shared<pipeline_interfaces::msg::TimedTrigger>();
    timed_trigger->time = dict_result["timed_trigger"].cast<double_t>();
  }

  if (dict_result.contains("sensory_stimuli")) {
    if (!py::isinstance<py::list>(dict_result["sensory_stimuli"])) {
      RCLCPP_ERROR(*logger_ptr, "sensory_stimuli must be a list.");
      return {false, timed_trigger, coil_target};
    }

    py::list py_sensory_stimuli = dict_result["sensory_stimuli"].cast<py::list>();
    if (!process_sensory_stimuli_list(py_sensory_stimuli, sensory_stimuli)) {
      return {false, timed_trigger, coil_target};
    }
  }

  if (dict_result.contains("events")) {
    if (!py::isinstance<py::list>(dict_result["events"])) {
      RCLCPP_ERROR(*logger_ptr, "events must be a list.");
      return {false, timed_trigger, coil_target};
    }

    py::list events = dict_result["events"].cast<py::list>();
    for (const auto& event : events) {
      double event_time = event.cast<double>();
      event_queue.push(event_time);
    }
  }

  return {true, timed_trigger, coil_target};
}

rclcpp::Logger* DeciderWrapper::logger_ptr = nullptr;
std::vector<LogEntry> DeciderWrapper::log_buffer;
uint8_t DeciderWrapper::current_processing_path;
