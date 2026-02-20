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

  /* Start IPC log server first, so cpp_bindings.log() can forward immediately. */
  log_server = std::make_unique<LogIpcServer>([](std::string&& msg) {
    std::lock_guard<std::mutex> lock(log_buffer_mutex);
    log_buffer.push_back({std::move(msg), LogLevel::INFO});
  });
  if (!log_server->start()) {
    RCLCPP_WARN(*logger_ptr, "Failed to start log IPC server; worker logs may be lost.");
  }

  /* Initialize the Python interpreter. */
  interpreter = std::make_unique<py::scoped_interpreter>();
  setup_custom_print();

  /* After interpreter init + setup_custom_print(), release the GIL
     so executor threads can acquire it in their callbacks. */
  main_thread_state = PyEval_SaveThread();
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

void DeciderWrapper::remove_modules(const std::string& base_directory) {
  py::module sys_module = py::module::import("sys");
  py::dict sys_modules = sys_module.attr("modules");
  py::module os_module = py::module::import("os");
  py::object path_obj = os_module.attr("path");

  std::vector<std::string> modules_to_remove;

  /* Ensure directory is an absolute path. */
  std::string abs_directory = py::str(path_obj.attr("abspath")(base_directory));

  /* Find modules that are in the specified directory or its subdirectories. */
  for (auto item : sys_modules) {
    std::string module_name_str = py::str(item.first).cast<std::string>();
    py::handle module_handle = item.second;

    try {
      if (py::hasattr(module_handle, "__file__")) {
        py::str module_file = module_handle.attr("__file__");
        std::string abs_module_file = py::str(path_obj.attr("abspath")(module_file));

        if (abs_module_file.find(abs_directory) == 0) {
          modules_to_remove.push_back(module_name_str);
          RCLCPP_DEBUG(*logger_ptr, "Marking module for removal: %s", module_name_str.c_str());
        }
      }
    } catch (const py::error_already_set& e) {
      RCLCPP_WARN(*logger_ptr, "Error processing module %s: %s", module_name_str.c_str(), e.what());
    }
  }

  /* Remove the modules. */
  for (const auto& module_name_str : modules_to_remove) {
    try {
      if (sys_modules.contains(module_name_str.c_str())) {
        sys_modules.attr("__delitem__")(module_name_str.c_str());
      } else {
        RCLCPP_WARN(*logger_ptr, "Module %s no longer in sys.modules, skipping removal", module_name_str.c_str());
      }
    } catch (const py::error_already_set& e) {
      RCLCPP_ERROR(*logger_ptr, "Failed to remove module %s: %s", module_name_str.c_str(), e.what());
    }
  }
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

  py::gil_scoped_acquire gil;

  this->sampling_frequency = sampling_frequency;
  if (this->sampling_frequency == 0) {
    RCLCPP_ERROR(*logger_ptr, "Sampling frequency must be greater than zero to interpret sample_window expressed in seconds.");
    return false;
  }

  /* Reset the module state. */
  bool success = reset_module_state();
  if (!success) {
    return false;
  }

  /* Local storage for logging */
  double default_window_earliest_seconds = 0.0;
  double default_window_latest_seconds = 0.0;
  std::unordered_map<std::string, std::pair<double, double>> event_window_seconds;

  /* Set up the Python environment. */
  py::module sys_module = py::module::import("sys");
  py::list sys_path = sys_module.attr("path");
  sys_path.append(module_directory);

  /* Remove already loaded modules to ensure their reloading. Only remove modules under the
     specified base directory to ensure that third-party imports such as NumPy are not reloaded -
     attempting to reload them may result in errors.

     Furthermore, use project directory as the base directory to ensure that previously loaded
     modules are removed even when the project is changed.

     TODO: Add tests. */
  remove_modules(project_directory);

  /* Import the module. */
  try {
    auto imported_module = py::module::import(module_name.c_str());
    decider_module = std::make_unique<py::module>(imported_module);

  } catch (const py::error_already_set &e) {
    std::string error_msg = std::string("Python import error: ") + e.what();
    RCLCPP_ERROR(*logger_ptr, "%s", error_msg.c_str());
    
    // Add error to log buffer so it can be published to UI
    {
      std::lock_guard<std::mutex> lock(log_buffer_mutex);
      log_buffer.push_back({error_msg, LogLevel::ERROR});
    }
    return false;

  } catch (const std::exception &e) {
    std::string error_msg = std::string("C++ error during import: ") + e.what();
    RCLCPP_ERROR(*logger_ptr, "%s", error_msg.c_str());
    
    // Add error to log buffer so it can be published to UI
    {
      std::lock_guard<std::mutex> lock(log_buffer_mutex);
      log_buffer.push_back({error_msg, LogLevel::ERROR});
    }
    
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
    {
      std::lock_guard<std::mutex> lock(log_buffer_mutex);
      log_buffer.push_back({error_msg, LogLevel::ERROR});
    }
    return false;

  } catch (const std::exception &e) {
    std::string error_msg = std::string("C++ error during initialization: ") + e.what();
    RCLCPP_ERROR(*logger_ptr, "%s", error_msg.c_str());
    
    // Add error to log buffer so it can be published to UI
    {
      std::lock_guard<std::mutex> lock(log_buffer_mutex);
      log_buffer.push_back({error_msg, LogLevel::ERROR});
    }
    return false;
  }

  /* Extract the configuration from decider_instance. */
  if (py::hasattr(*decider_instance, "get_configuration")) {
    py::dict config = decider_instance->attr("get_configuration")().cast<py::dict>();

    /* Helper lambdas to convert seconds to non-negative sample counts. */
    const auto to_look_back_samples = [this](double earliest_seconds) -> int {
      /* earliest_seconds is expected to be negative or zero. */
      double seconds = std::max(0.0, -earliest_seconds);
      return static_cast<int>(std::ceil(seconds * static_cast<double>(this->sampling_frequency)));
    };
    const auto to_look_ahead_samples = [this](double latest_seconds) -> int {
      /* latest_seconds is expected to be positive or zero. */
      double seconds = std::max(0.0, latest_seconds);
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

    /* Extract sample_window. */
    if (config.contains("sample_window")) {
      py::list sample_window = config["sample_window"].cast<py::list>();
      if (sample_window.size() == 2) {
        double earliest_seconds = sample_window[0].cast<double>();
        double latest_seconds = sample_window[1].cast<double>();
        
        /* Convert seconds to sample counts (ceil to ensure coverage). */
        this->look_back_samples = to_look_back_samples(earliest_seconds);
        this->look_ahead_samples = to_look_ahead_samples(latest_seconds);
        this->buffer_size = this->look_back_samples + this->look_ahead_samples + 1;

        /* Store window in seconds for later logging. */
        default_window_earliest_seconds = earliest_seconds;
        default_window_latest_seconds = latest_seconds;

        RCLCPP_DEBUG(*logger_ptr,
          "Configured default sample window: [%.6f s, %.6f s] -> look_back=%d samples, look_ahead=%d samples",
          earliest_seconds,
          latest_seconds,
          this->look_back_samples,
          this->look_ahead_samples);
      } else {
        RCLCPP_ERROR(*logger_ptr, "'sample_window' value in configuration is of incorrect length (should be two elements).");
        return false;
      }

    } else {
      RCLCPP_ERROR(*logger_ptr, "'sample_window' key not found in configuration dictionary.");
      return false;
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
    if (config.contains("periodic_processing_enabled")) {
      try {
        this->periodic_processing_enabled = config["periodic_processing_enabled"].cast<bool>();
      } catch (const py::cast_error& e) {
        RCLCPP_ERROR(*logger_ptr, "periodic_processing_enabled must be a boolean: %s", e.what());
        return false;
      }
    } else {
      RCLCPP_ERROR(*logger_ptr, "'periodic_processing_enabled' key not found in configuration dictionary.");
      return false;
    }

    /* Check that if periodic processing is enabled, the periodic_processing_interval is provided. */
    if (this->periodic_processing_enabled && !config.contains("periodic_processing_interval")) {
      RCLCPP_ERROR(*logger_ptr, "periodic_processing_enabled is true but 'periodic_processing_interval' is not provided in configuration dictionary.");
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

    /* Extract pulse_processor (optional). */
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
          double earliest_seconds = sample_window[0].cast<double>();
          double latest_seconds = sample_window[1].cast<double>();
          int pulse_look_back = to_look_back_samples(earliest_seconds);
          int pulse_look_ahead = to_look_ahead_samples(latest_seconds);
          
          this->pulse_look_back_samples = pulse_look_back;
          this->pulse_look_ahead_samples = pulse_look_ahead;
          this->has_custom_pulse_window = true;
          
          RCLCPP_DEBUG(*logger_ptr,
            "Registered custom sample window [%.6f s, %.6f s] for pulse_processor",
            earliest_seconds, latest_seconds);
        }
      } else {
        /* Simple format: value is the processor directly */
        this->pulse_processor = value;
      }
      
      if (!py::hasattr(this->pulse_processor, "__call__")) {
        RCLCPP_ERROR(*logger_ptr, "pulse_processor is not callable.");
        return false;
      }
      RCLCPP_DEBUG(*logger_ptr, "Registered pulse processor");
    }

    /* Extract event_processor (optional). */
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
          double earliest_seconds = sample_window[0].cast<double>();
          double latest_seconds = sample_window[1].cast<double>();
          int event_look_back = to_look_back_samples(earliest_seconds);
          int event_look_ahead = to_look_ahead_samples(latest_seconds);
          
          this->event_look_back_samples = event_look_back;
          this->event_look_ahead_samples = event_look_ahead;
          this->has_custom_event_window = true;
          
          RCLCPP_DEBUG(*logger_ptr,
            "Registered custom sample window [%.6f s, %.6f s] for event_processor",
            earliest_seconds, latest_seconds);
        }
      } else {
        /* Simple format: value is the processor directly */
        this->event_processor = value;
      }
      
      if (!py::hasattr(this->event_processor, "__call__")) {
        RCLCPP_ERROR(*logger_ptr, "event_processor is not callable.");
        return false;
      }
      RCLCPP_DEBUG(*logger_ptr, "Registered event processor");
    }
  } else {
    RCLCPP_ERROR(*logger_ptr, "get_configuration method not found in the Decider instance.");
    return false;
  }

  /* Calculate maximum buffer size needed to cover all windows. */
  int max_look_back = this->look_back_samples;
  int max_look_ahead = this->look_ahead_samples;
  
  if (this->has_custom_pulse_window) {
    max_look_back = std::max(max_look_back, this->pulse_look_back_samples);
    max_look_ahead = std::max(max_look_ahead, this->pulse_look_ahead_samples);
  }
  
  if (this->has_custom_event_window) {
    max_look_back = std::max(max_look_back, this->event_look_back_samples);
    max_look_ahead = std::max(max_look_ahead, this->event_look_ahead_samples);
  }
  
  this->buffer_size = max_look_back + max_look_ahead + 1;
  
  RCLCPP_DEBUG(*logger_ptr, "Maximum envelope: look_back=%d, look_ahead=%d, buffer size: %zu", 
               max_look_back, max_look_ahead, this->buffer_size);

  this->eeg_size = eeg_size;
  this->emg_size = emg_size;

  /* Initialize numpy arrays for default window. */
  py_time_offsets = std::make_unique<py::array_t<double>>(this->look_back_samples + this->look_ahead_samples + 1);

  std::vector<size_t> eeg_shape = {this->look_back_samples + this->look_ahead_samples + 1, eeg_size};
  py_eeg = std::make_unique<py::array_t<double>>(eeg_shape);

  std::vector<size_t> emg_shape = {this->look_back_samples + this->look_ahead_samples + 1, emg_size};
  py_emg = std::make_unique<py::array_t<double>>(emg_shape);
  
  /* Initialize numpy arrays for pulse processor if it has a custom window. */
  if (this->has_custom_pulse_window) {
    size_t pulse_buffer_size = this->pulse_look_back_samples + this->pulse_look_ahead_samples + 1;
    pulse_time_offsets = std::make_unique<py::array_t<double>>(pulse_buffer_size);
    
    std::vector<size_t> pulse_eeg_shape = {pulse_buffer_size, eeg_size};
    pulse_eeg = std::make_unique<py::array_t<double>>(pulse_eeg_shape);
    
    std::vector<size_t> pulse_emg_shape = {pulse_buffer_size, emg_size};
    pulse_emg = std::make_unique<py::array_t<double>>(pulse_emg_shape);
    
    RCLCPP_DEBUG(*logger_ptr, "Preallocated arrays for pulse_processor with buffer size %zu", pulse_buffer_size);
  }
  
  /* Initialize numpy arrays for event processor if it has a custom window. */
  if (this->has_custom_event_window) {
    size_t event_buffer_size = this->event_look_back_samples + this->event_look_ahead_samples + 1;
    event_time_offsets = std::make_unique<py::array_t<double>>(event_buffer_size);
    
    std::vector<size_t> event_eeg_shape = {event_buffer_size, eeg_size};
    event_eeg = std::make_unique<py::array_t<double>>(event_eeg_shape);
    
    std::vector<size_t> event_emg_shape = {event_buffer_size, emg_size};
    event_emg = std::make_unique<py::array_t<double>>(event_emg_shape);
    
    RCLCPP_DEBUG(*logger_ptr, "Preallocated arrays for event_processor with buffer size %zu", event_buffer_size);
  }

  /* Log the configuration. */
  RCLCPP_INFO(*logger_ptr, "Configuration:");
  RCLCPP_INFO(*logger_ptr, " ");
  RCLCPP_INFO(*logger_ptr, "  - Default sample window: %s[%.3f s, %.3f s]%s",
              bold_on.c_str(),
              default_window_earliest_seconds,
              default_window_latest_seconds,
              bold_off.c_str());
  
  if (this->has_custom_pulse_window) {
    double pulse_earliest = -this->pulse_look_back_samples / static_cast<double>(this->sampling_frequency);
    double pulse_latest = this->pulse_look_ahead_samples / static_cast<double>(this->sampling_frequency);
    RCLCPP_INFO(*logger_ptr, "  - Pulse processor window: %s[%.3f s, %.3f s]%s",
                bold_on.c_str(), pulse_earliest, pulse_latest, bold_off.c_str());
  }
  
  if (this->has_custom_event_window) {
    double event_earliest = -this->event_look_back_samples / static_cast<double>(this->sampling_frequency);
    double event_latest = this->event_look_ahead_samples / static_cast<double>(this->sampling_frequency);
    RCLCPP_INFO(*logger_ptr, "  - Event processor window: %s[%.3f s, %.3f s]%s",
                bold_on.c_str(), event_earliest, event_latest, bold_off.c_str());
  }
  
  if (!this->periodic_processing_enabled) {
    RCLCPP_INFO(*logger_ptr, "  - Periodic processing: %sDisabled%s", bold_on.c_str(), bold_off.c_str());
  } else {
    RCLCPP_INFO(*logger_ptr, "  - Periodic processing: %sEnabled%s (interval: %.3f s)", bold_on.c_str(), bold_off.c_str(), this->periodic_processing_interval);
  }
  if (this->pulse_lockout_duration > 0.0) {
    RCLCPP_INFO(*logger_ptr, "  - Pulse lockout duration: %s%.1f%s (s)", bold_on.c_str(), this->pulse_lockout_duration, bold_off.c_str());
  }
  RCLCPP_INFO(*logger_ptr, " ");

  return true;
}

void DeciderWrapper::destroy_instance() {
  py::gil_scoped_acquire gil;

  /* Setting to nullptr decrements the Python refcount; in CPython this triggers __del__ synchronously. */
  decider_instance = nullptr;
}

bool DeciderWrapper::reset_module_state() {
  py::gil_scoped_acquire gil;

  decider_instance = nullptr;
  decider_module = nullptr;

  py_time_offsets.reset();
  py_eeg.reset();
  py_emg.reset();
  
  pulse_time_offsets.reset();
  pulse_eeg.reset();
  pulse_emg.reset();
  
  event_time_offsets.reset();
  event_eeg.reset();
  event_emg.reset();

  return true;
}

bool DeciderWrapper::warm_up() {
  py::gil_scoped_acquire gil;

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
  auto time_offsets_ptr = py_time_offsets->mutable_data();
  auto eeg_ptr = py_eeg->mutable_data();
  auto emg_ptr = py_emg->mutable_data();

  // Dummy parameters for process method (constant across all rounds)
  double_t dummy_reference_time = 0.0;
  int dummy_reference_index = look_back_samples;
  bool dummy_is_coil_at_target = true;

  // Perform warm-up rounds with fresh random data for each round
  for (int round = 0; round < this->warm_up_rounds; round++) {
    // Generate fresh random data for this round
    for (size_t i = 0; i < buffer_size; i++) {
      time_offsets_ptr[i] = -static_cast<double>(buffer_size - 1 - i) / sampling_frequency;

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
        *py_time_offsets,
        *py_eeg,
        *py_emg,
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
      {
        std::lock_guard<std::mutex> lock(log_buffer_mutex);
        log_buffer.push_back({error_msg, LogLevel::ERROR});
      }
      
      return false;
    } catch (const std::exception& e) {
      std::string error_msg = std::string("C++ error: ") + e.what();
      RCLCPP_WARN(*logger_ptr, "  Round %d/%d: FAILED (%s)", round + 1, this->warm_up_rounds, error_msg.c_str());
      
      // Add error to log buffer so it can be published to UI
      {
        std::lock_guard<std::mutex> lock(log_buffer_mutex);
        log_buffer.push_back({error_msg, LogLevel::ERROR});
      }
      
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
  /* Restore the main thread state before destroying the interpreter */
  if (main_thread_state) {
    PyEval_RestoreThread(main_thread_state);
    main_thread_state = nullptr;
  }

  py_time_offsets.reset();
  py_eeg.reset();
  py_emg.reset();
  
  pulse_time_offsets.reset();
  pulse_eeg.reset();
  pulse_emg.reset();
  
  event_time_offsets.reset();
  event_eeg.reset();
  event_emg.reset();

  if (log_server) log_server->stop();
}

std::vector<LogEntry> DeciderWrapper::get_and_clear_logs() {
  std::lock_guard<std::mutex> lock(log_buffer_mutex);
  std::vector<LogEntry> logs = std::move(log_buffer);
  log_buffer.clear();
  return logs;
}

void DeciderWrapper::drain_logs() {
  if (log_server) {
    log_server->drain();
  }
}

std::size_t DeciderWrapper::get_buffer_size() const {
  return this->buffer_size;
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

int DeciderWrapper::get_look_ahead_samples() const {
  /* For a sample window like [-10, 5], look_ahead_samples is 5, which represents
     the number of samples we need to look ahead from the triggering sample. */
  return this->look_ahead_samples;
}

int DeciderWrapper::get_look_ahead_samples_for_pulse() const {
  if (this->has_custom_pulse_window) {
    return this->pulse_look_ahead_samples;
  }
  return this->look_ahead_samples;
}

int DeciderWrapper::get_look_ahead_samples_for_event() const {
  if (this->has_custom_event_window) {
    return this->event_look_ahead_samples;
  }
  return this->look_ahead_samples;
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
    bool pulse_trigger,
    bool has_event,
    std::priority_queue<double, std::vector<double>, std::greater<double>>& event_queue,
    bool is_coil_at_target) {

  py::gil_scoped_acquire gil;

  bool success = true;
  std::shared_ptr<pipeline_interfaces::msg::TimedTrigger> timed_trigger = nullptr;
  std::string coil_target;

  /* Determine which arrays to use and their parameters. */
  py::array_t<double>* time_offsets_to_use = py_time_offsets.get();
  py::array_t<double>* eeg_to_use = py_eeg.get();
  py::array_t<double>* emg_to_use = py_emg.get();
  int reference_index = this->look_back_samples;
  size_t num_samples = this->look_back_samples + this->look_ahead_samples + 1;
  
  /* Override with pulse-specific arrays if this is a pulse and it has a custom window. */
  if (pulse_trigger && this->has_custom_pulse_window) {
    time_offsets_to_use = pulse_time_offsets.get();
    eeg_to_use = pulse_eeg.get();
    emg_to_use = pulse_emg.get();
    reference_index = this->pulse_look_back_samples;
    num_samples = this->pulse_look_back_samples + this->pulse_look_ahead_samples + 1;
  }
  
  /* Override with event-specific arrays if this is an event and it has a custom window. */
  if (has_event && this->has_custom_event_window) {
    time_offsets_to_use = event_time_offsets.get();
    eeg_to_use = event_eeg.get();
    emg_to_use = event_emg.get();
    reference_index = this->event_look_back_samples;
    num_samples = this->event_look_back_samples + this->event_look_ahead_samples + 1;
  }
  
  /* Always extract the most recent num_samples from the ring buffer.
     By the time we process (after deferring for look-ahead), the buffer has advanced
     and contains all the samples we need at the end of the buffer, including the look-ahead samples. */
  size_t start_offset = this->buffer_size - num_samples;

  /* Fill the selected arrays from buffer at the calculated offset. */
  fill_arrays_from_buffer(buffer, reference_time, *time_offsets_to_use,
                          *eeg_to_use, *emg_to_use, start_offset, num_samples);

  /* Call the appropriate Python function. */
  py::object py_result;
  try {
    if (pulse_trigger) {
      /* Call pulse processor if registered. */
      if (!pulse_processor.is_none()) {
        py_result = pulse_processor(reference_time, reference_index, *time_offsets_to_use, *eeg_to_use, *emg_to_use, is_coil_at_target);
      } else {
        py_result = py::none();
      }
    } else if (has_event) {
      /* Call event processor if registered. */
      if (!event_processor.is_none()) {
        py_result = event_processor(reference_time, reference_index, *time_offsets_to_use, *eeg_to_use, *emg_to_use, is_coil_at_target);
      } else {
        py_result = py::none();
      }
    } else {
      /* Call standard process_periodic method (for periodic processing). */
      py_result = decider_instance->attr("process_periodic")(reference_time, reference_index, *time_offsets_to_use, *eeg_to_use, *emg_to_use, is_coil_at_target, false);
    }

  } catch(const py::error_already_set& e) {
    std::string error_msg = std::string("Python error: ") + e.what();
    RCLCPP_ERROR(*logger_ptr, "%s", error_msg.c_str());
    
    // Add error to log buffer so it can be published to UI
    {
      std::lock_guard<std::mutex> lock(log_buffer_mutex);
      log_buffer.push_back({error_msg, LogLevel::ERROR});
    }
    
    success = false;
    return {success, timed_trigger, coil_target};

  } catch(const std::exception& e) {
    std::string error_msg = std::string("C++ error: ") + e.what();
    RCLCPP_ERROR(*logger_ptr, "%s", error_msg.c_str());
    
    // Add error to log buffer so it can be published to UI
    {
      std::lock_guard<std::mutex> lock(log_buffer_mutex);
      log_buffer.push_back({error_msg, LogLevel::ERROR});
    }
    
    success = false;
    return {success, timed_trigger, coil_target};
  }

  /* If the return value is None, return early but mark it as successful. */
  if (py_result.is_none()) {
    return {success, timed_trigger, coil_target};
  }

  /* If the return value is not None, ensure that it is a dictionary. */
  if (!py::isinstance<py::dict>(py_result)) {
    RCLCPP_ERROR(*logger_ptr, "Python module should return a dictionary.");
    success = false;
    return {success, timed_trigger, coil_target};
  }

  /* Extract the dictionary from the result first */
  py::dict dict_result = py_result.cast<py::dict>();

  /* Validate that only allowed keys are present */
  std::vector<std::string> allowed_keys = {"timed_trigger", "sensory_stimuli", "events", "coil_target"};
  for (const auto& item : dict_result) {
    std::string key = py::str(item.first).cast<std::string>();
    if (std::find(allowed_keys.begin(), allowed_keys.end(), key) == allowed_keys.end()) {
      RCLCPP_ERROR(*logger_ptr, "Unexpected key '%s' in return value, only 'timed_trigger', 'sensory_stimuli', 'events', and 'coil_target' are allowed.", key.c_str());
      success = false;

      return {success, timed_trigger, coil_target};
    }
  }

  if (dict_result.contains("coil_target")) {
    coil_target = dict_result["coil_target"].cast<std::string>();
  }

  if (dict_result.contains("timed_trigger")) {
    timed_trigger = std::make_shared<pipeline_interfaces::msg::TimedTrigger>();
    timed_trigger->time = dict_result["timed_trigger"].cast<double_t>();
  }

  if (dict_result.contains("sensory_stimuli")) {
    if (!py::isinstance<py::list>(dict_result["sensory_stimuli"])) {
      RCLCPP_ERROR(*logger_ptr, "sensory_stimuli must be a list.");
      success = false;
      return {success, timed_trigger, coil_target};
    }

    py::list py_sensory_stimuli = dict_result["sensory_stimuli"].cast<py::list>();
    if (!process_sensory_stimuli_list(py_sensory_stimuli, sensory_stimuli)) {
      success = false;
      return {success, timed_trigger, coil_target};
    }
  }

  if (dict_result.contains("events")) {
    if (!py::isinstance<py::list>(dict_result["events"])) {
      RCLCPP_ERROR(*logger_ptr, "events must be a list.");
      success = false;
      return {success, timed_trigger, coil_target};
    }

    py::list events = dict_result["events"].cast<py::list>();
    for (const auto& event : events) {
      double event_time = event.cast<double>();
      event_queue.push(event_time);
    }
  }

  return {success, timed_trigger, coil_target};
}

rclcpp::Logger* DeciderWrapper::logger_ptr = nullptr;
std::vector<LogEntry> DeciderWrapper::log_buffer;
std::mutex DeciderWrapper::log_buffer_mutex;
