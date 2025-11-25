#include <pybind11/pybind11.h>
#include <pybind11/embed.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <rcpputils/filesystem_helper.hpp>
#include <cmath>
#include <chrono>
#include <cstdlib>

#include "decider_wrapper.h"
#include <eeg_msgs/msg/sample.hpp>

namespace py = pybind11;

DeciderWrapper::DeciderWrapper(rclcpp::Logger& logger) {
  logger_ptr = &logger;
  state = WrapperState::UNINITIALIZED;

  /* Initialize the Python interpreter. */
  interpreter = std::make_unique<py::scoped_interpreter>();
  setup_custom_print();
}

void DeciderWrapper::setup_custom_print() {
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

void DeciderWrapper::update_internal_imports(const std::string& base_directory) {
  internal_imports.clear();

  /* TODO: Modify this to use rcpputils instead of these ad-hoc lambda functions. */
  auto has_prefix = [](const std::string& str, const std::string& prefix) -> bool {
    return str.size() >= prefix.size() && str.compare(0, prefix.size(), prefix) == 0;
  };

  auto has_suffix = [](const std::string& str, const std::string& suffix) -> bool {
    return str.size() >= suffix.size() && str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
  };

  auto normalize_path = [](const std::string& path) -> std::string {
    std::string normalized = path;
    std::replace(normalized.begin(), normalized.end(), '\\', '/');
    return normalized;
  };

  auto get_filename = [](const std::string& path) -> std::string {
    size_t pos = path.find_last_of("/\\");
    return (pos == std::string::npos) ? path : path.substr(pos + 1);
  };

  py::module sys = py::module::import("sys");
  py::dict modules = sys.attr("modules");

  std::string normalized_base_dir = normalize_path(base_directory);

  for (const auto& item : modules) {
    py::handle key = item.first;
    py::handle value = item.second;

    if (py::hasattr(value, "__file__")) {
      std::string file_path = py::str(value.attr("__file__"));
      std::string normalized_file_path = normalize_path(file_path);

      /* Basic check if the file is within the base directory. */
      if (has_prefix(normalized_file_path, normalized_base_dir) &&
          has_suffix(normalized_file_path, ".py")) {

        /* Store only the filename for tracking. */
        std::string filename = get_filename(file_path);
        internal_imports.push_back(filename);
      }
    }
  }

  for (const auto& file : internal_imports) {
    RCLCPP_DEBUG(*logger_ptr, "Tracking file: %s", file.c_str());
  }
}

void DeciderWrapper::initialize_module(
    const std::string& project_directory,
    const std::string& module_directory,
    const std::string& module_name,
    const size_t eeg_data_size,
    const size_t emg_data_size,
    const uint16_t sampling_frequency,
    std::vector<pipeline_interfaces::msg::SensoryStimulus>& sensory_stimuli,
    std::priority_queue<std::pair<double, std::string>,
                       std::vector<std::pair<double, std::string>>,
                       std::greater<std::pair<double, std::string>>>& event_queue,
    std::mutex& event_queue_mutex) {

  this->sampling_frequency = sampling_frequency;

  /* Reset the module state. */
  reset_module_state();

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
    
    state = WrapperState::ERROR;
    return;

  } catch (const std::exception &e) {
    std::string error_msg = std::string("C++ error during import: ") + e.what();
    RCLCPP_ERROR(*logger_ptr, "%s", error_msg.c_str());
    
    // Add error to log buffer so it can be published to UI
    {
      std::lock_guard<std::mutex> lock(log_buffer_mutex);
      log_buffer.push_back({error_msg, LogLevel::ERROR});
    }
    
    state = WrapperState::ERROR;
    return;
  }

  /* Update the list of internal imports to watch for changes. */
  update_internal_imports(module_directory);

  /* Initialize Decider instance. */
  try {
    auto instance = decider_module->attr("Decider")(eeg_data_size, emg_data_size, sampling_frequency);
    decider_instance = std::make_unique<py::object>(instance);

  } catch (const py::error_already_set &e) {
    std::string error_msg = std::string("Python initialization error: ") + e.what();
    RCLCPP_ERROR(*logger_ptr, "%s", error_msg.c_str());
    
    // Add error to log buffer so it can be published to UI
    {
      std::lock_guard<std::mutex> lock(log_buffer_mutex);
      log_buffer.push_back({error_msg, LogLevel::ERROR});
    }
    
    state = WrapperState::ERROR;
    return;

  } catch (const std::exception &e) {
    std::string error_msg = std::string("C++ error during initialization: ") + e.what();
    RCLCPP_ERROR(*logger_ptr, "%s", error_msg.c_str());
    
    // Add error to log buffer so it can be published to UI
    {
      std::lock_guard<std::mutex> lock(log_buffer_mutex);
      log_buffer.push_back({error_msg, LogLevel::ERROR});
    }
    
    state = WrapperState::ERROR;
    return;
  }

  /* Check that the Python module has a process_eeg_trigger method (mandatory). */
  if (!py::hasattr(*decider_instance, "process_eeg_trigger")) {
    RCLCPP_ERROR(*logger_ptr, "Decider module must implement 'process_eeg_trigger' method.");
    state = WrapperState::ERROR;
    return;
  }

  /* Extract the configuration from decider_instance. */
  if (py::hasattr(*decider_instance, "get_configuration")) {
    py::dict config = decider_instance->attr("get_configuration")().cast<py::dict>();

    /* Validate that only allowed keys are present */
    std::vector<std::string> allowed_keys = {"sample_window", "predefined_sensory_stimuli", "periodic_processing_enabled", "periodic_processing_interval", "first_periodic_processing_at", "predefined_events", "pulse_lockout_duration", "event_processors"};
    for (const auto& item : config) {
      std::string key = py::str(item.first).cast<std::string>();
      if (std::find(allowed_keys.begin(), allowed_keys.end(), key) == allowed_keys.end()) {
        RCLCPP_ERROR(*logger_ptr, "Unexpected key '%s' in configuration dictionary. Only 'sample_window', 'predefined_sensory_stimuli', 'periodic_processing_enabled', 'periodic_processing_interval', 'first_periodic_processing_at', 'predefined_events', 'pulse_lockout_duration', and 'event_processors' are allowed.", key.c_str());
        state = WrapperState::ERROR;
        return;
      }
    }

    /* Extract sample_window. */
    if (config.contains("sample_window")) {
      py::list sample_window = config["sample_window"].cast<py::list>();
      if (sample_window.size() == 2) {
        int earliest_sample_index = sample_window[0].cast<int>();
        int latest_sample_index = sample_window[1].cast<int>();
        
        /* Convert from indices to positive sample counts.
           For window [-10, 5]: look_back_samples = 10, look_ahead_samples = 5 */
        this->look_back_samples = -earliest_sample_index;
        this->look_ahead_samples = latest_sample_index;
        this->buffer_size = this->look_back_samples + this->look_ahead_samples + 1;
      } else {
        RCLCPP_ERROR(*logger_ptr, "'sample_window' value in configuration is of incorrect length (should be two elements).");
        state = WrapperState::ERROR;
        return;
      }

    } else {
      RCLCPP_ERROR(*logger_ptr, "'sample_window' key not found in configuration dictionary.");
      state = WrapperState::ERROR;
      return;
    }

    /* Extract predefined_sensory_stimuli (optional). */
    if (config.contains("predefined_sensory_stimuli")) {
      if (!py::isinstance<py::list>(config["predefined_sensory_stimuli"])) {
        RCLCPP_ERROR(*logger_ptr, "predefined_sensory_stimuli must be a list.");
        state = WrapperState::ERROR;
        return;
      }

      py::list py_sensory_stimuli = config["predefined_sensory_stimuli"].cast<py::list>();
      if (!process_sensory_stimuli_list(py_sensory_stimuli, sensory_stimuli)) {
        state = WrapperState::ERROR;
        return;
      }
    }

    /* Extract periodic_processing_enabled (mandatory). */
    if (config.contains("periodic_processing_enabled")) {
      try {
        this->periodic_processing_enabled = config["periodic_processing_enabled"].cast<bool>();
      } catch (const py::cast_error& e) {
        RCLCPP_ERROR(*logger_ptr, "periodic_processing_enabled must be a boolean: %s", e.what());
        state = WrapperState::ERROR;
        return;
      }
    } else {
      RCLCPP_ERROR(*logger_ptr, "'periodic_processing_enabled' key not found in configuration dictionary.");
      state = WrapperState::ERROR;
      return;
    }

    /* Check that if periodic processing is enabled, the periodic_processing_interval is provided. */
    if (this->periodic_processing_enabled && !config.contains("periodic_processing_interval")) {
      RCLCPP_ERROR(*logger_ptr, "periodic_processing_enabled is true but 'periodic_processing_interval' is not provided in configuration dictionary.");
      state = WrapperState::ERROR;
      return;
    }

    /* Extract periodic_processing_interval. */
    if (config.contains("periodic_processing_interval")) {
      try {
        this->periodic_processing_interval = config["periodic_processing_interval"].cast<double>();
        if (this->periodic_processing_interval <= 0.0) {
          RCLCPP_ERROR(*logger_ptr, "periodic_processing_interval must be a positive number (got %.3f).", this->periodic_processing_interval);
          state = WrapperState::ERROR;
          return;
        }
      } catch (const py::cast_error& e) {
        RCLCPP_ERROR(*logger_ptr, "periodic_processing_interval must be a number: %s", e.what());
        state = WrapperState::ERROR;
        return;
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
          state = WrapperState::ERROR;
          return;
        }
      } catch (const py::cast_error& e) {
        RCLCPP_ERROR(*logger_ptr, "first_periodic_processing_at must be a number: %s", e.what());
        state = WrapperState::ERROR;
        return;
      }
    } else {
      /* Default to same as periodic_processing_interval. */
      this->first_periodic_processing_at = this->periodic_processing_interval;
    }

    /* Extract predefined_events (optional). */
    if (config.contains("predefined_events")) {
      py::list events = config["predefined_events"].cast<py::list>();
      {
        std::lock_guard<std::mutex> lock(event_queue_mutex);
        for (const auto& event : events) {
          py::dict event_dict = event.cast<py::dict>();

          std::string event_type = event_dict["type"].cast<std::string>();
          double event_time = event_dict["time"].cast<double>();

          event_queue.push(std::make_pair(event_time, event_type));
        }
      }
    }

    /* Extract pulse_lockout_duration (optional, defaults to 0.0). */
    if (config.contains("pulse_lockout_duration")) {
      try {
        this->pulse_lockout_duration = config["pulse_lockout_duration"].cast<double>();
        if (this->pulse_lockout_duration < 0.0) {
          RCLCPP_ERROR(*logger_ptr, "pulse_lockout_duration must be non-negative.");
          state = WrapperState::ERROR;
          return;
        }
      } catch (const py::cast_error& e) {
        RCLCPP_ERROR(*logger_ptr, "pulse_lockout_duration must be a number: %s", e.what());
        state = WrapperState::ERROR;
        return;
      }
    } else {
      this->pulse_lockout_duration = 0.0;
    }

    /* Initialize max window to default window */
    this->max_look_back_samples = this->look_back_samples;
    this->max_look_ahead_samples = this->look_ahead_samples;

    /* Extract event_processors. */
    if (config.contains("event_processors")) {
      if (!py::isinstance<py::dict>(config["event_processors"])) {
        RCLCPP_ERROR(*logger_ptr, "event_processors must be a dictionary.");
        state = WrapperState::ERROR;
        return;
      }

      py::dict processors = config["event_processors"].cast<py::dict>();
      this->event_processors.clear();
      this->event_sample_windows.clear();
      
      for (const auto& item : processors) {
        std::string event_type = py::str(item.first).cast<std::string>();
        py::object value = item.second.cast<py::object>();
        
        py::object processor;
        
        /* Check if value is a dict (with 'processor' and optional 'sample_window') or a callable */
        if (py::isinstance<py::dict>(value)) {
          py::dict processor_config = value.cast<py::dict>();
          
          /* Extract processor */
          if (!processor_config.contains("processor")) {
            RCLCPP_ERROR(*logger_ptr, "Event processor config for '%s' must contain 'processor' key.", event_type.c_str());
            state = WrapperState::ERROR;
            return;
          }
          processor = processor_config["processor"].cast<py::object>();
          
          /* Extract optional sample_window */
          if (processor_config.contains("sample_window")) {
            py::list sample_window = processor_config["sample_window"].cast<py::list>();
            if (sample_window.size() != 2) {
              RCLCPP_ERROR(*logger_ptr, "sample_window for event '%s' must have 2 elements.", event_type.c_str());
              state = WrapperState::ERROR;
              return;
            }
            int earliest = sample_window[0].cast<int>();
            int latest = sample_window[1].cast<int>();
            
            /* Update maximum envelope to cover this window */
            int event_look_back = -earliest;
            int event_look_ahead = latest;
            this->max_look_back_samples = std::max(this->max_look_back_samples, event_look_back);
            this->max_look_ahead_samples = std::max(this->max_look_ahead_samples, event_look_ahead);
            
            this->event_sample_windows[event_type] = std::make_pair(earliest, latest);
            RCLCPP_DEBUG(*logger_ptr, "Registered custom sample window [%d, %d] for event: %s", earliest, latest, event_type.c_str());
          }
        } else {
          /* Simple format: value is the processor directly */
          processor = value;
        }
        
        /* Verify that the processor is callable */
        if (!py::hasattr(processor, "__call__")) {
          RCLCPP_ERROR(*logger_ptr, "Event processor for '%s' is not callable.", event_type.c_str());
          state = WrapperState::ERROR;
          return;
        }
        
        this->event_processors[event_type] = processor;
        RCLCPP_DEBUG(*logger_ptr, "Registered event processor for: %s", event_type.c_str());
      }
    } else {
      RCLCPP_ERROR(*logger_ptr, "'event_processors' key not found in configuration dictionary.");
      state = WrapperState::ERROR;
      return;
    }
  } else {
    RCLCPP_ERROR(*logger_ptr, "get_configuration method not found in the Decider instance.");
    state = WrapperState::ERROR;
    return;
  }

  /* Calculate the maximum buffer size needed to cover all windows.
     The ring buffer will be sized to this maximum envelope. */
  size_t max_buffer_size = this->max_look_back_samples + this->max_look_ahead_samples + 1;
  this->buffer_size = max_buffer_size;
  
  RCLCPP_DEBUG(*logger_ptr, "Maximum envelope: [%d, %d], buffer size: %zu", 
               -this->max_look_back_samples, this->max_look_ahead_samples, max_buffer_size);

  this->eeg_data_size = eeg_data_size;
  this->emg_data_size = emg_data_size;

  /* Initialize numpy arrays for default sample window. */
  py_time_offsets = std::make_unique<py::array_t<double>>(buffer_size);
  py_valid = std::make_unique<py::array_t<bool>>(buffer_size);

  std::vector<size_t> eeg_data_shape = {buffer_size, eeg_data_size};
  py_eeg_data = std::make_unique<py::array_t<double>>(eeg_data_shape);

  std::vector<size_t> emg_data_shape = {buffer_size, emg_data_size};
  py_emg_data = std::make_unique<py::array_t<double>>(emg_data_shape);

  /* Initialize numpy arrays for custom event windows. */
  for (const auto& [event_type, window] : event_sample_windows) {
    int earliest = window.first;
    int latest = window.second;
    int event_look_back = -earliest;
    int event_look_ahead = latest;
    size_t event_buffer_size = event_look_back + event_look_ahead + 1;
    int event_reference_index = event_look_back;

    EventArrays arrays;
    arrays.time_offsets = std::make_unique<py::array_t<double>>(event_buffer_size);
    arrays.valid = std::make_unique<py::array_t<bool>>(event_buffer_size);
    
    std::vector<size_t> event_eeg_shape = {event_buffer_size, eeg_data_size};
    arrays.eeg_data = std::make_unique<py::array_t<double>>(event_eeg_shape);
    
    std::vector<size_t> event_emg_shape = {event_buffer_size, emg_data_size};
    arrays.emg_data = std::make_unique<py::array_t<double>>(event_emg_shape);
    
    arrays.buffer_size = event_buffer_size;
    arrays.reference_index = event_reference_index;

    event_arrays[event_type] = std::move(arrays);
    RCLCPP_DEBUG(*logger_ptr, "Preallocated arrays for event '%s' with buffer size %zu", event_type.c_str(), event_buffer_size);
  }

  state = WrapperState::READY;

  /* Log the configuration. */
  RCLCPP_INFO(*logger_ptr, "Configuration:");
  RCLCPP_INFO(*logger_ptr, " ");
  RCLCPP_INFO(*logger_ptr, "  - Default sample window: %s[%d, %d]%s", bold_on.c_str(), -this->look_back_samples, this->look_ahead_samples, bold_off.c_str());
  
  /* Log custom event windows if any */
  if (!event_sample_windows.empty()) {
    RCLCPP_INFO(*logger_ptr, "  - Event-specific windows:");
    for (const auto& [event_type, window] : event_sample_windows) {
      RCLCPP_INFO(*logger_ptr, "      '%s': %s[%d, %d]%s", event_type.c_str(), bold_on.c_str(), window.first, window.second, bold_off.c_str());
    }
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
}

void DeciderWrapper::reset_module_state() {
  decider_instance = nullptr;
  decider_module = nullptr;

  py_time_offsets.reset();
  py_valid.reset();
  py_eeg_data.reset();
  py_emg_data.reset();
  
  event_arrays.clear();

  state = WrapperState::UNINITIALIZED;
}

void DeciderWrapper::warm_up() {
  if (state != WrapperState::READY || !decider_instance) {
    RCLCPP_WARN(*logger_ptr, "Cannot warm up: module not ready or instance not available");
    return;
  }

  try {
    // Check if the Python module has a warm_up_rounds attribute
    int warm_up_rounds = 0;
    if (py::hasattr(*decider_instance, "warm_up_rounds")) {
      warm_up_rounds = decider_instance->attr("warm_up_rounds").cast<int>();
    }

    log_section_header("Warm-up");

    if (warm_up_rounds <= 0) {
      RCLCPP_INFO(*logger_ptr, "Warm-up disabled (warm_up_rounds = %d)", warm_up_rounds);
      RCLCPP_INFO(*logger_ptr, " ");
      log_section_header("Operation");
      return;
    }

    RCLCPP_INFO(*logger_ptr, "Starting %d warm-up rounds...", warm_up_rounds);

    // Initialize RNG with constant seed for reproducible warm-up data
    std::srand(12345);

    // Get pointers to numpy arrays
    auto time_offsets_ptr = py_time_offsets->mutable_data();
    auto valid_ptr = py_valid->mutable_data();
    auto eeg_data_ptr = py_eeg_data->mutable_data();
    auto emg_data_ptr = py_emg_data->mutable_data();

    // Dummy parameters for process method (constant across all rounds)
    double_t dummy_reference_time = 0.0;
    int dummy_reference_index = look_back_samples;
    bool dummy_ready_for_trial = true;
    bool dummy_is_coil_at_target = true;

    // Perform warm-up rounds with fresh random data for each round
    for (int round = 0; round < warm_up_rounds; round++) {
      // Generate fresh random data for this round
      for (size_t i = 0; i < buffer_size; i++) {
        time_offsets_ptr[i] = -static_cast<double>(buffer_size - 1 - i) / sampling_frequency;
        valid_ptr[i] = true;
        
        // Fill EEG data with small random values
        for (size_t j = 0; j < eeg_data_size; j++) {
          eeg_data_ptr[i * eeg_data_size + j] = (static_cast<double>(std::rand()) / RAND_MAX - 0.5) * 1e-6;
        }
        
        // Fill EMG data with small random values
        for (size_t j = 0; j < emg_data_size; j++) {
          emg_data_ptr[i * emg_data_size + j] = (static_cast<double>(std::rand()) / RAND_MAX - 0.5) * 1e-6;
        }
      }

      try {
        auto start_time = std::chrono::high_resolution_clock::now();
        
        py::object py_result = decider_instance->attr("process_periodic")(
          dummy_reference_time, 
          dummy_reference_index,
          *py_time_offsets, 
          *py_eeg_data, 
          *py_emg_data, 
          *py_valid, 
          dummy_ready_for_trial, 
          dummy_is_coil_at_target
        );
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration<double, std::milli>(end_time - start_time).count();
        
        RCLCPP_INFO(*logger_ptr, "  Round %d/%d: %.2f ms", 
                     round + 1, warm_up_rounds, duration);
        
      } catch (const py::error_already_set& e) {
        RCLCPP_WARN(*logger_ptr, "  Round %d/%d: FAILED (Python error: %s)", round + 1, warm_up_rounds, e.what());
        // Continue with remaining rounds
      } catch (const std::exception& e) {
        RCLCPP_WARN(*logger_ptr, "  Round %d/%d: FAILED (C++ error: %s)", round + 1, warm_up_rounds, e.what());
        // Continue with remaining rounds
      }
    }

    RCLCPP_INFO(*logger_ptr, " ");
    RCLCPP_INFO(*logger_ptr, "Warm-up completed successfully (%d rounds)", warm_up_rounds);
    RCLCPP_INFO(*logger_ptr, " ");
    
    // Clear any logs accumulated during warm-up to prevent them from being published
    get_and_clear_logs();
    
    log_section_header("Operation");

  } catch (const py::error_already_set& e) {
    RCLCPP_ERROR(*logger_ptr, "Warm-up failed with Python error: %s", e.what());
    // Clear logs from failed warm-up as well
    get_and_clear_logs();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(*logger_ptr, "Warm-up failed with C++ error: %s", e.what());
    // Clear logs from failed warm-up as well
    get_and_clear_logs();
  }
}

DeciderWrapper::~DeciderWrapper() {
  py_time_offsets.reset();
  py_valid.reset();
  py_eeg_data.reset();
  py_emg_data.reset();
  event_arrays.clear();
}

std::vector<LogEntry> DeciderWrapper::get_and_clear_logs() {
  std::lock_guard<std::mutex> lock(log_buffer_mutex);
  std::vector<LogEntry> logs = std::move(log_buffer);
  log_buffer.clear();
  return logs;
}

std::vector<std::vector<targeting_msgs::msg::ElectricTarget>> DeciderWrapper::get_targets() {
  std::vector<std::vector<targeting_msgs::msg::ElectricTarget>> targets;

  if (state != WrapperState::READY) {
    return targets;
  }

  try {
    py::list py_targets = decider_instance->attr("targets").cast<py::list>();

    for (const auto& py_target : py_targets) {
      std::vector<targeting_msgs::msg::ElectricTarget> target;
      py::list py_target_list = py_target.cast<py::list>();

      for (const auto& py_target_item : py_target_list) {
        py::dict py_target_dict = py_target_item.cast<py::dict>();

        targeting_msgs::msg::ElectricTarget electric_target;
        electric_target.displacement_x = py_target_dict["displacement_x"].cast<uint8_t>();
        electric_target.displacement_y = py_target_dict["displacement_y"].cast<uint8_t>();
        electric_target.rotation_angle = py_target_dict["rotation_angle"].cast<uint16_t>();
        electric_target.intensity = py_target_dict["intensity"].cast<uint8_t>();

        std::string algorithm = py_target_dict["algorithm"].cast<std::string>();
        if (algorithm == "least_squares") {
          electric_target.algorithm.value = targeting_msgs::msg::TargetingAlgorithm::LEAST_SQUARES;
        } else if (algorithm == "genetic") {
          electric_target.algorithm.value = targeting_msgs::msg::TargetingAlgorithm::GENETIC;
        } else {
          RCLCPP_WARN(*logger_ptr, "Unknown targeting algorithm: %s, defaulting to 'least squares'.", algorithm.c_str());
          electric_target.algorithm.value = targeting_msgs::msg::TargetingAlgorithm::LEAST_SQUARES;
        }

        target.push_back(electric_target);
      }
      targets.push_back(target);
    }

  } catch(const py::error_already_set& e) {
    RCLCPP_ERROR(*logger_ptr, "Python error: %s", e.what());
    state = WrapperState::ERROR;
    return targets;

  } catch(const std::exception& e) {
    RCLCPP_ERROR(*logger_ptr, "C++ error: %s", e.what());
    state = WrapperState::ERROR;
    return targets;
  }
  return targets;
}

std::vector<std::string> DeciderWrapper::get_internal_imports() const {
  return this->internal_imports;
}

WrapperState DeciderWrapper::get_state() const {
  return this->state;
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

int DeciderWrapper::get_look_ahead_samples_for_event(const std::string& event_type) const {
  /* Check if this event has a custom sample window. */
  auto window_it = event_sample_windows.find(event_type);
  if (window_it != event_sample_windows.end()) {
    /* Event has custom window - return its look-ahead. */
    int latest = window_it->second.second;
    return latest;
  }
  /* Event uses default window. */
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
      state = WrapperState::ERROR;
      return false;
    }
  }

  // 2) Type‐check & cast with per-field error reporting
  try {
    out_msg.time = py_sensory_stimulus["time"].cast<double>();
  } catch (const py::cast_error& e) {
    RCLCPP_ERROR(*logger_ptr, "'time' must be a float or int: %s", e.what());
    state = WrapperState::ERROR;
    return false;
  }

  try {
    out_msg.type = py_sensory_stimulus["type"].cast<std::string>();
  } catch (const py::cast_error& e) {
    RCLCPP_ERROR(*logger_ptr, "'type' must be a string: %s", e.what());
    state = WrapperState::ERROR;
    return false;
  }

  py::dict params;
  try {
    params = py_sensory_stimulus["parameters"].cast<py::dict>();
  } catch (const py::cast_error& e) {
    RCLCPP_ERROR(*logger_ptr, "'parameters' must be a dictionary: %s", e.what());
    state = WrapperState::ERROR;
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
      state = WrapperState::ERROR;
      return false;
    }

    std::string val;
    try {
      val = py::str(item.second);  // Serializes any Python value
    } catch (const std::exception& e) {
      RCLCPP_ERROR(*logger_ptr, "parameter value could not be serialized to string: %s", e.what());
      state = WrapperState::ERROR;
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
        state = WrapperState::ERROR;
        return false;
      }
      py::dict py_sensory_stimulus_dict = py_sensory_stimulus.cast<py::dict>();
      pipeline_interfaces::msg::SensoryStimulus msg;
      if (parse_sensory_stimulus_dict(py_sensory_stimulus_dict, msg)) {
        sensory_stimuli.push_back(msg);
      } else {
        RCLCPP_ERROR(*logger_ptr, "Failed to parse sensory_stimuli dictionary.");
        state = WrapperState::ERROR;
        return false;
      }
    }
  }
  return true;
}

void DeciderWrapper::fill_arrays_from_buffer(
    const RingBuffer<std::shared_ptr<eeg_msgs::msg::PreprocessedSample>>& buffer,
    double_t reference_time,
    py::array_t<double>& time_offsets,
    py::array_t<bool>& valid,
    py::array_t<double>& eeg_data,
    py::array_t<double>& emg_data,
    size_t start_offset,
    size_t num_samples) {
  
  auto time_offsets_ptr = time_offsets.mutable_data();
  auto valid_ptr = valid.mutable_data();
  auto eeg_data_ptr = eeg_data.mutable_data();
  auto emg_data_ptr = emg_data.mutable_data();

  size_t ring_idx = 0;
  size_t out_idx = 0;
  buffer.process_elements([&](const auto& sample_ptr) {
    /* Check if this sample is in the range we want */
    if (ring_idx >= start_offset && out_idx < num_samples) {
      const auto& sample = *sample_ptr;
      time_offsets_ptr[out_idx] = sample.time - reference_time;
      valid_ptr[out_idx] = sample.valid;
      std::memcpy(eeg_data_ptr + out_idx * eeg_data_size, sample.eeg_data.data(), eeg_data_size * sizeof(double));
      std::memcpy(emg_data_ptr + out_idx * emg_data_size, sample.emg_data.data(), emg_data_size * sizeof(double));
      out_idx++;
    }
    ring_idx++;
  });
}

/* TODO: Use struct for the return value. */
std::tuple<bool, std::shared_ptr<mtms_trial_interfaces::msg::Trial>, std::shared_ptr<pipeline_interfaces::msg::TimedTrigger>, std::string> DeciderWrapper::process(
    std::vector<pipeline_interfaces::msg::SensoryStimulus>& sensory_stimuli,
    const RingBuffer<std::shared_ptr<eeg_msgs::msg::PreprocessedSample>>& buffer,
    double_t reference_time,
    bool ready_for_trial,
    bool is_trigger,
    bool has_event,
    std::string event_type,
    std::priority_queue<std::pair<double, std::string>,
                       std::vector<std::pair<double, std::string>>,
                       std::greater<std::pair<double, std::string>>>& event_queue,
    std::mutex& event_queue_mutex,
    bool is_coil_at_target) {

  bool success = true;
  std::shared_ptr<mtms_trial_interfaces::msg::Trial> trial = nullptr;
  std::shared_ptr<pipeline_interfaces::msg::TimedTrigger> timed_trigger = nullptr;
  std::string coil_target;

  /* Determine which arrays to use. Default to standard arrays. */
  py::array_t<double>* time_offsets_to_use = py_time_offsets.get();
  py::array_t<bool>* valid_to_use = py_valid.get();
  py::array_t<double>* eeg_data_to_use = py_eeg_data.get();
  py::array_t<double>* emg_data_to_use = py_emg_data.get();
  int reference_index = this->look_back_samples;
  size_t num_samples = this->look_back_samples + this->look_ahead_samples + 1;
  
  /* Override with event-specific arrays if this event has a custom window. */
  if (has_event) {
    auto arrays_it = event_arrays.find(event_type);
    if (arrays_it != event_arrays.end()) {
      EventArrays& arrays = arrays_it->second;
      time_offsets_to_use = arrays.time_offsets.get();
      valid_to_use = arrays.valid.get();
      eeg_data_to_use = arrays.eeg_data.get();
      emg_data_to_use = arrays.emg_data.get();
      reference_index = arrays.reference_index;
      num_samples = arrays.buffer_size;
    }
  }
  
  /* Always extract the most recent num_samples from the ring buffer.
     By the time we process (after deferring for look-ahead), the buffer has advanced
     and contains all the samples we need at the end of the buffer, including the look-ahead samples. */
  size_t start_offset = this->buffer_size - num_samples;

  /* Fill the selected arrays from buffer at the calculated offset. */
  fill_arrays_from_buffer(buffer, reference_time, *time_offsets_to_use, *valid_to_use, 
                          *eeg_data_to_use, *emg_data_to_use, start_offset, num_samples);

  /* Call the appropriate Python function using the selected arrays. */
  py::object py_result;
  try {
    if (is_trigger) {
      /* Call process_eeg_trigger. */
      py_result = decider_instance->attr("process_eeg_trigger")(reference_time, reference_index, *time_offsets_to_use, *eeg_data_to_use, *emg_data_to_use, *valid_to_use, ready_for_trial, is_coil_at_target);
    } else if (has_event) {
      /* Call event processor for this event type. */
      auto processor_it = event_processors.find(event_type);
      if (processor_it == event_processors.end()) {
        std::string error_msg = std::string("No event processor registered for event type: ") + event_type;
        RCLCPP_ERROR(*logger_ptr, "%s", error_msg.c_str());
        
        // Add error to log buffer so it can be published to UI
        {
          std::lock_guard<std::mutex> lock(log_buffer_mutex);
          log_buffer.push_back({error_msg, LogLevel::ERROR});
        }
        
        state = WrapperState::ERROR;
        success = false;
        return {success, trial, timed_trigger, coil_target};
      }
      
      /* Call the event processor (arrays already selected and filled). */
      py_result = processor_it->second(reference_time, reference_index, *time_offsets_to_use, *eeg_data_to_use, *emg_data_to_use, *valid_to_use, ready_for_trial, is_coil_at_target);
    } else {
      /* Call standard process_periodic method (for periodic processing). */
      py_result = decider_instance->attr("process_periodic")(reference_time, reference_index, *time_offsets_to_use, *eeg_data_to_use, *emg_data_to_use, *valid_to_use, ready_for_trial, is_coil_at_target);
    }

  } catch(const py::error_already_set& e) {
    std::string error_msg = std::string("Python error: ") + e.what();
    RCLCPP_ERROR(*logger_ptr, "%s", error_msg.c_str());
    
    // Add error to log buffer so it can be published to UI
    {
      std::lock_guard<std::mutex> lock(log_buffer_mutex);
      log_buffer.push_back({error_msg, LogLevel::ERROR});
    }
    
    state = WrapperState::ERROR;
    success = false;

    return {success, trial, timed_trigger, coil_target};

  } catch(const std::exception& e) {
    std::string error_msg = std::string("C++ error: ") + e.what();
    RCLCPP_ERROR(*logger_ptr, "%s", error_msg.c_str());
    
    // Add error to log buffer so it can be published to UI
    {
      std::lock_guard<std::mutex> lock(log_buffer_mutex);
      log_buffer.push_back({error_msg, LogLevel::ERROR});
    }
    
    state = WrapperState::ERROR;
    success = false;

    return {success, trial, timed_trigger, coil_target};
  }

  /* If the return value is None, return early but mark it as successful. */
  if (py_result.is_none()) {
    return {success, trial, timed_trigger, coil_target};
  }

  /* If the return value is not None, ensure that it is a dictionary. */
  if (!py::isinstance<py::dict>(py_result)) {
    RCLCPP_ERROR(*logger_ptr, "Python module should return a dictionary.");
    state = WrapperState::ERROR;
    success = false;

    return {success, trial, timed_trigger, coil_target};
  }

  /* Extract the dictionary from the result first */
  py::dict dict_result = py_result.cast<py::dict>();

  /* Validate that only allowed keys are present */
  std::vector<std::string> allowed_keys = {"trial", "timed_trigger", "sensory_stimuli", "events", "coil_target"};
  for (const auto& item : dict_result) {
    std::string key = py::str(item.first).cast<std::string>();
    if (std::find(allowed_keys.begin(), allowed_keys.end(), key) == allowed_keys.end()) {
      RCLCPP_ERROR(*logger_ptr, "Unexpected key '%s' in return value, only 'trial', 'timed_trigger', 'sensory_stimuli', 'events', and 'coil_target' are allowed.", key.c_str());
      state = WrapperState::ERROR;
      success = false;

      return {success, trial, timed_trigger, coil_target};
    }
  }

  if (dict_result.contains("coil_target")) {
    coil_target = dict_result["coil_target"].cast<std::string>();
  }

  /* Extract the trial from the dictionary. */
  if (dict_result.contains("trial")) {
    /* If there is a trial in the dictionary, extract it and return early. */
    py::dict py_trial = dict_result["trial"].cast<py::dict>();

    trial = std::make_shared<mtms_trial_interfaces::msg::Trial>();

    /* Extract the targets from the dictionary. */
    if (!py_trial.contains("targets")) {
      RCLCPP_ERROR(*logger_ptr, "Trial dictionary does not contain the field: targets.");
      state = WrapperState::ERROR;
      success = false;

      return {success, trial, timed_trigger, coil_target};
    }

    py::list py_targets = py_trial["targets"].cast<py::list>();
    for (const auto& py_target : py_targets) {
      targeting_msgs::msg::ElectricTarget electric_target;
      py::dict py_target_dict = py_target.cast<py::dict>();

      electric_target.displacement_x = py_target_dict["displacement_x"].cast<uint8_t>();
      electric_target.displacement_y = py_target_dict["displacement_y"].cast<uint8_t>();
      electric_target.rotation_angle = py_target_dict["rotation_angle"].cast<uint16_t>();
      electric_target.intensity = py_target_dict["intensity"].cast<uint8_t>();

      std::string algorithm = py_target_dict["algorithm"].cast<std::string>();
      if (algorithm == "least_squares") {
        electric_target.algorithm.value = targeting_msgs::msg::TargetingAlgorithm::LEAST_SQUARES;
      } else if (algorithm == "genetic") {
        electric_target.algorithm.value = targeting_msgs::msg::TargetingAlgorithm::GENETIC;
      } else {
        RCLCPP_WARN(*logger_ptr, "Unknown targeting algorithm: %s, defaulting to 'least squares'.", algorithm.c_str());
        electric_target.algorithm.value = targeting_msgs::msg::TargetingAlgorithm::LEAST_SQUARES;
      }

      trial->targets.push_back(electric_target);
    }

    /* Extract the pulse times from the dictionary. */
    if (!py_trial.contains("pulse_times")) {
      RCLCPP_ERROR(*logger_ptr, "Trial dictionary does not contain the field: pulse_times.");
      state = WrapperState::ERROR;
      success = false;

      return {success, trial, timed_trigger, coil_target};
    }

    py::list py_pulse_times = py_trial["pulse_times"].cast<py::list>();
    auto first_pulse_time = py_pulse_times[0].cast<double_t>();
    for (const auto& py_pulse_time : py_pulse_times) {
      trial->pulse_times_since_trial_start.push_back(py_pulse_time.cast<double_t>() - first_pulse_time);
    }

    trial->timing.desired_start_time = first_pulse_time;
    trial->timing.allow_late = false;

    /* Extract the triggers from the dictionary. */
    if (!py_trial.contains("triggers")) {
      RCLCPP_ERROR(*logger_ptr, "Trial dictionary does not contain the field: triggers.");
      state = WrapperState::ERROR;
      success = false;

      return {success, trial, timed_trigger, coil_target};
    }

    py::list py_triggers = py_trial["triggers"].cast<py::list>();
    for (const auto& py_trigger : py_triggers) {
      mtms_trial_interfaces::msg::TriggerConfig trigger;
      py::dict py_trigger_dict = py_trigger.cast<py::dict>();

      trigger.enabled = py_trigger_dict["enabled"].cast<bool>();
      trigger.delay = py_trigger_dict["delay"].cast<double_t>();

      trial->triggers.push_back(trigger);
    }

    trial->config.voltage_tolerance_proportion_for_precharging = 0.03;
    trial->config.use_pulse_width_modulation_approximation = true;
    trial->config.recharge_after_trial = true;
    trial->config.dry_run = false;
  }

  if (dict_result.contains("timed_trigger")) {
    timed_trigger = std::make_shared<pipeline_interfaces::msg::TimedTrigger>();
    timed_trigger->time = dict_result["timed_trigger"].cast<double_t>();
  }

  if (dict_result.contains("sensory_stimuli")) {
    if (!py::isinstance<py::list>(dict_result["sensory_stimuli"])) {
      RCLCPP_ERROR(*logger_ptr, "sensory_stimuli must be a list.");
      state = WrapperState::ERROR;
      success = false;
      return {success, trial, timed_trigger, coil_target};
    }

    py::list py_sensory_stimuli = dict_result["sensory_stimuli"].cast<py::list>();
    if (!process_sensory_stimuli_list(py_sensory_stimuli, sensory_stimuli)) {
      state = WrapperState::ERROR;
      success = false;
      return {success, trial, timed_trigger, coil_target};
    }
  }

  if (dict_result.contains("events")) {
    if (!py::isinstance<py::list>(dict_result["events"])) {
      RCLCPP_ERROR(*logger_ptr, "events must be a list.");
      state = WrapperState::ERROR;
      success = false;
      return {success, trial, timed_trigger, coil_target};
    }

    py::list events = dict_result["events"].cast<py::list>();
    {
      std::lock_guard<std::mutex> lock(event_queue_mutex);
      for (const auto& event : events) {
        py::dict event_dict = event.cast<py::dict>();

        std::string event_type = event_dict["type"].cast<std::string>();
        double event_time = event_dict["time"].cast<double>();

        event_queue.push(std::make_pair(event_time, event_type));
      }
    }
  }

  return {success, trial, timed_trigger, coil_target};
}

rclcpp::Logger* DeciderWrapper::logger_ptr = nullptr;
std::vector<LogEntry> DeciderWrapper::log_buffer;
std::mutex DeciderWrapper::log_buffer_mutex;
