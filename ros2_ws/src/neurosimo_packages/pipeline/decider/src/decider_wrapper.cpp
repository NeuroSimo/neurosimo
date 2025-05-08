#include <pybind11/pybind11.h>
#include <pybind11/embed.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <rcpputils/filesystem_helper.hpp>

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
    std::priority_queue<std::pair<double, uint16_t>,
                       std::vector<std::pair<double, uint16_t>>,
                       std::greater<std::pair<double, uint16_t>>>& event_queue) {

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
    RCLCPP_ERROR(*logger_ptr, "Python error: %s", e.what());
    state = WrapperState::ERROR;
    return;

  } catch (const std::exception &e) {
    RCLCPP_ERROR(*logger_ptr, "C++ error: %s", e.what());
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
    RCLCPP_ERROR(*logger_ptr, "Python error: %s", e.what());
    state = WrapperState::ERROR;
    return;

  } catch (const std::exception &e) {
    RCLCPP_ERROR(*logger_ptr, "C++ error: %s", e.what());
    state = WrapperState::ERROR;
    return;
  }

  /* Extract the configuration from decider_instance. */
  if (py::hasattr(*decider_instance, "get_configuration")) {
    py::dict config = decider_instance->attr("get_configuration")().cast<py::dict>();

    /* Validate that only allowed keys are present */
    std::vector<std::string> allowed_keys = {"sample_window", "sensory_stimuli", "processing_interval_in_samples", "process_on_trigger", "events"};
    for (const auto& item : config) {
      std::string key = py::str(item.first).cast<std::string>();
      if (std::find(allowed_keys.begin(), allowed_keys.end(), key) == allowed_keys.end()) {
        RCLCPP_ERROR(*logger_ptr, "Unexpected key '%s' in configuration dictionary. Only 'sample_window', 'sensory_stimuli', 'processing_interval_in_samples', 'process_on_trigger', and 'events' are allowed.", key.c_str());
        state = WrapperState::ERROR;
        return;
      }
    }

    /* Extract sample_window. */
    if (config.contains("sample_window")) {
      py::list sample_window = config["sample_window"].cast<py::list>();
      if (sample_window.size() == 2) {
        this->earliest_sample = sample_window[0].cast<int>();
        this->latest_sample = sample_window[1].cast<int>();
        this->buffer_size = this->latest_sample - this->earliest_sample + 1;
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

    /* Extract sensory_stimuli. */
    if (config.contains("sensory_stimuli")) {
      if (!py::isinstance<py::list>(config["sensory_stimuli"])) {
        RCLCPP_ERROR(*logger_ptr, "sensory_stimuli must be a list.");
        state = WrapperState::ERROR;
        return;
      }

      py::list py_sensory_stimuli = config["sensory_stimuli"].cast<py::list>();
      if (!process_sensory_stimuli_list(py_sensory_stimuli, sensory_stimuli)) {
        state = WrapperState::ERROR;
        return;
      }
    } else {
      RCLCPP_ERROR(*logger_ptr, "'sensory_stimuli' key not found in configuration dictionary.");
      state = WrapperState::ERROR;
      return;
    }
    
    /* Extract processing_interval_in_samples. */
    if (config.contains("processing_interval_in_samples")) {
      this->processing_interval_in_samples = config["processing_interval_in_samples"].cast<uint16_t>();
    } else {
      RCLCPP_ERROR(*logger_ptr, "'processing_interval_in_samples' key not found in configuration dictionary.");
      state = WrapperState::ERROR;
      return;
    }

    /* Extract process_on_trigger. */
    if (config.contains("process_on_trigger")) {
      this->process_on_trigger = config["process_on_trigger"].cast<bool>();
    } else {
      RCLCPP_ERROR(*logger_ptr, "'process_on_trigger' key not found in configuration dictionary.");
      state = WrapperState::ERROR;
      return;
    }

    /* Extract events. */
    if (config.contains("events")) {
      py::list events = config["events"].cast<py::list>();
      for (const auto& event : events) {
        py::dict event_dict = event.cast<py::dict>();

        uint16_t event_type = event_dict["type"].cast<uint16_t>();
        double event_time = event_dict["time"].cast<double>();

        event_queue.push(std::make_pair(event_time, event_type));
      }
    } else {
      RCLCPP_ERROR(*logger_ptr, "'events' key not found in configuration dictionary.");
      state = WrapperState::ERROR;
      return;
    }
  } else {
    RCLCPP_ERROR(*logger_ptr, "get_configuration method not found in the Decider instance.");
    state = WrapperState::ERROR;
    return;
  }

  /* Initialize numpy arrays. */
  py_timestamps = std::make_unique<py::array_t<double>>(buffer_size);

  py_valid = std::make_unique<py::array_t<bool>>(buffer_size);

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
  if (this->processing_interval_in_samples == 0) {
    RCLCPP_INFO(*logger_ptr, "  - Processing interval: %sDisabled%s", bold_on.c_str(), bold_off.c_str());
  } else {
    RCLCPP_INFO(*logger_ptr, "  - Processing interval: %s%d%s (samples)", bold_on.c_str(), this->processing_interval_in_samples, bold_off.c_str());
  }
  RCLCPP_INFO(*logger_ptr, "  - Process on trigger: %s%s%s", bold_on.c_str(), this->process_on_trigger ? "Enabled" : "Disabled", bold_off.c_str());
  RCLCPP_INFO(*logger_ptr, " ");
}

void DeciderWrapper::reset_module_state() {
  decider_instance = nullptr;
  decider_module = nullptr;

  py_timestamps.reset();
  py_eeg_data.reset();
  py_emg_data.reset();

  state = WrapperState::UNINITIALIZED;
}

DeciderWrapper::~DeciderWrapper() {
  py_timestamps.reset();
  py_valid.reset();
  py_eeg_data.reset();
  py_emg_data.reset();
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

uint16_t DeciderWrapper::get_processing_interval_in_samples() const {
  return this->processing_interval_in_samples;
}

bool DeciderWrapper::is_processing_interval_enabled() const {
  return this->processing_interval_in_samples > 0;
}

bool DeciderWrapper::is_process_on_trigger_enabled() const {
  return this->process_on_trigger;
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

/* TODO: Use struct for the return value. */
std::tuple<bool, std::shared_ptr<mtms_trial_interfaces::msg::Trial>, std::shared_ptr<pipeline_interfaces::msg::TimedTrigger>> DeciderWrapper::process(
    std::vector<pipeline_interfaces::msg::SensoryStimulus>& sensory_stimuli,
    const RingBuffer<std::shared_ptr<eeg_msgs::msg::PreprocessedSample>>& buffer,
    double_t sample_time,
    bool ready_for_trial,
    bool is_trigger,
    bool has_event,
    uint16_t event_type) {

  bool success = true;
  std::shared_ptr<mtms_trial_interfaces::msg::Trial> trial = nullptr;
  std::shared_ptr<pipeline_interfaces::msg::TimedTrigger> timed_trigger = nullptr;

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
  auto valid_ptr = py_valid->mutable_data();
  auto eeg_data_ptr = py_eeg_data->mutable_data();
  auto emg_data_ptr = py_emg_data->mutable_data();

  buffer.process_elements([&](const auto& sample_ptr) {
    const auto& sample = *sample_ptr;

    *timestamps_ptr++ = sample.time - current_time;
    *valid_ptr++ = sample.valid;
    std::memcpy(eeg_data_ptr, sample.eeg_data.data(), eeg_data_size * sizeof(double));
    eeg_data_ptr += eeg_data_size;
    std::memcpy(emg_data_ptr, sample.emg_data.data(), emg_data_size * sizeof(double));
    emg_data_ptr += emg_data_size;
  });

  /* Call the Python function. */
  py::object py_result;
  try {
    py_result = decider_instance->attr("process")(current_time, *py_timestamps, *py_valid, *py_eeg_data, *py_emg_data, current_sample_index, ready_for_trial, is_trigger, has_event, event_type);

  } catch(const py::error_already_set& e) {
    RCLCPP_ERROR(*logger_ptr, "Python error: %s", e.what());
    state = WrapperState::ERROR;
    success = false;

    return {success, trial, timed_trigger};

  } catch(const std::exception& e) {
    RCLCPP_ERROR(*logger_ptr, "C++ error: %s", e.what());
    state = WrapperState::ERROR;
    success = false;

    return {success, trial, timed_trigger};
  }

  /* If the return value is None, return early but mark it as successful. */
  if (py_result.is_none()) {
    return {success, trial, timed_trigger};
  }

  /* If the return value is not None, ensure that it is a dictionary. */
  if (!py::isinstance<py::dict>(py_result)) {
    RCLCPP_ERROR(*logger_ptr, "Python module should return a dictionary.");
    state = WrapperState::ERROR;
    success = false;

    return {success, trial, timed_trigger};
  }

  py::dict dict_result = py_result.cast<py::dict>();

  /* Validate that only allowed keys are present */
  std::vector<std::string> allowed_keys = {"trial", "timed_trigger", "sensory_stimuli"};
  for (const auto& item : dict_result) {
    std::string key = py::str(item.first).cast<std::string>();
    if (std::find(allowed_keys.begin(), allowed_keys.end(), key) == allowed_keys.end()) {
      RCLCPP_ERROR(*logger_ptr, "Unexpected key '%s' in return value, only 'trial', 'timed_trigger', and 'sensory_stimuli' are allowed.", key.c_str());
      state = WrapperState::ERROR;
      success = false;

      return {success, trial, timed_trigger};
    }
  }

  if (dict_result.contains("trial")) {
    /* If there is a trial in the dictionary, extract it and return early. */
    py::dict py_trial = dict_result["trial"].cast<py::dict>();

    trial = std::make_shared<mtms_trial_interfaces::msg::Trial>();

    /* Extract the targets from the dictionary. */
    if (!py_trial.contains("targets")) {
      RCLCPP_ERROR(*logger_ptr, "Trial dictionary does not contain the field: targets.");
      state = WrapperState::ERROR;
      success = false;

      return {success, trial, timed_trigger};
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

      return {success, trial, timed_trigger};
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

      return {success, trial, timed_trigger};
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
      return {success, trial, timed_trigger};
    }

    py::list py_sensory_stimuli = dict_result["sensory_stimuli"].cast<py::list>();
    if (!process_sensory_stimuli_list(py_sensory_stimuli, sensory_stimuli)) {
      state = WrapperState::ERROR;
      success = false;
      return {success, trial, timed_trigger};
    }
  }
  return {success, trial, timed_trigger};
}

rclcpp::Logger* DeciderWrapper::logger_ptr = nullptr;
