#include "dataset_manager.h"

#include <fstream>
#include <sstream>
#include <vector>

DatasetManager::DatasetManager(rclcpp::Node* node) : node_(node) {
  service_ = node_->create_service<project_interfaces::srv::GetDatasetInfo>(
      "/eeg_simulator/dataset/get_info",
      std::bind(&DatasetManager::handle_get_dataset_info, this, std::placeholders::_1, std::placeholders::_2));
}

void DatasetManager::set_active_project(const std::string& project_name) {
  active_project_ = project_name;

  // Update data directory path
  std::ostringstream oss;
  oss << "projects/" << active_project_ << "/eeg_simulator/";
  data_directory_ = oss.str();

  RCLCPP_INFO(node_->get_logger(), "DatasetManager: Active project set to: %s, data directory: %s",
              active_project_.c_str(), data_directory_.c_str());
}

void DatasetManager::handle_get_dataset_info(
    const std::shared_ptr<project_interfaces::srv::GetDatasetInfo::Request> request,
    std::shared_ptr<project_interfaces::srv::GetDatasetInfo::Response> response) {

  /* Check if data directory is set */
  if (data_directory_.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "Data directory not set. Make sure a project is active.");
    response->success = false;
    return;
  }

  /* Check if the JSON file exists */
  std::string json_file_path = data_directory_ + "/" + request->filename;
  if (!std::filesystem::exists(json_file_path)) {
    RCLCPP_ERROR(node_->get_logger(), "Dataset file not found: %s", json_file_path.c_str());
    response->success = false;
    return;
  }

  /* Parse the dataset info directly from the JSON file */
  auto [success, dataset_info] = parse_dataset_info(request->filename, data_directory_);

  if (!success) {
    response->success = false;
    return;
  }

  response->dataset_info = dataset_info;
  response->success = true;

  RCLCPP_INFO(node_->get_logger(), "Dataset info requested for: %s", request->filename.c_str());
}

std::tuple<bool, project_interfaces::msg::DatasetInfo> DatasetManager::parse_dataset_info(
    const std::string& json_filename, const std::string& directory_path) {

  std::string json_file_path = directory_path + "/" + json_filename;
  std::ifstream file(json_file_path);

  if (!file.is_open()) {
    RCLCPP_ERROR(node_->get_logger(), "Error opening JSON file: %s", json_file_path.c_str());
    return std::make_tuple(false, project_interfaces::msg::DatasetInfo());
  }

  nlohmann::json json_data;
  project_interfaces::msg::DatasetInfo dataset_msg;

  try {
    file >> json_data;

    dataset_msg.json_filename = json_filename;

    /* Validate "name" field */
    if (json_data.contains("name") && json_data["name"].is_string()) {
      dataset_msg.name = json_data["name"];
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Mandatory field 'name' is missing or invalid in %s", json_filename.c_str());
      return std::make_tuple(false, project_interfaces::msg::DatasetInfo());
    }

    /* Validate "data_file" field. */
    if (json_data.contains("data_file") && json_data["data_file"].is_string()) {
      dataset_msg.data_filename = json_data["data_file"];
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Mandatory field 'data_file' is missing or invalid in %s", json_filename.c_str());
      return std::make_tuple(false, project_interfaces::msg::DatasetInfo());
    }

    /* Validate "session" object with sampling_frequency, num_eeg_channels, num_emg_channels. */
    if (json_data.contains("session") && json_data["session"].is_object()) {
      auto& session = json_data["session"];

      if (session.contains("sampling_frequency") && session["sampling_frequency"].is_number_integer()) {
        dataset_msg.sampling_frequency = session["sampling_frequency"];
      } else {
        RCLCPP_ERROR(node_->get_logger(), "Mandatory field 'session.sampling_frequency' is missing or invalid in %s", json_filename.c_str());
        return std::make_tuple(false, project_interfaces::msg::DatasetInfo());
      }

      if (session.contains("num_eeg_channels") && session["num_eeg_channels"].is_number_integer()) {
        dataset_msg.num_eeg_channels = session["num_eeg_channels"];
      } else {
        RCLCPP_ERROR(node_->get_logger(), "Mandatory field 'session.num_eeg_channels' is missing or invalid in %s", json_filename.c_str());
        return std::make_tuple(false, project_interfaces::msg::DatasetInfo());
      }

      if (session.contains("num_emg_channels") && session["num_emg_channels"].is_number_integer()) {
        dataset_msg.num_emg_channels = session["num_emg_channels"];
      } else {
        RCLCPP_ERROR(node_->get_logger(), "Mandatory field 'session.num_emg_channels' is missing or invalid in %s", json_filename.c_str());
        return std::make_tuple(false, project_interfaces::msg::DatasetInfo());
      }
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Mandatory object 'session' is missing or invalid in %s", json_filename.c_str());
      return std::make_tuple(false, project_interfaces::msg::DatasetInfo());
    }

    /* Read optional "loop" field, defaulting to false. */
    if (json_data.contains("loop") && json_data["loop"].is_boolean()) {
      dataset_msg.loop = json_data["loop"];
    } else {
      dataset_msg.loop = false;
    }

    /* Read optional "pulse_times" array. */
    std::vector<double_t> parsed_pulse_times;
    if (json_data.contains("pulse_times") && json_data["pulse_times"].is_array()) {
      for (const auto& pulse_time : json_data["pulse_times"]) {
        if (pulse_time.is_number()) {
          parsed_pulse_times.push_back(pulse_time.get<double>());
        } else {
          RCLCPP_WARN(node_->get_logger(), "Non-numeric value in pulse_times array in %s, skipping", json_filename.c_str());
        }
      }

      /* Warn if loop is enabled with pulse_times - this combination is not supported. */
      if (dataset_msg.loop && !parsed_pulse_times.empty()) {
        RCLCPP_WARN(node_->get_logger(), "Warning: pulse_times with loop=true is not supported in %s, pulse_times will be ignored", json_filename.c_str());
        parsed_pulse_times.clear();
      } else if (!parsed_pulse_times.empty()) {
        RCLCPP_INFO(node_->get_logger(), "Loaded %zu pulse times from %s", parsed_pulse_times.size(), json_filename.c_str());
      }
    }
    dataset_msg.pulse_count = parsed_pulse_times.size();

    /* Get sample count from the data file and compute duration. */
    std::string data_file_path = directory_path + "/" + dataset_msg.data_filename;
    auto [success, sample_count] = get_sample_count(data_file_path);

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "Error reading the dataset data file for %s, skipping...", json_filename.c_str());
      return std::make_tuple(false, project_interfaces::msg::DatasetInfo());
    }

    dataset_msg.duration = static_cast<double>(sample_count) / dataset_msg.sampling_frequency;

    return std::make_tuple(true, dataset_msg);

  } catch (const nlohmann::json::parse_error& ex) {
    RCLCPP_ERROR(node_->get_logger(), "JSON parse error in %s: %s", json_filename.c_str(), ex.what());
    return std::make_tuple(false, project_interfaces::msg::DatasetInfo());
  }
}

std::tuple<bool, size_t> DatasetManager::get_sample_count(const std::string& data_file_path) {
  std::ifstream data_file(data_file_path);

  if (!data_file.is_open()) {
    RCLCPP_ERROR(node_->get_logger(), "Error opening file: %s", data_file_path.c_str());
    return std::make_tuple(false, 0);
  }

  size_t line_count = 0;
  std::string line;
  while (std::getline(data_file, line)) {
    line_count++;
  }

  return std::make_tuple(true, line_count);
}