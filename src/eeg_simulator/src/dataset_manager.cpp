#include "dataset_manager.h"
#include "csv_reader.h"

#include <fstream>
#include <sstream>
#include <vector>

DatasetManager::DatasetManager(rclcpp::Node* node) : node_(node) {
  service_ = node_->create_service<neurosimo_project_interfaces::srv::GetDatasetInfo>(
      "/neurosimo/eeg_simulator/dataset/get_info",
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
    const std::shared_ptr<neurosimo_project_interfaces::srv::GetDatasetInfo::Request> request,
    std::shared_ptr<neurosimo_project_interfaces::srv::GetDatasetInfo::Response> response) {

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
  auto [success, dataset_info, event_times] = get_dataset_info(request->filename, data_directory_);

  if (!success) {
    response->success = false;
    return;
  }

  response->dataset_info = dataset_info;
  response->success = true;

  RCLCPP_INFO(node_->get_logger(), "Dataset info requested for: %s", request->filename.c_str());
}

std::tuple<bool, neurosimo_project_interfaces::msg::DatasetInfo, std::vector<double_t>> DatasetManager::get_dataset_info(
    const std::string& json_filename, const std::string& directory_path) {

  std::string json_file_path = directory_path + "/" + json_filename;
  std::ifstream file(json_file_path);

  if (!file.is_open()) {
    RCLCPP_ERROR(node_->get_logger(), "Error opening JSON file: %s", json_file_path.c_str());
    return std::make_tuple(false, neurosimo_project_interfaces::msg::DatasetInfo(), std::vector<double_t>());
  }

  nlohmann::json json_data;
  neurosimo_project_interfaces::msg::DatasetInfo dataset_msg;

  try {
    file >> json_data;

    dataset_msg.json_filename = json_filename;

    /* Validate "name" field */
    if (json_data.contains("name") && json_data["name"].is_string()) {
      dataset_msg.name = json_data["name"];
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Mandatory field 'name' is missing or invalid in %s", json_filename.c_str());
      return std::make_tuple(false, neurosimo_project_interfaces::msg::DatasetInfo(), std::vector<double_t>());
    }

    /* Validate "data_file" field. */
    if (json_data.contains("data_file") && json_data["data_file"].is_string()) {
      dataset_msg.data_filename = json_data["data_file"];
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Mandatory field 'data_file' is missing or invalid in %s", json_filename.c_str());
      return std::make_tuple(false, neurosimo_project_interfaces::msg::DatasetInfo(), std::vector<double_t>());
    }

    /* Validate "session" object with sampling_frequency, num_eeg_channels, num_emg_channels. */
    if (json_data.contains("session") && json_data["session"].is_object()) {
      auto& session = json_data["session"];

      if (session.contains("sampling_frequency") && session["sampling_frequency"].is_number_integer()) {
        dataset_msg.sampling_frequency = session["sampling_frequency"];
      } else {
        RCLCPP_ERROR(node_->get_logger(), "Mandatory field 'session.sampling_frequency' is missing or invalid in %s", json_filename.c_str());
        return std::make_tuple(false, neurosimo_project_interfaces::msg::DatasetInfo(), std::vector<double_t>());
      }

      if (session.contains("num_eeg_channels") && session["num_eeg_channels"].is_number_integer()) {
        dataset_msg.num_eeg_channels = session["num_eeg_channels"];
      } else {
        RCLCPP_ERROR(node_->get_logger(), "Mandatory field 'session.num_eeg_channels' is missing or invalid in %s", json_filename.c_str());
        return std::make_tuple(false, neurosimo_project_interfaces::msg::DatasetInfo(), std::vector<double_t>());
      }

      if (session.contains("num_emg_channels") && session["num_emg_channels"].is_number_integer()) {
        dataset_msg.num_emg_channels = session["num_emg_channels"];
      } else {
        RCLCPP_ERROR(node_->get_logger(), "Mandatory field 'session.num_emg_channels' is missing or invalid in %s", json_filename.c_str());
        return std::make_tuple(false, neurosimo_project_interfaces::msg::DatasetInfo(), std::vector<double_t>());
      }
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Mandatory object 'session' is missing or invalid in %s", json_filename.c_str());
      return std::make_tuple(false, neurosimo_project_interfaces::msg::DatasetInfo(), std::vector<double_t>());
    }

    /* Read optional "loop" field, defaulting to false. */
    if (json_data.contains("loop") && json_data["loop"].is_boolean()) {
      dataset_msg.loop = json_data["loop"];
    } else {
      dataset_msg.loop = false;
    }

    /* Read optional "event_file" field and load event times from CSV. */
    std::vector<double_t> parsed_event_times;
    if (json_data.contains("event_file") && json_data["event_file"].is_string()) {
      std::string event_file = json_data["event_file"];
      std::string event_file_path = directory_path + "/" + event_file;

      auto [success, event_times] = read_event_times_from_csv(event_file_path);
      if (!success) {
        return std::make_tuple(false, neurosimo_project_interfaces::msg::DatasetInfo(), std::vector<double_t>());
      }
      parsed_event_times = event_times;

      /* Warn if loop is enabled with event_file - this combination is not supported. */
      if (dataset_msg.loop && !parsed_event_times.empty()) {
        RCLCPP_WARN(node_->get_logger(), "Warning: event_file with loop=true is not supported in %s, event times will be ignored", json_filename.c_str());
        parsed_event_times.clear();
      } else if (!parsed_event_times.empty()) {
        RCLCPP_INFO(node_->get_logger(), "Loaded %zu event times from %s", parsed_event_times.size(), event_file.c_str());
      }
    }
    dataset_msg.trial_count = parsed_event_times.size();

    /* Get sample count from the data file and compute duration. */
    std::string data_file_path = directory_path + "/" + dataset_msg.data_filename;
    auto [success, sample_count] = get_sample_count(data_file_path);

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "Error reading the dataset data file for %s, skipping...", json_filename.c_str());
      return std::make_tuple(false, neurosimo_project_interfaces::msg::DatasetInfo(), std::vector<double_t>());
    }

    dataset_msg.duration = static_cast<double>(sample_count) / dataset_msg.sampling_frequency;

    return std::make_tuple(true, dataset_msg, parsed_event_times);

  } catch (const nlohmann::json::parse_error& ex) {
    RCLCPP_ERROR(node_->get_logger(), "JSON parse error in %s: %s", json_filename.c_str(), ex.what());
    return std::make_tuple(false, neurosimo_project_interfaces::msg::DatasetInfo(), std::vector<double_t>());
  }
}

std::tuple<bool, size_t> DatasetManager::get_sample_count(const std::string& data_file_path) {
  auto [success, line_count] = csv_reader::count_lines(data_file_path);
  if (!success) {
    RCLCPP_ERROR(node_->get_logger(), "Error reading file: %s", data_file_path.c_str());
  }
  return std::make_tuple(success, line_count);
}

std::tuple<bool, std::vector<double_t>> DatasetManager::read_event_times_from_csv(const std::string& event_file_path) {
  std::vector<double_t> event_times;

  if (!std::filesystem::exists(event_file_path)) {
    RCLCPP_ERROR(node_->get_logger(), "Event file not found: %s", event_file_path.c_str());
    return std::make_tuple(false, event_times);
  }

  std::ifstream event_file_stream(event_file_path);
  if (!event_file_stream.is_open()) {
    RCLCPP_ERROR(node_->get_logger(), "Error opening event file: %s", event_file_path.c_str());
    return std::make_tuple(false, event_times);
  }

  std::string line;
  while (std::getline(event_file_stream, line)) {
    /* Skip empty lines */
    if (line.empty() || line.find_first_not_of(" \t\r\n") == std::string::npos) {
      continue;
    }

    try {
      double_t event_time = std::stod(line);
      event_times.push_back(event_time);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(node_->get_logger(), "Error parsing event time from line '%s' in %s: %s",
                   line.c_str(), event_file_path.c_str(), e.what());
      return std::make_tuple(false, std::vector<double_t>());
    }
  }

  event_file_stream.close();

  return std::make_tuple(true, event_times);
}

std::tuple<bool, std::string> DatasetManager::load_dataset(
    const std::string& project_name,
    const neurosimo_project_interfaces::msg::DatasetInfo& dataset_info,
    std::vector<std::vector<double_t>>& buffer) {

  std::string data_filename = dataset_info.data_filename;
  std::string data_file_path = "projects/" + project_name + "/eeg_simulator/" + data_filename;

  RCLCPP_INFO(node_->get_logger(), "Loading dataset: %s", data_filename.c_str());

  size_t num_columns = static_cast<size_t>(dataset_info.num_eeg_channels + dataset_info.num_emg_channels);
  auto [success, error_msg] = csv_reader::parse_numeric_csv(data_file_path, num_columns, buffer);

  if (!success) {
    RCLCPP_ERROR(node_->get_logger(), "%s", error_msg.c_str());
    return std::make_tuple(false, error_msg);
  }

  RCLCPP_INFO(node_->get_logger(), "Finished loading data. Loaded %zu samples.", buffer.size());
  return std::make_tuple(true, "");
}