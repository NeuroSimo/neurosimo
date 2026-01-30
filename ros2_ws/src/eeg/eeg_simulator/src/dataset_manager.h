#ifndef DATASET_MANAGER_H
#define DATASET_MANAGER_H

#include <string>
#include <tuple>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "project_interfaces/srv/get_dataset_info.hpp"
#include "project_interfaces/msg/dataset_info.hpp"
#include <nlohmann/json.hpp>

class DatasetManager {
public:
  explicit DatasetManager(rclcpp::Node* node);

  void set_active_project(const std::string& project_name);

  std::tuple<bool, project_interfaces::msg::DatasetInfo, std::vector<double_t>> get_dataset_info(
      const std::string& json_filename, const std::string& directory_path);

  std::tuple<bool, std::string> load_dataset(
      const std::string& project_name,
      const project_interfaces::msg::DatasetInfo& dataset_info,
      std::vector<std::vector<double_t>>& buffer);

private:
  std::tuple<bool, size_t> get_sample_count(const std::string& data_file_path);
  std::tuple<bool, std::vector<double_t>> read_pulse_times_from_csv(const std::string& pulse_file_path);
  void handle_get_dataset_info(
      const std::shared_ptr<project_interfaces::srv::GetDatasetInfo::Request> request,
      std::shared_ptr<project_interfaces::srv::GetDatasetInfo::Response> response);

  rclcpp::Node* node_;
  rclcpp::Service<project_interfaces::srv::GetDatasetInfo>::SharedPtr service_;

  std::string active_project_;
  std::string data_directory_;
};

#endif // DATASET_MANAGER_H