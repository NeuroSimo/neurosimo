#include "protocol_loader.h"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <filesystem>
#include <set>
#include <algorithm>
#include <functional>

namespace experiment_coordinator {

ProtocolLoader::ProtocolLoader(rclcpp::Logger logger) : logger(logger) {}

LoadResult ProtocolLoader::load_from_file(const std::string& filepath) {
  LoadResult result;
  result.success = false;
  
  try {
    // Load YAML file
    YAML::Node yaml = YAML::LoadFile(filepath);
    
    Protocol protocol;
    
    // Parse name and description
    if (!yaml["name"]) {
      result.error_message = "Protocol missing required field 'name'";
      return result;
    }
    protocol.name = yaml["name"].as<std::string>();
    
    if (yaml["description"]) {
      protocol.description = yaml["description"].as<std::string>();
    }
    
    // Parse safety section
    if (!yaml["safety"]) {
      result.error_message = "Protocol missing required field 'safety'";
      return result;
    }
    const YAML::Node& safety = yaml["safety"];
    if (!safety["minimum_trial_interval"]) {
      result.error_message = "Protocol 'safety' section missing required field 'minimum_trial_interval'";
      return result;
    }
    protocol.minimum_trial_interval = safety["minimum_trial_interval"].as<double>();
    if (protocol.minimum_trial_interval <= 0) {
      result.error_message = "Protocol 'safety.minimum_trial_interval' must be positive (got " + std::to_string(protocol.minimum_trial_interval) + ")";
      return result;
    }
    
    // Parse runtime_parameters (optional)
    if (yaml["runtime_parameters"]) {
      const YAML::Node& runtime_parameters = yaml["runtime_parameters"];
      if (!runtime_parameters.IsMap()) {
        result.error_message = "Protocol 'runtime_parameters' must be a mapping";
        return result;
      }

      const std::set<std::string> allowed_types = {"float", "int", "string", "bool"};

      for (auto it = runtime_parameters.begin(); it != runtime_parameters.end(); ++it) {
        RuntimeParameter param;
        param.name = it->first.as<std::string>();
        const YAML::Node& definition = it->second;

        if (!definition.IsMap()) {
          result.error_message = "Runtime parameter '" + param.name + "' must be a mapping";
          return result;
        }

        if (!definition["type"]) {
          result.error_message = "Runtime parameter '" + param.name + "' missing required field 'type'";
          return result;
        }
        param.type = definition["type"].as<std::string>();
        if (allowed_types.count(param.type) == 0) {
          result.error_message = "Runtime parameter '" + param.name + "' has invalid type '" + param.type +
            "' (allowed: float, int, string, bool)";
          return result;
        }

        if (definition["label"]) {
          param.label = definition["label"].as<std::string>();
        }
        if (definition["unit"]) {
          param.unit = definition["unit"].as<std::string>();
        }
        if (definition["required"]) {
          param.required = definition["required"].as<bool>();
        }
        if (definition["min"]) {
          param.min = definition["min"].as<double>();
        }
        if (definition["max"]) {
          param.max = definition["max"].as<double>();
        }

        if (param.min.has_value() && param.max.has_value() && param.min.value() > param.max.value()) {
          result.error_message = "Runtime parameter '" + param.name + "' has 'min' greater than 'max'";
          return result;
        }

        protocol.runtime_parameters.push_back(param);
      }
    }

    // Parse stages
    if (!yaml["stages"]) {
      result.error_message = "Protocol missing required field 'stages'";
      return result;
    }
    
    const YAML::Node& stages = yaml["stages"];
    if (!stages.IsSequence()) {
      result.error_message = "Protocol 'stages' must be a sequence";
      return result;
    }
    
    for (size_t i = 0; i < stages.size(); ++i) {
      const YAML::Node& element = stages[i];
      
      if (element["stage"]) {
        // Parse stage element
        const YAML::Node& stage_node = element["stage"];
        
        Stage stage;
        
        if (!stage_node["name"]) {
          result.error_message = "Stage at index " + std::to_string(i) + " missing 'name'";
          return result;
        }
        stage.name = stage_node["name"].as<std::string>();

        if (!stage_node["trials"]) {
          result.error_message = "Stage '" + stage.name + "' missing 'trials'";
          return result;
        }

        const YAML::Node& trials_node = stage_node["trials"];

        if (!trials_node.IsScalar()) {
          result.error_message = "Stage '" + stage.name + "' 'trials' must be a number";
          return result;
        }
        stage.trials = trials_node.as<uint32_t>();

        /* Parse max_failures (optional, enables retry logic). */
        if (stage_node["max_failures"]) {
          uint32_t max_failures = stage_node["max_failures"].as<uint32_t>();
          if (max_failures == 0) {
            result.error_message = "Stage '" + stage.name + "' max_failures must be > 0";
            return result;
          }
          stage.max_failures = max_failures;
        }

        if (stage_node["notes"]) {
          stage.notes = stage_node["notes"].as<std::string>();
        }
        
        protocol.elements.push_back(ProtocolElement::create_stage(stage));
        
      } else if (element["rest"]) {
        // Parse rest element
        const YAML::Node& rest_node = element["rest"];
        
        Rest rest;
        
        if (rest_node["duration"]) {
          rest.duration = rest_node["duration"].as<double>();
        }
        
        if (rest_node["wait_until"]) {
          const YAML::Node& wait_until = rest_node["wait_until"];
          
          Rest::WaitUntil wu;
          
          if (!wait_until["anchor"]) {
            result.error_message = "Rest at index " + std::to_string(i) + " has wait_until but missing 'anchor'";
            return result;
          }
          wu.anchor = wait_until["anchor"].as<std::string>();
          
          if (!wait_until["offset"]) {
            result.error_message = "Rest at index " + std::to_string(i) + " has wait_until but missing 'offset'";
            return result;
          }
          wu.offset = wait_until["offset"].as<double>();
          
          rest.wait_until = wu;
        }
        
        // Validate that rest has either duration or wait_until
        if (!rest.duration.has_value() && !rest.wait_until.has_value()) {
          result.error_message = "Rest at index " + std::to_string(i) + " must have either 'duration' or 'wait_until'";
          return result;
        }
        
        if (rest.duration.has_value() && rest.wait_until.has_value()) {
          result.error_message = "Rest at index " + std::to_string(i) + " cannot have both 'duration' and 'wait_until'";
          return result;
        }
        
        if (rest_node["notes"]) {
          rest.notes = rest_node["notes"].as<std::string>();
        }
        
        protocol.elements.push_back(ProtocolElement::create_rest(rest));
        
      } else if (element["task"]) {
        // Parse task element
        const YAML::Node& task_node = element["task"];
        
        Task task;
        
        if (!task_node["name"]) {
          result.error_message = "Task at index " + std::to_string(i) + " missing 'name'";
          return result;
        }
        task.name = task_node["name"].as<std::string>();

        if (task_node["notes"]) {
          task.notes = task_node["notes"].as<std::string>();
        }
        
        protocol.elements.push_back(ProtocolElement::create_task(task));
        
      } else {
        result.error_message = "Element at index " + std::to_string(i) + " must be either 'stage', 'rest', or 'task'";
        return result;
      }
    }
    
    // Validate protocol
    std::string validation_error;
    if (!validate_protocol(protocol, validation_error)) {
      result.error_message = validation_error;
      return result;
    }
    
    result.success = true;
    result.protocol = protocol;
  } catch (const YAML::Exception& e) {
    result.error_message = std::string("YAML parsing error: ") + e.what();
    RCLCPP_ERROR(logger, "%s", result.error_message.c_str());
  } catch (const std::exception& e) {
    result.error_message = std::string("Error loading protocol: ") + e.what();
    RCLCPP_ERROR(logger, "%s", result.error_message.c_str());
  }
  
  return result;
}

bool ProtocolLoader::validate_protocol(const Protocol& protocol, std::string& error_message) {
  // Collect all stage and task names (both can serve as anchors)
  std::set<std::string> stage_names;
  std::set<std::string> task_names;
  std::set<std::string> anchor_names;  // union of stage + task names
  
  for (const auto& element : protocol.elements) {
    if (element.type == ProtocolElement::Type::STAGE) {
      const auto& stage = element.stage.value();
      
      // Check for duplicate stage names
      if (stage_names.count(stage.name) > 0) {
        error_message = "Duplicate stage name: " + stage.name;
        return false;
      }
      stage_names.insert(stage.name);
      anchor_names.insert(stage.name);
      
      // Validate trials > 0
      if (stage.trials == 0) {
        error_message = "Stage '" + stage.name + "' must have at least 1 trial";
        return false;
      }
    } else if (element.type == ProtocolElement::Type::TASK) {
      const auto& task = element.task.value();
      
      // Check for duplicate task names and conflicts with stage names
      if (task_names.count(task.name) > 0) {
        error_message = "Duplicate task name: " + task.name;
        return false;
      }
      if (stage_names.count(task.name) > 0) {
        error_message = "Task name conflicts with stage name: " + task.name;
        return false;
      }
      task_names.insert(task.name);
      anchor_names.insert(task.name);
    }
  }
  
  // Validate that wait_until anchors reference valid stages or tasks
  for (const auto& element : protocol.elements) {
    if (element.type == ProtocolElement::Type::REST) {
      const auto& rest = element.rest.value();
      
      if (rest.wait_until.has_value()) {
        const std::string& anchor = rest.wait_until.value().anchor;
        
        if (anchor_names.count(anchor) == 0) {
          error_message = "Rest references non-existent anchor: " + anchor;
          return false;
        }
      }
      
      // Validate duration/offset are positive
      if (rest.duration.has_value() && rest.duration.value() <= 0) {
        error_message = "Rest duration must be positive";
        return false;
      }
      
      if (rest.wait_until.has_value() && rest.wait_until.value().offset < 0) {
        error_message = "Rest wait_until offset must be non-negative";
        return false;
      }
    }
  }
  
  // Ensure protocol has at least one element
  if (protocol.elements.empty()) {
    error_message = "Protocol must have at least one stage or rest";
    return false;
  }
  
  return true;
}

LoadResult ProtocolLoader::load_from_project(
    const std::string& projects_directory,
    const std::string& project_name,
    const std::string& protocol_filename) {
  
  LoadResult result;
  result.success = false;
  
  /* Generate filepath from project name and protocol filename. */
  std::string filepath = projects_directory + "/" + project_name + "/protocols/" + protocol_filename;
  
  /* Check if the file exists. */
  if (!std::filesystem::exists(filepath)) {
    result.error_message = "Protocol file not found: " + filepath;
    RCLCPP_ERROR(logger, "%s", result.error_message.c_str());
    return result;
  }
  
  /* Load the protocol using existing method. */
  return load_from_file(filepath);
}

ProtocolInfoResult ProtocolLoader::get_protocol_info(
    const std::string& projects_directory,
    const std::string& project_name,
    const std::string& protocol_filename) {
  
  ProtocolInfoResult info_result;
  info_result.success = false;
  
  /* Load the protocol. */
  LoadResult load_result = load_from_project(projects_directory, project_name, protocol_filename);
  
  if (!load_result.success) {
    info_result.error_message = load_result.error_message;
    return info_result;
  }
  
  /* Convert to ROS message. */
  const auto& protocol = load_result.protocol.value();
  neurosimo_pipeline_interfaces::msg::ProtocolInfo info_msg = to_protocol_info_msg(protocol, protocol_filename);
  
  info_result.success = true;
  info_result.protocol_info = info_msg;
  
  return info_result;
}

neurosimo_pipeline_interfaces::msg::ProtocolInfo ProtocolLoader::to_protocol_info_msg(
    const Protocol& protocol, 
    const std::string& yaml_filename) {
  
  neurosimo_pipeline_interfaces::msg::ProtocolInfo info_msg;
  
  info_msg.yaml_filename = yaml_filename;
  info_msg.name = protocol.name;
  info_msg.description = protocol.description;
  
  /* Convert each protocol element to ROS message. */
  for (const auto& element : protocol.elements) {
    neurosimo_pipeline_interfaces::msg::ProtocolElementInfo element_msg;
    
    if (element.type == ProtocolElement::Type::STAGE) {
      const auto& stage = element.stage.value();
      
      element_msg.type = neurosimo_pipeline_interfaces::msg::ProtocolElementInfo::STAGE;
      element_msg.stage.name = stage.name;
      element_msg.stage.trials = stage.trials;
      element_msg.stage.notes = stage.notes;
      
    } else if (element.type == ProtocolElement::Type::REST) {
      const auto& rest = element.rest.value();
      
      element_msg.type = neurosimo_pipeline_interfaces::msg::ProtocolElementInfo::REST;
      element_msg.rest.notes = rest.notes;
      
    } else if (element.type == ProtocolElement::Type::TASK) {
      const auto& task = element.task.value();
      
      element_msg.type = neurosimo_pipeline_interfaces::msg::ProtocolElementInfo::TASK;
      element_msg.task.name = task.name;
      element_msg.task.notes = task.notes;
    }
    
    info_msg.elements.push_back(element_msg);
  }

  /* Convert runtime parameter descriptors to ROS message. */
  for (const auto& param : protocol.runtime_parameters) {
    neurosimo_pipeline_interfaces::msg::RuntimeParameterInfo param_msg;

    param_msg.name = param.name;
    param_msg.label = param.label;
    param_msg.type = param.type;
    param_msg.unit = param.unit;
    param_msg.required = param.required;

    param_msg.has_min = param.min.has_value();
    param_msg.min = param.min.value_or(0.0);

    param_msg.has_max = param.max.has_value();
    param_msg.max = param.max.value_or(0.0);

    info_msg.runtime_parameters.push_back(param_msg);
  }

  return info_msg;
}

} // namespace experiment_coordinator

