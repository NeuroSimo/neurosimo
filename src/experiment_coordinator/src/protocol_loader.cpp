#include "protocol_loader.h"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <filesystem>
#include <set>

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
        stage.trials = stage_node["trials"].as<uint32_t>();
        
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
        
      } else {
        result.error_message = "Element at index " + std::to_string(i) + " must be either 'stage' or 'rest'";
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
  // Collect all stage names
  std::set<std::string> stage_names;
  
  for (const auto& element : protocol.elements) {
    if (element.type == ProtocolElement::Type::STAGE) {
      const auto& stage = element.stage.value();
      
      // Check for duplicate stage names
      if (stage_names.count(stage.name) > 0) {
        error_message = "Duplicate stage name: " + stage.name;
        return false;
      }
      stage_names.insert(stage.name);
      
      // Validate trials > 0
      if (stage.trials == 0) {
        error_message = "Stage '" + stage.name + "' must have at least 1 trial";
        return false;
      }
    }
  }
  
  // Validate that wait_until anchors reference valid stages
  for (const auto& element : protocol.elements) {
    if (element.type == ProtocolElement::Type::REST) {
      const auto& rest = element.rest.value();
      
      if (rest.wait_until.has_value()) {
        const std::string& anchor = rest.wait_until.value().anchor;
        
        if (stage_names.count(anchor) == 0) {
          error_message = "Rest references non-existent stage anchor: " + anchor;
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
  pipeline_interfaces::msg::ProtocolInfo info_msg = to_protocol_info_msg(protocol, protocol_filename);
  
  info_result.success = true;
  info_result.protocol_info = info_msg;
  
  return info_result;
}

pipeline_interfaces::msg::ProtocolInfo ProtocolLoader::to_protocol_info_msg(
    const Protocol& protocol, 
    const std::string& yaml_filename) {
  
  pipeline_interfaces::msg::ProtocolInfo info_msg;
  
  info_msg.yaml_filename = yaml_filename;
  info_msg.name = protocol.name;
  info_msg.description = protocol.description;
  
  /* Convert each protocol element to ROS message. */
  for (const auto& element : protocol.elements) {
    pipeline_interfaces::msg::ProtocolElementInfo element_msg;
    
    if (element.type == ProtocolElement::Type::STAGE) {
      const auto& stage = element.stage.value();
      
      element_msg.type = pipeline_interfaces::msg::ProtocolElementInfo::STAGE;
      element_msg.stage.name = stage.name;
      element_msg.stage.trials = stage.trials;
      element_msg.stage.notes = stage.notes;
      
    } else if (element.type == ProtocolElement::Type::REST) {
      const auto& rest = element.rest.value();
      
      element_msg.type = pipeline_interfaces::msg::ProtocolElementInfo::REST;
      element_msg.rest.notes = rest.notes;
    }
    
    info_msg.elements.push_back(element_msg);
  }
  
  return info_msg;
}

} // namespace experiment_coordinator

