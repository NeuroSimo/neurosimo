#include "protocol_loader.h"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <filesystem>
#include <set>
#include <algorithm>
#include <random>
#include <functional>
#include <numeric>

namespace experiment_coordinator {

ProtocolLoader::ProtocolLoader(rclcpp::Logger logger) : logger(logger) {}

/**
 * @brief Build the flat trial_order array from trial_types and optionally shuffle it.
 */
static void build_trial_order(Stage& stage, int32_t subject_id) {
  stage.trial_order.clear();
  stage.trial_order.reserve(stage.trials);

  for (size_t type_idx = 0; type_idx < stage.trial_types.size(); ++type_idx) {
    for (uint32_t j = 0; j < stage.trial_types[type_idx].count; ++j) {
      stage.trial_order.push_back(type_idx);
    }
  }

  if (stage.order == "random") {
    std::mt19937 rng(static_cast<uint32_t>(subject_id));
    std::shuffle(stage.trial_order.begin(), stage.trial_order.end(), rng);
  }
}

LoadResult ProtocolLoader::load_from_file(const std::string& filepath, int32_t subject_id) {
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

        if (trials_node.IsScalar()) {
          /* Shorthand: trials: N  → N periodic trials. */
          stage.trials = trials_node.as<uint32_t>();
          TrialTypeEntry entry;
          entry.timing = neurosimo_pipeline_interfaces::msg::AttemptCommit::TRIAL_TIMING_PERIODIC;
          entry.count = stage.trials;
          stage.trial_types.push_back(entry);

        } else if (trials_node.IsSequence()) {
          /* Extended: trials is a list of {count, timing?, type?} dicts. */
          uint32_t total = 0;
          for (size_t j = 0; j < trials_node.size(); ++j) {
            const YAML::Node& entry_node = trials_node[j];

            if (!entry_node["count"]) {
              result.error_message = "Stage '" + stage.name + "' trial entry " + std::to_string(j) + " missing 'count'";
              return result;
            }

            TrialTypeEntry entry;
            entry.count = entry_node["count"].as<uint32_t>();
            if (entry.count == 0) {
              result.error_message = "Stage '" + stage.name + "' trial entry " + std::to_string(j) + " count must be > 0";
              return result;
            }

            /* Parse timing (defaults to periodic). */
            if (entry_node["timing"]) {
              std::string timing_str = entry_node["timing"].as<std::string>();
              if (timing_str == "periodic") {
                entry.timing = neurosimo_pipeline_interfaces::msg::AttemptCommit::TRIAL_TIMING_PERIODIC;
              } else if (timing_str == "predetermined") {
                entry.timing = neurosimo_pipeline_interfaces::msg::AttemptCommit::TRIAL_TIMING_PREDETERMINED;
              } else {
                result.error_message = "Stage '" + stage.name + "' trial entry " + std::to_string(j) + " has invalid timing '" + timing_str + "' (must be 'periodic' or 'predetermined')";
                return result;
              }
            }

            /* Parse type (required for predetermined). */
            if (entry_node["type"]) {
              entry.type = entry_node["type"].as<std::string>();
            }
            if (entry.timing == neurosimo_pipeline_interfaces::msg::AttemptCommit::TRIAL_TIMING_PREDETERMINED && entry.type.empty()) {
              result.error_message = "Stage '" + stage.name + "' trial entry " + std::to_string(j) + ": 'type' is required when timing is 'predetermined'";
              return result;
            }

            total += entry.count;
            stage.trial_types.push_back(entry);
          }
          stage.trials = total;

        } else {
          result.error_message = "Stage '" + stage.name + "' 'trials' must be a number or a list";
          return result;
        }

        /* Parse order (defaults to sequential). */
        if (stage_node["order"]) {
          stage.order = stage_node["order"].as<std::string>();
          if (stage.order != "sequential" && stage.order != "random") {
            result.error_message = "Stage '" + stage.name + "' has invalid order '" + stage.order + "' (must be 'sequential' or 'random')";
            return result;
          }
        }

        /* Parse max_failures (optional, enables retry logic). */
        if (stage_node["max_failures"]) {
          uint32_t max_failures = stage_node["max_failures"].as<uint32_t>();
          if (max_failures == 0) {
            result.error_message = "Stage '" + stage.name + "' max_failures must be > 0";
            return result;
          }
          stage.max_failures = max_failures;
        }

        /* Build trial_order array. */
        build_trial_order(stage, subject_id);
        
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
    const std::string& protocol_filename,
    int32_t subject_id) {
  
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
  return load_from_file(filepath, subject_id);
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
  
  return info_msg;
}

} // namespace experiment_coordinator

