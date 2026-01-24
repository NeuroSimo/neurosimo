#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <string>
#include <vector>
#include <optional>
#include <memory>
#include <map>

namespace experiment_coordinator {

/**
 * @brief Represents a stage with trials in the protocol
 */
struct Stage {
  std::string name;
  uint32_t trials;
  std::string notes;
};

/**
 * @brief Represents a rest period with duration or wait_until timing
 */
struct Rest {
  // Duration-based rest
  std::optional<double> duration;  // seconds
  
  // wait_until-based rest
  struct WaitUntil {
    std::string anchor;  // Reference to stage name
    double offset;       // Seconds after that stage started
  };
  std::optional<WaitUntil> wait_until;
  
  std::string notes;
};

/**
 * @brief Union type for protocol elements
 */
struct ProtocolElement {
  enum class Type {
    STAGE,
    REST
  };
  
  Type type;
  std::optional<Stage> stage;
  std::optional<Rest> rest;
  
  static ProtocolElement create_stage(const Stage& s) {
    ProtocolElement elem;
    elem.type = Type::STAGE;
    elem.stage = s;
    return elem;
  }
  
  static ProtocolElement create_rest(const Rest& r) {
    ProtocolElement elem;
    elem.type = Type::REST;
    elem.rest = r;
    return elem;
  }
};

/**
 * @brief Complete protocol definition
 */
struct Protocol {
  std::string name;
  std::string description;
  std::vector<ProtocolElement> elements;
};

/**
 * @brief Runtime state for experiment execution
 */
struct ExperimentState {
  // Current position in protocol
  size_t current_element_index = 0;
  uint32_t trial = 0;
  
  // Stage tracking
  std::string stage_name;
  std::map<std::string, double> stage_start_times;             // Map stage name -> sample start time
  std::map<std::string, double> stage_start_experiment_times;  // Map stage name -> experiment start time
  
  // Rest tracking
  bool in_rest = false;
  double rest_start_time = 0.0;            // Experiment time when rest started
  std::optional<double> rest_target_time;  // Experiment target time
  
  // Pause tracking
  bool paused = false;
  double total_pause_duration = 0.0;
  double pause_start_time = 0.0;  // Sample time when pause started
  
  // Timing
  double last_sample_time = 0.0;  // Most recent sample time seen
  
  // Counters
  uint32_t total_pulses = 0;
  
  // Flags
  bool protocol_complete = false;
  bool is_session_ongoing = false;
};

} // namespace experiment_coordinator

#endif // PROTOCOL_H

