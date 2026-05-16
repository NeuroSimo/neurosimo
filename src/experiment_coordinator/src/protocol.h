#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <string>
#include <vector>
#include <optional>
#include <memory>
#include <map>
#include <limits>
#include <cstdint>

namespace experiment_coordinator {

/**
 * @brief Trial timing modes
 */
enum class TrialTiming : uint8_t {
  PERIODIC = 0,
  PREDETERMINED = 1,
};

/**
 * @brief Describes one group of trials within a stage
 */
struct TrialTypeEntry {
  TrialTiming timing = TrialTiming::PERIODIC;
  std::string type;        // e.g. "low_iti", empty for periodic
  uint32_t count = 0;
};

/**
 * @brief Represents a stage with trials in the protocol
 */
struct Stage {
  std::string name;
  uint32_t trials;                              // total trial count (sum of all trial_types counts)
  std::string notes;
  std::vector<TrialTypeEntry> trial_types;      // breakdown by timing/type
  std::string order = "sequential";             // "sequential" or "random"
  std::vector<size_t> trial_order;              // indices into trial_types, length == trials
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
  double minimum_trial_interval;
  std::vector<ProtocolElement> elements;
};

/**
 * @brief Runtime state for experiment execution
 */
struct ExperimentState {
  // Current position in protocol
  size_t current_element_index = 0;
  uint32_t trial_in_stage = 0;
  
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
  uint32_t trial_in_session = 0;
  uint64_t attempt_in_session = 0;
  
  // Pending event flags (set when transition occurs, consumed on the next sample)
  bool is_new_stage_pending = false;
  bool is_attempt_start_pending = false;
  bool is_attempt_committed_pending = false;

  // Flags
  bool protocol_complete = false;
  bool is_session_ongoing = false;
};

} // namespace experiment_coordinator

#endif // PROTOCOL_H

