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
 * @brief Represents a stage with trials in the protocol
 */
struct Stage {
  std::string name;
  uint32_t trials;                              // total trial count
  std::string notes;
  std::optional<uint32_t> max_failures;         // per-stage cap on failed trials; absent means no retries
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
 * @brief Represents a task element (e.g., train_classifier)
 */
struct Task {
  std::string name;
  std::string notes;
};

/**
 * @brief Union type for protocol elements
 */
struct ProtocolElement {
  enum class Type {
    STAGE,
    REST,
    TASK
  };
  
  Type type;
  std::optional<Stage> stage;
  std::optional<Rest> rest;
  std::optional<Task> task;
  
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

  static ProtocolElement create_task(const Task& t) {
    ProtocolElement elem;
    elem.type = Type::TASK;
    elem.task = t;
    return elem;
  }
};

/**
 * @brief Descriptor for an optional runtime parameter set in the UI before a session starts
 */
struct RuntimeParameter {
  std::string name;                 // Key used to look up the value
  std::string label;                // Human-readable label shown in the UI
  std::string type;                 // 'float', 'int', 'string', or 'bool'
  std::string unit;                 // Optional unit shown next to the input
  std::optional<double> min;        // Optional minimum (numeric types)
  std::optional<double> max;        // Optional maximum (numeric types)
};

/**
 * @brief Complete protocol definition
 */
struct Protocol {
  std::string name;
  std::string description;
  double minimum_trial_interval;
  std::vector<ProtocolElement> elements;
  std::vector<RuntimeParameter> runtime_parameters;
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

  // Task tracking
  bool in_task = false;
  std::string task_name;
  uint64_t task_id = 0;                    // Monotonically increasing task ID within session
  
  // Pause tracking
  bool paused = false;
  bool pause_requested = false;            // Pause asked for while a trial runs ("Pausing...")
  bool trial_in_progress = false;          // True between a trial's commit and its verdict
  bool pending_attempt_commit = false;     // A trial start deferred due to pause; fire on resume
  double total_pause_duration = 0.0;
  double pause_start_time = 0.0;  // Sample time when pause started
  
  // Timing
  double last_sample_time = 0.0;  // Most recent sample time seen
  
  // Counters
  uint32_t trial_in_session = 0;
  uint64_t attempt_in_session = 0;
  uint32_t attempt_in_trial = 0;
  uint32_t failures_in_stage = 0;          // number of failed trials in current stage (for retry logic)
  
  // Pending event flags (set when transition occurs, consumed on the next sample)
  bool is_new_stage_pending = false;
  bool is_attempt_start_pending = false;

  // Flags
  bool protocol_complete = false;
  bool is_session_ongoing = false;
};

} // namespace experiment_coordinator

#endif // PROTOCOL_H

