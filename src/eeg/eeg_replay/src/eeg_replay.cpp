#include <filesystem>
#include <fstream>
#include <functional>

#include <nlohmann/json.hpp>

#include "eeg_replay.h"

using namespace std::chrono_literals;
using json = nlohmann::json;

using DataSourceState = system_interfaces::msg::DataSourceState;
using GlobalConfig = system_interfaces::msg::GlobalConfig;
using StreamInfo = eeg_interfaces::msg::StreamInfo;
using EegSample = eeg_interfaces::msg::Sample;
using ExperimentState = pipeline_interfaces::msg::ExperimentState;

const std::string EEG_ENRICHED_TOPIC = "/eeg/enriched";
const std::string EEG_PREPROCESSED_TOPIC = "/eeg/preprocessed";
const std::string EXPERIMENT_STATE_TOPIC = "/pipeline/experiment_state";


EegReplayNode::EegReplayNode() : Node("eeg_replay") {
  callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  /* QoS: transient local, keep last 1 (latched). */
  auto qos_persist_latest = rclcpp::QoS(1)
    .durability(rclcpp::DurabilityPolicy::TransientLocal)
    .history(rclcpp::HistoryPolicy::KeepLast);

  /* Publisher for data source state. */
  rclcpp::PublisherOptions pub_opts;
  pub_opts.callback_group = callback_group_;
  state_publisher_ = create_publisher<DataSourceState>(
    "/eeg_replay/state", qos_persist_latest, pub_opts);

  /* Client for aborting session. */
  abort_session_client_ = create_client<system_interfaces::srv::AbortSession>(
    "/session/abort",
    rclcpp::ServicesQoS(),
    callback_group_);

  /* Subscriber for global config. */
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = callback_group_;
  global_config_sub_ = create_subscription<GlobalConfig>(
    "/global_configurator/config", qos_persist_latest,
    [this](GlobalConfig::SharedPtr msg) { global_config_callback(msg); },
    sub_opts);

  /* Service servers. */
  initialize_service_ = create_service<eeg_interfaces::srv::InitializeEegReplayStream>(
    "/eeg_replay/initialize",
    std::bind(&EegReplayNode::handle_initialize, this, std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS(),
    callback_group_);

  start_streaming_service_ = create_service<eeg_interfaces::srv::StartStreaming>(
    "/eeg_replay/streaming/start",
    std::bind(&EegReplayNode::handle_start_streaming, this, std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS(),
    callback_group_);

  stop_streaming_service_ = create_service<eeg_interfaces::srv::StopStreaming>(
    "/eeg_replay/streaming/stop",
    std::bind(&EegReplayNode::handle_stop_streaming, this, std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS(),
    callback_group_);

  publish_state(DataSourceState::READY);
  RCLCPP_INFO(this->get_logger(), "EEG Replay Node initialized");
}

EegReplayNode::~EegReplayNode() {
  cleanup();
}


/* -------------------------------------------------------------------------- */
/*  Callbacks                                                                 */
/* -------------------------------------------------------------------------- */

void EegReplayNode::global_config_callback(const GlobalConfig::SharedPtr msg) {
  global_config_ = msg;
  RCLCPP_INFO(this->get_logger(), "Received global config: active_project=%s",
              msg->active_project.c_str());
}

void EegReplayNode::publish_state(uint8_t state) {
  DataSourceState msg;
  msg.state = state;
  state_publisher_->publish(msg);
}


/* -------------------------------------------------------------------------- */
/*  Initialize                                                                */
/* -------------------------------------------------------------------------- */

void EegReplayNode::handle_initialize(
  const std::shared_ptr<eeg_interfaces::srv::InitializeEegReplayStream::Request> request,
  std::shared_ptr<eeg_interfaces::srv::InitializeEegReplayStream::Response> response)
{
  RCLCPP_INFO(this->get_logger(),
    "Initializing replay: project=%s, bag_id=%s, play_preprocessed=%d",
    request->project_name.c_str(), request->bag_id.c_str(),
    request->play_preprocessed);

  if (!global_config_) {
    RCLCPP_ERROR(this->get_logger(), "Global config not yet received");
    return;
  }

  if (request->bag_id.empty()) {
    RCLCPP_ERROR(this->get_logger(), "bag_id is empty");
    return;
  }

  /* Construct paths.
   * Recordings:  projects/{project_name}/recordings/{bag_id}/
   * Metadata:    projects/{project_name}/recordings/{bag_id}.json */
  const std::string recordings_base =
    "projects/" + request->project_name + "/recordings";
  const std::string bag_path = recordings_base + "/" + request->bag_id;
  const std::string metadata_path = recordings_base + "/" + request->bag_id + ".json";

  if (!std::filesystem::exists(bag_path)) {
    RCLCPP_ERROR(this->get_logger(), "Recording path does not exist: %s", bag_path.c_str());
    return;
  }
  if (!std::filesystem::exists(metadata_path)) {
    RCLCPP_ERROR(this->get_logger(), "Metadata file not found: %s", metadata_path.c_str());
    return;
  }

  bag_filepath_ = bag_path;
  play_preprocessed_ = request->play_preprocessed;

  /* Read stream info from metadata JSON. */
  try {
    std::ifstream f(metadata_path);
    json metadata = json::parse(f);

    auto si = metadata.value("stream_info", json::object());
    stream_info_.sampling_frequency = si.value("sampling_frequency", 0);
    stream_info_.num_eeg_channels   = si.value("num_eeg_channels", 0);
    stream_info_.num_emg_channels   = si.value("num_emg_channels", 0);

    auto provenance   = metadata.value("provenance", json::object());
    auto fingerprints = provenance.value("fingerprints", json::object());
    data_source_fingerprint_ = fingerprints.value("data_source", uint64_t{0});

    RCLCPP_INFO(this->get_logger(),
      "Loaded stream info: %dHz, %d EEG channels, %d EMG channels",
      stream_info_.sampling_frequency,
      stream_info_.num_eeg_channels,
      stream_info_.num_emg_channels);
    RCLCPP_INFO(this->get_logger(),
      "Data source fingerprint: 0x%016lx", data_source_fingerprint_);

  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Error reading metadata: %s", e.what());
    return;
  }

  is_initialized_ = true;
  publish_state(DataSourceState::READY);
  response->stream_info = stream_info_;

  const std::string & topic = play_preprocessed_ ? EEG_PREPROCESSED_TOPIC : EEG_ENRICHED_TOPIC;
  RCLCPP_INFO(this->get_logger(),
    "Replay initialized successfully: %s (topic: %s)",
    bag_filepath_.c_str(), topic.c_str());
}


/* -------------------------------------------------------------------------- */
/*  Start / Stop streaming                                                    */
/* -------------------------------------------------------------------------- */

void EegReplayNode::handle_start_streaming(
  const std::shared_ptr<eeg_interfaces::srv::StartStreaming::Request> /*request*/,
  std::shared_ptr<eeg_interfaces::srv::StartStreaming::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Received start streaming request");

  if (!is_initialized_) {
    RCLCPP_ERROR(this->get_logger(), "Not initialized, cannot start streaming");
    response->success = false;
    return;
  }

  std::lock_guard<std::mutex> lock(playback_mutex_);

  if (is_streaming_.load()) {
    RCLCPP_WARN(this->get_logger(), "Bag playback already running");
    response->success = false;
    return;
  }

  stop_requested_.store(false);
  is_streaming_.store(true);

  if (playback_thread_.joinable()) {
    playback_thread_.join();
  }
  playback_thread_ = std::thread([this]() { playback_loop(); });

  publish_state(DataSourceState::RUNNING);
  response->success = true;
  RCLCPP_INFO(this->get_logger(), "Bag playback started successfully");
}

void EegReplayNode::handle_stop_streaming(
  const std::shared_ptr<eeg_interfaces::srv::StopStreaming::Request> /*request*/,
  std::shared_ptr<eeg_interfaces::srv::StopStreaming::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Received stop streaming request");

  std::lock_guard<std::mutex> lock(playback_mutex_);

  if (!is_streaming_.load()) {
    RCLCPP_WARN(this->get_logger(), "Not streaming, cannot stop");
    response->success = false;
    return;
  }

  stop_requested_.store(true);
  is_streaming_.store(false);

  if (playback_thread_.joinable()) {
    playback_thread_.join();
  }

  publish_state(DataSourceState::READY);
  response->success = true;
  response->data_source_fingerprint = data_source_fingerprint_;

  RCLCPP_INFO(this->get_logger(),
    "Bag playback stopped successfully (fingerprint: 0x%016lx)",
    data_source_fingerprint_);
}


/* -------------------------------------------------------------------------- */
/*  Playback loop                                                             */
/* -------------------------------------------------------------------------- */

void EegReplayNode::playback_loop() {
  const std::string & eeg_topic =
    play_preprocessed_ ? EEG_PREPROCESSED_TOPIC : EEG_ENRICHED_TOPIC;

  /* Create publishers for the duration of playback. */
  auto qos_persist_latest = rclcpp::QoS(rclcpp::KeepLast(1))
    .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
    .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

  auto eeg_pub = create_publisher<EegSample>(eeg_topic, 10);
  auto exp_pub = create_publisher<ExperimentState>(EXPERIMENT_STATE_TOPIC, qos_persist_latest);

  rclcpp::Serialization<EegSample> eeg_serializer;
  rclcpp::Serialization<ExperimentState> exp_serializer;

  /* Open bag. */
  rosbag2_cpp::Reader reader;
  rosbag2_storage::StorageOptions storage_opts;
  storage_opts.uri = bag_filepath_;

  rosbag2_cpp::ConverterOptions converter_opts;
  converter_opts.input_serialization_format  = "cdr";
  converter_opts.output_serialization_format = "cdr";

  try {
    reader.open(storage_opts, converter_opts);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open bag: %s", e.what());
    is_streaming_.store(false);
    publish_state(DataSourceState::READY);
    return;
  }

  /* Filter to only the topics we care about. */
  rosbag2_storage::StorageFilter filter;
  filter.topics = {eeg_topic, EXPERIMENT_STATE_TOPIC};
  reader.set_filter(filter);

  /* Pace playback by preserving original inter-message timing.
   * first_bag_ts / first_wall_time anchor bag time to wall time. */
  bool timing_initialized = false;
  rcutils_time_point_value_t first_bag_ts = 0;
  auto first_wall_time = std::chrono::steady_clock::now();

  while (reader.has_next() && !stop_requested_.load()) {
    auto bag_msg = reader.read_next();

    /* Sleep to preserve original timing between messages. */
    const auto bag_ts = bag_msg->time_stamp;
    if (!timing_initialized) {
      first_bag_ts    = bag_ts;
      first_wall_time = std::chrono::steady_clock::now();
      timing_initialized = true;
    } else {
      const auto elapsed_bag  = std::chrono::nanoseconds(bag_ts - first_bag_ts);
      const auto target_wall  = first_wall_time + elapsed_bag;
      const auto now_wall     = std::chrono::steady_clock::now();
      if (target_wall > now_wall) {
        std::this_thread::sleep_until(target_wall);
      }
    }

    if (stop_requested_.load()) break;

    /* Deserialize, patch timestamp, republish. */
    const auto & topic = bag_msg->topic_name;

    if (topic == eeg_topic) {
      EegSample eeg_msg;
      rclcpp::SerializedMessage serialized(*bag_msg->serialized_data);
      eeg_serializer.deserialize_message(&serialized, &eeg_msg);

      /* Get current high-resolution timestamp for both fields. */
      auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::high_resolution_clock::now().time_since_epoch()).count();

      /* Rewrite the timestamp so downstream consumers see wall-clock time. */
      eeg_msg.system_time_data_source_published = now_ns;

      /* Use the same timestamp for when experiment coordinator (replay) published this sample. */
      eeg_msg.system_time_experiment_coordinator_published = now_ns;

      eeg_pub->publish(eeg_msg);

    } else if (topic == EXPERIMENT_STATE_TOPIC) {
      ExperimentState exp_msg;
      rclcpp::SerializedMessage serialized(*bag_msg->serialized_data);
      exp_serializer.deserialize_message(&serialized, &exp_msg);
      exp_pub->publish(exp_msg);
    }
  }

  /* Playback ended. */
  const bool was_stopped = stop_requested_.load();
  is_streaming_.store(false);

  if (!was_stopped) {
    RCLCPP_INFO(this->get_logger(), "Bag playback finished, aborting session");
    publish_state(DataSourceState::READY);
    abort_session();
  }
}


/* -------------------------------------------------------------------------- */
/*  Session abort                                                             */
/* -------------------------------------------------------------------------- */

void EegReplayNode::abort_session() {
  if (!abort_session_client_->wait_for_service(1s)) {
    RCLCPP_ERROR(this->get_logger(),
      "AbortSession service not available, cannot abort session");
    return;
  }

  auto request = std::make_shared<system_interfaces::srv::AbortSession::Request>();
  request->source = "eeg_replay";
  abort_session_client_->async_send_request(request);
  RCLCPP_INFO(this->get_logger(), "Requested session abort: bag playback finished");
}


/* -------------------------------------------------------------------------- */
/*  Cleanup                                                                   */
/* -------------------------------------------------------------------------- */

void EegReplayNode::cleanup() {
  stop_requested_.store(true);
  if (playback_thread_.joinable()) {
    playback_thread_.join();
  }
}


/* -------------------------------------------------------------------------- */
/*  Main                                                                      */
/* -------------------------------------------------------------------------- */

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<EegReplayNode>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}