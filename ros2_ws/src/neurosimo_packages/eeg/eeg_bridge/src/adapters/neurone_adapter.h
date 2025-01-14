#ifndef MTMS_ROS2_WS_SRC_BRIDGES_EEG_BRIDGE_SRC_ADAPTERS_NEURONE_ADAPTER_H
#define MTMS_ROS2_WS_SRC_BRIDGES_EEG_BRIDGE_SRC_ADAPTERS_NEURONE_ADAPTER_H

#include <cstdlib>
#include <netinet/in.h>
#include <unistd.h>

#include <queue>
#include <vector>
#include <functional>

#include "rclcpp/rclcpp.hpp"

#include "eeg_adapter.h"

/// Maximum supported combined (EEG and bipolar) channel count in Bittium NeurOne.
const int MAX_CHANNELS = 160;

/// When triggers sent as channels, the value to determine trigger channel.
const uint16_t SOURCE_CHANNEL_FOR_TRIGGER = 65535;

/// Scaling factor for EXG and Tesla amplifiers in DC mode
const uint8_t DC_MODE_SCALE = 100;

/// Bittium stand-alone device ip, no other ip's used at the moment
const std::string EEG_DEVICE_IP = "192.168.200.220";

/// Frame type indexes of the NeurOne EEG device packets.
enum FrameType {
  MEASUREMENT_START = 1,
  SAMPLES = 2,
  TRIGGER = 3,
  MEASUREMENT_END = 4,
  HARDWARE_STATE = 5,
  JOIN = 128,
};

/// Start indexes of the MeasurementStart packet fields relative to start of the packet.
enum StartPacketFieldIndex {
  FRAME_TYPE = 0,
  MAIN_UNIT_NUM = 1,
  RESERVED = 2,
  SAMPLING_RATE_HZ = 4,
  SAMPLE_FORMAT = 8,
  TRIGGER_DEFS = 12,
  NUM_CHANNELS = 16,
  SOURCE_CHANNEL = 18
};

/// Start indexes of the Samples packet fields relative to start of the packet.
enum SamplesPacketFieldIndex {
  SAMPLE_PACKET_SEQ_NO = 4,
  SAMPLE_NUM_CHANNELS = 8,
  SAMPLE_NUM_SAMPLE_BUNDLES = 10,
  SAMPLE_FIRST_SAMPLE_INDEX = 12,
  SAMPLE_FIRST_SAMPLE_TIME = 20,
  SAMPLE_SAMPLES = 28,
};

/// Start indexes of the Trigger packet fields relative to start of the packet.
enum TriggerPacketFieldIndex {
  TRIGGER_NUM_TRIGGER = 2,
  TRIGGERS = 8,
};

/// Start indexes of the Trigger event type fields relative to start of the object.
enum TriggerEvent {
  MICROTIME = 0,
  SAMPLE_INDEX = 8,
  TYPE = 16,
  CODE = 17,
};

/// Indicator bit positions for the presence of trigger type. Indexing starts from 0.
enum TriggerBits {
  A_IN = 1,
  B_IN = 3,
};

/// Possible channel types from the Bittium NeurOne.
enum ChannelType {
  UNDEFINED_CHANNEL,
  EEG_CHANNEL,
  BIPOLAR_CHANNEL,
  TRIGGER_CHANNEL,
};

struct Trigger {
  double timestamp;
};

struct TriggerComparator {
  bool operator()(const Trigger& lhs, const Trigger& rhs) const {
    return lhs.timestamp > rhs.timestamp;
  }
};

class NeurOneAdapter : public EegAdapter {

public:
  NeurOneAdapter(uint16_t port);

  ~NeurOneAdapter() noexcept override { close(socket_); }

  std::tuple<PacketResult, eeg_msgs::msg::Sample, double> read_eeg_data_packet() override;

private:
  bool init_socket();
  bool request_measurement_start_packet() const;
  void handle_measurement_start_packet();

  std::tuple<eeg_msgs::msg::Sample, bool> handle_sample_packet();

  /** Parse and process the trigger packet.

      Process all the trigger events in the packet. If trigger in isolated trigger
      port B is present, the bool value indicating to include trigger in the next sample
      will be set to true. Trigger in isolated trigger port A are used for
      synchronisation. Other triggers are ignored.

      @return tuple of sync_trigger and sync_trigger_time, where sync_trigger is true if
      sync trigger is present and false otherwise. sync_trigger_time is the time of the
      trigger given in the packet in seconds since session start.
   */
  std::tuple<bool, double> handle_trigger_packet();

  bool read_eeg_data_from_socket();

  /** Convert big-endian int24 represented as 3 uint8_t bytes to int32_t.

      Assumes the given value to be in big-endian order and convert it to
      little-endian order. To handle sign of the value, assumes that right shift is made
      arithmetically, which is the case on most systems, but not guaranteed.

      @param value pointer to the first byte of big-endian int24.
      @return the actual value as int32_t. */
  static int32_t int24asint32(const uint8_t *value);

  uint8_t buffer[BUFFER_SIZE] = {0};

  ChannelType channel_types[MAX_CHANNELS] = {ChannelType::UNDEFINED_CHANNEL};

  int socket_ = -1;
  uint16_t port = 0;
  sockaddr_in socket_own = {};

  bool measurement_start_packet_received = false;
  uint16_t packets_since_measurement_start_packet_requested = 0;

  std::priority_queue<Trigger, std::vector<Trigger>, TriggerComparator> trigger_queue;
};

#endif // MTMS_ROS2_WS_SRC_BRIDGES_EEG_BRIDGE_SRC_ADAPTERS_NEURONE_ADAPTER_H
