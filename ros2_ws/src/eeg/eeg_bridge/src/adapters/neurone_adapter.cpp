#include "neurone_adapter.h"

#include <arpa/inet.h>
#include <bitset>
#include <memory>

const std::string LOGGER_NAME = "neurone_adapter";


NeurOneAdapter::NeurOneAdapter(std::shared_ptr<UdpSocket> socket)
    : EegAdapter(socket) {
}  

bool NeurOneAdapter::request_measurement_start_packet() const {
  /* Hard coded port for join request to Bittium NeurOne, as specified in the
     NeurOne manual. */
  const int conn_port = 5050;

  sockaddr_in dest_addr = {};
  memset(&dest_addr, 0, sizeof(dest_addr));

  dest_addr.sin_family = AF_INET;
  dest_addr.sin_port = htons(conn_port);

  if (inet_pton(AF_INET, EEG_DEVICE_IP.c_str(), &dest_addr.sin_addr) <= 0) {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Invalid address %s", EEG_DEVICE_IP.c_str());
    return false;
  }

  uint8_t join_packet[4] = {FrameType::JOIN, 0, 0, 0};
  if (!this->socket_->send_data(join_packet, sizeof(join_packet), dest_addr)) {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Failed to request measurement start packet.");
    return false;
  }

  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Measurement start packet requested.");
  return true;
}

void NeurOneAdapter::handle_measurement_start_packet(const uint8_t* buffer) {
  this->measurement_start_packet_received = true;

  this->sampling_frequency =
      ntohl(*reinterpret_cast<const uint32_t *>(buffer + StartPacketFieldIndex::SAMPLING_RATE_HZ));

  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "  Sampling frequency: %u Hz",
              this->sampling_frequency);

  uint16_t channel_count =
      ntohs(*reinterpret_cast<const uint16_t *>(buffer + StartPacketFieldIndex::NUM_CHANNELS));

  this->num_eeg_channels = 0;
  this->num_emg_channels = 0;

  for (uint16_t i = 0; i < channel_count; i++) {
    uint16_t source_channel = ntohs(
        *reinterpret_cast<const uint16_t *>(buffer + StartPacketFieldIndex::SOURCE_CHANNEL + 2 * i));

    if (source_channel == SOURCE_CHANNEL_FOR_TRIGGER) {
      this->channel_types[i] = ChannelType::TRIGGER_CHANNEL;
      continue;
    }

    /* The limits below come from NeurOne's amplifier configuration: each
       amplifier supports 40 channels in total; the monopolar (EEG) channels are
       numbered 1-32 and the bipolar (EMG) channels are numbered 33-40. The
       channels of the second amplifier are numbered after the channels of the
       first amplifier, thus, starting at 41. */
    if ((source_channel >= 33 && source_channel <= 40) ||
        (source_channel >= 73 && source_channel <= 80) ||
        (source_channel >= 113 && source_channel <= 120) ||
        (source_channel >= 153 && source_channel <= 160)) {
      this->num_emg_channels++;
      this->channel_types[i] = ChannelType::BIPOLAR_CHANNEL;
    } else {
      this->num_eeg_channels++;
      this->channel_types[i] = ChannelType::EEG_CHANNEL;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "  Number of EEG channels: %u", this->num_eeg_channels);
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "  Number of EMG channels: %u", this->num_emg_channels);
}

AdapterSample NeurOneAdapter::handle_sample_packet(const uint8_t* buffer) {
  auto adapter_sample = AdapterSample();

  uint16_t num_sample_bundles = ntohs(
      *reinterpret_cast<const uint16_t *>(buffer + SamplesPacketFieldIndex::SAMPLE_NUM_SAMPLE_BUNDLES));

  if (num_sample_bundles != 1) {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),
                 "Invalid bundle size received: %u. The supported bundle size is 1. "
                 "Please ensure that the sampling frequency and the packet frequency "
                 "are set to the same value in the EEG software.",
                 num_sample_bundles);
    return adapter_sample;
  }

  /* Note: be64toh is Linux specific. */
  uint64_t sample_index = be64toh(
      *reinterpret_cast<const uint64_t *>(buffer + SamplesPacketFieldIndex::SAMPLE_FIRST_SAMPLE_INDEX));

  uint64_t sample_time_us = be64toh(
      *reinterpret_cast<const uint64_t *>(buffer + SamplesPacketFieldIndex::SAMPLE_FIRST_SAMPLE_TIME));
  double sample_time_s = (double)sample_time_us * 1e-6;

  uint16_t channel_count =
      ntohs(*reinterpret_cast<const uint16_t *>(buffer + SamplesPacketFieldIndex::SAMPLE_NUM_CHANNELS));

  for (uint16_t i = 0; i < channel_count; i++) {
    const uint8_t *buffer_offset = buffer + SamplesPacketFieldIndex::SAMPLE_SAMPLES + 3 * i;
    int32_t value = int24asint32(const_cast<uint8_t *>(buffer_offset));

    /* Scale the value by the scale factor (gain) and change from nV to uV*/
    double result_uV = value * DC_MODE_SCALE * 1e-3;

    ChannelType channel_type = channel_types[i];
    switch (channel_type) {

    case ChannelType::EEG_CHANNEL:
      adapter_sample.eeg.push_back(result_uV);
      break;

    case ChannelType::BIPOLAR_CHANNEL:
      adapter_sample.emg.push_back(result_uV);
      break;

    case ChannelType::TRIGGER_CHANNEL: {
      auto triggers = std::bitset<32>(value);
      adapter_sample.trigger_a = triggers[TriggerBits::A_IN];
      adapter_sample.trigger_b = triggers[TriggerBits::B_IN];
      break;
    }

    default:
      continue;
    }
  }

  adapter_sample.time = sample_time_s;
  adapter_sample.sample_index = sample_index;

  return adapter_sample;
}

AdapterPacket NeurOneAdapter::process_packet(const uint8_t* buffer, [[maybe_unused]] size_t buffer_size) {
  /* Return variables */
  AdapterPacket packet;
  packet.result = INTERNAL;
  packet.sample = AdapterSample();

  uint8_t frame_type = buffer[0];

  switch (frame_type) {

  case FrameType::MEASUREMENT_START:
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Measurement start packet received");
    handle_measurement_start_packet(buffer);
    packet.result = INTERNAL;
    break;

  case FrameType::SAMPLES:
    packet.sample = handle_sample_packet(buffer);
    packet.result = SAMPLE;
    break;

  case FrameType::MEASUREMENT_END:
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Measurement ended on the EEG device.");
    this->num_emg_channels = UNSET_CHANNEL_COUNT;
    this->num_eeg_channels = UNSET_CHANNEL_COUNT;
    this->sampling_frequency = UNSET_SAMPLING_FREQUENCY;

    packet.result = END;
    break;

  case FrameType::TRIGGER:
    RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME), "Standalone trigger packet received (ignored).");
    packet.result = INTERNAL;
    break;

  case FrameType::HARDWARE_STATE:
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),
                "Hardware state packet received. Currently no effect");
    break;
  default:
    RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME), "Unknown frame type received, doing nothing.");
  }

  /* The first packet in a new stream is the measurement start packet. If the first received
     packet is something else, the socket started reading in the middle of the measurement, thus
     request the measurement start packet.

     In addition, re-request it every 1000 packets. This is because NeurOne does not seem to always
     respond to the first request. */
  if (!measurement_start_packet_received && frame_type != FrameType::MEASUREMENT_START) {
    if (packets_since_measurement_start_packet_requested == 0) {
      request_measurement_start_packet();
    }
    packets_since_measurement_start_packet_requested = (packets_since_measurement_start_packet_requested + 1) % 1000;
  }

  return packet;
}

int32_t NeurOneAdapter::int24asint32(const uint8_t *value) {
  /* Starting from the most significant bit and shifting everything to the left
     by 8 bits handles the sign extension automatically for us, if arithmetic
     shift is used which is the case on most modern systems. Assumes big-endian
     byte order in the pointer. */
  int32_t result =
      ((static_cast<int32_t>(value[0]) << 24) | (static_cast<int32_t>(value[1]) << 16) |
       (static_cast<int32_t>(value[2]) << 8)) >>
      8;

  return result;
}
