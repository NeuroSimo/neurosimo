#include <bitset>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "turbolink_adapter.h"

const std::string LOGGER_NAME = "turbolink_adapter";

TurboLinkAdapter::TurboLinkAdapter(std::shared_ptr<UdpSocket> socket, uint32_t sampling_frequency,
                                   uint8_t eeg_channel_count)
    : EegAdapter(socket) {
  this->num_emg_channels = AUX_CHANNEL_COUNT;
  this->num_eeg_channels = eeg_channel_count;
  this->sampling_frequency = sampling_frequency;
}

std::tuple<AdapterSample, bool> TurboLinkAdapter::handle_packet(const uint8_t* buffer) {
  auto adapter_sample = AdapterSample();
  bool trigger_a = false;

  uint32_t token = *reinterpret_cast<const uint32_t *>(buffer + SamplePacketFieldIndex::TOKEN);

  if (token != 0x0050) {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Sample packet started with unknown token %x",
                 token);
  }

  uint32_t sample_index = *reinterpret_cast<const uint32_t *>(buffer + SamplePacketFieldIndex::SAMPLE_COUNTER);

  uint32_t trigger_bits = *reinterpret_cast<const uint32_t *>(buffer + SamplePacketFieldIndex::TRIGGER_BITS);

  for (uint8_t i = 0; i < this->num_emg_channels; i++) {
    const uint8_t *data = buffer + SamplePacketFieldIndex::AUX_CHANNELS + 4 * i;
    float_t value = *reinterpret_cast<const float *>(data);
    adapter_sample.emg.push_back(value);
  }
  for (uint8_t i = 0; i < this->num_eeg_channels; i++) {
    const uint8_t *data = buffer + SamplePacketFieldIndex::EEG_CHANNELS + 4 * i;
    float_t value = *reinterpret_cast<const float *>(data);
    adapter_sample.eeg.push_back(value);
  }

  auto triggers = std::bitset<32>(trigger_bits);
  adapter_sample.trigger_b = triggers[TriggerBitPosition::PULSE_TRIGGER_BIT];
  trigger_a = triggers[TriggerBitPosition::SYNC_TRIGGER_BIT];

  if (adapter_sample.trigger_b) {
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Trigger received with sample %d", sample_index);
  }

  /* Turbolink has no timestamp so interpret time from sampling frequency */
  adapter_sample.time = (double)sample_index / this->sampling_frequency;
  adapter_sample.sample_index = sample_index;
  adapter_sample.trigger_a = false;  // Will be set in process_packet

  return {adapter_sample, trigger_a};
}

AdapterPacket TurboLinkAdapter::process_packet(const uint8_t* buffer, [[maybe_unused]] size_t buffer_size) {
  /* Return variables */
  AdapterPacket packet;
  packet.result = INTERNAL;
  packet.sample = AdapterSample();

  bool trigger_a = false;

  std::tie(packet.sample, trigger_a) = handle_packet(buffer);
  packet.result = SAMPLE;
  if (trigger_a) {
    packet.sample.trigger_a = true;
  }

  return packet;
}

float_t TurboLinkAdapter::convert_be_float_to_host(uint8_t *buffer) {
  uint32_t little_endian_value = ntohl(*reinterpret_cast<uint32_t *>(buffer));
  return *reinterpret_cast<float *>(&little_endian_value);
}
