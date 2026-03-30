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

void TurboLinkAdapter::handle_packet(const uint8_t* buffer, AdapterPacket& out_packet) {
  auto& sample = out_packet.sample;

  /* Pre-size vectors once so subsequent fills never allocate. */
  if (this->needs_vector_resize) {
    sample.eeg.resize(this->num_eeg_channels);
    sample.emg.resize(this->num_emg_channels);
    this->needs_vector_resize = false;
  }

  uint32_t token = *reinterpret_cast<const uint32_t *>(buffer + SamplePacketFieldIndex::TOKEN);

  if (token != 0x0050) {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Sample packet started with unknown token %x",
                 token);
  }

  uint32_t sample_index = *reinterpret_cast<const uint32_t *>(buffer + SamplePacketFieldIndex::SAMPLE_COUNTER);

  uint32_t trigger_bits = *reinterpret_cast<const uint32_t *>(buffer + SamplePacketFieldIndex::TRIGGER_BITS);

  for (uint8_t i = 0; i < this->num_emg_channels; i++) {
    const uint8_t *data = buffer + SamplePacketFieldIndex::AUX_CHANNELS + 4 * i;
    sample.emg[i] = *reinterpret_cast<const float *>(data);
  }
  for (uint8_t i = 0; i < this->num_eeg_channels; i++) {
    const uint8_t *data = buffer + SamplePacketFieldIndex::EEG_CHANNELS + 4 * i;
    sample.eeg[i] = *reinterpret_cast<const float *>(data);
  }

  auto triggers = std::bitset<32>(trigger_bits);
  sample.trigger_b = triggers[TriggerBitPosition::PULSE_TRIGGER_BIT];
  sample.trigger_a = triggers[TriggerBitPosition::SYNC_TRIGGER_BIT];

  if (sample.trigger_b) {
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Trigger received with sample %d", sample_index);
  }

  /* Turbolink has no timestamp so interpret time from sampling frequency */
  sample.time = (double)sample_index / this->sampling_frequency;
  sample.sample_index = sample_index;
}

void TurboLinkAdapter::process_packet(const uint8_t* buffer, [[maybe_unused]] size_t buffer_size, AdapterPacket& out_packet) {
  handle_packet(buffer, out_packet);
  out_packet.result = SAMPLE;
}

float_t TurboLinkAdapter::convert_be_float_to_host(uint8_t *buffer) {
  uint32_t little_endian_value = ntohl(*reinterpret_cast<uint32_t *>(buffer));
  return *reinterpret_cast<float *>(&little_endian_value);
}
