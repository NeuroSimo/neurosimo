#ifndef EEG_BRIDGE_SRC_ADAPTERS_TURBOLINK_ADAPTER_H
#define EEG_BRIDGE_SRC_ADAPTERS_TURBOLINK_ADAPTER_H

#include "eeg_adapter.h"
#include "socket.h"

enum SamplePacketFieldIndex {
  TOKEN = 0,
  SAMPLE_COUNTER = 4,
  TRIGGER_BITS = 8,
  AUX_CHANNELS = 12,
  EEG_CHANNELS = 44,
};

/// Define which trigger bits are used for what triggers.
enum TriggerBitPosition {
  SYNC_TRIGGER_BIT = 0,
  PULSE_TRIGGER_BIT = 1,
};

/// Turbolink has constant 8 AUX channels for EMG.
const uint8_t AUX_CHANNEL_COUNT = 8;

class TurboLinkAdapter : public EegAdapter {

public:
  TurboLinkAdapter(std::shared_ptr<UdpSocket> socket, uint32_t sampling_frequency, uint8_t eeg_channel_count);
  ~TurboLinkAdapter() noexcept override = default;

  AdapterPacket process_packet(const uint8_t* buffer, size_t buffer_size) override;

private:
  std::tuple<AdapterSample, bool> handle_packet(const uint8_t* buffer);

  static float_t convert_be_float_to_host(uint8_t *buffer);
};

#endif // EEG_BRIDGE_SRC_ADAPTERS_TURBOLINK_ADAPTER_H
