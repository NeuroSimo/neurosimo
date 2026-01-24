#ifndef EEG_ADAPTER_H
#define EEG_ADAPTER_H

#include <cmath>
#include <cstdlib>
#include <memory>
#include <netinet/in.h>
#include <vector>

#include "eeg_interfaces/msg/eeg_device_info.hpp"
#include "socket.h"

/** UDP datagram length is limited by Ethernet MTU (IP layer fragmentation isn't
    supported). */
const int BUFFER_SIZE = 1472;

const uint32_t UNSET_SAMPLING_FREQUENCY = 0;
const uint8_t UNSET_CHANNEL_COUNT = 0;

/** Result type for adapter packet processing. */
enum AdapterPacketResult {
  SAMPLE,    // Sample data (may include trigger_a and/or trigger_b)
  INTERNAL,  // Internal/configuration packet
  ERROR,     // Error reading packet
  END        // Measurement ended
};

/** Internal data structure representing raw sample from EEG device adapter.
    This keeps the adapter layer independent of ROS message types.
    The distinction between trigger_a (latency measurement) and trigger_b
    (pulse trigger) is explicit in field names. */
struct AdapterSample {
  std::vector<double> eeg;       // EEG channels in μV
  std::vector<double> emg;       // EMG channels in μV
  double time;                   // Sample timestamp in seconds
  uint64_t sample_index;         // Sample index
  bool trigger_a;                // Latency measurement trigger (Port A)
  bool trigger_b;                // Pulse trigger (Port B)
};

/** Packet returned from adapter process_packet().
    Contains both the result type and the sample data (when applicable). */
struct AdapterPacket {
  AdapterPacketResult result;
  AdapterSample sample;           // Valid when result == ADAPTER_SAMPLE
  double trigger_a_timestamp;     // Valid when result contains trigger_a
};

class EegAdapter {
public:
  EegAdapter(std::shared_ptr<UdpSocket> socket) : socket_(socket) {}
  virtual ~EegAdapter() = default;

  /** Process a packet from buffer data that has already been read from socket.

      This method processes packet data that has been pre-read from the UDP socket.
      The packet can update the adapter configuration and state or return a sample.

      @param buffer The buffer containing the packet data
      @param buffer_size The size of valid data in the buffer
      @return AdapterPacket containing the result type, sample data (when applicable),
      and trigger_a timestamp for latency measurement triggers. */
  virtual AdapterPacket process_packet(const uint8_t* buffer, size_t buffer_size) = 0;

  /// Get EEG device configuration info
  eeg_interfaces::msg::EegDeviceInfo get_device_info() const {
    auto device_info_msg = eeg_interfaces::msg::EegDeviceInfo();
    device_info_msg.sampling_frequency = sampling_frequency;
    device_info_msg.num_eeg_channels = num_eeg_channels;
    device_info_msg.num_emg_channels = num_emg_channels;

    return device_info_msg;
  }

protected:
  std::shared_ptr<UdpSocket> socket_;

public:
  /// Sampling frequency in Hz.
  uint32_t sampling_frequency = UNSET_SAMPLING_FREQUENCY;

  /// Number of channel dedicated to EEG.
  uint8_t num_eeg_channels = UNSET_CHANNEL_COUNT;

  /** Refers to bipolar channels that can be used for ECG, EOG etc., in addition
      to EMG. Named kept for consistency with other parts of the system, should
      probably be renamed at some point. */
  uint8_t num_emg_channels = UNSET_CHANNEL_COUNT;
};

#endif // EEG_ADAPTER_H
