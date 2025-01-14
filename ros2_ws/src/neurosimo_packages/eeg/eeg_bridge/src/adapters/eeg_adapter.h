#ifndef MTMS_EEG_ADAPTER_H
#define MTMS_EEG_ADAPTER_H

#include <cmath>
#include <cstdlib>
#include <netinet/in.h>

#include "eeg_msgs/msg/eeg_info.hpp"
#include "eeg_msgs/msg/sample.hpp"

/** UDP datagram length is limited by Ethernet MTU (IP layer fragmentation isn’t
    supported). */
const int BUFFER_SIZE = 1472;

const uint32_t UNSET_SAMPLING_FREQUENCY = 0;
const uint8_t UNSET_CHANNEL_COUNT = 0;

enum PacketResult { SAMPLE, SAMPLE_WITH_SYNC, SYNC_TRIGGER, INTERNAL, ERROR, END };

class EegAdapter {
public:
  virtual ~EegAdapter() = default;

  /** Read next packet from the EEG device.

      Process the next packet received from the device. The packet can update
      the adapter configuration and state or return a sample.

      @return type of the packet read, sample packet if packet was a sample and
      sync trigger timestamp in seconds, if sync trigger was in the sample or sync trigger in
      the packet. */
  virtual std::tuple<PacketResult, eeg_msgs::msg::Sample, double> read_eeg_data_packet() = 0;

  /// Get EEG device configuration info
  eeg_msgs::msg::EegInfo get_eeg_info() const {
    auto eeg_info_msg = eeg_msgs::msg::EegInfo();
    eeg_info_msg.sampling_frequency = sampling_frequency;
    eeg_info_msg.num_of_eeg_channels = num_of_eeg_channels;
    eeg_info_msg.num_of_emg_channels = num_of_emg_channels;

    return eeg_info_msg;
  }

protected:
  /// Sampling frequency in Hz.
  uint32_t sampling_frequency = UNSET_SAMPLING_FREQUENCY;

  /// Number of channel dedicated to EEG.
  uint8_t num_of_eeg_channels = UNSET_CHANNEL_COUNT;

  /** Refers to bipolar channels that can be used for ECG, EOG etc., in addition
      to EMG. Named kept for consistency with other parts of the system, should
      probably be renamed at some point. */
  uint8_t num_of_emg_channels = UNSET_CHANNEL_COUNT;
};

#endif // MTMS_EEG_ADAPTER_H
