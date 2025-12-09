#include "neurone_adapter.h"

#include <arpa/inet.h>
#include <bitset>
#include <sys/socket.h>

const std::string LOGGER_NAME = "neurone_adapter";


NeurOneAdapter::NeurOneAdapter(uint16_t port) {
  this->port = port;
  bool success = init_socket();
  if (!success) {
    /*socket initialisation failed. Close socket manually before throwing
    exception, as destructor is not called if constructor fails */
    if (this->socket_ != -1) {
      close(this->socket_);
    }
    throw std::runtime_error("Failed to initialise socket");
  }
}

bool NeurOneAdapter::init_socket() {
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Binding to port: %d", this->port);

  this->socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (this->socket_ == -1) {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),
                 "Error: Failed to create socket file descriptor.");
    return false;
  }

  /* Initialize socket_own with zeros. */
  memset(&(this->socket_own), 0, sizeof(this->socket_own));

  socket_own.sin_family = AF_INET;
  socket_own.sin_port = htons(this->port);
  socket_own.sin_addr.s_addr = htonl(INADDR_ANY);

  /* Bind socket to address */
  if (bind(this->socket_, (struct sockaddr *)&(this->socket_own), sizeof(this->socket_own)) == -1) {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),
                 "Error: Failed to bind socket file descriptor to socket. Reason %s",
                 strerror(errno));
    return false;
  }

  /* Set socket timeout to 1 second */
  timeval read_timeout{};
  read_timeout.tv_sec = 1;
  read_timeout.tv_usec = 0;
  setsockopt(this->socket_, SOL_SOCKET, SO_RCVTIMEO, &read_timeout, sizeof(read_timeout));

  /* Attempt to set socket buffer size to 10 MB (both receive and send buffers
     separately). */
  int requested_buffer_size = 1024 * 1024 * 10;
  if (setsockopt(this->socket_, SOL_SOCKET, SO_RCVBUF, &requested_buffer_size,
                 sizeof(requested_buffer_size)) == -1) {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Failed to set socket buffer size.");
  }

  /* Check the actual buffer size. */
  int total_buffer_size;
  socklen_t optlen = sizeof(total_buffer_size);
  if (getsockopt(this->socket_, SOL_SOCKET, SO_RCVBUF, &total_buffer_size, &optlen) == -1) {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Failed to get socket buffer size.");
  }

  /* In Linux, the kernel doubles the buffer size for bookkeeping overhead. */
  int receive_buffer_size = total_buffer_size / 2;

  if (receive_buffer_size < requested_buffer_size) {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),
                 "Failed to set socket receive buffer size to %d bytes, actual "
                 "size is %d bytes.",
                 requested_buffer_size, receive_buffer_size);
    return false;
  }
  return true;
}

bool NeurOneAdapter::read_eeg_data_from_socket() {
  auto success = recvfrom(this->socket_, this->buffer, BUFFER_SIZE, 0, nullptr, nullptr);
  if (success == -1) {
    RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME), "No data received, reason: %s", strerror(errno));
    return false;
  }

  return true;
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
  }

  uint8_t join_packet[4] = {FrameType::JOIN, 0, 0, 0};
  auto success = sendto(this->socket_, join_packet, sizeof(join_packet), 0,
                        (struct sockaddr *)&dest_addr, sizeof(dest_addr));
  if (success == -1) {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Failed to request measurement start packet.");
    return false;
  }

  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Measurement start packet requested.");
  return true;
}

void NeurOneAdapter::handle_measurement_start_packet() {
  this->measurement_start_packet_received = true;

  this->sampling_frequency =
      ntohl(*reinterpret_cast<uint32_t *>(buffer + StartPacketFieldIndex::SAMPLING_RATE_HZ));

  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "  Sampling frequency: %u Hz",
              this->sampling_frequency);

  uint16_t channel_count =
      ntohs(*reinterpret_cast<uint16_t *>(buffer + StartPacketFieldIndex::NUM_CHANNELS));

  this->num_of_eeg_channels = 0;
  this->num_of_emg_channels = 0;

  for (uint16_t i = 0; i < channel_count; i++) {
    uint16_t source_channel = ntohs(
        *reinterpret_cast<uint16_t *>(buffer + StartPacketFieldIndex::SOURCE_CHANNEL + 2 * i));

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
      this->num_of_emg_channels++;
      this->channel_types[i] = ChannelType::BIPOLAR_CHANNEL;
    } else {
      this->num_of_eeg_channels++;
      this->channel_types[i] = ChannelType::EEG_CHANNEL;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "  Number of EEG channels: %u", this->num_of_eeg_channels);
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "  Number of EMG channels: %u", this->num_of_emg_channels);
}

std::tuple<AdapterSample, bool> NeurOneAdapter::handle_sample_packet() {
  auto adapter_sample = AdapterSample();
  bool trigger_a_from_channel = false;

  uint16_t num_sample_bundles = ntohs(
      *reinterpret_cast<uint16_t *>(buffer + SamplesPacketFieldIndex::SAMPLE_NUM_SAMPLE_BUNDLES));

  if (num_sample_bundles != 1) {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME),
                 "Invalid bundle size received: %u. The supported bundle size is 1. "
                 "Please ensure that the sampling frequency and the packet frequency "
                 "are set to the same value in the EEG software.",
                 num_sample_bundles);
    return {AdapterSample(), trigger_a_from_channel};
  }

  /* Note: be64toh is Linux specific. */
  uint64_t sample_index = be64toh(
      *reinterpret_cast<uint64_t *>(buffer + SamplesPacketFieldIndex::SAMPLE_FIRST_SAMPLE_INDEX));

  uint64_t sample_time_us = be64toh(
      *reinterpret_cast<uint64_t *>(buffer + SamplesPacketFieldIndex::SAMPLE_FIRST_SAMPLE_TIME));
  double sample_time_s = (double)sample_time_us * 1e-6;

  uint16_t channel_count =
      ntohs(*reinterpret_cast<uint16_t *>(buffer + SamplesPacketFieldIndex::SAMPLE_NUM_CHANNELS));

  for (uint16_t i = 0; i < channel_count; i++) {
    uint8_t *buffer_offset = buffer + SamplesPacketFieldIndex::SAMPLE_SAMPLES + 3 * i;
    int32_t value = int24asint32(buffer_offset);

    /* Scale the value by the scale factor (gain) and change from nV to uV*/
    double result_uV = value * DC_MODE_SCALE * 1e-3;

    ChannelType channel_type = channel_types[i];
    switch (channel_type) {
    case ChannelType::EEG_CHANNEL:
      adapter_sample.eeg_data.push_back(result_uV);
      break;
    case ChannelType::BIPOLAR_CHANNEL:
      adapter_sample.emg_data.push_back(result_uV);
      break;
    case ChannelType::TRIGGER_CHANNEL: {
      auto triggers = std::bitset<32>(value);
      adapter_sample.trigger_b = triggers[TriggerBits::B_IN];
      trigger_a_from_channel = triggers[TriggerBits::A_IN];

      if (adapter_sample.trigger_b) {
        RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Received a trigger with sample %lu",
                    sample_index);
      }
      break;
    }
    default:
      continue;
    }
  }

  bool trigger_b_from_queue = false;

  /* Process all triggers with timestamps <= sample_time_s. */
  while (!trigger_queue.empty() && trigger_queue.top().timestamp <= sample_time_s) {
    trigger_queue.pop();

    /* Set the flag to tag the sample. */
    trigger_b_from_queue = true;

    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Flagging sample %lu with trigger", sample_index);
  }
  adapter_sample.trigger_b = trigger_b_from_queue;

  adapter_sample.time = sample_time_s;
  adapter_sample.index = sample_index;
  adapter_sample.trigger_a = false;  // Will be set by trigger queue or packet processing

  return {adapter_sample, trigger_a_from_channel};
}

std::tuple<bool, double> NeurOneAdapter::handle_trigger_packet() {
  uint16_t trigger_count =
      ntohs(*reinterpret_cast<uint16_t *>(buffer + TriggerPacketFieldIndex::TRIGGER_NUM_TRIGGER));

  bool trigger = false;
  uint64_t trigger_time = 0;

  for (uint16_t i = 0; i < trigger_count; i++) {
    int trigger_event_base_index = TriggerPacketFieldIndex::TRIGGERS + 20 * i;

    uint8_t type = buffer[trigger_event_base_index + TriggerEvent::TYPE];

    uint64_t sample_index = be64toh(*reinterpret_cast<uint64_t *>(
        buffer + trigger_event_base_index + TriggerEvent::SAMPLE_INDEX));

    uint64_t trigger_microtime = be64toh(*reinterpret_cast<uint64_t *>(
      buffer + trigger_event_base_index + TriggerEvent::MICROTIME));

    double trigger_time_s = static_cast<double>(trigger_microtime) * 1e-6;

    uint8_t trigger_channel = (type >> 4);

    if (trigger_channel == 1) {
      trigger = true;
      trigger_time = trigger_microtime;
    } else if (trigger_channel == 2) {
      /* Store the trigger timestamp in the min-heap. */
      Trigger trigger_event;
      trigger_event.timestamp = trigger_time_s;
      trigger_queue.push(trigger_event);

      RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Received a trigger packet to port B at time: %f",
          trigger_time_s);
    } else {
      RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME), "Unknown trigger port: %u", trigger_channel);
    }
  }

  double trigger_time_s = static_cast<double>(trigger_time) * 1e-6;
  return {trigger, trigger_time_s};
}

AdapterPacket NeurOneAdapter::read_eeg_data_packet() {
  /* Return variables */
  AdapterPacket packet;
  packet.result = INTERNAL;
  packet.sample = AdapterSample();
  packet.trigger_a_timestamp = -1.0; // in seconds

  bool trigger_a = false;

  bool success = read_eeg_data_from_socket();
  if (!success) {
    RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME), "Timeout while reading EEG data");
    packet.result = ERROR;
    return packet;
  }

  uint8_t frame_type = this->buffer[0];

  switch (frame_type) {
  case FrameType::MEASUREMENT_START:
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Measurement start packet received");
    handle_measurement_start_packet();
    packet.result = INTERNAL;
    break;
  case FrameType::SAMPLES:
    std::tie(packet.sample, trigger_a) = handle_sample_packet();
    packet.result = SAMPLE;
    if (trigger_a) {
      packet.trigger_a_timestamp = packet.sample.time;
      packet.sample.trigger_a = true;
    }
    break;
  case FrameType::TRIGGER:
    std::tie(trigger_a, packet.trigger_a_timestamp) = handle_trigger_packet();
    packet.result = trigger_a ? TRIGGER_ONLY : INTERNAL;
    break;
  case FrameType::MEASUREMENT_END:
    RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME), "Measurement end packet received.");
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Measurement ended on the EEG device.");
    packet.result = END;
    break;

  case FrameType::HARDWARE_STATE:
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME),
                "Hardware state packet received. Currently no effect");
    break;
  default:
    RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME), "Unknown frame type received, doing nothing.");
  }

  /* The first packet in a new stream is the measurement start packet. If the first received
     packet is something else, the socket started reading middle of the measurement, thus
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
