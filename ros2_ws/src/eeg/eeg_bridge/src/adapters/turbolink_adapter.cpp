#include <arpa/inet.h>
#include <bitset>
#include <sys/socket.h>

#include "rclcpp/rclcpp.hpp"

#include "turbolink_adapter.h"

const std::string LOGGER_NAME = "turbolink_adapter";

TurboLinkAdapter::TurboLinkAdapter(uint16_t port, uint32_t sampling_frequency,
                                   uint8_t eeg_channel_count)
    : port(port) {
  bool success = init_socket();
  if (!success) {
    /*socket initialisation failed. Close socket manually before throwing
    exception, as destructor is not called if constructor fails */
    if (this->socket_ != -1) {
      close(this->socket_);
    }
    throw std::runtime_error("Failed to initialise socket");
  }

  this->num_of_emg_channels = AUX_CHANNEL_COUNT;
  this->num_of_eeg_channels = eeg_channel_count;
  this->sampling_frequency = sampling_frequency;
}

bool TurboLinkAdapter::init_socket() {
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

bool TurboLinkAdapter::read_eeg_from_socket() {
  auto success = recvfrom(this->socket_, this->buffer, BUFFER_SIZE, 0, nullptr, nullptr);
  if (success == -1) {
    RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME), "No data received, reason: %s", strerror(errno));
    return false;
  }

  return true;
}

std::tuple<AdapterSample, bool> TurboLinkAdapter::handle_packet() {
  auto adapter_sample = AdapterSample();
  bool trigger_a = false;

  uint32_t token = *reinterpret_cast<uint32_t *>(buffer + SamplePacketFieldIndex::TOKEN);

  if (token != 0x0050) {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Sample packet started with unknown token %x",
                 token);
  }

  uint32_t sample_index = *reinterpret_cast<uint32_t *>(buffer + SamplePacketFieldIndex::SAMPLE_COUNTER);

  uint32_t trigger_bits = *reinterpret_cast<uint32_t *>(buffer + SamplePacketFieldIndex::TRIGGER_BITS);

  for (uint8_t i = 0; i < this->num_of_emg_channels; i++) {
    uint8_t *data = buffer + SamplePacketFieldIndex::AUX_CHANNELS + 4 * i;
    float_t value = *reinterpret_cast<float *>(data);
    adapter_sample.emg.push_back(value);
  }
  for (uint8_t i = 0; i < this->num_of_eeg_channels; i++) {
    uint8_t *data = buffer + SamplePacketFieldIndex::EEG_CHANNELS + 4 * i;
    float_t value = *reinterpret_cast<float *>(data);
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
  adapter_sample.trigger_a = false;  // Will be set in read_eeg_packet

  return {adapter_sample, trigger_a};
}

AdapterPacket TurboLinkAdapter::read_eeg_packet() {
  /* Return variables */
  AdapterPacket packet;
  packet.result = INTERNAL;
  packet.sample = AdapterSample();
  packet.trigger_a_timestamp = -1.0; // in seconds

  bool trigger_a = false;

  bool success = read_eeg_from_socket();
  if (!success) {
    RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME), "Timeout while reading EEG data");
    packet.result = ERROR;
    return packet;
  }
  std::tie(packet.sample, trigger_a) = handle_packet();
  packet.result = SAMPLE;
  if (trigger_a) {
    packet.trigger_a_timestamp = packet.sample.time;
    packet.sample.trigger_a = true;
  }

  return packet;
}

float_t TurboLinkAdapter::convert_be_float_to_host(uint8_t *buffer) {
  uint32_t little_endian_value = ntohl(*reinterpret_cast<uint32_t *>(buffer));
  return *reinterpret_cast<float *>(&little_endian_value);
}
