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

bool TurboLinkAdapter::read_eeg_data_from_socket() {
  auto success = recvfrom(this->socket_, this->buffer, BUFFER_SIZE, 0, nullptr, nullptr);
  if (success == -1) {
    RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME), "No data received, reason: %s", strerror(errno));
    return false;
  }

  return true;
}

std::tuple<eeg_msgs::msg::Sample, bool> TurboLinkAdapter::handle_packet() {
  auto sample = eeg_msgs::msg::Sample();
  bool sync_trigger_received = false;

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
    sample.emg_data.push_back(value);
  }
  for (uint8_t i = 0; i < this->num_of_eeg_channels; i++) {
    uint8_t *data = buffer + SamplePacketFieldIndex::EEG_CHANNELS + 4 * i;
    float_t value = *reinterpret_cast<float *>(data);
    sample.eeg_data.push_back(value);
  }

  auto triggers = std::bitset<32>(trigger_bits);
  sample.is_trigger = triggers[TriggerBitPosition::PULSE_TRIGGER_BIT];
  sync_trigger_received = triggers[TriggerBitPosition::SYNC_TRIGGER_BIT];

  if (sample.is_trigger) {
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Trigger received with sample %d", sample_index);
  }

  /* Turbolink has no timestamp so interpret time from sampling frequency */
  sample.time = (double)sample_index / this->sampling_frequency;
  sample.index = sample_index;

  sample.metadata.num_of_eeg_channels = this->num_of_eeg_channels;
  sample.metadata.num_of_emg_channels = this->num_of_emg_channels;
  sample.metadata.sampling_frequency = this->sampling_frequency;
  sample.metadata.is_simulation = false;

  return {sample, sync_trigger_received};
}

std::tuple<PacketResult, eeg_msgs::msg::Sample, double>
TurboLinkAdapter::read_eeg_data_packet() {
  /* Return variables */
  PacketResult result_type = PacketResult::INTERNAL;
  auto sample = eeg_msgs::msg::Sample();
  double sync_trigger_time = -1.0; // in seconds

  bool sync_trigger = false;

  bool success = read_eeg_data_from_socket();
  if (!success) {
    RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME), "Timeout while reading EEG data");
    return {PacketResult::ERROR, sample, -1.0};
  }
  std::tie(sample, sync_trigger) = handle_packet();
  result_type = PacketResult::SAMPLE;
  if (sync_trigger) {
    sync_trigger_time = sample.time;
    result_type = PacketResult::SAMPLE_WITH_SYNC;
  }

  return {result_type, sample, sync_trigger_time};
}

float_t TurboLinkAdapter::convert_be_float_to_host(uint8_t *buffer) {
  uint32_t little_endian_value = ntohl(*reinterpret_cast<uint32_t *>(buffer));
  return *reinterpret_cast<float *>(&little_endian_value);
}
