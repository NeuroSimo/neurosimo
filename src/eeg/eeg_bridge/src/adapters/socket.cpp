#include "socket.h"

#include <rclcpp/rclcpp.hpp>

UdpSocket::UdpSocket(uint16_t port) : port(port) {}

UdpSocket::~UdpSocket() noexcept {
  if (this->socket_ != -1) {
    close(this->socket_);
  }
}

bool UdpSocket::init_socket() {
  RCLCPP_INFO(rclcpp::get_logger("udp_socket"), "Binding to port: %d", this->port);

  this->socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (this->socket_ == -1) {
    RCLCPP_ERROR(rclcpp::get_logger("udp_socket"),
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
    RCLCPP_ERROR(rclcpp::get_logger("udp_socket"),
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
    RCLCPP_ERROR(rclcpp::get_logger("udp_socket"), "Failed to set socket buffer size.");
  }

  /* Check the actual buffer size. */
  int total_buffer_size;
  socklen_t optlen = sizeof(total_buffer_size);
  if (getsockopt(this->socket_, SOL_SOCKET, SO_RCVBUF, &total_buffer_size, &optlen) == -1) {
    RCLCPP_ERROR(rclcpp::get_logger("udp_socket"), "Failed to get socket buffer size.");
  }

  /* In Linux, the kernel doubles the buffer size for bookkeeping overhead. */
  int receive_buffer_size = total_buffer_size / 2;

  if (receive_buffer_size < requested_buffer_size) {
    RCLCPP_ERROR(rclcpp::get_logger("udp_socket"),
                 "Failed to set socket receive buffer size to %d bytes, actual "
                 "size is %d bytes.",
                 requested_buffer_size, receive_buffer_size);
    return false;
  }
  return true;
}

bool UdpSocket::read_data(uint8_t* buffer, size_t buffer_size) {
  auto success = recvfrom(this->socket_, buffer, buffer_size, 0, nullptr, nullptr);
  if (success == -1) {
    RCLCPP_DEBUG(rclcpp::get_logger("udp_socket"), "No data received, reason: %s", strerror(errno));
    return false;
  }

  return true;
}

bool UdpSocket::send_data(const uint8_t* data, size_t data_size, const sockaddr_in& dest_addr) {
  auto success = sendto(this->socket_, data, data_size, 0,
                        (struct sockaddr *)&dest_addr, sizeof(dest_addr));
  if (success == -1) {
    RCLCPP_ERROR(rclcpp::get_logger("udp_socket"), "Failed to send data.");
    return false;
  }

  return true;
}