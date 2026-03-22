#ifndef EEG_BRIDGE_SRC_ADAPTERS_SOCKET_H
#define EEG_BRIDGE_SRC_ADAPTERS_SOCKET_H

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"

class UdpSocket {
public:
  UdpSocket(uint16_t port);
  ~UdpSocket() noexcept;

  /** Initialize the UDP socket with binding and buffer configuration */
  bool init_socket();

  /** Read data from the socket into the provided buffer
   *
   * @param buffer Buffer to read data into
   * @param buffer_size Size of the buffer
   * @return true if data was read successfully, false otherwise
   */
  bool read_data(uint8_t* buffer, size_t buffer_size);

  /** Send data to a specific destination address
   *
   * @param data Data to send
   * @param data_size Size of data to send
   * @param dest_addr Destination address structure
   * @return true if data was sent successfully, false otherwise
   */
  bool send_data(const uint8_t* data, size_t data_size, const sockaddr_in& dest_addr);

private:
  int socket_ = -1;
  uint16_t port = 0;
  sockaddr_in socket_own = {};
};

#endif // EEG_BRIDGE_SRC_ADAPTERS_SOCKET_H