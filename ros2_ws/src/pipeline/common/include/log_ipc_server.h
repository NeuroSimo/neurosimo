#ifndef LOG_IPC_SERVER_H
#define LOG_IPC_SERVER_H

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <atomic>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <string>
#include <thread>

class LogIpcServer {
public:
  using SinkFn = std::function<void(std::string&&)>;

  explicit LogIpcServer(SinkFn sink) : sink_(std::move(sink)) {}

  bool start() {
    path_ = "/tmp/neurosimo-log-" + std::to_string(::getpid()) + ".sock";
    ::unlink(path_.c_str());

    fd_ = ::socket(AF_UNIX, SOCK_DGRAM, 0);
    if (fd_ < 0) return false;

    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    std::snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", path_.c_str());

    if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
      ::close(fd_);
      fd_ = -1;
      return false;
    }

    ::setenv("NEUROSIMO_LOG_SOCK", path_.c_str(), 1);

    running_.store(true);
    thread_ = std::thread([this] { this->loop(); });
    return true;
  }

  void stop() {
    if (!running_.exchange(false)) return;

    if (fd_ >= 0) {
      ::shutdown(fd_, SHUT_RDWR);
      ::close(fd_);
      fd_ = -1;
    }

    if (thread_.joinable()) thread_.join();

    if (!path_.empty()) {
      ::unlink(path_.c_str());
      path_.clear();
    }
    ::unsetenv("NEUROSIMO_LOG_SOCK");
  }

  ~LogIpcServer() { stop(); }

  LogIpcServer(const LogIpcServer&) = delete;
  LogIpcServer& operator=(const LogIpcServer&) = delete;

private:
  void loop() {
    char buf[8192];

    while (running_.load()) {
      ssize_t n = ::recvfrom(fd_, buf, sizeof(buf) - 1, 0, nullptr, nullptr);
      if (n <= 0) {
        continue;
      }
      buf[n] = '\0';
      sink_(std::string(buf, static_cast<size_t>(n)));
    }
  }

  SinkFn sink_;
  std::atomic<bool> running_{false};
  std::thread thread_;
  int fd_ = -1;
  std::string path_;
};

namespace log_ipc_client {

inline int get_client_fd() {
  static thread_local int fd = -1;
  if (fd >= 0) return fd;
  fd = ::socket(AF_UNIX, SOCK_DGRAM, 0);
  return fd;
}

inline bool send_to_parent(const std::string& s) {
  const char* path = ::getenv("NEUROSIMO_LOG_SOCK");
  if (!path || !*path) return false;

  int fd = get_client_fd();
  if (fd < 0) return false;

  sockaddr_un addr{};
  addr.sun_family = AF_UNIX;
  std::snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", path);

  ::sendto(fd, s.data(), s.size(), 0, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
  return true;
}

} // namespace log_ipc_client

#endif // LOG_IPC_SERVER_H
