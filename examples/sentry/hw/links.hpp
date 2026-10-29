// The sim's connections to the robot's two computers, as the physical robot
// wires them:
//
//  - McbLink: the MCB's peripherals. The MCB firmware's hosted build
//    (aruw-mcb, ARUW_SIM_HARDWARE_SOCKET) connects to a Unix SOCK_SEQPACKET
//    socket and trades raw CAN frames, UART bytes and IMU register bursts
//    with it; the format is in aruw-mcb's
//    src/aruwsrc/communication/sim/sim_hardware_bridge.cpp.
//  - Pty: a pseudo-terminal standing in for a serial device the Jetson
//    opens -- the MCB's UART, each LD19 -- symlinked to a stable path.
//  - SeqpacketServer: a plain message socket, for the camera feed.
#pragma once

#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

#include "wire.hpp"

namespace sentry::hw {

// A listening Unix SOCK_SEQPACKET socket with at most one client at a time.
class SeqpacketServer {
 public:
  explicit SeqpacketServer(std::string path) : path_(std::move(path)) {
    ::unlink(path_.c_str());
    listen_fd_ = ::socket(AF_UNIX, SOCK_SEQPACKET | SOCK_NONBLOCK, 0);
    if (listen_fd_ < 0) {
      throw std::runtime_error("socket: " + std::string(std::strerror(errno)));
    }
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    if (path_.size() >= sizeof(address.sun_path)) {
      throw std::runtime_error("socket path too long: " + path_);
    }
    std::strncpy(address.sun_path, path_.c_str(), sizeof(address.sun_path) - 1);
    if (::bind(
            listen_fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)
        ) != 0 ||
        ::listen(listen_fd_, 1) != 0) {
      throw std::runtime_error(
          "bind/listen " + path_ + ": " + std::string(std::strerror(errno))
      );
    }
    // Generous buffers: a stalled peer should lose data, not stall the sim.
    buffer_size_ = 4 << 20;
  }
  SeqpacketServer(const SeqpacketServer&) = delete;
  SeqpacketServer& operator=(const SeqpacketServer&) = delete;
  ~SeqpacketServer() {
    if (client_fd_ >= 0) {
      ::close(client_fd_);
    }
    ::close(listen_fd_);
    ::unlink(path_.c_str());
  }

  bool Connected() const { return client_fd_ >= 0; }
  const std::string& Path() const { return path_; }

  // Accepts a waiting client, if there's none yet. Returns true on a new one.
  bool Accept() {
    if (client_fd_ >= 0) {
      return false;
    }
    const int fd = ::accept4(listen_fd_, nullptr, nullptr, SOCK_NONBLOCK);
    if (fd < 0) {
      return false;
    }
    ::setsockopt(
        fd, SOL_SOCKET, SO_SNDBUF, &buffer_size_, sizeof(buffer_size_)
    );
    ::setsockopt(
        fd, SOL_SOCKET, SO_RCVBUF, &buffer_size_, sizeof(buffer_size_)
    );
    client_fd_ = fd;
    return true;
  }

  // Sends one message; dropped if the client is gone or not keeping up.
  void Send(std::span<const std::uint8_t> message) {
    if (client_fd_ < 0) {
      return;
    }
    const ssize_t sent = ::send(
        client_fd_, message.data(), message.size(), MSG_DONTWAIT | MSG_NOSIGNAL
    );
    if (sent < 0 && (errno == EPIPE || errno == ECONNRESET)) {
      Drop();
    }
  }

  // Calls on_message for every message waiting. Returns false once the
  // client has hung up (and forgets it, so another may connect).
  bool Receive(
      const std::function<void(std::span<const std::uint8_t>)>& on_message
  ) {
    if (client_fd_ < 0) {
      return false;
    }
    std::array<std::uint8_t, 65536> buffer{};
    while (true) {
      const ssize_t size =
          ::recv(client_fd_, buffer.data(), buffer.size(), MSG_DONTWAIT);
      if (size > 0) {
        on_message(std::span<const std::uint8_t>(
            buffer.data(), static_cast<std::size_t>(size)
        ));
        continue;
      }
      if (size == 0 || (errno != EAGAIN && errno != EINTR)) {
        Drop();
        return false;
      }
      return true;
    }
  }

 private:
  void Drop() {
    ::close(client_fd_);
    client_fd_ = -1;
  }

  std::string path_;
  int listen_fd_ = -1;
  int client_fd_ = -1;
  int buffer_size_ = 0;
};

// The MCB's side of every peripheral, over McbLink's wire format.
class McbLink {
 public:
  enum Bus : std::uint8_t { kCan1 = 0, kCan2 = 1 };
  // tap::communication::serial::Uart::UartPort.
  enum Port : std::uint8_t {
    kUart1 = 0,  // DR16 receiver
    kUart2 = 1,  // to the Jetson (TX)
    kUart3 = 2,  // from the Jetson (RX)
    kUart6 = 3,  // referee system
    kUart7 = 4,
    kUart8 = 5,
  };
  // tap::arch::hosted::HardwareBackend::ImuChip.
  enum Chip : std::uint8_t { kMpu6500 = 0, kIsm330 = 1 };

  struct CanFrame {
    Bus bus;
    std::uint32_t id;
    std::uint8_t length;
    std::array<std::uint8_t, 8> data;
  };

  explicit McbLink(const std::string& path) : server_(path) {}

  const std::string& Path() const { return server_.Path(); }
  bool Connected() const { return hello_ && server_.Connected(); }

  // Accepts the MCB and reads everything it has sent since the last call.
  // Returns false once it has disconnected.
  bool Poll(
      const std::function<void(const CanFrame&)>& on_can,
      const std::function<void(Port, std::span<const std::uint8_t>)>& on_uart
  ) {
    if (server_.Accept()) {
      hello_ = false;
    }
    return server_.Receive([&](std::span<const std::uint8_t> message) {
      switch (message[0]) {
        case 0x00:
          hello_ = true;
          break;
        case 0x01:
          if (message.size() >= 16) {
            CanFrame frame{
                static_cast<Bus>(message[1]),
                Get<std::uint32_t>(message, 4),
                message[2],
                {}
            };
            std::memcpy(frame.data.data(), message.data() + 8, 8);
            on_can(frame);
          }
          break;
        case 0x02:
          if (message.size() >= 4) {
            const auto length = Get<std::uint16_t>(message, 2);
            on_uart(
                static_cast<Port>(message[1]),
                message.subspan(
                    4, std::min<std::size_t>(length, message.size() - 4)
                )
            );
          }
          break;
        default:
          break;
      }
    });
  }

  void SendCan(Bus bus, std::uint32_t id, std::span<const std::uint8_t> data) {
    std::array<std::uint8_t, 16> message{};
    message[0] = 0x01;
    message[1] = bus;
    message[2] =
        static_cast<std::uint8_t>(std::min<std::size_t>(data.size(), 8));
    message[3] = 0;
    std::memcpy(message.data() + 4, &id, 4);
    std::memcpy(message.data() + 8, data.data(), message[2]);
    Send(message);
  }

  void SendUart(Port port, std::span<const std::uint8_t> bytes) {
    for (std::size_t offset = 0; offset < bytes.size(); offset += 1024) {
      const std::size_t chunk =
          std::min<std::size_t>(1024, bytes.size() - offset);
      Bytes message{0x02, port};
      Put<std::uint16_t>(message, static_cast<std::uint16_t>(chunk));
      message.insert(
          message.end(), bytes.begin() + offset, bytes.begin() + offset + chunk
      );
      Send(message);
    }
  }

  void SendImu(
      Chip chip, std::uint8_t instance, std::span<const std::uint8_t> burst
  ) {
    Bytes message{
        0x03, chip, instance, static_cast<std::uint8_t>(burst.size())
    };
    message.insert(message.end(), burst.begin(), burst.end());
    Send(message);
  }

  void SendTime(std::uint64_t time_us, std::uint32_t step_us) {
    Bytes message{0x04};
    Put(message, time_us);
    Put(message, step_us);
    Send(message);
  }

 private:
  void Send(std::span<const std::uint8_t> message) {
    if (Connected()) {
      server_.Send(message);
    }
  }

  SeqpacketServer server_;
  bool hello_ = false;
};

// A pseudo-terminal: the sim holds the master, the Jetson software opens the
// slave through a symlink, as it would a /dev/tty device. Raw mode; baud
// rate settings on the slave are accepted and ignored.
class Pty {
 public:
  explicit Pty(std::string link) : link_(std::move(link)) {
    fd_ = ::posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0 || ::grantpt(fd_) != 0 || ::unlockpt(fd_) != 0) {
      throw std::runtime_error(
          "posix_openpt: " + std::string(std::strerror(errno))
      );
    }
    termios tio{};
    ::tcgetattr(fd_, &tio);
    ::cfmakeraw(&tio);
    ::tcsetattr(fd_, TCSANOW, &tio);
    const char* slave = ::ptsname(fd_);
    if (slave == nullptr) {
      throw std::runtime_error("ptsname failed");
    }
    slave_ = slave;
    // Configure the slave side raw too, so nothing is echoed or translated
    // before the reader sets its own attributes.
    const int slave_fd = ::open(slave, O_RDWR | O_NOCTTY);
    if (slave_fd >= 0) {
      ::tcgetattr(slave_fd, &tio);
      ::cfmakeraw(&tio);
      ::tcsetattr(slave_fd, TCSANOW, &tio);
      // Keep one slave handle open so the master never sees a hangup while
      // the reader reconnects.
      keep_open_fd_ = slave_fd;
    }
    ::unlink(link_.c_str());
    if (::symlink(slave, link_.c_str()) != 0) {
      throw std::runtime_error(
          "symlink " + link_ + ": " + std::string(std::strerror(errno))
      );
    }
  }
  Pty(const Pty&) = delete;
  Pty& operator=(const Pty&) = delete;
  ~Pty() {
    ::unlink(link_.c_str());
    if (keep_open_fd_ >= 0) {
      ::close(keep_open_fd_);
    }
    ::close(fd_);
  }

  const std::string& Link() const { return link_; }

  // Writes what fits; a reader that isn't keeping up loses bytes, as a UART
  // overrun would.
  void Write(std::span<const std::uint8_t> bytes) {
    std::size_t offset = 0;
    while (offset < bytes.size()) {
      const ssize_t written =
          ::write(fd_, bytes.data() + offset, bytes.size() - offset);
      if (written <= 0) {
        return;
      }
      offset += static_cast<std::size_t>(written);
    }
  }

  // Everything the reader has written since the last call.
  Bytes Read() {
    Bytes out;
    std::array<std::uint8_t, 4096> buffer{};
    while (true) {
      const ssize_t size = ::read(fd_, buffer.data(), buffer.size());
      if (size <= 0) {
        break;
      }
      out.insert(out.end(), buffer.begin(), buffer.begin() + size);
    }
    return out;
  }

 private:
  std::string link_;
  std::string slave_;
  int fd_ = -1;
  int keep_open_fd_ = -1;
};

}  // namespace sentry::hw
