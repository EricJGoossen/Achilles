// Byte-level helpers shared by the hardware emulation: little-endian packing
// and the two CRC families on the sentry's wires -- DJI's serial protocol
// (CRC-8/MAXIM header check, CRC-16/MCRF4XX frame check), used by the
// referee system and the MCB <-> Jetson link alike, and the LD19 lidar's
// own CRC-8 (polynomial 0x4D, MSB first, initial value 0).
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <vector>

namespace sentry::hw {

using Bytes = std::vector<std::uint8_t>;

template <typename T>
void Put(Bytes& out, T value) {
  std::array<std::uint8_t, sizeof(T)> raw{};
  std::memcpy(raw.data(), &value, sizeof(T));  // the sim only runs on LE hosts
  out.insert(out.end(), raw.begin(), raw.end());
}

template <typename T>
T Get(std::span<const std::uint8_t> in, std::size_t offset) {
  T value{};
  std::memcpy(&value, in.data() + offset, sizeof(T));
  return value;
}

namespace detail {

// A reflected CRC table (LSB first).
template <typename T>
constexpr std::array<T, 256> ReflectedTable(T poly) {
  std::array<T, 256> table{};
  for (std::size_t i = 0; i < 256; ++i) {
    T crc = static_cast<T>(i);
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 1) != 0 ? static_cast<T>((crc >> 1) ^ poly)
                           : static_cast<T>(crc >> 1);
    }
    table[i] = crc;
  }
  return table;
}

constexpr std::array<std::uint8_t, 256> MsbFirstTable8(std::uint8_t poly) {
  std::array<std::uint8_t, 256> table{};
  for (std::size_t i = 0; i < 256; ++i) {
    auto crc = static_cast<std::uint8_t>(i);
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x80) != 0 ? static_cast<std::uint8_t>((crc << 1) ^ poly)
                              : static_cast<std::uint8_t>(crc << 1);
    }
    table[i] = crc;
  }
  return table;
}

inline constexpr auto kDjiCrc8Table = ReflectedTable<std::uint8_t>(0x8C);
inline constexpr auto kDjiCrc16Table = ReflectedTable<std::uint16_t>(0x8408);
inline constexpr auto kLd19Crc8Table = MsbFirstTable8(0x4D);

}  // namespace detail

inline std::uint8_t DjiCrc8(std::span<const std::uint8_t> data) {
  std::uint8_t crc = 0xFF;
  for (std::uint8_t byte : data) {
    crc = detail::kDjiCrc8Table[crc ^ byte];
  }
  return crc;
}

inline std::uint16_t DjiCrc16(std::span<const std::uint8_t> data) {
  std::uint16_t crc = 0xFFFF;
  for (std::uint8_t byte : data) {
    crc = static_cast<std::uint16_t>(
        (crc >> 8) ^ detail::kDjiCrc16Table[(crc ^ byte) & 0xFF]
    );
  }
  return crc;
}

inline std::uint8_t Ld19Crc8(std::span<const std::uint8_t> data) {
  std::uint8_t crc = 0;
  for (std::uint8_t byte : data) {
    crc = detail::kLd19Crc8Table[crc ^ byte];
  }
  return crc;
}

// One DJI-serial frame: 0xA5, data length, sequence, header CRC-8, command
// id, data, frame CRC-16 -- the framing of both the referee system's UART
// and the MCB <-> Jetson link.
inline Bytes DjiSerialFrame(
    std::uint16_t command, std::span<const std::uint8_t> data, std::uint8_t seq
) {
  Bytes frame;
  frame.push_back(0xA5);
  Put<std::uint16_t>(frame, static_cast<std::uint16_t>(data.size()));
  frame.push_back(seq);
  frame.push_back(DjiCrc8(frame));
  Put<std::uint16_t>(frame, command);
  frame.insert(frame.end(), data.begin(), data.end());
  Put<std::uint16_t>(frame, DjiCrc16(frame));
  return frame;
}

// Splits a DJI-serial byte stream into frames, for listening in on a link.
// Bytes that don't make a valid frame are skipped, as the receivers do.
class DjiSerialParser {
 public:
  struct Frame {
    std::uint16_t command;
    Bytes data;
  };

  template <typename OnFrame>
  void Feed(std::span<const std::uint8_t> bytes, OnFrame&& on_frame) {
    buffer_.insert(buffer_.end(), bytes.begin(), bytes.end());
    std::size_t start = 0;
    while (true) {
      while (start < buffer_.size() && buffer_[start] != 0xA5) {
        ++start;
      }
      if (buffer_.size() - start < 5) {
        break;
      }
      std::span<const std::uint8_t> header(buffer_.data() + start, 5);
      if (DjiCrc8(header.first(4)) != header[4]) {
        ++start;
        continue;
      }
      const auto length = Get<std::uint16_t>(header, 1);
      const std::size_t total = 5 + 2 + std::size_t{length} + 2;
      if (buffer_.size() - start < total) {
        break;
      }
      std::span<const std::uint8_t> frame(buffer_.data() + start, total);
      if (DjiCrc16(frame.first(total - 2)) ==
          Get<std::uint16_t>(frame, total - 2)) {
        Frame parsed{
            Get<std::uint16_t>(frame, 5),
            Bytes(frame.begin() + 7, frame.begin() + 7 + length)
        };
        on_frame(parsed);
        start += total;
      } else {
        ++start;
      }
    }
    buffer_.erase(
        buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(start)
    );
    if (buffer_.size() > 65536) {
      buffer_.clear();
    }
  }

 private:
  Bytes buffer_;
};

}  // namespace sentry::hw
