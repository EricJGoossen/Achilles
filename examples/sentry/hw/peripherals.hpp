// The rest of the sentry's peripherals:
//
//  - Dr16: the remote's receiver, 18-byte DBUS frames every 14 ms on UART1,
//    switch positions from an operator script.
//  - Referee: the referee system's serial feed on UART6 -- game status,
//    robot status, every robot's HP, power and heat -- at its rates.
//  - Ld19Packer: an LD19's serial output, 12 points to a packet.
//  - Absolute encoders on turret major's yaw: the CAN encoder on the motor
//    pulley and the Lamprey on the output, as SentryImuCalibrateCommand's
//    binned alignment expects them.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <span>
#include <utility>
#include <vector>

#include "wire.hpp"

namespace sentry::hw {

inline constexpr double kTwoPi = 2.0 * std::numbers::pi;

inline double WrapTwoPi(double angle) {
  angle = std::fmod(angle, kTwoPi);
  return angle < 0.0 ? angle + kTwoPi : angle;
}

// taproot's Remote::SwitchState.
enum class Switch : std::uint8_t { kUp = 1, kDown = 2, kMid = 3 };

// One DBUS frame: sticks centered, both switches as given, no keys.
inline std::array<std::uint8_t, 18> Dr16Frame(Switch left, Switch right) {
  constexpr std::uint16_t kCenter = 1024;
  const std::array<std::uint16_t, 4> channels{
      kCenter, kCenter, kCenter, kCenter
  };
  std::array<std::uint8_t, 18> frame{};
  // Four 11-bit channels, packed little-endian from bit 0.
  std::uint64_t bits = 0;
  for (std::size_t i = 0; i < channels.size(); ++i) {
    bits |= std::uint64_t{channels[i]} << (11 * i);
  }
  for (std::size_t i = 0; i < 6; ++i) {
    frame[i] = static_cast<std::uint8_t>((bits >> (8 * i)) & 0xFF);
  }
  // Byte 5's top four bits hold the switches: right in 4-5, left in 6-7.
  frame[5] = static_cast<std::uint8_t>(
      (frame[5] & 0x0F) | (static_cast<std::uint8_t>(right) << 4) |
      (static_cast<std::uint8_t>(left) << 6)
  );
  // Mouse and keys zero; the wheel centered.
  frame[16] = static_cast<std::uint8_t>(kCenter & 0xFF);
  frame[17] = static_cast<std::uint8_t>(kCenter >> 8);
  return frame;
}

// The operator: a list of (time, left, right) switch changes.
struct OperatorStep {
  float time;
  Switch left;
  Switch right;
};

class Referee {
 public:
  struct Config {
    std::uint8_t robot_id = 7;  // red sentry; 107 is blue's
    std::uint16_t sentry_hp = 400;
    std::uint16_t sentry_max_hp = 400;
    std::uint16_t chassis_power_limit = 100;  // W
    float match_start = 5.0F;          // s of sim time when the match starts
    std::uint16_t match_length = 420;  // s
  };

  explicit Referee(Config config) : config_(config) {}

  // Every frame due at `now` (s), concatenated.
  Bytes Poll(float now) {
    Bytes out;
    if (Due(now, next_status_, 1.0F)) {
      Append(out, 0x0001, GameStatus(now));
    }
    if (Due(now, next_robot_, 0.1F)) {
      Append(out, 0x0201, RobotStatus());
    }
    if (Due(now, next_hp_, 1.0F / 3.0F)) {
      Append(out, 0x0003, AllRobotHp());
    }
    if (Due(now, next_power_, 0.02F)) {
      Append(out, 0x0202, PowerHeat());
    }
    return out;
  }

 private:
  static bool Due(float now, float& next, float period) {
    if (now < next) {
      return false;
    }
    next = now + period;
    return true;
  }

  void Append(Bytes& out, std::uint16_t command, const Bytes& data) {
    const Bytes frame = DjiSerialFrame(command, data, seq_++);
    out.insert(out.end(), frame.begin(), frame.end());
  }

  // game_status (0x0001): type RMUL (4) and stage in one byte, stage time
  // remaining, sync timestamp.
  Bytes GameStatus(float now) const {
    // Stages: 0 not started, 1 preparing, 2 self-check, 3 countdown,
    // 4 in progress, 5 calculating results.
    const bool started = now >= config_.match_start;
    const std::uint8_t stage = started ? 4 : 1;
    const float elapsed = started ? now - config_.match_start : 0.0F;
    const auto remaining = static_cast<std::uint16_t>(
        std::max(0.0F, static_cast<float>(config_.match_length) - elapsed)
    );
    Bytes data{static_cast<std::uint8_t>((stage << 4) | 4)};
    Put<std::uint16_t>(data, remaining);
    Put<std::uint64_t>(data, 0);
    return data;
  }

  // robot_status (0x0201).
  Bytes RobotStatus() const {
    Bytes data{config_.robot_id, 1};
    Put<std::uint16_t>(data, config_.sentry_hp);
    Put<std::uint16_t>(data, config_.sentry_max_hp);
    Put<std::uint16_t>(data, 80);   // barrel cooling rate
    Put<std::uint16_t>(data, 400);  // barrel heat limit
    Put<std::uint16_t>(data, config_.chassis_power_limit);
    data.push_back(0b111);  // gimbal, chassis and shooter power on
    return data;
  }

  // game_robot_HP (0x0003): this side's robots -- hero, engineer, standard
  // 3 and 4, (reserved), sentry, outpost, base.
  Bytes AllRobotHp() const {
    Bytes data;
    for (std::uint16_t hp : {200, 250, 200, 200, 0}) {
      Put<std::uint16_t>(data, hp);
    }
    Put<std::uint16_t>(data, config_.sentry_hp);
    Put<std::uint16_t>(data, 1500);  // outpost
    Put<std::uint16_t>(data, 5000);  // base
    return data;
  }

  // power_heat_data (0x0202): reserved words, energy buffer, barrel heat.
  static Bytes PowerHeat() {
    Bytes data;
    Put<std::uint16_t>(data, 0);
    Put<std::uint16_t>(data, 0);
    Put<float>(data, 0.0F);
    Put<std::uint16_t>(data, 60);  // J of buffer energy
    Put<std::uint16_t>(data, 0);
    Put<std::uint16_t>(data, 0);
    return data;
  }

  Config config_;
  std::uint8_t seq_ = 0;
  float next_status_ = 0.0F;
  float next_robot_ = 0.0F;
  float next_hp_ = 0.0F;
  float next_power_ = 0.0F;
};

// Packs an LD19's samples into its 47-byte packets: header 0x54 0x2C, rotor
// speed (deg/s), start angle, 12 x (distance mm, intensity), end angle,
// timestamp (ms, wrapping at 30000) and CRC-8. Angles are clockwise seen
// from above, in hundredths of a degree from the sensor's zero.
class Ld19Packer {
 public:
  struct Point {
    float angle_cw_deg;
    float range_m;  // non-finite: no return
  };

  void Add(const Point& point) { points_.push_back(point); }

  // Every complete packet, consuming its points.
  template <typename OnPacket>
  void Drain(float speed_deg_per_s, float now, OnPacket&& on_packet) {
    constexpr std::size_t kPerPacket = 12;
    while (points_.size() >= kPerPacket) {
      Bytes packet{0x54, 0x2C};
      Put<std::uint16_t>(
          packet, static_cast<std::uint16_t>(std::lround(speed_deg_per_s))
      );
      Put<std::uint16_t>(packet, Centidegrees(points_[0].angle_cw_deg));
      for (std::size_t i = 0; i < kPerPacket; ++i) {
        const float range = points_[i].range_m;
        const bool hit = std::isfinite(range);
        Put<std::uint16_t>(
            packet,
            hit ? static_cast<std::uint16_t>(
                      std::clamp(std::lround(range * 1000.0F), 0L, 65535L)
                  )
                : 0
        );
        packet.push_back(hit ? 200 : 0);
      }
      Put<std::uint16_t>(
          packet, Centidegrees(points_[kPerPacket - 1].angle_cw_deg)
      );
      Put<std::uint16_t>(
          packet,
          static_cast<std::uint16_t>(
              static_cast<std::uint64_t>(now * 1000.0F) % 30000U
          )
      );
      packet.push_back(Ld19Crc8(packet));
      on_packet(std::span<const std::uint8_t>(packet));
      points_.erase(points_.begin(), points_.begin() + kPerPacket);
    }
  }

 private:
  static std::uint16_t Centidegrees(float degrees) {
    float wrapped = std::fmod(degrees, 360.0F);
    if (wrapped < 0.0F) {
      wrapped += 360.0F;
    }
    return static_cast<std::uint16_t>(std::lround(wrapped * 100.0F) % 36000);
  }

  std::vector<Point> points_;
};

// Turret major's yaw encoders. The output turns once per 95/30 turns of the
// motor pulley; the pulley's CAN encoder (4096 counts, mounted inverted)
// and the Lamprey on the output (centidegrees, read through the firmware's
// LAMPREY_CALIBRATION_MAP) are installed with the offsets the firmware's
// alignment constants imply: the Lamprey reads SENTRY_YAW_ALIGNMENT_OFFSET
// with the turret centered, and lamprey - pulley * 30/95 =
// BINNED_ALIGNMENT_OFFSET.
class TurretMajorEncoders {
 public:
  // (raw lamprey angle, true angle) pairs, radians: the firmware's
  // LAMPREY_CALIBRATION_MAP.
  explicit TurretMajorEncoders(std::vector<std::pair<float, float>> lamprey_map)
      : lamprey_map_(std::move(lamprey_map)) {}

  static constexpr double kPulleyRatio = 30.0 / 95.0;  // output per pulley turn
  static constexpr double kLampreyOffset = 267.0 * std::numbers::pi / 180.0;
  static constexpr double kBinnedOffset = 3.712;

  // The pulley encoder frame (0x1E0): raw counts and field strength.
  std::array<std::uint8_t, 8> PulleyFrame(double output_angle) const {
    const double pulley = output_angle / kPulleyRatio +
                          (kLampreyOffset - kBinnedOffset) / kPulleyRatio;
    auto counts =
        static_cast<std::uint32_t>(WrapTwoPi(pulley) / kTwoPi * 4096.0) % 4096U;
    counts = 4095U - counts;  // mounted inverted
    return {
        static_cast<std::uint8_t>(counts & 0xFF),
        static_cast<std::uint8_t>(counts >> 8),
        0x20,
        0x03,
        0,
        0,
        0,
        0
    };
  }

  // The Lamprey frame (0x1E7): the raw angle in centidegrees.
  std::array<std::uint8_t, 8> LampreyFrame(double output_angle) const {
    const double truth = WrapTwoPi(output_angle + kLampreyOffset);
    const double raw = RawForTrue(truth);
    const auto centidegrees = static_cast<std::uint16_t>(
        std::lround(raw * 180.0 / std::numbers::pi * 100.0) % 36000
    );
    return {
        static_cast<std::uint8_t>(centidegrees & 0xFF),
        static_cast<std::uint8_t>(centidegrees >> 8),
        0,
        0,
        0,
        0,
        0,
        0
    };
  }

 private:
  // Inverts the calibration map by linear interpolation on its true side.
  double RawForTrue(double truth) const {
    if (lamprey_map_.size() < 2) {
      return truth;
    }
    for (std::size_t i = 1; i < lamprey_map_.size(); ++i) {
      const auto [raw0, true0] = lamprey_map_[i - 1];
      const auto [raw1, true1] = lamprey_map_[i];
      if (truth >= std::min(true0, true1) && truth <= std::max(true0, true1) &&
          true1 != true0) {
        return raw0 + (raw1 - raw0) * (truth - true0) / (true1 - true0);
      }
    }
    return truth;
  }

  std::vector<std::pair<float, float>> lamprey_map_;
};

}  // namespace sentry::hw
