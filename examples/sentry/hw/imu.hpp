// IMUs as the MCB reads them.
//
//  - The main MCB rides on turret major and reads three chips over SPI: an
//    MPU6500 and two ISM330DHCXs. Their gyro/accelerometer readings come
//    from the sim's sensors (mounted as the firmware's
//    turretMajorImuTransforms say) and go to the firmware as the chips'
//    own data-register bursts, scaled and laid out as the datasheets give.
//  - The chassis and the widow each carry a turret MCB: a second board
//    that runs its own attitude filter and reports on CAN (TurretMCBCanComm
//    in aruw-mcb). TurretMcb plays that board's firmware: offset
//    calibration on request, a Mahony filter, and the X/Y/Z axis frames
//    and status frame the main MCB parses.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <span>
#include <vector>

#include "wire.hpp"

namespace sentry::hw {

inline constexpr float kGravity = 9.80665F;

struct Vec3f {
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
};

inline std::int16_t SaturateInt16(float value) {
  return static_cast<std::int16_t>(
      std::clamp(std::lround(value), -32768L, 32767L)
  );
}

// MPU6500 burst from ACCEL_XOUT_H: accel, temperature, gyro, big endian.
// +-8 g (4096 LSB/g) and +-2000 dps (16.384 LSB per deg/s), as taproot's
// Mpu6500 configures it.
inline std::array<std::uint8_t, 14> Mpu6500Burst(
    Vec3f gyro, Vec3f accel, float temperature
) {
  constexpr float kAccelLsb = 4096.0F / kGravity;
  constexpr float kGyroLsb = 16.384F * 180.0F / std::numbers::pi_v<float>;
  const std::array<std::int16_t, 7> words{
      SaturateInt16(accel.x * kAccelLsb),
      SaturateInt16(accel.y * kAccelLsb),
      SaturateInt16(accel.z * kAccelLsb),
      SaturateInt16((temperature - 21.0F) * 333.87F),
      SaturateInt16(gyro.x * kGyroLsb),
      SaturateInt16(gyro.y * kGyroLsb),
      SaturateInt16(gyro.z * kGyroLsb),
  };
  std::array<std::uint8_t, 14> burst{};
  for (std::size_t i = 0; i < words.size(); ++i) {
    const auto word = static_cast<std::uint16_t>(words[i]);
    burst[2 * i] = static_cast<std::uint8_t>(word >> 8);
    burst[2 * i + 1] = static_cast<std::uint8_t>(word & 0xFF);
  }
  return burst;
}

// ISM330DHCX burst from OUT_TEMP_L: temperature, gyro, accel, little
// endian. +-1000 dps (35 mdps/LSB) and +-4 g (0.122 mg/LSB), aruw-mcb's
// ISM330 defaults.
inline std::array<std::uint8_t, 14> Ism330Burst(
    Vec3f gyro, Vec3f accel, float temperature
) {
  constexpr float kGyroLsb =
      1.0F / (35.0e-3F * std::numbers::pi_v<float> / 180.0F);
  constexpr float kAccelLsb = 1.0F / (0.122e-3F * kGravity);
  const std::array<std::int16_t, 7> words{
      SaturateInt16((temperature - 25.0F) * 256.0F),
      SaturateInt16(gyro.x * kGyroLsb),
      SaturateInt16(gyro.y * kGyroLsb),
      SaturateInt16(gyro.z * kGyroLsb),
      SaturateInt16(accel.x * kAccelLsb),
      SaturateInt16(accel.y * kAccelLsb),
      SaturateInt16(accel.z * kAccelLsb),
  };
  std::array<std::uint8_t, 14> burst{};
  for (std::size_t i = 0; i < words.size(); ++i) {
    const auto word = static_cast<std::uint16_t>(words[i]);
    burst[2 * i] = static_cast<std::uint8_t>(word & 0xFF);
    burst[2 * i + 1] = static_cast<std::uint8_t>(word >> 8);
  }
  return burst;
}

// Mahony's complementary attitude filter (gyro integration, corrected
// toward the accelerometer's gravity), the filter taproot's AbstractIMU and
// the turret MCB firmware run.
class MahonyFilter {
 public:
  MahonyFilter(float kp, float ki) : kp_(kp), ki_(ki) {}

  void Reset() {
    q_ = {1.0F, 0.0F, 0.0F, 0.0F};
    integral_ = {};
  }

  void Update(Vec3f gyro, Vec3f accel, float dt) {
    float gx = gyro.x;
    float gy = gyro.y;
    float gz = gyro.z;
    const float norm =
        std::sqrt(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
    if (norm > 1e-6F) {
      const float ax = accel.x / norm;
      const float ay = accel.y / norm;
      const float az = accel.z / norm;
      const auto [q0, q1, q2, q3] = q_;
      // Gravity's direction as the current attitude predicts it.
      const float vx = 2.0F * (q1 * q3 - q0 * q2);
      const float vy = 2.0F * (q0 * q1 + q2 * q3);
      const float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
      const float ex = ay * vz - az * vy;
      const float ey = az * vx - ax * vz;
      const float ez = ax * vy - ay * vx;
      if (ki_ > 0.0F) {
        integral_.x += 2.0F * ki_ * ex * dt;
        integral_.y += 2.0F * ki_ * ey * dt;
        integral_.z += 2.0F * ki_ * ez * dt;
      }
      gx += 2.0F * kp_ * ex + integral_.x;
      gy += 2.0F * kp_ * ey + integral_.y;
      gz += 2.0F * kp_ * ez + integral_.z;
    }
    auto [q0, q1, q2, q3] = q_;
    const float half_dt = 0.5F * dt;
    const float n0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * half_dt;
    const float n1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * half_dt;
    const float n2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * half_dt;
    const float n3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * half_dt;
    const float qn = std::sqrt(n0 * n0 + n1 * n1 + n2 * n2 + n3 * n3);
    q_ = {n0 / qn, n1 / qn, n2 / qn, n3 / qn};
  }

  float Roll() const {
    const auto [q0, q1, q2, q3] = q_;
    return std::atan2(q0 * q1 + q2 * q3, 0.5F - q1 * q1 - q2 * q2);
  }
  float Pitch() const {
    const auto [q0, q1, q2, q3] = q_;
    return std::asin(std::clamp(-2.0F * (q1 * q3 - q0 * q2), -1.0F, 1.0F));
  }
  float Yaw() const {
    const auto [q0, q1, q2, q3] = q_;
    return std::atan2(q1 * q2 + q0 * q3, 0.5F - q2 * q2 - q3 * q3);
  }

 private:
  float kp_;
  float ki_;
  std::array<float, 4> q_{1.0F, 0.0F, 0.0F, 0.0F};
  Vec3f integral_{};
};

// A turret MCB: the second board on the chassis or on the widow, running
// its IMU filter on every sample and reporting at 500 Hz.
class TurretMcb {
 public:
  // taproot's ImuInterface::ImuState.
  enum class State : std::uint8_t {
    kNotConnected = 0,
    kNotCalibrated = 1,
    kCalibrating = 2,
    kCalibrated = 3
  };

  explicit TurretMcb(std::uint8_t bus) : bus_(bus) {}

  std::uint8_t Bus() const { return bus_; }

  // A frame from the main MCB. 0x1F7 carries its command bitmask
  // (TurretMCBCanComm::TxCommandMsgBitmask); bit 1 asks for calibration.
  void HandleFrame(
      std::uint8_t bus, std::uint32_t id, std::span<const std::uint8_t> data
  ) {
    if (bus != bus_) {
      return;
    }
    if (id == 0x1F7 && (data[0] & 0x02) != 0 && state_ != State::kCalibrating) {
      state_ = State::kCalibrating;
      samples_ = 0;
      gyro_sum_ = {};
      accel_sum_ = {};
    } else if (id == 0x1F4) {
      // CALIBRATION_SAMPLES sync: the sample count to calibrate over.
      const auto count = Get<std::uint16_t>(data, 0);
      if (count > 0) {
        calibration_samples_ = count;
      }
    }
  }

  // One IMU sample, `dt` after the last, in the board's mounted frame: rad/s
  // and m/s^2 (specific force, +g up at rest).
  void Sample(Vec3f gyro, Vec3f accel, float dt) {
    if (state_ == State::kCalibrating) {
      gyro_sum_ = {
          gyro_sum_.x + gyro.x, gyro_sum_.y + gyro.y, gyro_sum_.z + gyro.z
      };
      accel_sum_ = {
          accel_sum_.x + accel.x,
          accel_sum_.y + accel.y,
          accel_sum_.z + accel.z - kGravity
      };
      if (++samples_ >= calibration_samples_) {
        const auto n = static_cast<float>(samples_);
        gyro_offset_ = {gyro_sum_.x / n, gyro_sum_.y / n, gyro_sum_.z / n};
        accel_offset_ = {accel_sum_.x / n, accel_sum_.y / n, accel_sum_.z / n};
        filter_.Reset();
        state_ = State::kCalibrated;
      }
      gyro_ = {};
      accel_ = {};
      return;
    }
    gyro_ = {
        gyro.x - gyro_offset_.x,
        gyro.y - gyro_offset_.y,
        gyro.z - gyro_offset_.z
    };
    accel_ = {
        accel.x - accel_offset_.x,
        accel.y - accel_offset_.y,
        accel.z - accel_offset_.z
    };
    filter_.Update(gyro_, accel_, dt);
  }

  // The X, Y and Z axis frames (0x1FB-0x1FD): angle, rate and acceleration
  // per axis, with a shared sequence number.
  template <typename SendFrame>
  void SendAxes(SendFrame&& send) {
    constexpr float kAngleScale = 65535.0F / (2.0F * std::numbers::pi_v<float>);
    constexpr float kGyroLsb = 16.384F * 180.0F / std::numbers::pi_v<float>;
    const std::array<float, 3> angles{
        filter_.Roll(), filter_.Pitch(), filter_.Yaw()
    };
    const std::array<float, 3> rates{gyro_.x, gyro_.y, gyro_.z};
    const std::array<float, 3> accels{accel_.x, accel_.y, accel_.z};
    ++seq_;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      Bytes data;
      Put<std::int16_t>(data, SaturateInt16(angles[axis] * kAngleScale));
      Put<std::int16_t>(data, SaturateInt16(rates[axis] * kGyroLsb));
      Put<std::int16_t>(data, SaturateInt16(accels[axis] * 100.0F));
      data.push_back(seq_);
      send(
          bus_,
          static_cast<std::uint32_t>(0x1FB + axis),
          std::span<const std::uint8_t>(data)
      );
    }
  }

  // The status frame (0x1FA): limit switches, IMU state, temperature.
  template <typename SendFrame>
  void SendStatus(SendFrame&& send) {
    Bytes data{0, static_cast<std::uint8_t>(state_)};
    Put<std::int16_t>(data, 4000);  // 40.00 C
    send(bus_, 0x1FAU, std::span<const std::uint8_t>(data));
  }

 private:
  std::uint8_t bus_;
  State state_ = State::kNotCalibrated;
  MahonyFilter filter_{0.1F, 0.0F};
  std::uint32_t calibration_samples_ = 2000;
  std::uint32_t samples_ = 0;
  Vec3f gyro_sum_{};
  Vec3f accel_sum_{};
  Vec3f gyro_offset_{};
  Vec3f accel_offset_{};
  Vec3f gyro_{};
  Vec3f accel_{};
  std::uint8_t seq_ = 0;
};

}  // namespace sentry::hw
