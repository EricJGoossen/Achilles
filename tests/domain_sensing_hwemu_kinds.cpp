// The kinds added for hardware emulation: the attitude estimate, the limit
// switch, a camera's pinhole intrinsics, and a motor's winding temperature --
// each against its own closed-form answer.
#include <gtest/gtest.h>

#include <cmath>
#include <numbers>
#include <stdexcept>

#include "domain/actuation/dc_motor.hpp"
#include "domain/attachment/attachment.hpp"
#include "domain/math/quaternion.hpp"
#include "domain/math/vector3.hpp"
#include "domain/sensing/attitude.hpp"
#include "domain/sensing/camera.hpp"
#include "domain/sensing/limit_switch.hpp"
#include "domain/spatial/transform.hpp"

using achilles::domain::actuation::DcMotor;
using achilles::domain::actuation::DcMotorCommand;
using achilles::domain::actuation::DcMotorOutput;
using achilles::domain::actuation::DcMotorState;
using achilles::domain::attachment::AttachmentOptions;
using achilles::domain::attachment::AttachmentTick;
using achilles::domain::math::Quaternion;
using achilles::domain::math::Vector3;
using achilles::domain::sensing::Attitude;
using achilles::domain::sensing::Camera;
using achilles::domain::sensing::LimitSwitch;
using achilles::domain::sensing::SwitchReading;
using achilles::domain::sensing::SwitchState;
using achilles::domain::spatial::Transform;

namespace {

constexpr float kPi = std::numbers::pi_v<float>;

Quaternion<float> About(const Vector3<float>& axis, float angle) {
  return Quaternion<float>::Exp(axis * angle);
}

Vector3<float> ReadAttitude(const Attitude& sensor, const Transform<float>& pose) {
  Attitude::ErrorModel::State state;
  Vector3<float> out;
  sensor.Read(AttachmentTick{0.002F}, pose, &state, &out);
  return out;
}

}  // namespace

// Z-Y-X: world_from_sensor = Rz(yaw) Ry(pitch) Rx(roll), read straight back.
TEST(Attitude, ReadsZyxEulerAngles) {
  const float roll = 0.3F;
  const float pitch = -0.4F;
  const float yaw = 2.5F;
  const Quaternion<float> q = About(Vector3<float>::UnitZ(), yaw) *
                              About(Vector3<float>::UnitY(), pitch) *
                              About(Vector3<float>::UnitX(), roll);
  const Attitude sensor(AttachmentOptions{}, Attitude::Params{});
  const auto rpy = ReadAttitude(sensor, Transform<float>(Vector3<float>(1, 2, 3), q));
  EXPECT_NEAR(rpy.X(), roll, 1e-5F);
  EXPECT_NEAR(rpy.Y(), pitch, 1e-5F);
  EXPECT_NEAR(rpy.Z(), yaw, 1e-5F);
}

// The mount turns the sensor on its body: a body yawed by a, carrying a
// sensor yawed a further b on it, reads a + b -- wrapped into (-pi, pi].
TEST(Attitude, ComposesTheMountAndWraps) {
  Attitude::Params params;
  params.body_from_sensor = Transform<float>(Vector3<float>(0, 0, 0), About(Vector3<float>::UnitZ(), 2.0F));
  const Attitude sensor(AttachmentOptions{}, params);
  const auto rpy =
      ReadAttitude(sensor, Transform<float>(Vector3<float>(0, 0, 0), About(Vector3<float>::UnitZ(), 2.0F)));
  EXPECT_NEAR(rpy.Z(), 4.0F - (2.0F * kPi), 1e-5F);
}

// A bias moves the estimate, and the result still wraps.
TEST(Attitude, ItsErrorModelActsOnTheAngles) {
  Attitude::Params params;
  params.error.bias.bias = Vector3<float>(0.0F, 0.0F, 0.5F);
  const Attitude sensor(AttachmentOptions{}, params);
  const auto rpy =
      ReadAttitude(sensor, Transform<float>(Vector3<float>(0, 0, 0), About(Vector3<float>::UnitZ(), 3.0F)));
  EXPECT_NEAR(rpy.Z(), 3.5F - (2.0F * kPi), 1e-5F);
}

namespace {

bool Press(const LimitSwitch& sw, SwitchState* state, float angle) {
  SwitchReading out;
  sw.Read(
      AttachmentTick{0.002F},
      Transform<float>(Vector3<float>(0, 0, 0), About(Vector3<float>::UnitX(), angle)),
      state,
      &out
  );
  return out.pressed;
}

}  // namespace

// Pressed at or past press_above; released only `hysteresis` back.
TEST(LimitSwitch, PressesPastItsThresholdWithHysteresis) {
  LimitSwitch::Params params;
  params.has_above = true;
  params.press_above = 0.5F;
  params.hysteresis = 0.1F;
  const LimitSwitch sw(AttachmentOptions{}, params);
  SwitchState state;
  EXPECT_FALSE(Press(sw, &state, 0.45F));
  EXPECT_TRUE(Press(sw, &state, 0.5F));
  EXPECT_TRUE(Press(sw, &state, 0.45F));  // within the hysteresis: still held
  EXPECT_FALSE(Press(sw, &state, 0.39F));
}

TEST(LimitSwitch, BelowAndNormallyClosed) {
  LimitSwitch::Params params;
  params.has_below = true;
  params.press_below = -0.2F;
  params.normally_closed = true;
  const LimitSwitch sw(AttachmentOptions{}, params);
  SwitchState state;
  EXPECT_TRUE(Press(sw, &state, 0.0F));    // released: a closed contact
  EXPECT_FALSE(Press(sw, &state, -0.3F));  // pressed: opens
}

TEST(LimitSwitch, NeedsAThreshold) {
  EXPECT_THROW(LimitSwitch(AttachmentOptions{}, LimitSwitch::Params{}), std::invalid_argument);
}

// Intrinsics in pixels go in and come back out, and a point on the optical
// axis lands on the principal point.
TEST(Camera, IntrinsicsFromConfig) {
  Camera::Params params;
  params.width = 960;
  params.height = 540;
  params.fx = 610.0F;
  params.fy = 612.0F;
  params.cx = 470.0F;
  params.cy = 280.0F;
  const Camera camera(AttachmentOptions{}, params);
  const auto in = camera.GetIntrinsics();
  EXPECT_NEAR(in.fx, 610.0F, 1e-3F);
  EXPECT_NEAR(in.fy, 612.0F, 1e-3F);
  EXPECT_NEAR(in.cx, 470.0F, 1e-3F);
  EXPECT_NEAR(in.cy, 280.0F, 1e-3F);
  // Sensor axes: x forward, y left, z up.
  const auto center = camera.ImagePoint(Vector3<float>(2.0F, 0.0F, 0.0F));
  EXPECT_NEAR(center[0] * 960.0F, 470.0F, 1e-2F);
  EXPECT_NEAR(center[1] * 540.0F, 280.0F, 1e-2F);
  // A point 0.1 m right (-y) of the axis at 2 m: u = fx * 0.1 / 2 + cx.
  const auto right = camera.ImagePoint(Vector3<float>(2.0F, -0.1F, 0.0F));
  EXPECT_NEAR(right[0] * 960.0F, (610.0F * 0.05F) + 470.0F, 1e-2F);
}

TEST(Camera, HasRoomForARealsense) {
  EXPECT_GE(Camera::kMaxWidth, 960U);
  EXPECT_GE(Camera::kMaxHeight, 540U);
  Camera::Params params;
  params.width = 848;
  params.height = 480;
  EXPECT_NO_THROW(Camera(AttachmentOptions{}, params));
}

namespace {

DcMotor::Params ThermalMotor() {
  DcMotor::Params params;
  params.torque_constant = 0.3F;
  params.back_emf_constant = 0.3F;
  params.resistance = 0.2F;
  params.bus_voltage = 24.0F;
  params.max_current = 20.0F;
  params.gear_ratio = 0.1F;
  params.ambient_temperature = 25.0F;
  params.thermal_resistance = 2.0F;  // K/W
  params.thermal_time_constant = 1.0F;
  return params;
}

}  // namespace

// 10 A through 0.2 ohm is 20 W; at 2 K/W the winding settles 40 K over
// ambient, rising as 1 - exp(-t / tau).
TEST(Motor, WindingHeatsWithItsLoss) {
  const Vector3<float> axis(0.0F, 0.0F, 1.0F);
  const DcMotor motor(AttachmentOptions{}, axis, ThermalMotor());
  DcMotor::Subspace subspace{};
  subspace(2, 0) = 1.0F;
  const DcMotor::Velocity qd(0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
  DcMotorState state;
  DcMotorOutput out;
  DcMotorCommand command = DcMotorCommand::Current(10.0F);
  constexpr float kDt = 0.001F;
  for (int i = 0; i < 1000; ++i) {  // one time constant
    motor.Drive(AttachmentTick{kDt}, qd, subspace, &command, &state, &out);
  }
  EXPECT_NEAR(out.temperature, 25.0F + (40.0F * (1.0F - std::exp(-1.0F))), 0.1F);
  for (int i = 0; i < 9000; ++i) {
    motor.Drive(AttachmentTick{kDt}, qd, subspace, &command, &state, &out);
  }
  EXPECT_NEAR(out.temperature, 65.0F, 0.1F);
}

TEST(Motor, WithoutAThermalResistanceItStaysAtAmbient) {
  DcMotor::Params params = ThermalMotor();
  params.thermal_resistance = 0.0F;
  const DcMotor motor(AttachmentOptions{}, Vector3<float>(0.0F, 0.0F, 1.0F), params);
  DcMotor::Subspace subspace{};
  subspace(2, 0) = 1.0F;
  DcMotorState state;
  DcMotorOutput out;
  DcMotorCommand command = DcMotorCommand::Current(10.0F);
  motor.Drive(AttachmentTick{0.01F}, DcMotor::Velocity(0, 0, 0, 0, 0, 0), subspace, &command, &state, &out);
  EXPECT_FLOAT_EQ(out.temperature, 25.0F);
}
