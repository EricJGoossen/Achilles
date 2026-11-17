// The actuator demo: a four-wheeled robot (waypoint_robot.arow) that drives
// itself around a loop of waypoints. Everything control-related lives in
// this file -- the sim only ever sees torques through the actuators it
// hands out.
//
// Usage: waypoint_robot <waypoint_robot.arow> <sim_config.yaml> [--headless]
//                  [--seconds N]
// (scripts/run-waypoint-robot.sh passes this folder's files). Run in real time,
// with the display built (ACHILLES_WITH_DISPLAY), it opens a window each for
// the loop from above (waypoint_robot_camera.arow), the wheels' torques and
// speeds, and the chassis's pose; closing them stops the run. Headless runs
// print the robot's pose once per simulated second instead.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <numbers>
#include <span>
#include <string>
#include <string_view>

#include "../shared/real_time.hpp"
#include "algorithms/conventions.hpp"
#include "display/renderer.hpp"
#include "display/views/camera/camera_view.hpp"
#include "display/views/reading_view.hpp"
#include "domain/sensing/camera.hpp"
#include "interface/attachment_handle.hpp"
#include "interface/simulation.hpp"

namespace {

using achilles::algorithms::ScalarOperationT;
using achilles::interface::AngularActuator;
using achilles::interface::SensorHandle;
using achilles::interface::Simulation;
namespace truth = achilles::interface::truth;

// Geometry, matching shared/robots/wheeled_robot.arow.
constexpr float kWheelRadius = 0.1F;
constexpr float kHalfTrack = 0.27F;  // axle center to each side's wheels

// Controller tuning.
constexpr float kCruiseSpeed = 0.8F;       // m/s along the path
constexpr float kTurnGain = 2.5F;          // rad/s of yaw rate per rad of error
constexpr float kMaxYawRate = 2.0F;        // rad/s
constexpr float kWaypointRadius = 0.3F;    // m: close enough to move on
constexpr float kWheelTorqueLimit = 3.0F;  // N*m per wheel
constexpr float kSpeedGain = 0.6F;         // N*m per rad/s of speed error
constexpr float kSpeedIntegralGain = 2.0F;

constexpr ScalarOperationT kDt = 0.004F;

// A figure-eight-ish loop on the ground plane, as (x, z) points.
constexpr std::array<std::array<float, 2>, 8> kWaypoints = {{
    {0.0F, 1.5F},
    {1.2F, 2.6F},
    {2.4F, 1.5F},
    {1.2F, 0.4F},
    {0.0F, 1.5F},
    {-1.2F, 2.6F},
    {-2.4F, 1.5F},
    {-1.2F, 0.4F},
}};

// A wheel's speed controller: drives its wheel toward a target speed with a PI
// loop, limited to what the motor can deliver, through the actuator the sim
// hands out.
class WheelMotor {
 public:
  WheelMotor(AngularActuator motor, SensorHandle<truth::JointVelocity> speed)
      : motor_(motor), speed_(speed) {}

  // The wheel's spin rate about its axle: its joint velocity's one
  // coordinate.
  float Speed() const { return speed_.Read().value.Angular().X(); }

  // One control tick: torque toward `target_speed` (rad/s) given the
  // wheel's `measured_speed`. Integrates only while unsaturated, so a long
  // stall doesn't wind up a huge correction.
  void Drive(float target_speed, float measured_speed, float dt) {
    float error = target_speed - measured_speed;
    float torque = (kSpeedGain * error) + (kSpeedIntegralGain * integral_);
    if (std::abs(torque) < kWheelTorqueLimit) {
      integral_ += error * dt;
    }
    motor_.Apply(std::clamp(torque, -kWheelTorqueLimit, kWheelTorqueLimit));
  }

 private:
  AngularActuator motor_;
  SensorHandle<truth::JointVelocity> speed_;
  float integral_ = 0.0F;
};

struct Pose {
  float x;
  float z;
  float heading;  // yaw about +y; 0 faces +z
};

Pose ChassisPose(const SensorHandle<truth::WorldTransform>& sensor) {
  auto world = sensor.Read().value;
  const auto& q = world.Rotation();
  // The chassis's local +z (its forward) in the world: R * z.
  float forward_x = 2.0F * ((q.X() * q.Z()) + (q.W() * q.Y()));
  float forward_z = 1.0F - (2.0F * ((q.X() * q.X()) + (q.Y() * q.Y())));
  return {
      world.Translation().X(),
      world.Translation().Z(),
      std::atan2(forward_x, forward_z)
  };
}

float WrapAngle(float angle) {
  constexpr float kPi = std::numbers::pi_v<float>;
  while (angle > kPi) {
    angle -= 2.0F * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0F * kPi;
  }
  return angle;
}

// Steers toward the current waypoint and moves on once it's reached: aim
// the heading at it, and slow down while the heading is far off so the
// robot turns instead of swinging wide.
class WaypointFollower {
 public:
  struct Command {
    float speed;     // m/s forward
    float yaw_rate;  // rad/s about +y
  };

  Command Update(const Pose& pose) {
    const auto& target = kWaypoints[next_];
    float dx = target[0] - pose.x;
    float dz = target[1] - pose.z;
    if (std::hypot(dx, dz) < kWaypointRadius) {
      next_ = (next_ + 1) % kWaypoints.size();
      ++reached_;
      return Update(pose);
    }
    float error = WrapAngle(std::atan2(dx, dz) - pose.heading);
    float speed = kCruiseSpeed * std::max(0.0F, std::cos(error));
    float yaw_rate = std::clamp(kTurnGain * error, -kMaxYawRate, kMaxYawRate);
    return {speed, yaw_rate};
  }

  std::size_t Reached() const { return reached_; }

 private:
  std::size_t next_ = 0;
  std::size_t reached_ = 0;
};

struct Wheels {
  WheelMotor left_front;
  WheelMotor left_rear;
  WheelMotor right_front;
  WheelMotor right_rear;
};

WheelMotor AttachWheel(const Simulation& sim, const std::string& name) {
  auto speed = sim.Sensor<truth::JointVelocity>(name + "_speed");
  return {sim.Actuator<truth::Torque>(name + "_motor"), speed};
}

// Skid steering: a yaw rate w about +y moves a wheel at lateral offset x
// backward by w * x, so the wheels on each side get v -/+ w * kHalfTrack.
// Rolling without slip, a wheel spinning at +s about its axle moves its
// axle forward (+z) at s * kWheelRadius.
void DriveWheels(
    Wheels& wheels, const WaypointFollower::Command& command, float dt
) {
  float left = (command.speed - (command.yaw_rate * kHalfTrack)) / kWheelRadius;
  float right =
      (command.speed + (command.yaw_rate * kHalfTrack)) / kWheelRadius;
  for (WheelMotor* wheel : {&wheels.left_front, &wheels.left_rear}) {
    wheel->Drive(left, wheel->Speed(), dt);
  }
  for (WheelMotor* wheel : {&wheels.right_front, &wheels.right_rear}) {
    wheel->Drive(right, wheel->Speed(), dt);
  }
}

struct Args {
  std::string arow_path;
  std::string config_path;
  bool headless = false;
  float seconds = 0.0F;  // 0: run until stopped
};

bool ParseArgs(int argc, char** argv, Args& args) {
  std::span<char*> argv_span(argv, static_cast<std::size_t>(argc));
  if (argv_span.size() < 3) {
    return false;
  }
  args.arow_path = argv_span[1];
  args.config_path = argv_span[2];
  for (std::size_t i = 3; i < argv_span.size(); ++i) {
    std::string_view arg = argv_span[i];
    if (arg == "--headless") {
      args.headless = true;
    } else if (arg == "--seconds" && i + 1 < argv_span.size()) {
      args.seconds = std::strtof(argv_span[++i], nullptr);
    } else {
      return false;
    }
  }
  if (args.headless && args.seconds <= 0.0F) {
    args.seconds = 30.0F;
  }
  return true;
}

}  // namespace

// The whole demo; throws on a bad scene (a missing joint, say).
int Run(const Args& args) {
  Simulation sim;
  if (!sim.LoadArowFile(args.arow_path) ||
      !sim.LoadConfigFile(args.config_path) || !sim.Init()) {
    return 1;
  }
  // Semi-implicit Euler: the integrator with collision.
  sim.SetIntegrator(Simulation::Integrator::kSemiImplicitEuler);

  const auto chassis = sim.Sensor<truth::WorldTransform>("chassis_pose");
  Wheels wheels{
      AttachWheel(sim, "left_front"),
      AttachWheel(sim, "left_rear"),
      AttachWheel(sim, "right_front"),
      AttachWheel(sim, "right_rear"),
  };
  WaypointFollower follower;

  // One control tick per physics tick: read the state the last Step left,
  // command the wheels, step. The actuators' torque applies to exactly the
  // Step that follows.
  float sim_time = 0.0F;
  float next_report = 0.0F;
  auto tick = [&] {
    Pose pose = ChassisPose(chassis);
    DriveWheels(wheels, follower.Update(pose), kDt);
    bool ok = sim.Step(kDt);
    sim_time += kDt;
    if (args.headless && sim_time >= next_report) {
      std::cout << std::fixed << std::setprecision(2) << "t=" << std::setw(5)
                << sim_time << "s  x=" << std::setw(6) << pose.x
                << "  z=" << std::setw(6) << pose.z
                << "  heading=" << std::setw(7)
                << pose.heading * 180.0F /
                       std::numbers::pi_v<float> << " deg  waypoints="
                << follower.Reached() << '\n';
      next_report += 1.0F;
    }
    return ok;
  };

  if (args.headless) {
    while (sim_time < args.seconds) {
      if (!tick()) {
        return 1;
      }
    }
    return 0;
  }

  // Real time, with what the actuators do to the robot: the loop from above,
  // the torque each wheel's motor applied and the speed it got the wheel to
  // (each four on one plot), and where that put the chassis.
  achilles::examples::RunInRealTime(
      "Achilles: waypoint robot",
      kDt,
      [&](achilles::display::Renderer& renderer) {
        using achilles::display::ReadingView;
        using achilles::display::EffortChannel;
        using achilles::display::MotionChannel;
        using achilles::display::PoseChannel;
        constexpr std::array<std::string_view, 4> kWheels = {
            "left front", "left rear", "right front", "right rear"
        };
        renderer.Attach(
            achilles::display::CameraView(
                sim.Sensor<achilles::domain::sensing::Camera>(
                    "waypoint_robot_camera"
                )
            ),
            "Overhead camera"
        );
        renderer.Attach(
            ReadingView<truth::Torque, 4>(
                {sim.Actuator<truth::Torque>("left_front_motor"),
                 sim.Actuator<truth::Torque>("left_rear_motor"),
                 sim.Actuator<truth::Torque>("right_front_motor"),
                 sim.Actuator<truth::Torque>("right_rear_motor")},
                {.channels = {EffortChannel::kEffort}, .names = kWheels}
            ),
            "Wheel torques"
        );
        renderer.Attach(
            ReadingView<truth::JointVelocity, 4>(
                {sim.Sensor<truth::JointVelocity>("left_front_speed"),
                 sim.Sensor<truth::JointVelocity>("left_rear_speed"),
                 sim.Sensor<truth::JointVelocity>("right_front_speed"),
                 sim.Sensor<truth::JointVelocity>("right_rear_speed")},
                {.channels = {MotionChannel::kAngularX}, .names = kWheels}
            ),
            "Wheel speeds"
        );
        // On the ground (+y up): where it is across it, and its heading.
        renderer.Attach(
            ReadingView<truth::WorldTransform>(
                sim.Sensor<truth::WorldTransform>("chassis_pose"),
                {.channels =
                     {PoseChannel::kX, PoseChannel::kZ, PoseChannel::kTurnY}}
            ),
            "Chassis pose"
        );
      },
      tick,
      [&] { return args.seconds > 0.0F && sim_time >= args.seconds; }
  );
  return 0;
}

int main(int argc, char** argv) {
  Args args;
  if (!ParseArgs(argc, argv, args)) {
    std::cerr
        << "usage: waypoint_robot <waypoint_robot.arow> <sim_config.yaml> "
           "[--headless] [--seconds N]\n";
    return 1;
  }
  try {
    return Run(args);
  } catch (const std::exception& e) {
    std::cerr << "waypoint_robot: " << e.what() << '\n';
    return 1;
  }
}
