// The actuator demo: a four-wheeled box robot (box_robot.arow) that drives
// itself around a loop of waypoints. Everything control-related lives in
// this file -- the sim only ever sees torques through the actuators it
// hands out.
//
// Usage: box_robot <box_robot.arow> <sim_config.yaml> [--headless]
//                  [--seconds N]
// (scripts/run-box-robot.sh passes this folder's files). Headless runs
// print the robot's pose once per simulated second instead of drawing it.
#include <algorithm>
#include <array>
#include <chrono>
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
#include <utility>

#include "algorithms/aba/aba_data.hpp"
#include "algorithms/aba/aba_step.hpp"
#include "algorithms/conventions.hpp"
#include "domain/archetype.hpp"
#include "engine/topology/orderings.hpp"
#include "interface/actuator.hpp"
#include "interface/simulation.hpp"

namespace {

using achilles::algorithms::ScalarOperationT;
using achilles::algorithms::aba::ABAAlgorithm;
using achilles::algorithms::aba::ABAField;
using achilles::engine::topology::TopologicalOrdering;
using achilles::interface::AngularActuator;
using achilles::interface::JointAddress;
using achilles::interface::Simulation;

// Geometry, matching box_robot.arow.
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

// A wheel's motor: drives its wheel toward a target speed with a PI loop,
// limited to what the motor can deliver. A model of a real actuator,
// built on the AngularActuator the sim hands out.
class WheelMotor : public AngularActuator {
 public:
  WheelMotor(AngularActuator base, std::size_t row)
      : AngularActuator(std::move(base)), row_(row) {}

  std::size_t Row() const { return row_; }

  // One control tick: torque toward `target_speed` (rad/s) given the
  // wheel's `measured_speed`. Integrates only while unsaturated, so a long
  // stall doesn't wind up a huge correction.
  void Drive(float target_speed, float measured_speed, float dt) {
    float error = target_speed - measured_speed;
    float torque = (kSpeedGain * error) + (kSpeedIntegralGain * integral_);
    if (std::abs(torque) < kWheelTorqueLimit) {
      integral_ += error * dt;
    }
    Apply(std::clamp(torque, -kWheelTorqueLimit, kWheelTorqueLimit));
  }

 private:
  std::size_t row_;
  float integral_ = 0.0F;
};

struct Pose {
  float x;
  float z;
  float heading;  // yaw about +y; 0 faces +z
};

Pose ChassisPose(const Simulation& sim, std::size_t row) {
  auto world = sim.ViewFor<ABAAlgorithm>()
                   .Load<ABAField::kWorldTransform, ScalarOperationT>(row);
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

// A wheel's spin rate about its axle: its joint velocity's one coordinate.
float WheelSpeed(const Simulation& sim, std::size_t row) {
  return sim.ViewFor<ABAAlgorithm>()
      .Load<ABAField::kJointVelocity, ScalarOperationT>(row)
      .Angular()
      .X();
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

// The row each joint lives at in ABA's Views, which TopologicalOrdering
// orders.
std::size_t Row(const Simulation& sim, const JointAddress& joint) {
  return sim.RowFor<TopologicalOrdering>(joint);
}

WheelMotor AttachWheel(const Simulation& sim, std::string name) {
  JointAddress joint{"box_robot", std::move(name)};
  // Each wheel turns about its own axle, local X.
  return sim.AttachActuator<WheelMotor>(joint, {1, 0, 0}, Row(sim, joint));
}

// Skid steering: a yaw rate w about +y moves a wheel at lateral offset x
// backward by w * x, so the wheels on each side get v -/+ w * kHalfTrack.
// Rolling without slip, a wheel spinning at +s about its axle moves its
// axle forward (+z) at s * kWheelRadius.
void DriveWheels(
    Wheels& wheels,
    const Simulation& sim,
    const WaypointFollower::Command& command,
    float dt
) {
  float left = (command.speed + (command.yaw_rate * kHalfTrack)) / kWheelRadius;
  float right =
      (command.speed - (command.yaw_rate * kHalfTrack)) / kWheelRadius;
  for (WheelMotor* wheel : {&wheels.left_front, &wheels.left_rear}) {
    wheel->Drive(left, WheelSpeed(sim, wheel->Row()), dt);
  }
  for (WheelMotor* wheel : {&wheels.right_front, &wheels.right_rear}) {
    wheel->Drive(right, WheelSpeed(sim, wheel->Row()), dt);
  }
}

struct Args {
  std::string arow_path;
  std::string config_path;
  bool headless = false;
  float seconds = 0.0F;  // 0: run until the window closes
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

  const std::size_t chassis = Row(sim, {"box_robot", "chassis"});
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
    Pose pose = ChassisPose(sim, chassis);
    DriveWheels(wheels, sim, follower.Update(pose), kDt);
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

  // Real time, the same fixed-step accumulator src/main.cpp uses.
  constexpr float kMaxFrameTime = 0.25F;
  float accumulator = 0.0F;
  auto last_time = std::chrono::steady_clock::now();
  bool running = true;
  while (running && (args.seconds <= 0.0F || sim_time < args.seconds)) {
    auto now = std::chrono::steady_clock::now();
    accumulator += std::min(
        std::chrono::duration<float>(now - last_time).count(), kMaxFrameTime
    );
    last_time = now;
    while (running && accumulator >= kDt) {
      running = tick();
      accumulator -= kDt;
    }
    if (running) {
      running = sim.Render();
    }
  }
  return 0;
}

int main(int argc, char** argv) {
  Args args;
  if (!ParseArgs(argc, argv, args)) {
    std::cerr << "usage: box_robot <box_robot.arow> <sim_config.yaml> "
                 "[--headless] [--seconds N]\n";
    return 1;
  }
  try {
    return Run(args);
  } catch (const std::exception& e) {
    std::cerr << "box_robot: " << e.what() << '\n';
    return 1;
  }
}
