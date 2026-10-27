// The balancing demo: eight two-wheeled robots (balancer.arow, 8 copies),
// each starting tipped 5 degrees forward and catching itself with the same
// PD loop on its body's pitch. Then, left to right, a box (box.arow, one
// copy per robot) drops onto each robot's tray -- from higher up each time,
// and further off center, alternately in front and behind -- and the robot
// has to absorb the hit, then lean away from the off-center load to keep
// balancing with the box riding on top.
// Everything control-related lives in this file -- the sim only ever sees
// torques through the actuators it hands out.
//
// Usage: balancers <balancers.arow> <sim_config.yaml> [--headless]
//                  [--seconds N] [--csv PATH]
// (scripts/run-balancers.sh passes this folder's files). Headless runs
// print each robot's pitch every half simulated second instead of drawing
// it; --csv also writes every robot's pitch and box's height every 10 ms,
// for plotting.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numbers>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "algorithms/aba/aba_data.hpp"
#include "algorithms/aba/aba_step.hpp"
#include "algorithms/conventions.hpp"
#include "domain/archetype.hpp"
#include "engine/topology/ordering_policy.hpp"
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

constexpr std::size_t kRobotCount = 8;  // balancer.arow's copies

// Geometry, matching balancer.arow.
constexpr float kWheelRadius = 0.1F;
constexpr float kRobotSpacing = 0.6F;  // m between neighbors along +x

// Controller tuning. Each wheel gets
//   tau = kp * pitch + kd * pitch_rate + kSpeedGain * v + kPositionGain * z
// where v and z are the robot's speed and distance along +z from where it
// started (z = 0). The speed and position terms keep a balancing robot from
// rolling away. Their sign is the counterintuitive one: to stop, a robot
// moving forward must first lean back, and the only way to lean back is to
// drive forward harder.
constexpr float kPitchGain = 10.0F;        // N*m per rad
constexpr float kPitchRateGain = 1.2F;     // N*m per rad/s
constexpr float kSpeedGain = 1.5F;         // N*m per m/s
constexpr float kPositionGain = 0.5F;      // N*m per m
constexpr float kWheelTorqueLimit = 2.5F;  // N*m per wheel
// Past this tilt a robot has fallen; its motors cut out, like a real
// robot's tip-over cutoff, instead of spinning it across the floor.
constexpr float kFallenPitch = 60.0F * std::numbers::pi_v<float> / 180.0F;

// Box i is held above robot i until kFirstDrop + i * kDropInterval, then
// let go. It's held kDropHeights[i] above the robot's tray (kTrayHeight
// up), shifted kDropOffsets[i] along z off its center.
constexpr float kTrayHeight = 0.74F;
constexpr float kFirstDrop = 1.5F;
constexpr float kDropInterval = 0.75F;
constexpr std::array<float, kRobotCount> kDropHeights = {
    0.1F, 0.25F, 0.4F, 0.55F, 0.7F, 0.85F, 1.0F, 1.2F
};
constexpr std::array<float, kRobotCount> kDropOffsets = {
    0.0F, 0.03F, -0.05F, 0.07F, -0.08F, 0.1F, -0.11F, 0.12F
};

constexpr ScalarOperationT kDt = 0.004F;

// The row each joint lives at in ABA's Views, which TopologicalOrdering
// orders.
std::size_t Row(const Simulation& sim, const JointAddress& joint) {
  return sim.RowFor<TopologicalOrdering>(joint);
}

// Moves a free joint's body to `position` (world, since every free joint
// here hangs off the world root), keeping its rotation, and stops it.
// .arow copies all start from the same data, so this is how the example
// spreads them out -- and how it holds each box until its drop.
void Place(const Simulation& sim, std::size_t row, float x, float y, float z) {
  auto view = sim.ViewFor<ABAAlgorithm>();
  auto pose = view.Load<ABAField::kJointPosition, ScalarOperationT>(row);
  const auto& q = pose.Rotation();
  view.Store<ABAField::kJointPosition, ScalarOperationT>(
      row, decltype(pose)(x, y, z, q.W(), q.X(), q.Y(), q.Z())
  );
  view.Store<ABAField::kJointVelocity, ScalarOperationT>(row, {});
}

// One robot: its body's row, its two wheel motors, and its PD loop.
class Balancer {
 public:
  Balancer(const Simulation& sim, std::size_t index)
      : body_(Row(sim, {"balancer", "body", index})),
        left_row_(Row(sim, {"balancer", "left_wheel", index})),
        right_row_(Row(sim, {"balancer", "right_wheel", index})),
        left_(Attach(sim, "left_wheel", index)),
        right_(Attach(sim, "right_wheel", index)) {}

  std::size_t Body() const { return body_; }

  // Pitch toward +z, radians: the body's up axis R * y leaning over.
  float Pitch(const Simulation& sim) const {
    auto world = sim.ViewFor<ABAAlgorithm>()
                     .Load<ABAField::kWorldTransform, ScalarOperationT>(body_);
    const auto& q = world.Rotation();
    float up_y = 1.0F - (2.0F * ((q.X() * q.X()) + (q.Z() * q.Z())));
    float up_z = 2.0F * ((q.Y() * q.Z()) + (q.W() * q.X()));
    return std::atan2(up_z, up_y);
  }

  bool Fallen(const Simulation& sim) const {
    return std::abs(Pitch(sim)) > kFallenPitch;
  }

  // One control tick: read pitch, pitch rate, speed and distance, and
  // apply the same clamped torque to both wheels.
  void Control(const Simulation& sim) {
    float pitch = Pitch(sim);
    if (std::abs(pitch) > kFallenPitch) {
      return;
    }
    auto view = sim.ViewFor<ABAAlgorithm>();
    // The body only pitches (it never yaws: both wheels always get the
    // same torque), so its angular velocity about its own X is the pitch
    // rate.
    float pitch_rate =
        view.Load<ABAField::kJointVelocity, ScalarOperationT>(body_)
            .Angular()
            .X();
    // A wheel's spin over the ground is its own joint's spin plus the
    // body's; rolling without slip, the axle moves at that times r.
    float wheel_spin =
        0.5F *
        (view.Load<ABAField::kJointVelocity, ScalarOperationT>(left_row_)
             .Angular()
             .X() +
         view.Load<ABAField::kJointVelocity, ScalarOperationT>(right_row_)
             .Angular()
             .X());
    float speed = (wheel_spin + pitch_rate) * kWheelRadius;
    float z = view.Load<ABAField::kWorldTransform, ScalarOperationT>(body_)
                  .Translation()
                  .Z();

    float torque = (kPitchGain * pitch) + (kPitchRateGain * pitch_rate) +
                   (kSpeedGain * speed) + (kPositionGain * z);
    torque = std::clamp(torque, -kWheelTorqueLimit, kWheelTorqueLimit);
    left_.Apply(torque);
    right_.Apply(torque);
  }

 private:
  // Each wheel turns about its own axle, local X.
  static AngularActuator Attach(
      const Simulation& sim, const char* wheel, std::size_t index
  ) {
    return sim.AttachActuator<AngularActuator>(
        {"balancer", wheel, index}, {1, 0, 0}
    );
  }

  std::size_t body_;
  std::size_t left_row_;
  std::size_t right_row_;
  AngularActuator left_;
  AngularActuator right_;
};

float RobotX(std::size_t index) {
  return (static_cast<float>(index) -
          (static_cast<float>(kRobotCount - 1) / 2.0F)) *
         kRobotSpacing;
}

struct Args {
  std::string arow_path;
  std::string config_path;
  std::string csv_path;
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
    } else if (arg == "--csv" && i + 1 < argv_span.size()) {
      args.csv_path = argv_span[++i];
    } else {
      return false;
    }
  }
  if (args.headless && args.seconds <= 0.0F) {
    args.seconds = 10.0F;
  }
  return true;
}

constexpr float RadToDeg(float radians) {
  return radians * 180.0F / std::numbers::pi_v<float>;
}

// Holds each box above its tray until its drop time.
void HoldUndroppedBoxes(
    Simulation& sim, const std::vector<std::size_t>& boxes, float sim_time
) {
  for (std::size_t i = 0; i < kRobotCount; ++i) {
    float drop_time = kFirstDrop + (static_cast<float>(i) * kDropInterval);
    if (sim_time < drop_time) {
      Place(
          sim,
          boxes[i],
          RobotX(i),
          kTrayHeight + kDropHeights[i],
          kDropOffsets[i]
      );
    }
  }
}

void WriteCsvHeader(std::ofstream& csv) {
  csv << "t";
  for (std::size_t i = 0; i < kRobotCount; ++i) {
    csv << ",pitch_deg_" << i;
  }
  for (std::size_t i = 0; i < kRobotCount; ++i) {
    csv << ",box_y_" << i;
  }
  csv << '\n';
}

void WriteCsvRow(
    std::ofstream& csv,
    const Simulation& sim,
    const std::vector<Balancer>& robots,
    const std::vector<std::size_t>& boxes,
    float sim_time
) {
  csv << sim_time;
  for (const Balancer& robot : robots) {
    csv << ',' << RadToDeg(robot.Pitch(sim));
  }
  for (std::size_t box : boxes) {
    csv << ','
        << sim.ViewFor<ABAAlgorithm>()
               .Load<ABAField::kWorldTransform, ScalarOperationT>(box)
               .Translation()
               .Y();
  }
  csv << '\n';
}

void PrintPitches(
    const Simulation& sim, const std::vector<Balancer>& robots, float sim_time
) {
  std::cout << std::fixed << std::setprecision(1) << "t=" << std::setw(5)
            << sim_time << "s  pitch (deg):";
  for (const Balancer& robot : robots) {
    std::cout << std::setw(7) << RadToDeg(robot.Pitch(sim));
  }
  std::cout << '\n';
}

// Real time, the same fixed-step accumulator src/main.cpp uses. Runs until
// the window closes, a tick fails, or sim_time (advanced by tick) reaches
// seconds; seconds <= 0 runs until the window closes.
template <typename TickT>
void RunRealTime(
    Simulation& sim, float seconds, const float& sim_time, TickT& tick
) {
  constexpr float kMaxFrameTime = 0.25F;
  float accumulator = 0.0F;
  auto last_time = std::chrono::steady_clock::now();
  bool running = true;
  while (running && (seconds <= 0.0F || sim_time < seconds)) {
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

  std::vector<Balancer> robots;
  robots.reserve(kRobotCount);
  for (std::size_t i = 0; i < kRobotCount; ++i) {
    robots.emplace_back(sim, i);
    float y = sim.ViewFor<ABAAlgorithm>()
                  .Load<ABAField::kJointPosition, ScalarOperationT>(
                      robots.back().Body()
                  )
                  .Translation()
                  .Y();
    Place(sim, robots.back().Body(), RobotX(i), y, 0.0F);
  }
  std::vector<std::size_t> boxes;
  boxes.reserve(kRobotCount);
  for (std::size_t i = 0; i < kRobotCount; ++i) {
    boxes.push_back(Row(sim, {"box", "box", i}));
  }

  std::ofstream csv;
  if (!args.csv_path.empty()) {
    csv.open(args.csv_path);
    WriteCsvHeader(csv);
  }

  // One control tick per physics tick: read the state the last Step left,
  // command the wheels, step. The actuators' torque applies to exactly the
  // Step that follows.
  float sim_time = 0.0F;
  float next_report = 0.0F;
  float next_sample = 0.0F;
  auto tick = [&] {
    HoldUndroppedBoxes(sim, boxes, sim_time);
    for (Balancer& robot : robots) {
      robot.Control(sim);
    }
    if (csv.is_open() && sim_time >= next_sample) {
      WriteCsvRow(csv, sim, robots, boxes, sim_time);
      next_sample += 0.01F;
    }
    if (args.headless && sim_time >= next_report) {
      PrintPitches(sim, robots, sim_time);
      next_report += 0.5F;
    }
    bool ok = sim.Step(kDt);
    sim_time += kDt;
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

  RunRealTime(sim, args.seconds, sim_time, tick);
  return 0;
}

int main(int argc, char** argv) {
  Args args;
  if (!ParseArgs(argc, argv, args)) {
    std::cerr << "usage: balancers <balancers.arow> <sim_config.yaml> "
                 "[--headless] [--seconds N] [--csv PATH]\n";
    return 1;
  }
  try {
    return Run(args);
  } catch (const std::exception& e) {
    std::cerr << "balancers: " << e.what() << '\n';
    return 1;
  }
}
