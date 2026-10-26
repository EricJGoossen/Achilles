// The balancing demo: eight two-wheeled robots (balancer.arow, 8 copies),
// each starting tipped 5 degrees forward, each running the same PD loop on
// its body's pitch but with its own gains -- from too soft to hold itself
// up to well tuned. Partway through, a box (box.arow) is thrown at one of
// the well-tuned robots. Everything control-related lives in this file -- the
// sim only ever sees torques through the actuators it hands out.
//
// Usage: balancers <balancers.arow> <sim_config.yaml> [--headless]
//                  [--seconds N] [--csv PATH]
// (scripts/run-balancers.sh passes this folder's files). Headless runs
// print each robot's pitch every half simulated second instead of drawing
// it; --csv also writes every robot's pitch every 10 ms, for plotting.
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
#include <utility>
#include <vector>

#include "algorithms/aba/aba_data.hpp"
#include "algorithms/aba/aba_step.hpp"
#include "algorithms/conventions.hpp"
#include "algorithms/registry.hpp"
#include "domain/archetype.hpp"
#include "engine/memory/sim_allocator.hpp"
#include "engine/topology/ordering_plan.hpp"
#include "engine/topology/ordering_policy.hpp"
#include "interface/actuator.hpp"
#include "interface/archetype_loader.hpp"
#include "interface/simulation.hpp"

namespace {

using achilles::algorithms::ScalarOperationT;
using achilles::algorithms::aba::ABAAlgorithm;
using achilles::algorithms::aba::ABAField;
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
// started (z = 0). Only kp and kd vary from robot to robot; the speed and
// position terms keep a balancing robot from rolling away. Their sign is
// the counterintuitive one: to stop, a robot moving forward must first
// lean back, and the only way to lean back is to drive forward harder.
constexpr float kSpeedGain = 1.5F;         // N*m per m/s
constexpr float kPositionGain = 0.5F;      // N*m per m
constexpr float kWheelTorqueLimit = 2.5F;  // N*m per wheel
// Past this tilt a robot has fallen; its motors cut out, like a real
// robot's tip-over cutoff, instead of spinning it across the floor.
constexpr float kFallenPitch = 60.0F * std::numbers::pi_v<float> / 180.0F;

struct Gains {
  float kp;  // N*m per rad of pitch
  float kd;  // N*m per rad/s of pitch rate
};

// Left to right along +x. Gravity tips the body with about 8.8 N*m per rad
// (m g l = 3 * 9.81 * 0.3), shared by two wheels, so kp has to beat about
// 4.4 per wheel; too little kd and every catch overshoots into a growing
// wobble. From the left: one too soft and two too undamped, all down
// within 2 s; one right at the edge whose wobble grows until it falls at
// about 4 s; one just inside it that keeps wobbling about 5 degrees
// without ever settling; and three that settle within half a second.
constexpr std::array<Gains, kRobotCount> kGains = {{
    {3.0F, 0.6F},
    {10.0F, 0.0F},
    {10.0F, 0.35F},
    {4.0F, 0.6F},
    {4.5F, 0.6F},
    {5.0F, 1.2F},
    {10.0F, 1.2F},
    {20.0F, 1.2F},
}};

// The box is held kThrowDistance in front of (-z of) this robot until
// kThrowTime, then thrown at it: up and forward at kThrowSpeed (y, z), so
// it arcs into the upper body about 0.3 s later.
constexpr std::size_t kThrowTarget = 6;
constexpr float kThrowTime = 5.0F;
constexpr float kThrowHeight = 0.55F;
constexpr float kThrowDistance = 1.2F;
constexpr std::array<float, 2> kThrowSpeed = {1.5F, 4.0F};

constexpr ScalarOperationT kDt = 0.004F;

// Which row of the sim's Views each joint lives at. Simulation doesn't hand
// that out, so the example rebuilds the same row order ABA's fields use:
// the TopologicalOrdering plan over the same archetypes and lane width the
// sim's allocator was built with (SimAllocator::BuildPlans does exactly
// this).
class JointRows {
 public:
  explicit JointRows(std::vector<achilles::domain::Archetype> archetypes)
      : archetypes_(std::move(archetypes)), plan_(Plan(archetypes_)) {}

  std::size_t Row(const JointAddress& joint) const {
    return plan_.Layout().ToSorted(
        achilles::domain::ResolveJoint(archetypes_, joint)
    );
  }

 private:
  static achilles::engine::topology::OrderingPlan Plan(
      std::span<const achilles::domain::Archetype> archetypes
  ) {
    using Allocator = achilles::engine::memory::SimAllocatorForT<
        achilles::algorithms::RegisteredAlgorithms>;
    std::vector<achilles::domain::ArchetypeTreeStructure> tree;
    tree.reserve(archetypes.size());
    for (const achilles::domain::Archetype& archetype : archetypes) {
      tree.push_back(archetype.TreeStructure());
    }
    return achilles::engine::topology::TopologicalOrdering::Build(
        tree, Allocator::RequiredLaneSize()
    );
  }

  std::vector<achilles::domain::Archetype> archetypes_;
  achilles::engine::topology::OrderingPlan plan_;
};

// Moves a free joint's body to `position` (world, since every free joint
// here hangs off the world root), keeping its rotation, and stops it.
// .arow copies all start from the same data, so this is how the example
// spreads them out -- and how it holds the box until the throw.
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
  Balancer(const Simulation& sim, const JointRows& rows, std::size_t index)
      : gains_(kGains[index]),
        body_(rows.Row({"balancer", "body", index})),
        left_row_(rows.Row({"balancer", "left_wheel", index})),
        right_row_(rows.Row({"balancer", "right_wheel", index})),
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

    float torque = (gains_.kp * pitch) + (gains_.kd * pitch_rate) +
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

  Gains gains_;
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
    args.seconds = 12.0F;
  }
  return true;
}

constexpr float RadToDeg(float radians) {
  return radians * 180.0F / std::numbers::pi_v<float>;
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

  const JointRows rows(achilles::interface::LoadArchetypes(args.arow_path));
  std::vector<Balancer> robots;
  robots.reserve(kRobotCount);
  for (std::size_t i = 0; i < kRobotCount; ++i) {
    robots.emplace_back(sim, rows, i);
    float y = sim.ViewFor<ABAAlgorithm>()
                  .Load<ABAField::kJointPosition, ScalarOperationT>(
                      robots.back().Body()
                  )
                  .Translation()
                  .Y();
    Place(sim, robots.back().Body(), RobotX(i), y, 0.0F);
  }
  const std::size_t box = rows.Row({"box", "box"});

  std::ofstream csv;
  if (!args.csv_path.empty()) {
    csv.open(args.csv_path);
    csv << "t";
    for (std::size_t i = 0; i < kRobotCount; ++i) {
      csv << ",pitch_deg_" << i;
    }
    csv << '\n';
  }

  // One control tick per physics tick: read the state the last Step left,
  // command the wheels, step. The actuators' torque applies to exactly the
  // Step that follows.
  float sim_time = 0.0F;
  float next_report = 0.0F;
  float next_sample = 0.0F;
  bool thrown = false;
  auto tick = [&] {
    if (sim_time < kThrowTime) {
      Place(sim, box, RobotX(kThrowTarget), kThrowHeight, -kThrowDistance);
    } else if (!thrown) {
      // The box is unrotated, so its own frame's linear velocity is the
      // world's.
      sim.ViewFor<ABAAlgorithm>()
          .Store<ABAField::kJointVelocity, ScalarOperationT>(
              box, {0.0F, 0.0F, 0.0F, 0.0F, kThrowSpeed[0], kThrowSpeed[1]}
          );
      thrown = true;
    }
    for (Balancer& robot : robots) {
      robot.Control(sim);
    }
    if (csv.is_open() && sim_time >= next_sample) {
      csv << sim_time;
      for (const Balancer& robot : robots) {
        csv << ',' << RadToDeg(robot.Pitch(sim));
      }
      csv << '\n';
      next_sample += 0.01F;
    }
    if (args.headless && sim_time >= next_report) {
      std::cout << std::fixed << std::setprecision(1) << "t=" << std::setw(5)
                << sim_time << "s  pitch (deg):";
      for (const Balancer& robot : robots) {
        std::cout << std::setw(7) << RadToDeg(robot.Pitch(sim));
      }
      std::cout << '\n';
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
