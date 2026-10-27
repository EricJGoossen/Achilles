// The ball-on-plate demo: a plate (plate_rig.arow) on a two-axis pivot
// driven by two motors, and a ball (ball.arow) on it. The plate starts
// tipped steeply with the ball at rest on its high corner, and is held
// there for a moment while the ball rolls downhill; then the controller
// takes over, has to stop the ball before it runs off the far edge, bring
// it to the center, and finally roll it around a figure-8. A ball on a
// tilted plate rolls downhill, so the only way to keep it is to keep
// tipping the plate under it.
//
// What makes it hard is what makes a real one hard: the controller never
// sees the ball directly, only through a camera-like sensor (Camera) that
// reports its position 50 times a second, 40 ms late, with a little noise,
// and the plate can only tilt as fast as a hobby servo would let it
// (kTiltRateLimit). Its velocity estimate, differenced from those noisy,
// late samples, is what keeps the plate twitching.
//
// Everything control-related lives in this file -- the sim only ever sees
// torques through the actuators it hands out.
//
// Usage: ball_balance <ball_balance.arow> <sim_config.yaml> [--headless]
//                     [--seconds N] [--csv PATH]
// (scripts/run-ball-balance.sh passes this folder's files). Headless runs
// print the ball's distance from its target and the plate's tilt every
// half simulated second instead of drawing it; --csv also writes the
// ball's true and measured position, its target, and the plate's tilt
// every 10 ms, for plotting.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numbers>
#include <random>
#include <span>
#include <string>
#include <string_view>

#include "algorithms/aba/aba_data.hpp"
#include "algorithms/aba/aba_step.hpp"
#include "algorithms/conventions.hpp"
#include "engine/topology/ordering_policy.hpp"
#include "interface/actuator.hpp"
#include "interface/simulation.hpp"
#include "util/tmp.hpp"

namespace {

using achilles::algorithms::ScalarOperationT;
using achilles::algorithms::aba::ABAAlgorithm;
using achilles::algorithms::aba::ABAField;
using achilles::algorithms::aba::ABAStep;
using achilles::engine::topology::TopologicalOrdering;
using achilles::interface::AngularActuator;
using achilles::interface::JointAddress;
using achilles::interface::Simulation;

constexpr float kPi = std::numbers::pi_v<float>;
constexpr float kRadToDeg = 180.0F / kPi;

// Geometry, matching plate_rig.arow and ball.arow.
constexpr float kPivotHeight = 0.5F;
constexpr float kPlateThickness = 0.02F;
constexpr float kBallRadius = 0.08F;

// The start: the plate tipped kStartTilt (about X, about Z) -- its -z, +x
// corner high -- with the ball at rest at kStartSpot (x, z) on it, near
// that corner. The plate is held there until kHoldTime, so the ball is
// already rolling fast toward the far corner when the controller takes
// over.
constexpr std::array<float, 2> kStartTilt = {0.12F, 0.09F};
constexpr std::array<float, 2> kStartSpot = {0.14F, -0.14F};
constexpr float kHoldTime = 0.5F;

// The sensor: a camera looking down at the plate, reporting the ball's
// position on it every kSensorPeriod, kSensorLatency after it was taken,
// off by kSensorNoise (one standard deviation, each axis).
constexpr float kSensorPeriod = 1.0F / 30.0F;
constexpr float kSensorLatency = 0.05F;
constexpr float kSensorNoise = 0.002F;  // m
// How much of each new differenced velocity to blend in per sample -- a
// light low-pass, since differencing noisy positions amplifies the noise.
constexpr float kVelocitySmoothing = 0.5F;

// Outer loop: the plate tilt that steers the ball toward its target. A
// ball rolling without slip on a plate tilted by theta accelerates at
// 5/7 g sin(theta) downhill, so the tilt for an acceleration a is about
// a / (5/7 g). The plate tilts for the acceleration its target is making
// (so a moving target isn't chased from behind), plus a PD on how far the
// ball is from it, which makes the ball a damped spring about the target:
// here about 4 rad/s, 0.7 damped -- about as stiff as the sensor's delay
// allows.
constexpr float kRollingAcceleration = 5.0F / 7.0F * 9.81F;  // per rad
constexpr float kBallGain = 1.3F;      // rad of tilt per m of error
constexpr float kBallRateGain = 0.6F;  // rad of tilt per m/s
constexpr float kMaxTilt = 0.25F;      // rad, about 14 degrees
// The fastest the commanded tilt may change, like a hobby servo's speed.
constexpr float kTiltRateLimit = 3.0F;  // rad/s

// Inner loop: each pivot's motor drives its angle to the commanded tilt,
// much faster (about 25 rad/s) than the ball moves, and stiff enough that
// the ball's own weight only bends it about a degree.
constexpr float kPivotGain = 25.0F;        // N*m per rad
constexpr float kPivotRateGain = 1.5F;     // N*m per rad/s
constexpr float kPivotTorqueLimit = 8.0F;  // N*m

// From kPathStart, the ball is steered around a figure-8, kPathSize (x, z)
// across each lobe, once per kPathPeriod -- easing up from nothing over
// kPathRampTime so the target never jumps. Until then, the plate's center.
constexpr float kPathStart = 3.0F;
constexpr float kPathRampTime = 1.0F;
constexpr std::array<float, 2> kPathSize = {0.12F, 0.08F};
constexpr float kPathPeriod = 4.0F;

constexpr ScalarOperationT kDt = 0.004F;

// The row each joint lives at in ABA's Views, which TopologicalOrdering
// orders.
std::size_t Row(const Simulation& sim, const JointAddress& joint) {
  return sim.RowFor<TopologicalOrdering>(joint);
}

using Vec3 = std::array<float, 3>;

// The plate's rotation, the gimbal's about X after the plate's own about
// Z, applied to `v`.
Vec3 PlateRotate(const std::array<float, 2>& tilt, const Vec3& v) {
  float cz = std::cos(tilt[1]);
  float sz = std::sin(tilt[1]);
  Vec3 u = {(v[0] * cz) - (v[1] * sz), (v[0] * sz) + (v[1] * cz), v[2]};
  float cx = std::cos(tilt[0]);
  float sx = std::sin(tilt[0]);
  return {u[0], (u[1] * cx) - (u[2] * sx), (u[1] * sx) + (u[2] * cx)};
}

// Where the plate is steering its ball: position, velocity and acceleration
// on the plate, along x and z.
struct BallTarget {
  float x = 0.0F;
  float z = 0.0F;
  float velocity_x = 0.0F;
  float velocity_z = 0.0F;
  float acceleration_x = 0.0F;
  float acceleration_z = 0.0F;
};

// The target at `time`: the center until kPathStart, then the figure-8
// x = a sin(wt), z = b sin(2wt). (The feedforward ignores the ramp's own
// small rate of change.)
BallTarget Target(float time) {
  if (time < kPathStart) {
    return {};
  }
  float t = time - kPathStart;
  float ramp = std::min(1.0F, t / kPathRampTime);
  float a = kPathSize[0] * ramp;
  float b = kPathSize[1] * ramp;
  float w = 2.0F * kPi / kPathPeriod;
  return {
      a * std::sin(w * t),
      b * std::sin(2.0F * w * t),
      a * w * std::cos(w * t),
      2.0F * b * w * std::cos(2.0F * w * t),
      -a * w * w * std::sin(w * t),
      -4.0F * b * w * w * std::sin(2.0F * w * t),
  };
}

// The camera: samples the ball's true position on the plate every
// kSensorPeriod, adds noise, and hands each sample over only once
// kSensorLatency has passed. Seeded, so every run sees the same noise.
class Camera {
 public:
  struct Sample {
    float ready_time;
    float x;
    float z;
  };

  // Takes a new sample at `time` if one is due.
  void Observe(float time, float x, float z) {
    if (time < next_sample_) {
      return;
    }
    pending_.push_back(
        {time + kSensorLatency, x + noise_(rng_), z + noise_(rng_)}
    );
    next_sample_ += kSensorPeriod;
  }

  // The next sample ready by `time`, if any, removed from the queue.
  bool Next(float time, Sample& sample) {
    if (pending_.empty() || pending_.front().ready_time > time) {
      return false;
    }
    sample = pending_.front();
    pending_.pop_front();
    return true;
  }

 private:
  std::deque<Sample> pending_;
  float next_sample_ = 0.0F;
  std::mt19937 rng_{7};
  std::normal_distribution<float> noise_{0.0F, kSensorNoise};
};

// The plate: its rows, its two pivot motors, the camera watching its ball,
// and the two loops.
class Plate {
 public:
  explicit Plate(const Simulation& sim)
      : gimbal_(Row(sim, {"plate_rig", "gimbal"})),
        plate_(Row(sim, {"plate_rig", "plate"})),
        ball_(Row(sim, {"ball", "ball"})),
        tilt_x_(sim.AttachActuator<AngularActuator>(
            {"plate_rig", "gimbal"}, {1, 0, 0}
        )),
        tilt_z_(sim.AttachActuator<AngularActuator>(
            {"plate_rig", "plate"}, {0, 0, 1}
        )) {}

  // Tips the plate to kStartTilt and sets the ball down at rest at
  // kStartSpot on it. Call once, before the first Step.
  void SetUp(Simulation& sim) {
    SetPivot(sim, gimbal_, kXAxis, kStartTilt[0]);
    SetPivot(sim, plate_, kZAxis, kStartTilt[1]);
    command_ = kStartTilt;
    // The ball touches the plate's top face at kStartSpot, its center r
    // along the plate's normal from there; its frame (an unrotated one) is
    // r below its center.
    Vec3 contact = PlateRotate(
        kStartTilt, {kStartSpot[0], kPlateThickness, kStartSpot[1]}
    );
    Vec3 normal = PlateRotate(kStartTilt, {0.0F, 1.0F, 0.0F});
    auto view = sim.ViewFor<ABAAlgorithm>();
    auto pose = view.Load<ABAField::kJointPosition, ScalarOperationT>(ball_);
    view.Store<ABAField::kJointPosition, ScalarOperationT>(
        ball_,
        decltype(pose)(
            contact[0] + (kBallRadius * normal[0]),
            kPivotHeight + contact[1] + (kBallRadius * normal[1]) - kBallRadius,
            contact[2] + (kBallRadius * normal[2]),
            1.0F,
            0.0F,
            0.0F,
            0.0F
        )
    );
    view.Store<ABAField::kJointVelocity, ScalarOperationT>(ball_, {});
    // World transforms only follow joint positions when the dynamics run;
    // run them once, without advancing time, so the first look at the
    // ball sees where it now is.
    sim.StepWith<achilles::util::TypeList<ABAStep>>(0.0F);
  }

  // The ball's center relative to the plate's pivot, in the scene's axes: (x,
  // y, z). Close enough to the plate's own axes at these tilts.
  Vec3 BallOnPlate(const Simulation& sim) const {
    auto view = sim.ViewFor<ABAAlgorithm>();
    auto ball = view.Load<ABAField::kWorldTransform, ScalarOperationT>(ball_);
    auto pivot = view.Load<ABAField::kWorldTransform, ScalarOperationT>(plate_)
                     .Translation();
    // The ball's frame is at its bottom; its center is r along the frame's
    // own +y, R * (0, r, 0), wherever rolling has turned it.
    const auto& q = ball.Rotation();
    float up_x = 2.0F * ((q.X() * q.Y()) - (q.W() * q.Z()));
    float up_y = 1.0F - (2.0F * ((q.X() * q.X()) + (q.Z() * q.Z())));
    float up_z = 2.0F * ((q.Y() * q.Z()) + (q.W() * q.X()));
    // Then from world axes back to the scene's own (ball_balance.arow's
    // world joint turned its +y to the world's +z): world (x, y, z) is
    // scene (x, z, -y).
    float x = ball.Translation().X() + (kBallRadius * up_x) - pivot.X();
    float y = ball.Translation().Y() + (kBallRadius * up_y) - pivot.Y();
    float z = ball.Translation().Z() + (kBallRadius * up_z) - pivot.Z();
    return {x, z, -y};
  }

  // What the camera last reported: (x, z).
  std::array<float, 2> Measured() const { return {measured_x_, measured_z_}; }

  // The two pivots' angles, radians: (about X, about Z).
  std::array<float, 2> Tilts(const Simulation& sim) const {
    return {PivotAngle(sim, gimbal_, kXAxis), PivotAngle(sim, plate_, kZAxis)};
  }

  // One control tick at `time`: let the camera look, update the outer loop
  // on each sample it hands over, slew the commanded tilt toward what the
  // outer loop wants, and drive the pivots to it. Until kHoldTime, the
  // plate is just held at kStartTilt.
  void Control(const Simulation& sim, float time) {
    Vec3 ball = BallOnPlate(sim);
    camera_.Observe(time, ball[0], ball[2]);
    Camera::Sample sample{};
    while (camera_.Next(time, sample)) {
      Measure(sample);
    }

    std::array<float, 2> wanted = kStartTilt;
    if (time >= kHoldTime) {
      wanted = OuterLoop(Target(measured_time_), Target(time));
    }
    float step = kTiltRateLimit * kDt;
    for (std::size_t axis = 0; axis < 2; ++axis) {
      command_[axis] += std::clamp(wanted[axis] - command_[axis], -step, step);
    }
    Drive(sim, tilt_x_, gimbal_, command_[0], kXAxis);
    Drive(sim, tilt_z_, plate_, command_[1], kZAxis);
  }

 private:
  enum Axis : std::uint8_t { kXAxis, kZAxis };

  // Folds one camera sample into the position and velocity estimate.
  void Measure(const Camera::Sample& sample) {
    if (has_measurement_) {
      velocity_x_ += kVelocitySmoothing *
                     (((sample.x - measured_x_) / kSensorPeriod) - velocity_x_);
      velocity_z_ += kVelocitySmoothing *
                     (((sample.z - measured_z_) / kSensorPeriod) - velocity_z_);
    }
    measured_time_ = sample.ready_time - kSensorLatency;
    measured_x_ = sample.x;
    measured_z_ = sample.z;
    has_measurement_ = true;
  }

  // The tilt (about X, about Z) that steers the measured ball toward its
  // target. The measurement is a picture of the past, so it's compared
  // against `seen`, where the target was when the picture was taken -- not
  // where it is `now`, which would read the sensor's own lag as error; only
  // the feedforward, the tilt the path itself needs, uses `now`. Tilting
  // about +X by a positive angle lowers the plate's +z edge, so the ball
  // rolls toward +z; about +Z, it raises the +x edge, so the ball rolls
  // toward -x.
  std::array<float, 2> OuterLoop(const BallTarget& seen, const BallTarget& now)
      const {
    if (!has_measurement_) {
      return {0.0F, 0.0F};
    }
    float tilt_x = (now.acceleration_z / kRollingAcceleration) -
                   (kBallGain * (measured_z_ - seen.z)) -
                   (kBallRateGain * (velocity_z_ - seen.velocity_z));
    float tilt_z = -(now.acceleration_x / kRollingAcceleration) +
                   (kBallGain * (measured_x_ - seen.x)) +
                   (kBallRateGain * (velocity_x_ - seen.velocity_x));
    return {
        std::clamp(tilt_x, -kMaxTilt, kMaxTilt),
        std::clamp(tilt_z, -kMaxTilt, kMaxTilt)
    };
  }

  // A pivot joint's angle about its one axis. A rotation by angle a about
  // one axis is the quaternion (cos a/2, sin a/2 * axis).
  static float PivotAngle(const Simulation& sim, std::size_t row, Axis axis) {
    const auto& q = sim.ViewFor<ABAAlgorithm>()
                        .Load<ABAField::kJointPosition, ScalarOperationT>(row)
                        .Rotation();
    return 2.0F * std::atan2(axis == kXAxis ? q.X() : q.Z(), q.W());
  }

  // Turns a pivot joint to `angle` about its one axis, at rest.
  static void SetPivot(
      const Simulation& sim, std::size_t row, Axis axis, float angle
  ) {
    auto view = sim.ViewFor<ABAAlgorithm>();
    auto pose = view.Load<ABAField::kJointPosition, ScalarOperationT>(row);
    float c = std::cos(angle / 2.0F);
    float s = std::sin(angle / 2.0F);
    view.Store<ABAField::kJointPosition, ScalarOperationT>(
        row,
        axis == kXAxis ? decltype(pose)(0, 0, 0, c, s, 0, 0)
                       : decltype(pose)(0, 0, 0, c, 0, 0, s)
    );
    view.Store<ABAField::kJointVelocity, ScalarOperationT>(row, {});
  }

  // A pivot's motor: PD from its joint's angle to `target`, clamped.
  static void Drive(
      const Simulation& sim,
      const AngularActuator& motor,
      std::size_t row,
      float target,
      Axis axis
  ) {
    // kJointVelocity holds generalized velocities, one per column of the
    // joint's motion subspace -- so a one-axis pivot's rate is always the
    // first, whichever axis it turns about.
    float angle_rate =
        sim.ViewFor<ABAAlgorithm>()
            .Load<ABAField::kJointVelocity, ScalarOperationT>(row)
            .Angular()
            .X();
    float angle = PivotAngle(sim, row, axis);
    float torque =
        (kPivotGain * (target - angle)) - (kPivotRateGain * angle_rate);
    motor.Apply(std::clamp(torque, -kPivotTorqueLimit, kPivotTorqueLimit));
  }

  std::size_t gimbal_;
  std::size_t plate_;
  std::size_t ball_;
  AngularActuator tilt_x_;
  AngularActuator tilt_z_;
  Camera camera_;
  std::array<float, 2> command_ = {0.0F, 0.0F};
  bool has_measurement_ = false;
  float measured_x_ = 0.0F;
  float measured_z_ = 0.0F;
  float measured_time_ = 0.0F;  // when the last sample was taken
  float velocity_x_ = 0.0F;
  float velocity_z_ = 0.0F;
};

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
    args.seconds = 15.0F;
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

  Plate plate(sim);
  plate.SetUp(sim);

  std::ofstream csv;
  if (!args.csv_path.empty()) {
    csv.open(args.csv_path);
    csv << "t,x,z,measured_x,measured_z,target_x,target_z,tilt_x,tilt_z\n";
  }

  // One control tick per physics tick: read the state the last Step left,
  // command the pivots, step. The actuators' torque applies to exactly the
  // Step that follows.
  float sim_time = 0.0F;
  float next_report = 0.0F;
  float next_sample = 0.0F;
  auto tick = [&] {
    plate.Control(sim, sim_time);
    if (csv.is_open() && sim_time >= next_sample) {
      Vec3 ball = plate.BallOnPlate(sim);
      auto measured = plate.Measured();
      BallTarget target = Target(sim_time);
      auto tilts = plate.Tilts(sim);
      csv << sim_time << ',' << ball[0] << ',' << ball[2] << ',' << measured[0]
          << ',' << measured[1] << ',' << target.x << ',' << target.z << ','
          << tilts[0] << ',' << tilts[1] << '\n';
      next_sample += 0.01F;
    }
    if (args.headless && sim_time >= next_report) {
      Vec3 ball = plate.BallOnPlate(sim);
      BallTarget target = Target(sim_time);
      auto tilts = plate.Tilts(sim);
      std::cout << std::fixed << std::setprecision(1) << "t=" << std::setw(5)
                << sim_time << "s  ball (cm): " << std::setw(6)
                << 100.0F * ball[0] << std::setw(6) << 100.0F * ball[2]
                << "  off target: " << std::setw(5)
                << 100.0F * std::hypot(ball[0] - target.x, ball[2] - target.z)
                << "  tilt (deg): " << std::setw(6) << kRadToDeg * tilts[0]
                << std::setw(6) << kRadToDeg * tilts[1] << '\n';
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
    std::cerr << "usage: ball_balance <ball_balance.arow> <sim_config.yaml> "
                 "[--headless] [--seconds N] [--csv PATH]\n";
    return 1;
  }
  try {
    return Run(args);
  } catch (const std::exception& e) {
    std::cerr << "ball_balance: " << e.what() << '\n';
    return 1;
  }
}
