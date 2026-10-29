// The sensing demo: a four-wheeled robot (robot.arow) finds its way out of
// a maze it has never seen (maze.arow), using nothing but its own sensors
// -- a lidar, a gyroscope and four wheel encoders, all with noise. The
// controller never reads the sim's state; it only sees sensor readings and
// only acts through wheel torques:
//
//  - Odometry: heading from the gyroscope, its turn-on bias calibrated out
//    while the robot sits still for its first second; distance from the
//    wheel encoders. The pose estimate drifts, as a real robot's does.
//  - Mapping: every lidar scan goes into an occupancy grid at the
//    estimated pose -- each beam's cells free up to its hit, the hit cell
//    occupied.
//  - Planning: A* from the estimated pose to the exit over that grid,
//    walls grown by the robot's size, cells it hasn't seen yet assumed
//    open; replanned on every scan, so the robot heads optimistically
//    toward the exit and backs out of each dead end once it sees one.
//  - Driving: pure pursuit along the path, turned into left and right wheel
//    speeds, each held by a PI loop on its encoder's velocity.
//
// Usage: maze <maze.arow> <sim_config.yaml> [--headless] [--seconds N]
//             [--seed N]
// (scripts/run-maze.sh passes this folder's files). Headless runs print
// the true and estimated pose once per simulated second instead of drawing
// them, and stop at the exit. --seed keys every sensor's noise (default 1).
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numbers>
#include <queue>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "algorithms/aba/aba_data.hpp"
#include "algorithms/aba/aba_step.hpp"
#include "algorithms/conventions.hpp"
#include "algorithms/sensing/encoder.hpp"
#include "algorithms/sensing/gyroscope.hpp"
#include "algorithms/sensing/lidar.hpp"
#include "domain/archetype.hpp"
#include "engine/topology/orderings.hpp"
#include "interface/actuator.hpp"
#include "interface/sensor_handle.hpp"
#include "interface/simulation.hpp"

namespace {

using achilles::algorithms::ScalarOperationT;
using achilles::algorithms::aba::ABAAlgorithm;
using achilles::algorithms::aba::ABAField;
using achilles::algorithms::sensing::Encoder;
using achilles::algorithms::sensing::Gyroscope;
using achilles::algorithms::sensing::Lidar;
using achilles::algorithms::sensing::LidarScan;
using achilles::engine::topology::TopologicalOrdering;
using achilles::interface::AngularActuator;
using achilles::interface::JointAddress;
using achilles::interface::SensorHandle;
using achilles::interface::Simulation;

constexpr float kPi = std::numbers::pi_v<float>;

// Geometry, matching robot.arow and maze.arow.
constexpr float kWheelRadius = 0.1F;
constexpr float kHalfTrack = 0.27F;  // axle center to each side's wheels
constexpr float kStartX = -3.75F;
constexpr float kStartY = -3.75F;
constexpr float kStartHeading = kPi / 2;  // facing +y
constexpr float kExitX = 3.75F;
constexpr float kExitY = 5.25F;

// The lidar, matching sensors/lidar.arow: beam i points
// -pi + (i + 0.5) * 2 pi / kLidarBeams from the robot's forward,
// counterclockwise.
constexpr std::size_t kLidarBeams = 360;
constexpr float kLidarMaxRange = 8.0F;

// The occupancy grid: kGridCells squared cells of kCellSize, covering the
// maze and its exit. Log-odds per cell, clamped so a few contrary scans can
// still flip it.
constexpr float kCellSize = 0.1F;
constexpr int kGridCells = 136;
constexpr std::size_t kGridArea =
    static_cast<std::size_t>(kGridCells) * kGridCells;
constexpr float kGridOrigin = -6.8F;  // its lower-left corner, x and y
constexpr float kHitLogOdds = 0.9F;
constexpr float kMissLogOdds = -0.4F;
constexpr float kMinLogOdds = -2.0F;
constexpr float kMaxLogOdds = 3.5F;
constexpr float kOccupiedLogOdds = 0.6F;

// Planning: a path keeps kRobotRadius clear of every wall it knows of, and
// prefers staying kComfortRadius clear.
constexpr float kRobotRadius = 0.4F;
constexpr float kComfortRadius = 0.7F;
constexpr float kBlockedCost = 200.0F;  // per cell inside kRobotRadius
constexpr float kCloseCost = 4.0F;      // per cell, at the wall's edge

// Driving.
constexpr float kCalibrationTime = 1.0F;   // s standing still at the start
constexpr float kCruiseSpeed = 0.6F;       // m/s
constexpr float kLookahead = 0.5F;         // m along the path
constexpr float kTurnGain = 2.5F;          // rad/s of yaw rate per rad of error
constexpr float kMaxYawRate = 1.5F;        // rad/s
constexpr float kTurnInPlace = 0.8F;       // rad of error: stop and turn
constexpr float kExitRadius = 0.4F;        // m: close enough to the exit
constexpr float kWheelTorqueLimit = 3.0F;  // N*m per wheel
constexpr float kSpeedGain = 0.6F;         // N*m per rad/s of speed error
constexpr float kSpeedIntegralGain = 2.0F;

constexpr ScalarOperationT kDt = 0.004F;

float WrapAngle(float angle) {
  while (angle > kPi) {
    angle -= 2.0F * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0F * kPi;
  }
  return angle;
}

struct Pose {
  float x;
  float y;
  float heading;  // counterclockwise from +x, seen from above
};

// A wheel's motor and encoder: drives its wheel toward a target speed with
// a PI loop on the encoder's velocity, limited to what the motor can
// deliver.
class Wheel {
 public:
  Wheel(AngularActuator motor, SensorHandle<Encoder> encoder)
      : motor_(std::move(motor)), encoder_(encoder) {}

  // Reads the encoder; call once per tick, before Drive and Angle.
  void Sense(const Simulation& sim) { reading_ = sim.Read(encoder_).value; }

  // The wheel's turn about its axle so far (rad).
  float Angle() const { return reading_.position; }

  // Integrates only while unsaturated, so a long stall doesn't wind up a
  // huge correction.
  void Drive(float target_speed, float dt) {
    float error = target_speed - reading_.velocity;
    float torque = (kSpeedGain * error) + (kSpeedIntegralGain * integral_);
    if (std::abs(torque) < kWheelTorqueLimit) {
      integral_ += error * dt;
    }
    motor_.Apply(std::clamp(torque, -kWheelTorqueLimit, kWheelTorqueLimit));
  }

 private:
  AngularActuator motor_;
  SensorHandle<Encoder> encoder_;
  achilles::algorithms::sensing::EncoderReading reading_;
  float integral_ = 0.0F;
};

// Dead reckoning: heading from the gyroscope's yaw rate, distance from the
// wheels' mean turn. The gyroscope's bias is the mean of its readings over
// the first kCalibrationTime, while the robot stands still.
class Odometry {
 public:
  Odometry() : pose_{kStartX, kStartY, kStartHeading} {}

  bool Calibrated() const { return calibrated_; }
  const Pose& Estimate() const { return pose_; }

  void Update(float time, float yaw_rate, float wheel_angle, float dt) {
    if (!calibrated_) {
      bias_sum_ += yaw_rate;
      ++bias_samples_;
      last_wheel_angle_ = wheel_angle;
      if (time >= kCalibrationTime) {
        bias_ = bias_sum_ / static_cast<float>(bias_samples_);
        calibrated_ = true;
      }
      return;
    }
    float turn = (yaw_rate - bias_) * dt;
    float distance = (wheel_angle - last_wheel_angle_) * kWheelRadius;
    last_wheel_angle_ = wheel_angle;
    float mid_heading = pose_.heading + (turn / 2);
    pose_.x += distance * std::cos(mid_heading);
    pose_.y += distance * std::sin(mid_heading);
    pose_.heading = WrapAngle(pose_.heading + turn);
  }

 private:
  Pose pose_;
  bool calibrated_ = false;
  float bias_ = 0.0F;
  float bias_sum_ = 0.0F;
  int bias_samples_ = 0;
  float last_wheel_angle_ = 0.0F;
};

struct Cell {
  int x;
  int y;
  friend bool operator==(const Cell&, const Cell&) = default;
};

// What the robot knows of the maze: log-odds of each cell being occupied,
// from every scan so far, placed at the pose estimate at the time.
class OccupancyGrid {
 public:
  OccupancyGrid() : log_odds_(kGridArea, 0.0F) {}

  static bool Contains(Cell cell) {
    return cell.x >= 0 && cell.y >= 0 && cell.x < kGridCells &&
           cell.y < kGridCells;
  }
  static Cell CellAt(float x, float y) {
    return {
        static_cast<int>(std::floor((x - kGridOrigin) / kCellSize)),
        static_cast<int>(std::floor((y - kGridOrigin) / kCellSize))
    };
  }
  static std::array<float, 2> Center(Cell cell) {
    return {
        kGridOrigin + ((static_cast<float>(cell.x) + 0.5F) * kCellSize),
        kGridOrigin + ((static_cast<float>(cell.y) + 0.5F) * kCellSize)
    };
  }
  static std::size_t Index(Cell cell) {
    return (static_cast<std::size_t>(cell.y) * kGridCells) +
           static_cast<std::size_t>(cell.x);
  }

  bool Occupied(Cell cell) const {
    return log_odds_[Index(cell)] > kOccupiedLogOdds;
  }

  void Integrate(const Pose& pose, const LidarScan& scan) {
    for (std::size_t i = 0; i < scan.beams; ++i) {
      float azimuth =
          -kPi + ((static_cast<float>(i) + 0.5F) * 2.0F * kPi / kLidarBeams);
      float angle = pose.heading + azimuth;
      bool hit = std::isfinite(scan.ranges[i]);
      float range = hit ? scan.ranges[i] : kLidarMaxRange;
      Cell end = CellAt(
          pose.x + (range * std::cos(angle)), pose.y + (range * std::sin(angle))
      );
      Trace(CellAt(pose.x, pose.y), end, hit);
    }
  }

 private:
  void Add(Cell cell, float amount) {
    float& value = log_odds_[Index(cell)];
    value = std::clamp(value + amount, kMinLogOdds, kMaxLogOdds);
  }

  // Bresenham from `from` to `to`: every cell before `to` is seen free,
  // `to` itself occupied if the beam hit something there.
  void Trace(Cell from, Cell to, bool hit) {
    int dx = std::abs(to.x - from.x);
    int dy = -std::abs(to.y - from.y);
    int step_x = from.x < to.x ? 1 : -1;
    int step_y = from.y < to.y ? 1 : -1;
    int error = dx + dy;
    Cell cell = from;
    while (cell != to) {
      if (!Contains(cell)) {
        return;
      }
      Add(cell, kMissLogOdds);
      int twice = 2 * error;
      if (twice >= dy) {
        error += dy;
        cell.x += step_x;
      }
      if (twice <= dx) {
        error += dx;
        cell.y += step_y;
      }
    }
    if (hit && Contains(to)) {
      Add(to, kHitLogOdds);
    }
  }

  std::vector<float> log_odds_;
};

// A* over the grid, 8-connected. A cell's cost grows as it nears a known
// wall, steeply inside kRobotRadius -- steep rather than forbidden, so a
// pose estimate that drifts against a wall can still plan its way out.
// Unknown cells cost the same as known free ones: the robot assumes open
// space until it sees otherwise.
class Planner {
 public:
  // The path from `start` to `goal`, as cell centers, start first; empty
  // if there is none.
  std::vector<std::array<float, 2>> Plan(
      const OccupancyGrid& grid, Cell start, Cell goal
  ) {
    ComputeClearance(grid);
    constexpr float kInf = std::numeric_limits<float>::infinity();
    std::vector<float> cost(kGridArea, kInf);
    std::vector<std::size_t> came_from(kGridArea, 0);
    using Entry = std::pair<float, std::size_t>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<>> open;
    auto heuristic = [&](Cell cell) {
      return std::hypot(
          static_cast<float>(cell.x - goal.x),
          static_cast<float>(cell.y - goal.y)
      );
    };
    cost[OccupancyGrid::Index(start)] = 0.0F;
    open.emplace(heuristic(start), OccupancyGrid::Index(start));
    while (!open.empty()) {
      auto [priority, index] = open.top();
      open.pop();
      Cell cell{
          static_cast<int>(index % kGridCells),
          static_cast<int>(index / kGridCells)
      };
      if (cell == goal) {
        return Walk(came_from, start, goal);
      }
      if (priority > cost[index] + heuristic(cell) + 1e-3F) {
        continue;  // stale
      }
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          Cell next{cell.x + dx, cell.y + dy};
          if ((dx == 0 && dy == 0) || !OccupancyGrid::Contains(next)) {
            continue;
          }
          std::size_t next_index = OccupancyGrid::Index(next);
          float step =
              (dx != 0 && dy != 0) ? std::numbers::sqrt2_v<float> : 1.0F;
          float next_cost =
              cost[index] + (step * (1.0F + Penalty(clearance_[next_index])));
          if (next_cost < cost[next_index]) {
            cost[next_index] = next_cost;
            came_from[next_index] = index;
            open.emplace(next_cost + heuristic(next), next_index);
          }
        }
      }
    }
    return {};
  }

 private:
  static float Penalty(float clearance) {
    if (clearance < kRobotRadius) {
      return kBlockedCost;
    }
    if (clearance < kComfortRadius) {
      return kCloseCost * (kComfortRadius - clearance) /
             (kComfortRadius - kRobotRadius);
    }
    return 0.0F;
  }

  // Each cell's distance to the nearest occupied cell, out to
  // kComfortRadius (beyond that, it doesn't matter how far).
  void ComputeClearance(const OccupancyGrid& grid) {
    constexpr int kReach = static_cast<int>(kComfortRadius / kCellSize) + 1;
    clearance_.assign(kGridArea, kComfortRadius);
    for (int y = 0; y < kGridCells; ++y) {
      for (int x = 0; x < kGridCells; ++x) {
        if (!grid.Occupied({x, y})) {
          continue;
        }
        for (int dy = -kReach; dy <= kReach; ++dy) {
          for (int dx = -kReach; dx <= kReach; ++dx) {
            Cell near{x + dx, y + dy};
            if (!OccupancyGrid::Contains(near)) {
              continue;
            }
            float distance =
                kCellSize *
                std::hypot(static_cast<float>(dx), static_cast<float>(dy));
            float& clearance = clearance_[OccupancyGrid::Index(near)];
            clearance = std::min(clearance, distance);
          }
        }
      }
    }
  }

  static std::vector<std::array<float, 2>> Walk(
      const std::vector<std::size_t>& came_from, Cell start, Cell goal
  ) {
    std::vector<std::array<float, 2>> path;
    std::size_t index = OccupancyGrid::Index(goal);
    const std::size_t start_index = OccupancyGrid::Index(start);
    while (true) {
      path.push_back(OccupancyGrid::Center(
          {static_cast<int>(index % kGridCells),
           static_cast<int>(index / kGridCells)}
      ));
      if (index == start_index) {
        break;
      }
      index = came_from[index];
    }
    std::ranges::reverse(path);
    return path;
  }

  std::vector<float> clearance_;
};

struct Command {
  float speed;     // m/s forward
  float yaw_rate;  // rad/s, counterclockwise from above
};

// Pure pursuit: steer at the point kLookahead along the path, slowing as
// the heading error grows and turning in place when it's large.
Command Follow(const Pose& pose, std::span<const std::array<float, 2>> path) {
  if (path.empty()) {
    return {0.0F, 0.0F};
  }
  const std::array<float, 2>* target = &path.back();
  for (const auto& point : path) {
    if (std::hypot(point[0] - pose.x, point[1] - pose.y) >= kLookahead) {
      target = &point;
      break;
    }
  }
  float error = WrapAngle(
      std::atan2((*target)[1] - pose.y, (*target)[0] - pose.x) - pose.heading
  );
  float yaw_rate = std::clamp(kTurnGain * error, -kMaxYawRate, kMaxYawRate);
  if (std::abs(error) > kTurnInPlace) {
    return {0.0F, yaw_rate};
  }
  return {kCruiseSpeed * std::cos(error), yaw_rate};
}

// The robot's whole brain: its sensors, its estimate of where it is, its
// map, its plan, and its wheels.
class Navigator {
 public:
  explicit Navigator(const Simulation& sim)
      : lidar_(sim.FindSensor<Lidar>({"lidar", 0})),
        gyroscope_(sim.FindSensor<Gyroscope>({"gyroscope", 0})),
        wheels_{
            AttachWheel(sim, "left_front"),
            AttachWheel(sim, "left_rear"),
            AttachWheel(sim, "right_front"),
            AttachWheel(sim, "right_rear")
        } {}

  // One control tick: read every sensor (as of the last Step), update the
  // pose estimate, and on a new scan the map and plan; then command the
  // wheels for the next Step.
  void Tick(const Simulation& sim, float time, float dt) {
    float wheel_angle = 0.0F;
    for (Wheel& wheel : wheels_) {
      wheel.Sense(sim);
      wheel_angle += wheel.Angle() / static_cast<float>(wheels_.size());
    }
    odometry_.Update(time, sim.Read(gyroscope_).value.Y(), wheel_angle, dt);

    const auto scan = sim.Read(lidar_);
    if (odometry_.Calibrated() && scan.sequence != last_scan_) {
      last_scan_ = scan.sequence;
      grid_.Integrate(odometry_.Estimate(), scan.value);
      Replan();
    }

    Command command{0.0F, 0.0F};
    if (odometry_.Calibrated() && !AtExit()) {
      command = Follow(odometry_.Estimate(), path_);
    }
    // Skid steering: a yaw rate w moves a wheel at lateral offset +/-
    // kHalfTrack (left/right) at v -/+ w * kHalfTrack.
    float left =
        (command.speed - (command.yaw_rate * kHalfTrack)) / kWheelRadius;
    float right =
        (command.speed + (command.yaw_rate * kHalfTrack)) / kWheelRadius;
    wheels_[0].Drive(left, dt);
    wheels_[1].Drive(left, dt);
    wheels_[2].Drive(right, dt);
    wheels_[3].Drive(right, dt);
  }

  bool AtExit() const {
    const Pose& pose = odometry_.Estimate();
    return std::hypot(kExitX - pose.x, kExitY - pose.y) < kExitRadius;
  }
  const Pose& Estimate() const { return odometry_.Estimate(); }
  const OccupancyGrid& Map() const { return grid_; }
  std::span<const std::array<float, 2>> Path() const { return path_; }

 private:
  static Wheel AttachWheel(const Simulation& sim, const std::string& name) {
    // Each wheel turns about its own axle, local X.
    return {
        sim.AttachActuator<AngularActuator>({"robot", name}, {1, 0, 0}),
        sim.FindSensor<Encoder>({name + "_encoder", 0})
    };
  }

  void Replan() {
    const Pose& pose = odometry_.Estimate();
    path_ = planner_.Plan(
        grid_,
        OccupancyGrid::CellAt(pose.x, pose.y),
        OccupancyGrid::CellAt(kExitX, kExitY)
    );
  }

  SensorHandle<Lidar> lidar_;
  SensorHandle<Gyroscope> gyroscope_;
  std::array<Wheel, 4> wheels_;  // left front, left rear, right front, rear
  Odometry odometry_;
  OccupancyGrid grid_;
  Planner planner_;
  std::vector<std::array<float, 2>> path_;
  std::uint32_t last_scan_ = 0;
};

// Where the robot really is -- for the printout only; the controller never
// sees it.
Pose TruePose(const Simulation& sim, std::size_t chassis) {
  auto world = sim.ViewFor<ABAAlgorithm>()
                   .Load<ABAField::kWorldTransform, ScalarOperationT>(chassis);
  // The chassis's forward is its local +z.
  auto forward = world.Rotation().Rotate({0.0F, 0.0F, 1.0F});
  return {
      world.Translation().X(),
      world.Translation().Y(),
      std::atan2(forward.Y(), forward.X())
  };
}

struct Args {
  std::string arow_path;
  std::string config_path;
  bool headless = false;
  float seconds = 0.0F;  // 0: run until the window closes
  std::uint64_t seed = 1;
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
    } else if (arg == "--seed" && i + 1 < argv_span.size()) {
      args.seed = std::strtoull(argv_span[++i], nullptr, 10);
    } else {
      return false;
    }
  }
  if (args.headless && args.seconds <= 0.0F) {
    args.seconds = 180.0F;
  }
  return true;
}

}  // namespace

// The whole demo; throws on a bad scene (a missing joint, say).
int Run(const Args& args) {
  Simulation sim;
  if (!sim.LoadArowFile(args.arow_path) ||
      !sim.LoadConfigFile(args.config_path) || !sim.Init(args.seed)) {
    return 1;
  }
  // Semi-implicit Euler: the integrator with collision.
  sim.SetIntegrator(Simulation::Integrator::kSemiImplicitEuler);

  const std::size_t chassis =
      sim.RowFor<TopologicalOrdering>({"robot", "chassis"});
  Navigator navigator(sim);

  // One control tick per physics tick: read what the last Step left,
  // command the wheels, step. The actuators' torque applies to exactly the
  // Step that follows.
  float sim_time = 0.0F;
  float next_report = 0.0F;
  auto tick = [&] {
    navigator.Tick(sim, sim_time, kDt);
    bool ok = sim.Step(kDt);
    sim_time += kDt;
    if (args.headless && sim_time >= next_report) {
      Pose truth = TruePose(sim, chassis);
      const Pose& estimate = navigator.Estimate();
      constexpr float kRadToDeg = 180.0F / kPi;
      std::cout << std::fixed << std::setprecision(2) << "t=" << std::setw(6)
                << sim_time << "s  true=(" << std::setw(5) << truth.x << ", "
                << std::setw(5) << truth.y << ", " << std::setw(7)
                << truth.heading * kRadToDeg << ")  estimate=(" << std::setw(5)
                << estimate.x << ", " << std::setw(5) << estimate.y << ", "
                << std::setw(7) << estimate.heading * kRadToDeg
                << ")  error=" << std::setprecision(3)
                << std::hypot(truth.x - estimate.x, truth.y - estimate.y)
                << " m  path=" << navigator.Path().size() << " cells\n";
      next_report += 1.0F;
    }
    return ok;
  };

  if (args.headless) {
    while (sim_time < args.seconds && !navigator.AtExit()) {
      if (!tick()) {
        return 1;
      }
    }
    if (navigator.AtExit()) {
      Pose truth = TruePose(sim, chassis);
      std::cout << "reached the exit at t=" << sim_time << "s, truly "
                << std::hypot(kExitX - truth.x, kExitY - truth.y)
                << " m from it\n";
      return 0;
    }
    std::cout << "still in the maze after " << sim_time << "s\n";
    return 1;
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
    std::cerr << "usage: maze <maze.arow> <sim_config.yaml> [--headless] "
                 "[--seconds N] [--seed N]\n";
    return 1;
  }
  try {
    return Run(args);
  } catch (const std::exception& e) {
    std::cerr << "maze: " << e.what() << '\n';
    return 1;
  }
}
