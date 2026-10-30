// The sentry co-simulation: ARUW's sentry (aruw-mcb SENTRY_ACHLYS) on the
// ARC 2026 field, its real MCB firmware and its real Jetson software
// (Project Otto) attached to the simulated hardware at the same interfaces
// the robot's physical parts present.
//
// The robot's software is split across two computers and two repos, and
// this keeps the split:
//
//   aruw-mcb hosted build  <-- CAN frames, UART bytes, IMU bursts -->  this
//                           (Unix socket, run-dir/mcb.sock)
//   Project Otto           <-- MCB UART bytes      (pty run-dir/mcb_uart)
//                           <-- LD19 packets        (ptys run-dir/lidar_*)
//
// The RealSense isn't modelled yet: examples/sentry/otto_bridge feeds Otto
// blank frames at the camera's rate, so its loop runs but sees no tags.
//
// Everything between them -- which firmware runs, how the EKF fuses wheel
// odometry and IMUs, how Otto plans and localizes, the messages each sends
// the other -- is theirs, unmodified apart from the hosted hardware hooks.
// This program only plays the hardware:
//
//  - Physics: a planar chassis on four omni wheels (an explicit roller model
//    below: traction along each wheel's rolling direction, nearly free
//    along its axle), turret major, the widow's yaw and pitch, the LD19s'
//    spinning heads; collisions with the field's walls.
//  - DJI motors (C620/M3508 on the wheels and turret major, GM6020s in
//    current mode on the widow): CAN commands in, torque out, 1 kHz
//    feedback frames back (hw/dji_motor.hpp).
//  - Turret major's absolute encoders (pulley CAN encoder and Lamprey).
//  - IMUs: the main MCB's MPU6500 and two ISM330s on turret major, read as
//    register bursts; the chassis and widow turret MCBs, which run their
//    own attitude filter and report over CAN (hw/imu.hpp).
//  - The DR16 remote: an operator who calibrates the IMUs, then switches
//    to autonomous navigation (left switch up, right mid).
//  - The referee system: robot id, game stage and time, HP, power.
//  - Two LD19 lidars and the MCB <-> Jetson UART.
//
// Physics steps at 500 Hz; the hardware talks at 1 kHz. It runs in real time
// (the software on both computers lives on wall-clock time), stamps every step
// to the MCB so the firmware's clock follows the simulation if it ever falls
// behind, and logs the true pose next to the MCB's estimate (overheard on the
// UART) to run-dir/pose_log.csv.
//
// Usage: sentry_sim <field.arow> <sim_config.yaml> [--headless]
//            [--seconds N] [--seed N] [--run-dir DIR]
//            [--calibrate-at S] [--autonav-at S]
// scripts/run-sentry-sim.sh starts all three programs.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <numbers>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include "algorithms/aba/aba_data.hpp"
#include "algorithms/aba/aba_step.hpp"
#include "algorithms/conventions.hpp"
#include "algorithms/sensing/accelerometer.hpp"
#include "algorithms/sensing/encoder.hpp"
#include "algorithms/sensing/gyroscope.hpp"
#include "algorithms/sensing/lidar.hpp"
#include "engine/topology/orderings.hpp"
#include "hw/dji_motor.hpp"
#include "hw/imu.hpp"
#include "hw/links.hpp"
#include "hw/peripherals.hpp"
#include "hw/wire.hpp"
#include "interface/actuator.hpp"
#include "interface/sensor_handle.hpp"
#include "interface/simulation.hpp"
#include "sentry_geometry.hpp"

namespace {

using achilles::algorithms::ScalarOperationT;
using achilles::algorithms::aba::ABAAlgorithm;
using achilles::algorithms::aba::ABAField;
using achilles::algorithms::sensing::Accelerometer;
using achilles::algorithms::sensing::Encoder;
using achilles::algorithms::sensing::Gyroscope;
using achilles::algorithms::sensing::Lidar;
using achilles::engine::topology::TopologicalOrdering;
using achilles::interface::AngularActuator;
using achilles::interface::LinearActuator;
using achilles::interface::SensorHandle;
using achilles::interface::Simulation;
using sentry::hw::Bytes;
using sentry::hw::DjiMotor;
using sentry::hw::McbLink;
using sentry::hw::Switch;
using sentry::hw::Vec3f;

constexpr float kPi = std::numbers::pi_v<float>;
// Physics runs at the MCB's 500 Hz control rate; the hardware talks at
// 1 kHz (DJI feedback, the MPU6500), so every step is two hardware ticks,
// the first reporting the joints extrapolated half a step back.
constexpr ScalarOperationT kPhysicsDt = 0.002F;
constexpr double kHardwareDt = 0.001;
constexpr const char* kRobot = "sentry";

// --- The sentry's geometry and mass: robot/sentry_coarse.urdf's, via
// sentry_geometry.hpp (generated with sentry.arow) ---
using sentry::geometry::kRobotMass;
using sentry::geometry::kWheelAxleInertia;
using sentry::geometry::kWheelOffset;
using sentry::geometry::kWheelRadius;
constexpr float kChassisGearRatio = 17.0F / 268.0F;  // CHASSIS_GEARBOX_RATIO
constexpr float kTurretMajorGearRatio = (187.0F / 3591.0F) * (30.0F / 95.0F);

// The omni wheels' rollers: traction along each wheel's rolling direction
// sticks, up to friction; across it, the rollers turn almost freely.
// Traction is solved per step as the force that ends the step without slip
// (static friction), since an explicit damper soft enough to be stable left the
// wheel all but free above ~70 Hz and the firmware's wheel PID, tuned for a
// loaded wheel, chattered it against the friction cap.
// The robot's mass as one wheel sees it: a quarter of it, which overstates how
// easily the chassis yields (the other wheels hold it too), so the solve
// undershoots rather than overshoots.
constexpr float kWheelEffectiveMass = kRobotMass / 4.0F;
constexpr float kFrictionCoefficient = 0.8F;
constexpr float kRollerDragPerSpeed = 4.0F;  // N per m/s across the wheel
constexpr float kRollerCoulombDrag = 1.0F;   // N

// The LD19's head: 10 rev/s, clockwise seen from above.
constexpr float kLidarHeadRate = -2.0F * kPi * 10.0F;
constexpr float kLidarHeadGain = 0.025F;  // N*m per rad/s

// Rates.
constexpr float kTurretMcbReportPeriod = 0.002F;
constexpr float kTurretMcbStatusPeriod = 0.02F;
constexpr float kIsm330Period = 1.0F / 833.0F;
constexpr float kEncoderPeriod = 0.002F;
constexpr float kRemotePeriod = 0.014F;
constexpr float kLogPeriod = 0.02F;

// The firmware's LAMPREY_CALIBRATION_MAP (sentry_turret_constants.hpp).
const std::vector<std::pair<float, float>> kLampreyMap = {
    {0.000000F, 0.000000F}, {0.075346F, 0.075346F}, {0.218188F, 0.253815F},
    {0.404305F, 0.435452F}, {0.548394F, 0.613726F}, {0.733642F, 0.793966F},
    {0.897285F, 0.974016F}, {1.051091F, 1.151980F}, {1.175210F, 1.331009F},
    {1.362350F, 1.511143F}, {1.431943F, 1.690569F}, {1.575330F, 1.869899F},
    {1.723736F, 2.050683F}, {1.836452F, 2.229278F}, {1.985133F, 2.408738F},
    {2.129243F, 2.587978F}, {2.273207F, 2.767971F}, {2.402318F, 2.947488F},
    {2.621315F, 3.127540F}, {2.759315F, 3.306994F}, {2.919486F, 3.485407F},
    {3.116810F, 3.664799F}, {3.384219F, 3.845500F}, {3.654829F, 4.024267F},
    {3.905801F, 4.204214F}, {4.128683F, 4.384434F}, {4.349133F, 4.562612F},
    {4.543364F, 4.743505F}, {4.544728F, 4.741508F}, {4.668600F, 4.921982F},
    {4.837042F, 5.102610F}, {5.039588F, 5.280833F}, {5.290755F, 5.460778F},
    {5.568966F, 5.641462F}, {5.810764F, 5.819294F}, {6.023868F, 6.000088F},
    {6.209723F, 6.180309F}, {6.283185F, 6.283185F}
};

Vec3f ToVec3f(const achilles::domain::sensing::Vec3& v) {
  return {v.X(), v.Y(), v.Z()};
}

float WrapAngle(float angle) { return std::remainder(angle, 2.0F * kPi); }

struct Args {
  std::string arow_path;
  std::string config_path;
  bool headless = false;
  float seconds = 0.0F;  // 0: until stopped
  std::uint64_t seed = 1;
  std::string run_dir = "/tmp/sentry-sim";
  float calibrate_at = 3.0F;
  float autonav_at = 25.0F;
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
    const bool has_value = i + 1 < argv_span.size();
    if (arg == "--headless") {
      args.headless = true;
    } else if (arg == "--seconds" && has_value) {
      args.seconds = std::strtof(argv_span[++i], nullptr);
    } else if (arg == "--seed" && has_value) {
      args.seed = std::strtoull(argv_span[++i], nullptr, 10);
    } else if (arg == "--run-dir" && has_value) {
      args.run_dir = argv_span[++i];
    } else if (arg == "--calibrate-at" && has_value) {
      args.calibrate_at = std::strtof(argv_span[++i], nullptr);
    } else if (arg == "--autonav-at" && has_value) {
      args.autonav_at = std::strtof(argv_span[++i], nullptr);
    } else {
      return false;
    }
  }
  return true;
}

// A wheel: its joint, motor, shaft encoder and where it pushes the chassis.
struct Wheel {
  std::string name;
  float x;  // position on the chassis (m), x forward, y left
  float y;
  float roll_x;  // rolling direction (unit) for positive wheel speed
  float roll_y;
  DjiMotor motor;
  AngularActuator actuator;
  SensorHandle<Encoder> shaft;
};

// A motor-driven turret joint.
struct TurretJoint {
  DjiMotor motor;
  AngularActuator actuator;
  SensorHandle<Encoder> shaft;
};

struct LidarUnit {
  std::string name;
  AngularActuator head;
  SensorHandle<Encoder> head_angle;
  SensorHandle<Lidar> scan;
  std::unique_ptr<sentry::hw::Pty> pty;
  sentry::hw::Ld19Packer packer;
  std::uint32_t last_sequence = 0;
};

struct ImuHandles {
  SensorHandle<Gyroscope> gyro;
  SensorHandle<Accelerometer> accel;
};

// The MCB's pose estimate, overheard on its UART to the Jetson. On the
// sentry the "chassis" pose the MCB reports is turret major's
// (SentryTransformAdapter::getVisionWorldToChassis): the chassis center's
// position with turret major's world yaw -- the frame the lidars ride in.
struct OverheardOdometry {
  bool valid = false;
  float x = 0.0F;
  float y = 0.0F;
  float yaw = 0.0F;
  float turret_yaw = 0.0F;  // the widow's world yaw
  float turret_pitch = 0.0F;
  std::uint32_t setpoints = 0;  // autonav paths the Jetson has sent
  std::size_t last_path_points = 0;
};

class SentrySim {
 public:
  SentrySim(Simulation& sim, const Args& args)
      : sim_(sim),
        args_(args),
        mcb_(args.run_dir + "/mcb.sock"),
        vision_uart_(args.run_dir + "/mcb_uart"),
        encoders_(kLampreyMap),
        referee_(sentry::hw::Referee::Config{
            .match_start = args.autonav_at,
        }),
        chassis_row_(sim.RowFor<TopologicalOrdering>({kRobot, "chassis"})),
        field_row_(sim.RowFor<TopologicalOrdering>({"field", "world"})),
        chassis_x_(
            sim.AttachActuator<LinearActuator>({kRobot, "chassis"}, {1, 0, 0})
        ),
        chassis_y_(
            sim.AttachActuator<LinearActuator>({kRobot, "chassis"}, {0, 1, 0})
        ),
        chassis_yaw_(
            sim.AttachActuator<AngularActuator>({kRobot, "chassis"}, {0, 0, 1})
        ),
        major_imus_{
            ImuHandles{
                Gyro("major_ism330_0_gyro"), Accel("major_ism330_0_accel")
            },
            ImuHandles{
                Gyro("major_ism330_1_gyro"), Accel("major_ism330_1_accel")
            },
            ImuHandles{
                Gyro("major_mpu6500_gyro"), Accel("major_mpu6500_accel")
            },
        },
        chassis_imu_{Gyro("chassis_mcb_gyro"), Accel("chassis_mcb_accel")},
        widow_imu_{Gyro("widow_mcb_gyro"), Accel("widow_mcb_accel")},
        chassis_mcb_(McbLink::kCan2),
        widow_mcb_(McbLink::kCan1) {
    using sentry::hw::kGm6020Current;
    using sentry::hw::kM3508;
    const float s = 1.0F / std::numbers::sqrt2_v<float>;
    const float a = kWheelOffset;
    // Ids, buses and inversion as sentry_control.cpp declares the motors;
    // rolling directions as tools/generate_scene.py's wheel_directions.
    struct WheelSpec {
      const char* name;
      float x, y, dx, dy;
      std::uint8_t id;
      std::uint16_t offset;
    };
    const std::array<WheelSpec, 4> wheels{{
        {"left_front", a, a, s, -s, 2, 1234},
        {"right_front", a, -a, -s, -s, 1, 4321},
        {"left_back", -a, a, s, s, 3, 777},
        {"right_back", -a, -a, -s, s, 4, 6000},
    }};
    for (const auto& spec : wheels) {
      wheels_.push_back(Wheel{
          spec.name,
          spec.x,
          spec.y,
          spec.dx,
          spec.dy,
          DjiMotor(
              kM3508,
              McbLink::kCan2,
              spec.id,
              kChassisGearRatio,
              true,
              spec.offset
          ),
          sim.AttachActuator<AngularActuator>({kRobot, spec.name}, {1, 0, 0}),
          sim.FindSensor<Encoder>({std::string(spec.name) + "_shaft", 0}),
      });
    }
    turret_major_ = std::make_unique<TurretJoint>(TurretJoint{
        DjiMotor(kM3508, McbLink::kCan2, 5, kTurretMajorGearRatio, true, 5000),
        sim.AttachActuator<AngularActuator>(
            {kRobot, "turret_major"}, {0, 0, 1}
        ),
        sim.FindSensor<Encoder>({"turret_major_shaft", 0}),
    });
    widow_yaw_ = std::make_unique<TurretJoint>(TurretJoint{
        DjiMotor(kGm6020Current, McbLink::kCan1, 6, 1.0F, false, 2693),
        sim.AttachActuator<AngularActuator>({kRobot, "widow_yaw"}, {0, 0, 1}),
        sim.FindSensor<Encoder>({"widow_yaw_shaft", 0}),
    });
    widow_pitch_ = std::make_unique<TurretJoint>(TurretJoint{
        DjiMotor(kGm6020Current, McbLink::kCan1, 5, 1.0F, true, 7357),
        sim.AttachActuator<AngularActuator>({kRobot, "widow_pitch"}, {0, 1, 0}),
        sim.FindSensor<Encoder>({"widow_pitch_shaft", 0}),
    });
    for (const char* name : {"lidar_front", "lidar_rear"}) {
      lidars_.push_back(LidarUnit{
          name,
          sim.AttachActuator<AngularActuator>({kRobot, name}, {0, 0, 1}),
          sim.FindSensor<Encoder>({std::string(name) + "_head", 0}),
          sim.FindSensor<Lidar>({name, 0}),
          std::make_unique<sentry::hw::Pty>(args.run_dir + "/" + name),
          {},
      });
    }
    log_.open(args.run_dir + "/pose_log.csv");
    // true_yaw/mcb_yaw: turret major's world yaw, the yaw of the frame the
    // MCB reports as the chassis; chassis_yaw: the chassis's own.
    log_ << "time,true_x,true_y,true_yaw,mcb_x,mcb_y,mcb_yaw,true_turret_yaw,"
            "mcb_turret_yaw,"
            "true_turret_pitch,mcb_turret_pitch,chassis_yaw,turret_major,widow_"
            "yaw,"
            "autonav_paths\n";
    operator_ = {
        {0.0F, Switch::kDown, Switch::kDown},  // manual drive, sticks centered
        {args.calibrate_at, Switch::kUp, Switch::kDown},  // IMU calibration
        {args.autonav_at, Switch::kUp, Switch::kMid},     // autonav + auto aim
    };
  }

  // One hardware tick (1 kHz): every other one, the MCB's commands in and a
  // physics step; every one, the hardware's replies out.
  bool Tick() {
    ExchangeWithMcb();
    ExchangeWithJetson();
    double lag = 0.0;
    if (tick_ % 2 == 0) {
      ApplyEfforts();
      if (!sim_.Step(kPhysicsDt)) {
        return false;
      }
      lag = -kHardwareDt;  // physics is now a tick ahead of time_
    }
    time_ += kHardwareDt;
    ++tick_;
    mcb_.SendTime(static_cast<std::uint64_t>(std::llround(time_ * 1e6)), 1000U);
    Report(lag);
    return true;
  }

  // Holds time at zero until the MCB connects, so the firmware boots at t = 0
  // and the operator script (IMU calibration first) can't run ahead of it.
  void WaitForMcb() {
    while (!mcb_.Connected()) {
      ExchangeWithMcb();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  float Time() const { return static_cast<float>(time_); }
  bool McbConnected() const { return mcb_.Connected(); }
  const OverheardOdometry& Estimate() const { return overheard_; }

  // The chassis's true pose in the field frame (the field's world joint;
  // field.arow sets it off so the viewer centres on the field): x, y, yaw.
  std::array<float, 3> TruePose() const {
    const auto view = sim_.ViewFor<ABAAlgorithm>();
    const auto world =
        view.Load<ABAField::kWorldTransform, ScalarOperationT>(field_row_)
            .Inverse() *
        view.Load<ABAField::kWorldTransform, ScalarOperationT>(chassis_row_);
    const auto forward = world.Rotation().Rotate({1.0F, 0.0F, 0.0F});
    return {
        world.Translation().X(),
        world.Translation().Y(),
        std::atan2(forward.Y(), forward.X())
    };
  }

  // The true pose of what the MCB reports as the chassis: the chassis
  // center, facing turret major's way.
  std::array<float, 3> TrueReportedPose() const {
    auto pose = TruePose();
    pose[2] =
        WrapAngle(pose[2] + sim_.Read(turret_major_->shaft).value.position);
    return pose;
  }

  void PrintStatus() const {
    const auto truth = TrueReportedPose();
    std::cout << std::fixed << std::setprecision(2) << "t=" << std::setw(7)
              << time_ << "s  true=(" << std::setw(5) << truth[0] << ", "
              << std::setw(5) << truth[1] << ", " << std::setw(7)
              << truth[2] * 180.0F / kPi << ")";
    if (overheard_.valid) {
      std::cout << "  mcb=(" << std::setw(5) << overheard_.x << ", "
                << std::setw(5) << overheard_.y << ", " << std::setw(7)
                << overheard_.yaw * 180.0F / kPi
                << ")  error=" << std::setprecision(3)
                << std::hypot(truth[0] - overheard_.x, truth[1] - overheard_.y)
                << " m";
    } else {
      std::cout << "  mcb="
                << (mcb_.Connected() ? "(no odometry yet)" : "(not connected)");
    }
    std::cout << "  paths=" << overheard_.setpoints << '\n' << std::flush;
  }

 private:
  SensorHandle<Gyroscope> Gyro(const std::string& name) const {
    return sim_.FindSensor<Gyroscope>({name, 0});
  }
  SensorHandle<Accelerometer> Accel(const std::string& name) const {
    return sim_.FindSensor<Accelerometer>({name, 0});
  }

  std::vector<DjiMotor*> Motors() {
    std::vector<DjiMotor*> motors;
    for (Wheel& wheel : wheels_) {
      motors.push_back(&wheel.motor);
    }
    motors.push_back(&turret_major_->motor);
    motors.push_back(&widow_yaw_->motor);
    motors.push_back(&widow_pitch_->motor);
    return motors;
  }

  void ExchangeWithMcb() {
    const auto now = static_cast<float>(time_);
    const bool was_connected = mcb_.Connected();
    mcb_.Poll(
        [&](const McbLink::CanFrame& frame) {
          const std::span<const std::uint8_t> data(
              frame.data.data(), frame.length
          );
          for (DjiMotor* motor : Motors()) {
            motor->HandleCommand(frame.bus, frame.id, data, now);
          }
          chassis_mcb_.HandleFrame(frame.bus, frame.id, data);
          widow_mcb_.HandleFrame(frame.bus, frame.id, data);
        },
        [&](McbLink::Port port, std::span<const std::uint8_t> bytes) {
          if (port == McbLink::kUart2) {
            vision_uart_.Write(bytes);
            Overhear(bytes);
          }
        }
    );
    if (mcb_.Connected() && !was_connected) {
      std::cout << "MCB connected on " << mcb_.Path() << '\n';
    }
  }

  void ExchangeWithJetson() {
    const Bytes from_jetson = vision_uart_.Read();
    if (!from_jetson.empty()) {
      mcb_.SendUart(McbLink::kUart3, from_jetson);
      jetson_parser_.Feed(
          from_jetson,
          [&](const sentry::hw::DjiSerialParser::Frame& frame) {
            if (frame.command == 13 && frame.data.size() >= 12) {
              ++overheard_.setpoints;
              overheard_.last_path_points =
                  sentry::hw::Get<std::uint32_t>(frame.data, 8);
            }
          }
      );
    }
  }

  // The odometry message (type 1) the MCB sends the Jetson: timestamp, the
  // chassis's x, y, z, roll, pitch, yaw, a turret count, then each turret's
  // x, y, z, roll, pitch, yaw.
  void Overhear(std::span<const std::uint8_t> bytes) {
    mcb_parser_.Feed(
        bytes,
        [&](const sentry::hw::DjiSerialParser::Frame& frame) {
          if (frame.command == 1 && frame.data.size() >= 4 + 6 * 4) {
            overheard_.valid = true;
            overheard_.x = sentry::hw::Get<float>(frame.data, 4);
            overheard_.y = sentry::hw::Get<float>(frame.data, 8);
            overheard_.yaw = sentry::hw::Get<float>(frame.data, 24);
            if (frame.data.size() >= 29 + 6 * 4) {
              overheard_.turret_pitch =
                  sentry::hw::Get<float>(frame.data, 29 + 16);
              overheard_.turret_yaw =
                  sentry::hw::Get<float>(frame.data, 29 + 20);
            }
          }
        }
    );
  }

  void ApplyEfforts() {
    const auto now = static_cast<float>(time_);
    const auto view = sim_.ViewFor<ABAAlgorithm>();
    // The chassis's velocity, in its own frame.
    const auto velocity =
        view.Load<ABAField::kSpatialVelocity, ScalarOperationT>(chassis_row_);
    const float vx = velocity.Linear().X();
    const float vy = velocity.Linear().Y();
    const float wz = velocity.Angular().Z();

    float force_x = 0.0F;
    float force_y = 0.0F;
    float torque_z = 0.0F;
    const float normal =
        kRobotMass * sentry::hw::kGravity / static_cast<float>(wheels_.size());
    for (Wheel& wheel : wheels_) {
      const float omega = sim_.Read(wheel.shaft).value.velocity;
      // The wheel's contact point moves with the chassis...
      const float px = vx - wz * wheel.y;
      const float py = vy + wz * wheel.x;
      const float along = px * wheel.roll_x + py * wheel.roll_y;
      const float across = -px * wheel.roll_y + py * wheel.roll_x;
      // ...and its rim with the wheel. The difference is slip.
      const float slip = omega * kWheelRadius - along;
      const float motor_torque = wheel.motor.JointTorque(omega, now);
      // The force that ends this step without slip: it slows the rim by
      // F r^2 / I per second and speeds the contact point by F / m.
      const float stick =
          (slip + kPhysicsDt * kWheelRadius * motor_torque / kWheelAxleInertia
          ) /
          (kPhysicsDt * (kWheelRadius * kWheelRadius / kWheelAxleInertia +
                         1.0F / kWheelEffectiveMass));
      const float traction = std::clamp(
          stick, -kFrictionCoefficient * normal, kFrictionCoefficient * normal
      );
      const float drag = -kRollerDragPerSpeed * across -
                         kRollerCoulombDrag * std::tanh(across / 0.01F);
      const float fx = traction * wheel.roll_x - drag * wheel.roll_y;
      const float fy = traction * wheel.roll_y + drag * wheel.roll_x;
      force_x += fx;
      force_y += fy;
      torque_z += wheel.x * fy - wheel.y * fx;
      wheel.actuator.Apply(motor_torque - traction * kWheelRadius);
    }
    chassis_x_.Apply(force_x);
    chassis_y_.Apply(force_y);
    chassis_yaw_.Apply(torque_z);

    for (TurretJoint* joint :
         {turret_major_.get(), widow_yaw_.get(), widow_pitch_.get()}) {
      const float omega = sim_.Read(joint->shaft).value.velocity;
      joint->actuator.Apply(joint->motor.JointTorque(omega, now));
    }
    for (LidarUnit& lidar : lidars_) {
      const float omega = sim_.Read(lidar.head_angle).value.velocity;
      lidar.head.Apply(kLidarHeadGain * (kLidarHeadRate - omega));
    }
  }

  void SendCan(
      std::uint8_t bus, std::uint32_t id, std::span<const std::uint8_t> data
  ) {
    mcb_.SendCan(static_cast<McbLink::Bus>(bus), id, data);
  }

  bool Every(float period, double& next) {
    if (time_ + 1e-9 < next) {
      return false;
    }
    next += period;
    return true;
  }

  // `lag`: how far behind the physics state time_ is (0 or minus a tick).
  void Report(double lag) {
    const auto now = static_cast<float>(time_);
    const bool physics_stepped = lag != 0.0;
    // DJI motor feedback, 1 kHz.
    for (Wheel& wheel : wheels_) {
      SendMotorFeedback(wheel.motor, wheel.shaft, lag);
    }
    for (TurretJoint* joint :
         {turret_major_.get(), widow_yaw_.get(), widow_pitch_.get()}) {
      SendMotorFeedback(joint->motor, joint->shaft, lag);
    }
    if (Every(kEncoderPeriod, next_encoder_)) {
      const double major = sim_.Read(turret_major_->shaft).value.position;
      SendCan(McbLink::kCan2, 0x1E0, encoders_.PulleyFrame(major));
      SendCan(McbLink::kCan2, 0x1E7, encoders_.LampreyFrame(major));
    }

    // The main MCB's chips. The ISM330s' registers update at their 833 Hz
    // output data rate, the MPU6500's at 1 kHz.
    {
      const auto& mpu = major_imus_[2];
      mcb_.SendImu(
          McbLink::kMpu6500,
          0,
          sentry::hw::Mpu6500Burst(
              ToVec3f(sim_.Read(mpu.gyro).value),
              ToVec3f(sim_.Read(mpu.accel).value),
              50.0F
          )
      );
    }
    if (Every(kIsm330Period, next_ism330_)) {
      for (std::uint8_t instance = 0; instance < 2; ++instance) {
        const auto& ism = major_imus_[instance];
        mcb_.SendImu(
            McbLink::kIsm330,
            instance,
            sentry::hw::Ism330Burst(
                ToVec3f(sim_.Read(ism.gyro).value),
                ToVec3f(sim_.Read(ism.accel).value),
                35.0F
            )
        );
      }
    }

    // The turret MCBs, sampling their IMUs once per physics step.
    auto send = [&](std::uint8_t bus,
                    std::uint32_t id,
                    std::span<const std::uint8_t> data) {
      SendCan(bus, id, data);
    };
    if (physics_stepped) {
      chassis_mcb_.Sample(
          ToVec3f(sim_.Read(chassis_imu_.gyro).value),
          ToVec3f(sim_.Read(chassis_imu_.accel).value),
          kPhysicsDt
      );
      widow_mcb_.Sample(
          ToVec3f(sim_.Read(widow_imu_.gyro).value),
          ToVec3f(sim_.Read(widow_imu_.accel).value),
          kPhysicsDt
      );
    }
    if (Every(kTurretMcbReportPeriod, next_turret_mcb_)) {
      chassis_mcb_.SendAxes(send);
      widow_mcb_.SendAxes(send);
    }
    if (Every(kTurretMcbStatusPeriod, next_turret_status_)) {
      chassis_mcb_.SendStatus(send);
      widow_mcb_.SendStatus(send);
    }

    // The operator's remote and the referee system.
    if (Every(kRemotePeriod, next_remote_)) {
      const sentry::hw::OperatorStep* step = &operator_.front();
      for (const auto& candidate : operator_) {
        if (now >= candidate.time) {
          step = &candidate;
        }
      }
      mcb_.SendUart(
          McbLink::kUart1, sentry::hw::Dr16Frame(step->left, step->right)
      );
    }
    const Bytes referee = referee_.Poll(now);
    if (!referee.empty()) {
      mcb_.SendUart(McbLink::kUart6, referee);
    }

    if (physics_stepped) {
      ReportLidars();
    }
    if (Every(kLogPeriod, next_log_)) {
      Log();
    }
  }

  void SendMotorFeedback(
      const DjiMotor& motor, SensorHandle<Encoder> shaft, double lag
  ) {
    const auto reading = sim_.Read(shaft).value;
    SendCan(
        motor.Bus(),
        motor.FeedbackId(),
        motor.Feedback(
            reading.position + reading.velocity * lag, reading.velocity
        )
    );
  }

  // Each head's newest 9-beam sample, if it took one this tick, becomes
  // LD19 points at the head's angle: the sensor's zero is the head's +x,
  // and LD19 angles run clockwise.
  void ReportLidars() {
    const auto now = static_cast<float>(time_);
    for (LidarUnit& lidar : lidars_) {
      const auto scan = sim_.Read(lidar.scan);
      if (scan.sequence == lidar.last_sequence) {
        continue;
      }
      lidar.last_sequence = scan.sequence;
      const float head = sim_.Read(lidar.head_angle).value.position;
      const auto beams = scan.value.beams;
      const float fov = 7.2F * kPi / 180.0F;
      // Beam i points (i + 0.5) * fov / beams - fov / 2 counterclockwise of
      // the head's +x; clockwise order is reverse beam order.
      for (std::uint32_t k = 0; k < beams; ++k) {
        const std::uint32_t i = beams - 1 - k;
        const float azimuth =
            head +
            ((static_cast<float>(i) + 0.5F) * fov / static_cast<float>(beams)) -
            (fov / 2);
        lidar.packer.Add({-azimuth * 180.0F / kPi, scan.value.ranges[i]});
      }
      lidar.packer.Drain(
          std::abs(kLidarHeadRate) * 180.0F / kPi,
          now,
          [&](std::span<const std::uint8_t> packet) {
            lidar.pty->Write(packet);
          }
      );
    }
  }

  void Log() {
    const auto chassis = TruePose();
    const auto truth = TrueReportedPose();
    const float major = sim_.Read(turret_major_->shaft).value.position;
    const float widow_yaw = sim_.Read(widow_yaw_->shaft).value.position;
    const float pitch = sim_.Read(widow_pitch_->shaft).value.position;
    log_ << std::fixed << std::setprecision(4) << time_ << ',' << truth[0]
         << ',' << truth[1] << ',' << truth[2] << ',';
    if (overheard_.valid) {
      log_ << overheard_.x << ',' << overheard_.y << ',' << overheard_.yaw;
    } else {
      log_ << ",,";
    }
    log_ << ',' << WrapAngle(chassis[2] + major + widow_yaw) << ',';
    if (overheard_.valid) {
      log_ << overheard_.turret_yaw;
    }
    log_ << ',' << pitch << ',';
    if (overheard_.valid) {
      log_ << overheard_.turret_pitch;
    }
    log_ << ',' << chassis[2] << ',' << major << ',' << widow_yaw << ','
         << overheard_.setpoints << '\n';
  }

  Simulation& sim_;
  Args args_;
  McbLink mcb_;
  sentry::hw::Pty vision_uart_;
  sentry::hw::TurretMajorEncoders encoders_;
  sentry::hw::Referee referee_;
  std::size_t chassis_row_;
  std::size_t field_row_;
  LinearActuator chassis_x_;
  LinearActuator chassis_y_;
  AngularActuator chassis_yaw_;
  std::vector<Wheel> wheels_;
  std::unique_ptr<TurretJoint> turret_major_;
  std::unique_ptr<TurretJoint> widow_yaw_;
  std::unique_ptr<TurretJoint> widow_pitch_;
  std::vector<LidarUnit> lidars_;
  std::array<ImuHandles, 3> major_imus_;
  ImuHandles chassis_imu_;
  ImuHandles widow_imu_;
  sentry::hw::TurretMcb chassis_mcb_;
  sentry::hw::TurretMcb widow_mcb_;
  std::vector<sentry::hw::OperatorStep> operator_;
  sentry::hw::DjiSerialParser mcb_parser_;
  sentry::hw::DjiSerialParser jetson_parser_;
  OverheardOdometry overheard_;
  std::ofstream log_;
  double time_ = 0.0;
  std::uint64_t tick_ = 0;
  double next_encoder_ = 0.0;
  double next_ism330_ = 0.0;
  double next_turret_mcb_ = 0.0;
  double next_turret_status_ = 0.0;
  double next_remote_ = 0.0;
  double next_log_ = 0.0;
};

int Run(const Args& args) {
  std::filesystem::create_directories(args.run_dir);
  Simulation sim;
  if (!sim.LoadArowFile(args.arow_path) ||
      !sim.LoadConfigFile(args.config_path) || !sim.Init(args.seed)) {
    return 1;
  }
  // Semi-implicit Euler: the integrator with collision.
  sim.SetIntegrator(Simulation::Integrator::kSemiImplicitEuler);

  SentrySim sentry(sim, args);
  std::cout << "sentry sim: MCB socket " << args.run_dir
            << "/mcb.sock, Jetson UART " << args.run_dir << "/mcb_uart, lidars "
            << args.run_dir << "/lidar_front and " << args.run_dir
            << "/lidar_rear\n"
            << std::flush;
  sentry.WaitForMcb();

  // Real time: the MCB and the Jetson both run on wall-clock time. If the
  // sim falls behind it says so, and the MCB's clock (stamped every step)
  // slows down with it.
  using Clock = std::chrono::steady_clock;
  const auto start = Clock::now();
  auto last_render = start;
  float next_status = 1.0F;
  float max_lag = 0.0F;
  while (args.seconds <= 0.0F || sentry.Time() < args.seconds) {
    if (!sentry.Tick()) {
      return 1;
    }
    const auto due = start + std::chrono::duration<double>(sentry.Time());
    const auto now = Clock::now();
    if (due > now) {
      std::this_thread::sleep_until(due);
    } else {
      max_lag =
          std::max(max_lag, std::chrono::duration<float>(now - due).count());
    }
    if (sentry.Time() >= next_status) {
      sentry.PrintStatus();
      if (max_lag > 0.05F) {
        std::cout << "  (running behind real time by up to " << max_lag
                  << " s)\n";
      }
      max_lag = 0.0F;
      next_status += 1.0F;
    }
    if (!args.headless &&
        Clock::now() - last_render > std::chrono::milliseconds(16)) {
      last_render = Clock::now();
      if (!sim.Render()) {
        break;
      }
    }
  }
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  Args args;
  if (!ParseArgs(argc, argv, args)) {
    std::cerr
        << "usage: sentry_sim <field.arow> <sim_config.yaml> [--headless] "
           "[--seconds N] "
           "[--seed N] [--run-dir DIR] [--calibrate-at S] [--autonav-at S]\n";
    return 1;
  }
  try {
    return Run(args);
  } catch (const std::exception& e) {
    std::cerr << "sentry_sim: " << e.what() << '\n';
    return 1;
  }
}
