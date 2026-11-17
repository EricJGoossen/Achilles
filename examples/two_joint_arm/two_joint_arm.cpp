// The double pendulum demo: the two-joint arm (two_joint_arm.arow) let go
// to swing under gravity, with nothing driving it. Its motion is chaotic --
// two runs a hair apart soon look nothing alike -- so what it shows off is
// the integrator: RK4 at a 2 ms step keeps the swing going essentially
// indefinitely, without visibly bleeding energy, where a cruder scheme
// slowly damps it out.
//
// It watches the arm through its camera, and plots what makes the motion
// chaotic: both joints' angles on one plot and both joints' rates on
// another, from the arm's truth sensors (shoulder_angle.arow and the rest).
//
// Usage: two_joint_arm <two_joint_arm.arow> <sim_config.yaml> [--headless]
//                      [--seconds N] [--integrator NAME]
// (scripts/run-double-pendulum.sh passes this folder's files). Headless runs
// print both angles once per simulated second instead, for N seconds
// (default 20). --integrator picks another integrator to compare (see
// algorithms/integrators.hpp for their names).
#include <cstddef>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <optional>
#include <span>
#include <string>
#include <string_view>

#include "../shared/real_time.hpp"
#include "algorithms/integrators.hpp"
#include "display/renderer.hpp"
#include "display/views/camera/camera_view.hpp"
#include "display/views/reading_view.hpp"
#include "domain/sensing/camera.hpp"
#include "interface/attachment_handle.hpp"
#include "interface/simulation.hpp"

namespace {

using achilles::domain::sensing::Camera;
using achilles::interface::Simulation;
namespace truth = achilles::interface::truth;

constexpr float kDt = 0.002F;

struct Args {
  std::string arow_path;
  std::string config_path;
  bool headless = false;
  float seconds = 0.0F;  // 0: run until stopped
  Simulation::Integrator integrator = Simulation::Integrator::kRungeKutta4;
};

bool ParseArgs(int argc, char** argv, Args& args) {
  const std::span<char*> argv_span(argv, static_cast<std::size_t>(argc));
  if (argv_span.size() < 3) {
    return false;
  }
  args.arow_path = argv_span[1];
  args.config_path = argv_span[2];
  for (std::size_t i = 3; i < argv_span.size(); ++i) {
    const std::string_view arg = argv_span[i];
    if (arg == "--headless") {
      args.headless = true;
    } else if (arg == "--seconds" && i + 1 < argv_span.size()) {
      args.seconds = std::strtof(argv_span[++i], nullptr);
    } else if (arg == "--integrator" && i + 1 < argv_span.size()) {
      const std::optional<Simulation::Integrator> integrator =
          achilles::algorithms::IntegratorFromName(argv_span[++i]);
      if (!integrator.has_value()) {
        return false;
      }
      args.integrator = *integrator;
    } else {
      return false;
    }
  }
  if (args.headless && args.seconds <= 0.0F) {
    args.seconds = 20.0F;
  }
  return true;
}

// A revolute joint's angle about its own X, read from its position.
float AngleOf(
    const achilles::interface::SensorHandle<truth::JointPosition>& sensor
) {
  return sensor.Read().value.Rotation().Log().X();
}

int Run(const Args& args) {
  Simulation sim;
  if (!sim.LoadArowFile(args.arow_path) ||
      !sim.LoadConfigFile(args.config_path) || !sim.Init()) {
    return 1;
  }
  sim.SetIntegrator(args.integrator);
  const auto shoulder = sim.Sensor<truth::JointPosition>("shoulder_angle");
  const auto elbow = sim.Sensor<truth::JointPosition>("elbow_angle");

  float sim_time = 0.0F;
  float next_report = 0.0F;
  const auto tick = [&] {
    const bool ok = sim.Step(kDt);
    sim_time += kDt;
    if (args.headless && sim_time >= next_report) {
      std::cout << std::fixed << std::setprecision(3) << "t=" << std::setw(6)
                << sim_time << "s  shoulder=" << std::setw(7)
                << AngleOf(shoulder) << " rad  elbow=" << std::setw(7)
                << AngleOf(elbow) << " rad\n";
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

  // Real time, with the arm through its camera, and both joints' angles and
  // rates, each pair on one plot.
  achilles::examples::RunInRealTime(
      sim,
      "Achilles: double pendulum",
      kDt,
      [&](achilles::display::Renderer& renderer) {
        using achilles::display::ReadingView;
        using achilles::display::MotionChannel;
        using achilles::display::PoseChannel;
        renderer.Attach(
            achilles::display::CameraView{},
            "Arm camera",
            sim.Sensor<Camera>("arm_camera")
        );
        renderer.Attach(
            ReadingView<truth::JointPosition, 2>{
                .channels = {PoseChannel::kTurnX},
                .names = {"shoulder", "elbow"}
            },
            "Joint angles",
            shoulder,
            elbow
        );
        renderer.Attach(
            ReadingView<truth::JointVelocity, 2>{
                .channels = {MotionChannel::kAngularX},
                .names = {"shoulder", "elbow"}
            },
            "Joint rates",
            sim.Sensor<truth::JointVelocity>("shoulder_rate"),
            sim.Sensor<truth::JointVelocity>("elbow_rate")
        );
      },
      tick,
      [&] { return args.seconds > 0.0F && sim_time >= args.seconds; }
  );
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  Args args;
  if (!ParseArgs(argc, argv, args)) {
    std::cerr << "usage: two_joint_arm <two_joint_arm.arow> <sim_config.yaml> "
                 "[--headless] [--seconds N] [--integrator NAME]\n";
    return 1;
  }
  try {
    return Run(args);
  } catch (const std::exception& e) {
    std::cerr << "two_joint_arm: " << e.what() << '\n';
    return 1;
  }
}
