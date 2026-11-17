// The domino demo: ten dominoes in a row, each 1.25 times the size of the
// one before (dominoes.arow), the first set falling. What it shows off is
// contact: each falling domino's momentum carried into the next, bigger one
// through the contact solver, so the chain runs on to the end -- each fall a
// little later and a little heavier than the last.
//
// It watches the chain through its camera, and plots every domino's tilt
// (its pose sensor's turn about z) on one plot, one line each, so the wave
// can be seen moving down the row: each line drops as its domino falls, the
// next starting a little after it.
//
// Usage: dominoes <dominoes.arow> <sim_config.yaml> [--headless]
//                 [--seconds N]
// (scripts/run-dominoes.sh passes this folder's files). Headless runs print
// every domino's tilt once per simulated second instead, for N seconds
// (default 10).
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <span>
#include <string>
#include <string_view>
#include <utility>

#include "../shared/real_time.hpp"
#include "display/renderer.hpp"
#include "display/views/camera/camera_view.hpp"
#include "display/views/reading_view.hpp"
#include "domain/sensing/camera.hpp"
#include "interface/attachment_handle.hpp"
#include "interface/simulation.hpp"

namespace {

using achilles::interface::Simulation;
namespace truth = achilles::interface::truth;

constexpr float kDt = 0.002F;
constexpr std::size_t kDominoes = 10;
constexpr float kRadToDeg = 57.2957795F;

struct Args {
  std::string arow_path;
  std::string config_path;
  bool headless = false;
  float seconds = 0.0F;  // 0: run until stopped
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
    } else {
      return false;
    }
  }
  if (args.headless && args.seconds <= 0.0F) {
    args.seconds = 10.0F;
  }
  return true;
}

int Run(const Args& args) {
  Simulation sim;
  if (!sim.LoadArowFile(args.arow_path) ||
      !sim.LoadConfigFile(args.config_path) || !sim.Init()) {
    return 1;
  }
  // Semi-implicit Euler: the integrator with collision.
  sim.SetIntegrator(Simulation::Integrator::kSemiImplicitEuler);
  // Every domino's pose, domino_1_pose on: an array, as one display shows
  // them all.
  const auto poses = [&]<std::size_t... Is>(std::index_sequence<Is...>) {
    return std::array{sim.Sensor<truth::WorldTransform>(
        "domino_" + std::to_string(Is + 1) + "_pose"
    )...};
  }(std::make_index_sequence<kDominoes>{});

  float sim_time = 0.0F;
  float next_report = 0.0F;
  const auto tick = [&] {
    const bool ok = sim.Step(kDt);
    sim_time += kDt;
    if (args.headless && sim_time >= next_report) {
      std::cout << std::fixed << std::setprecision(1) << "t=" << std::setw(5)
                << sim_time << "s  tilt (deg):";
      for (const auto& pose : poses) {
        std::cout << std::setw(6)
                  << std::abs(pose.Read().value.Rotation().Log().Z()) *
                         kRadToDeg;
      }
      std::cout << '\n';
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

  // Real time, with the chain through its camera, and every domino's tilt
  // on one plot.
  achilles::examples::RunInRealTime(
      "Achilles: dominoes",
      kDt,
      [&](achilles::display::Renderer& renderer) {
        renderer.Attach(
            achilles::display::CameraView(
                sim.Sensor<achilles::domain::sensing::Camera>("dominoes_camera")
            ),
            "Domino camera"
        );
        renderer.Attach(
            achilles::display::ReadingView<truth::WorldTransform, kDominoes>(
                poses,
                {.channels = {achilles::display::PoseChannel::kTurnZ},
                 .names = {"1", "2", "3", "4", "5", "6", "7", "8", "9", "10"}}
            ),
            "Domino tilt"
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
    std::cerr << "usage: dominoes <dominoes.arow> <sim_config.yaml> "
                 "[--headless] [--seconds N]\n";
    return 1;
  }
  try {
    return Run(args);
  } catch (const std::exception& e) {
    std::cerr << "dominoes: " << e.what() << '\n';
    return 1;
  }
}
