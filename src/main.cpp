// NOLINTBEGIN(misc-include-cleaner) -- main.cpp is deliberately a kitchen
// sink while actively developing: it includes every header so that a
// single-TU compile (and clang-tidy run, via scripts/check-tidy.sh's
// "every header must be reachable from a src/*.cpp file" invariant)
// exercises the whole codebase, not just what main() itself calls. This
// is not the standard for the rest of the codebase -- it's specific to
// this file.
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>

#include "algorithms/aba/aba_data.hpp"
#include "algorithms/aba/aba_ops.hpp"
#include "algorithms/aba/aba_step.hpp"
#include "algorithms/conventions.hpp"
#include "algorithms/registry.hpp"
#include "algorithms/sim_config.hpp"
#include "domain/archetype.hpp"
#include "domain/joint_topology.hpp"
#include "domain/math/activation_mask.hpp"
#include "domain/math/matrix.hpp"
#include "domain/math/quaternion.hpp"
#include "domain/math/vector3.hpp"
#include "domain/math/vector6.hpp"
#include "domain/spatial/dual.hpp"
#include "domain/spatial/inertia.hpp"
#include "domain/spatial/transform.hpp"
#include "engine/algorithm_contract.hpp"
#include "engine/assembler.hpp"
#include "engine/field_contract.hpp"
#include "engine/memory/arena.hpp"
#include "engine/memory/binding.hpp"
#include "engine/memory/sim_allocator.hpp"
#include "engine/op_contract.hpp"
#include "engine/pass/algorithm_step.hpp"
#include "engine/pass/op_invoker.hpp"
#include "engine/pass/sim_context.hpp"
#include "engine/pass/traversals.hpp"
#include "engine/topology/layout.hpp"
#include "engine/topology/layout_policy.hpp"
#include "engine/topology/ordering_policy.hpp"
#include "engine/view/view.hpp"
#include "engine/view/view_contract.hpp"
#include "engine/view/view_factory.hpp"
#include "interface/archetype_loader.hpp"
#include "interface/sim_config_loader.hpp"
#include "interface/simulation.hpp"
#include "render/mat4.hpp"
#include "util/buffer.hpp"
#include "util/io.hpp"
#include "util/simd_ops.hpp"
#include "util/tmp.hpp"
#include "util/yaml.hpp"
// NOLINTEND(misc-include-cleaner)

namespace {

using Integrator = achilles::interface::Simulation::Integrator;

struct CliArgs {
  bool headless = false;
  std::string arow_path;
  std::string config_path;
  std::string energy_log_path;
  // Substepping is off: it exists to shrink kSemiImplicitEuler's own
  // per-step error, and kRungeKutta4 below gets that accuracy from its
  // own order instead (see Simulation::SetSubsteps / Integrator).
  int substeps = 1;
  // RK4 at a 2ms step. RK4 is the integrator MuJoCo itself points at for
  // energy-conserving systems, and it measured far and away the best on
  // this engine's own scenes -- roughly three orders of magnitude less
  // energy drift than semi-implicit Euler at the same step, and the only
  // scheme benchmarked here that stayed stable out to dt = 1/20.
  //
  // Note what this costs: 4 ABA evaluations per tick at 500 ticks per
  // simulated second is 2000 evaluations/sim-second, which is the most
  // expensive setting in the benchmark, not the cheapest. RK4's own
  // efficiency argument is that it buys quality by taking *larger* steps
  // (at dt = 0.05 it holds drift near 0.58 for only 80 evaluations/sim-
  // second, a third of what Euler at 1/120 costs). A 2ms step spends that
  // headroom on fidelity instead: drift falls well below what the 1/120
  // row already measured at 0.001. Raise dt toward 0.0333 or 0.05 to trade
  // that back for throughput.
  Integrator integrator = Integrator::kRungeKutta4;
  float dt = 0.002F;
};

// Parses argv into CliArgs, or returns std::nullopt (having already
// printed the specific problem to stderr) for a missing flag argument or
// an unrecognized --integrator value. Split out of main() itself purely to
// keep main()'s own cognitive complexity down -- this has no dependency on
// anything main() sets up.
std::optional<Integrator> ParseIntegrator(std::string_view name) {
  if (name == "euler") {
    return Integrator::kSemiImplicitEuler;
  }
  if (name == "verlet") {
    return Integrator::kVelocityVerlet;
  }
  if (name == "midpoint") {
    return Integrator::kImplicitMidpoint;
  }
  if (name == "mujoco" || name == "implicit") {
    return Integrator::kImplicitVelocity;
  }
  if (name == "rk4") {
    return Integrator::kRungeKutta4;
  }
  if (name == "implicitfast") {
    return Integrator::kImplicitFast;
  }
  std::cerr << R"(--integrator must be one of euler, verlet, midpoint, )"
               R"(implicit, implicitfast, rk4; got ")"
            << name << "\"\n";
  return std::nullopt;
}

std::optional<CliArgs> ParseArgs(int argc, char** argv) {
  CliArgs args;
  for (int i = 1; i < argc; ++i) {
    std::string_view arg = argv[i];
    if (arg == "--headless") {
      args.headless = true;
    } else if (arg == "--energy-log") {
      if (++i >= argc) {
        std::cerr << "--energy-log requires a path argument\n";
        return std::nullopt;
      }
      args.energy_log_path = argv[i];
    } else if (arg == "--substeps") {
      if (++i >= argc) {
        std::cerr << "--substeps requires an integer argument\n";
        return std::nullopt;
      }
      args.substeps = std::atoi(argv[i]);
    } else if (arg == "--dt") {
      if (++i >= argc) {
        std::cerr << "--dt requires a floating-point seconds argument\n";
        return std::nullopt;
      }
      args.dt = std::strtof(argv[i], nullptr);
    } else if (arg == "--integrator") {
      if (++i >= argc) {
        std::cerr << R"(--integrator requires one of euler, verlet, midpoint, )"
                     R"(implicit, implicitfast, rk4)"
                  << '\n';
        return std::nullopt;
      }
      std::optional<Integrator> integrator = ParseIntegrator(argv[i]);
      if (!integrator.has_value()) {
        return std::nullopt;
      }
      args.integrator = *integrator;
    } else if (args.arow_path.empty()) {
      args.arow_path = arg;
    } else {
      args.config_path = arg;
    }
  }
  return args;
}

}  // namespace

// achilles <scene.arow> [sim_config.yaml] [--headless] [--energy-log <path>]
//   [--substeps N] [--dt seconds] [--integrator
//   euler|verlet|midpoint|implicit|implicitfast|rk4]
//
// Loads and initializes a Simulation from a .arow file (see
// interface/archetype_loader.hpp for its format, and examples/ for a
// sample scene), then steps it at a fixed 1/120s timestep. Visual by
// default -- Simulation::Step renders one frame per call (see
// interface/simulation.hpp's own comment on SetHeadless) -- so the loop
// below is the same shape either way: it just runs until Step() says stop,
// which happens on error, or (only when visual) once the render window is
// closed. See scripts/run-example.sh for the common case of running this
// against examples/two_joint_arm.arow. --energy-log writes a CSV of system
// energy over time (see Simulation::EnableEnergyLog) -- graph it with
// scripts/plot.py. --substeps sets Simulation::SetSubsteps, --integrator
// sets Simulation::Integrator and --dt the fixed timestep (see their own
// comments); CliArgs's own defaults (kRungeKutta4, dt = 2ms, substeps=1)
// are the CLI's, not Simulation's -- library callers (tests, embedders)
// still get the original unsubstepped semi-implicit-Euler behavior at
// whatever dt they pass unless they ask for something else. Pass
// --integrator euler to see the older, visibly-dissipative behavior for
// comparison, or raise --dt to trade fidelity back for throughput.
int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr
        << "usage: achilles <scene.arow> [sim_config.yaml] "
           "[--headless] [--energy-log <path>] [--substeps N] [--dt seconds] "
           "[--integrator euler|verlet|midpoint|implicit|implicitfast|rk4]\n";
    return 1;
  }

  std::optional<CliArgs> args = ParseArgs(argc, argv);
  if (!args.has_value()) {
    return 1;
  }

  achilles::interface::Simulation sim;
  if (!sim.LoadArowFile(args->arow_path)) {
    return 1;
  }
  if (!args->config_path.empty() && !sim.LoadConfigFile(args->config_path)) {
    return 1;
  }
  if (!sim.Init()) {
    return 1;
  }
  sim.SetIntegrator(args->integrator);
  sim.SetHeadless(args->headless);
  sim.SetSubsteps(args->substeps);
  if (!args->energy_log_path.empty() &&
      !sim.EnableEnergyLog(args->energy_log_path)) {
    return 1;
  }

  const float dt = args->dt;
  constexpr float kMaxFrameTime = 0.25F;
  float accumulator = 0.0F;
  auto last_time = std::chrono::steady_clock::now();

  bool running = true;
  while (running) {
    auto now = std::chrono::steady_clock::now();
    float elapsed = std::chrono::duration<float>(now - last_time).count();
    last_time = now;
    // Clamped so a debugger pause or a hitch (e.g. window drag) doesn't
    // make the next real frame try to catch up with a huge burst of steps.
    accumulator += std::min(elapsed, kMaxFrameTime);

    while (running && accumulator >= dt) {
      running = sim.Step(dt);
      accumulator -= dt;
    }
  }

  return 0;
}
