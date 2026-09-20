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
  // 1, not kSemiImplicitEuler's earlier 8: kImplicitMidpoint's own inner
  // iteration already buys far better energy behavior than substepped
  // Euler did, at a fraction of the ABA evaluations (4 per tick here vs.
  // 8 for substepped Euler) -- see the energy-log comparison this
  // default is based on (PR description / conversation history has the
  // numbers: mean drift ~0.003 here vs. ~1.0-1.3 for Euler at substeps=8
  // over the same 30s run).
  int substeps = 1;
  Integrator integrator = Integrator::kImplicitMidpoint;
};

// Parses argv into CliArgs, or returns std::nullopt (having already
// printed the specific problem to stderr) for a missing flag argument or
// an unrecognized --integrator value. Split out of main() itself purely to
// keep main()'s own cognitive complexity down -- this has no dependency on
// anything main() sets up.
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
    } else if (arg == "--integrator") {
      if (++i >= argc) {
        std::cerr << R"(--integrator requires "euler", "verlet", or )"
                     R"("midpoint")"
                  << '\n';
        return std::nullopt;
      }
      std::string_view name = argv[i];
      if (name == "euler") {
        args.integrator = Integrator::kSemiImplicitEuler;
      } else if (name == "verlet") {
        args.integrator = Integrator::kVelocityVerlet;
      } else if (name == "midpoint") {
        args.integrator = Integrator::kImplicitMidpoint;
      } else {
        std::cerr << R"(--integrator must be "euler", "verlet", or )"
                     R"("midpoint", got ")"
                  << name << "\"\n";
        return std::nullopt;
      }
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
//   [--substeps N] [--integrator euler|verlet|midpoint]
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
// scripts/plot.py. --substeps sets Simulation::SetSubsteps and --integrator
// sets Simulation::Integrator (see their own comments); CliArgs's own
// defaults (kImplicitMidpoint, substeps=1) are the CLI's, not Simulation's
// -- library callers (tests, embedders) still get the original
// unsubstepped semi-implicit-Euler behavior unless they ask for something
// else. Pass --integrator euler [--substeps 8] to see the older,
// visibly-dissipative behavior for comparison.
int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "usage: achilles <scene.arow> [sim_config.yaml] "
                 "[--headless] [--energy-log <path>] [--substeps N] "
                 "[--integrator euler|verlet|midpoint]\n";
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

  constexpr float kDt = 1.0F / 120.0F;
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

    while (running && accumulator >= kDt) {
      running = sim.Step(kDt);
      accumulator -= kDt;
    }
  }

  return 0;
}
