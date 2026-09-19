// NOLINTBEGIN(misc-include-cleaner) -- main.cpp is deliberately a kitchen
// sink while actively developing: it includes every header so that a
// single-TU compile (and clang-tidy run, via scripts/check-tidy.sh's
// "every header must be reachable from a src/*.cpp file" invariant)
// exercises the whole codebase, not just what main() itself calls. This
// is not the standard for the rest of the codebase -- it's specific to
// this file.
#include <algorithm>
#include <chrono>
#include <iostream>
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

// achilles <scene.arow> [sim_config.yaml] [--headless]
//
// Loads and initializes a Simulation from a .arow file (see
// interface/archetype_loader.hpp for its format, and examples/ for a
// sample scene), then steps it at a fixed 1/120s timestep. Visual by
// default -- Simulation::Step renders one frame per call (see
// interface/simulation.hpp's own comment on SetHeadless) -- so the loop
// below is the same shape either way: it just runs until Step() says stop,
// which happens on error, or (only when visual) once the render window is
// closed. See scripts/run-example.sh for the common case of running this
// against examples/two_joint_arm.arow.
int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr
        << "usage: achilles <scene.arow> [sim_config.yaml] [--headless]\n";
    return 1;
  }

  bool headless = false;
  std::string arow_path;
  std::string config_path;
  for (int i = 1; i < argc; ++i) {
    std::string_view arg = argv[i];
    if (arg == "--headless") {
      headless = true;
    } else if (arow_path.empty()) {
      arow_path = arg;
    } else {
      config_path = arg;
    }
  }

  achilles::interface::Simulation sim;
  if (!sim.LoadArowFile(arow_path)) {
    return 1;
  }
  if (!config_path.empty() && !sim.LoadConfigFile(config_path)) {
    return 1;
  }
  if (!sim.Init()) {
    return 1;
  }
  sim.SetHeadless(headless);

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
