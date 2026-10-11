// Benchmark / trace harness for comparing achilles against MuJoCo (see
// bench/README.md for the full workflow).
//
//   achilles_bench throughput <scene.arow> <sim_config.yaml> <integrator>
//                  <dt> <copies> <steps> [trials]
//     Loads <scene.arow> with `copies: <copies>` injected, then times
//     <steps> headless Simulation::Step(dt) calls (best of [trials], default
//     5). Prints one JSON line.
//
//   achilles_bench trace <scene.arow> <sim_config.yaml> <integrator> <dt>
//                  <steps> <out.csv>
//     Runs a single instance and writes, for t = 0 and after every step,
//     each joint's raw state: rotation quaternion (w, x, y, z), the six
//     generalized-velocity slots, and the six qdd slots ABA reports for that
//     exact (q, qd). bench/compare_mujoco.py turns these into joint angles.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <unistd.h>
#include <vector>

#include "algorithms/aba/aba_data.hpp"
#include "algorithms/conventions.hpp"
#include "domain/joint_topology.hpp"
#include "engine/topology/ordering_policy.hpp"
#include "interface/simulation.hpp"

namespace {

using achilles::algorithms::ScalarOperationT;
using achilles::algorithms::aba::ABAAlgorithm;
using achilles::algorithms::aba::ABAField;
using achilles::engine::topology::TopologicalOrdering;
using achilles::interface::Simulation;
using Integrator = Simulation::Integrator;
using Clock = std::chrono::steady_clock;

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
  if (name == "implicit") {
    return Integrator::kImplicitVelocity;
  }
  if (name == "rk4") {
    return Integrator::kRungeKutta4;
  }
  if (name == "implicitfast") {
    return Integrator::kImplicitFast;
  }
  return std::nullopt;
}

// The loader only allows `copies` on a non-root archetype, so batching N
// robots needs a single fixed root to hang them off: one joint with no
// active DOF (all-zero subspace, empty activation mask), i.e. welded to the
// fixed base. Its own inertia never reaches any arm -- a zero-DOF joint on
// a fixed base never moves -- so it only costs one extra ABA row per tick.
constexpr std::string_view kWorldArow = R"(archetype: bench_world
joints:
  - name: origin
    parent: null
    fields:
      joint_subspace: [0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0,
                       0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0]
      joint_activation_mask: [0]
      fixed_joint_transform: {translation: [0, 0, 0], rotation: [1, 0, 0, 0]}
      rigid_body_inertia: [1.0, 0, 0, 0, 1.0, 1.0, 1.0, 0, 0, 0]
      joint_position: {translation: [0, 0, 0], rotation: [1, 0, 0, 0]}
      joint_velocity: [0, 0, 0, 0, 0, 0]
      joint_torque: [0, 0, 0, 0, 0, 0]
      visual_extents: [0.01, 0.01, 0.01]
      visual_color: [0, 0, 0]
)";

// Writes `arow_path` plus kWorldArow into a fresh temp directory, with
// `copies: n` and an attach-to-bench_world injected right after the
// scene's own `archetype:` line, so any single-robot scene file can be
// benchmarked at any batch size. Returns the rewritten scene's path.
std::filesystem::path WithCopies(const std::string& arow_path, std::size_t n) {
  std::ifstream in(arow_path);
  std::ostringstream out;
  std::string line;
  bool inserted = false;
  while (std::getline(in, line)) {
    out << line << '\n';
    if (!inserted && line.rfind("archetype:", 0) == 0) {
      out << "copies: " << n << '\n'
          << "includes: [bench_world.arow]\n"
          << "attach: {archetype: bench_world, joint: origin}\n";
      inserted = true;
    }
  }
  if (!inserted) {
    throw std::runtime_error("no top-level `archetype:` line in " + arow_path);
  }
  std::filesystem::path dir =
      std::filesystem::temp_directory_path() /
      ("achilles_bench_" + std::to_string(getpid()) + "_" + std::to_string(n));
  std::filesystem::create_directories(dir);
  std::ofstream(dir / "bench_world.arow") << kWorldArow;
  std::ofstream(dir / "scene.arow") << out.str();
  return dir / "scene.arow";
}

bool MakeSim(
    Simulation& sim, const std::string& arow, const std::string& config,
    Integrator integrator
) {
  if (!sim.LoadArowFile(arow) || !sim.LoadConfigFile(config) || !sim.Init()) {
    return false;
  }
  sim.SetIntegrator(integrator);
  sim.SetHeadless(true);
  return true;
}

// The real (non-padding) rows of a single-instance serial chain, root first.
// Padding rows also resolve their parent to the base row, but never have
// children, so the longest chain hanging off the base row is the real one.
std::vector<std::size_t> ChainRows(const achilles::domain::JointTopology& t) {
  std::size_t base = t.Size();
  std::vector<std::size_t> best;
  for (std::size_t root = 0; root < base; ++root) {
    if (t[root] != base) {
      continue;
    }
    std::vector<std::size_t> chain{root};
    bool extended = true;
    while (extended) {
      extended = false;
      for (std::size_t row = 0; row < base; ++row) {
        if (row != chain.back() && t[row] == chain.back()) {
          chain.push_back(row);
          extended = true;
          break;
        }
      }
    }
    if (chain.size() > best.size()) {
      best = chain;
    }
  }
  return best;
}

bool AllFinite(const Simulation& sim) {
  auto view = sim.ViewFor<ABAAlgorithm>();
  auto topology = sim.TopologyFor<TopologicalOrdering>();
  for (std::size_t row = 0; row < topology.Size(); ++row) {
    auto q = view.Load<ABAField::kJointPosition, ScalarOperationT>(row);
    if (!std::isfinite(q.Rotation().W()) || !std::isfinite(q.Rotation().X())) {
      return false;
    }
  }
  return true;
}

int Throughput(int argc, char** argv) {
  if (argc < 8) {
    std::cerr << "throughput <arow> <config> <integrator> <dt> <copies> "
                 "<steps> [trials]\n";
    return 1;
  }
  std::optional<Integrator> integrator = ParseIntegrator(argv[4]);
  if (!integrator.has_value()) {
    std::cerr << "unknown integrator " << argv[4] << '\n';
    return 1;
  }
  const float dt = std::strtof(argv[5], nullptr);
  const auto copies = static_cast<std::size_t>(std::atol(argv[6]));
  const long steps = std::atol(argv[7]);
  const int trials = argc > 8 ? std::atoi(argv[8]) : 5;

  std::filesystem::path arow = WithCopies(argv[2], copies);
  Simulation sim;
  bool ok = MakeSim(sim, arow.string(), argv[3], *integrator);
  std::filesystem::remove_all(arow.parent_path());
  if (!ok) {
    return 1;
  }
  std::size_t padded_rows = sim.TopologyFor<TopologicalOrdering>().Size();

  // Warm-up: touch every page and let the branch predictors settle.
  for (long i = 0; i < std::max(1L, steps / 10); ++i) {
    sim.Step(dt);
  }

  std::vector<double> seconds;
  for (int trial = 0; trial < trials; ++trial) {
    auto start = Clock::now();
    for (long i = 0; i < steps; ++i) {
      sim.Step(dt);
    }
    seconds.push_back(
        std::chrono::duration<double>(Clock::now() - start).count()
    );
  }
  std::sort(seconds.begin(), seconds.end());
  double best = seconds.front();
  double median = seconds[seconds.size() / 2];

  std::printf(
      "{\"engine\": \"achilles\", \"integrator\": \"%s\", \"dt\": %.9g, "
      "\"copies\": %zu, \"padded_rows\": %zu, \"simd_lanes\": %zu, "
      "\"steps\": %ld, \"trials\": %d, \"best_s\": %.6f, \"median_s\": %.6f, "
      "\"batch_steps_per_s\": %.1f, \"robot_steps_per_s\": %.1f, "
      "\"finite\": %s}\n",
      argv[4], static_cast<double>(dt), copies, padded_rows,
      achilles::algorithms::BatchOperationT::size, steps, trials, best, median,
      static_cast<double>(steps) / best,
      static_cast<double>(steps) * static_cast<double>(copies) / best,
      AllFinite(sim) ? "true" : "false"
  );
  return 0;
}

template <typename SixT>
void WriteSix(std::ostream& out, const SixT& x) {
  auto ang = x.Angular();
  auto lin = x.Linear();
  out << ',' << ang.X() << ',' << ang.Y() << ',' << ang.Z() << ',' << lin.X()
      << ',' << lin.Y() << ',' << lin.Z();
}

int Trace(int argc, char** argv) {
  if (argc < 8) {
    std::cerr << "trace <arow> <config> <integrator> <dt> <steps> <out.csv>\n";
    return 1;
  }
  std::optional<Integrator> integrator = ParseIntegrator(argv[4]);
  if (!integrator.has_value()) {
    std::cerr << "unknown integrator " << argv[4] << '\n';
    return 1;
  }
  const float dt = std::strtof(argv[5], nullptr);
  const long steps = std::atol(argv[6]);

  // `wrapped` loads the scene the way Throughput does (one copy hung off
  // bench_world) -- the trace then starts with bench_world's own row, and
  // should otherwise match the unwrapped trace exactly.
  const bool wrapped = argc > 8 && std::string_view(argv[8]) == "wrapped";
  std::filesystem::path arow =
      wrapped ? WithCopies(argv[2], 1) : std::filesystem::path(argv[2]);
  Simulation sim;
  bool ok = MakeSim(sim, arow.string(), argv[3], *integrator);
  if (wrapped) {
    std::filesystem::remove_all(arow.parent_path());
  }
  if (!ok) {
    return 1;
  }
  std::vector<std::size_t> rows =
      ChainRows(sim.TopologyFor<TopologicalOrdering>());

  std::ofstream out(argv[7]);
  out.precision(17);
  out << "t";
  for (std::size_t j = 0; j < rows.size(); ++j) {
    out << ",j" << j << "_qw,j" << j << "_qx,j" << j << "_qy,j" << j << "_qz";
    for (int k = 0; k < 6; ++k) {
      out << ",j" << j << "_v" << k;
    }
    for (int k = 0; k < 6; ++k) {
      out << ",j" << j << "_a" << k;
    }
  }
  out << '\n';

  auto record = [&](double t) {
    sim.RefreshAccelerationForCurrentState();
    auto view = sim.ViewFor<ABAAlgorithm>();
    out << t;
    for (std::size_t row : rows) {
      auto q = view.Load<ABAField::kJointPosition, ScalarOperationT>(row);
      out << ',' << q.Rotation().W() << ',' << q.Rotation().X() << ','
          << q.Rotation().Y() << ',' << q.Rotation().Z();
      WriteSix(out, view.Load<ABAField::kJointVelocity, ScalarOperationT>(row));
      WriteSix(
          out, view.Load<ABAField::kJointAcceleration, ScalarOperationT>(row)
      );
    }
    out << '\n';
  };

  record(0.0);
  for (long i = 1; i <= steps; ++i) {
    if (!sim.Step(dt)) {
      return 1;
    }
    record(static_cast<double>(i) * static_cast<double>(dt));
  }
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  std::string_view mode = argc > 1 ? argv[1] : "";
  if (mode == "throughput") {
    return Throughput(argc, argv);
  }
  if (mode == "trace") {
    return Trace(argc, argv);
  }
  std::cerr << "usage: achilles_bench throughput|trace ...\n";
  return 1;
}
