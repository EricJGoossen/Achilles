#pragma once

#include <cstdint>

#include "algorithms/conventions.hpp"

namespace achilles::algorithms {

// Which of MuJoCo's primal constraint solvers the contact solver runs
// (opt.solver). Both minimize the same cost, so a converged answer doesn't
// depend on the choice; they differ in how fast they get there.
enum class ContactSolver : std::uint8_t {
  kNewton,  // H^-1 grad steps, H = M + J^T D J (MuJoCo's default)
  kCG,      // M^-1-preconditioned conjugate gradient

  kCount
};

// The contact solver's settings: which solver, and when it stops early.
// Example: config.contact_solver.solver = ContactSolver::kCG;
struct ContactSolverConfig {
  ContactSolver solver = ContactSolver::kNewton;
  // The solve stops once an iteration's expected improvement, scaled by
  // 1 / trace(M) as MuJoCo scales it, is below this (opt.tolerance).
  // MuJoCo's own 1e-8 is for doubles; in floats rounding keeps the
  // improvement from getting that small, so the solve would run to its
  // iteration cap.
  float tolerance = 1e-6F;
};

// The whole-simulation, caller-supplied config every hosted Algorithm's Step
// may read (see engine::pass::SimContext::Step/engine::pass::Step, both
// generic over whatever Config type a caller passes -- this is just the one
// this codebase actually uses). One concrete struct for the whole sim, not
// a per-algorithm/templated shape: today only ABA reads
// world_base_transform/base_velocity/base_acceleration (its base-row seed
// state -- base_acceleration is typically -gravity for the common
// fixed-base case), but adding an algorithm that needs its own static
// config means adding a field here, the same "new field, not new plumbing"
// shape Traits<F> already has for a field's own per-tick data.
//
// Lives here, not engine::pass, so engine::pass never has to name
// algorithms::Transform/Velocity/Acceleration -- the generic engine layer
// stays generic, and only algorithms (and whatever composes it, e.g.
// interface::Simulation) needs to know this struct's shape.
struct SimConfig {
  Transform world_base_transform = Transform::Identity();
  Velocity base_velocity = Velocity::Zero();
  Acceleration base_acceleration = Acceleration::Zero();
  ContactSolverConfig contact_solver;

  // Same values as a default-constructed SimConfig -- named, the same way
  // Transform::Identity()/Velocity::Zero() are, so interface::LoadSimConfig
  // (interface/sim_config_loader.hpp) has an explicit thing to fall back
  // to (a missing file, unparsable YAML, or one bad field) rather than
  // relying on an implicit {}.
  static SimConfig Default() { return SimConfig{}; }
};

}  // namespace achilles::algorithms
