#pragma once

#include <cstddef>
#include <cstdint>

#include "algorithms/conventions.hpp"

namespace achilles::algorithms {

// Which of MuJoCo's primal solvers the contact solver runs
// (algorithms/contact_solver): both minimize the same cost, so a scene's
// answer doesn't depend on the choice, only how fast it gets there.
//
//   kNewton (MuJoCo's default) -- each direction is -H^-1 grad, with the
//     Hessian H = M + J^T D J factorized once per tick the way ABA
//     factorizes M (articulated-body inertias, each body's plus its own
//     contact rows' D w w^T). Exact when every contact is against a body
//     that doesn't move with the tree (a floor, a wall); between two moving
//     bodies the cross terms are left out, which still converges, just not
//     quadratically. A few iterations.
//   kCG -- Hager-Zhang conjugate gradient in M's metric. Cheaper per
//     iteration, many more of them.
enum class ContactSolver : std::uint8_t { kNewton, kCG };

struct ContactSolverConfig {
  ContactSolver solver = ContactSolver::kNewton;
  // The most iterations a tick runs; fewer once converged.
  std::size_t iterations = 20;
  // MuJoCo's opt.tolerance, on its improvement test: stop once the step the
  // next direction predicts, 1/2 grad^T P grad (P = H^-1 for Newton, M^-1
  // for CG), scaled by 1 / trace(M) (MuJoCo's 1 / (meaninertia nv)), is
  // below it. 0 never stops early: always `iterations`.
  float tolerance = 1e-8F;
  // Newton only: refactor H at every iteration, not once per tick -- exact
  // Newton when the set of active rows changes mid-solve, at one more
  // backward pass per iteration.
  bool refactor_every_iteration = false;
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
