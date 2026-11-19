#pragma once

#include "algorithms/base/every_row_step.hpp"
#include "algorithms/conventions.hpp"
#include "algorithms/sim_config.hpp"
#include "algorithms/verlet/verlet_data.hpp"
#include "algorithms/verlet/verlet_ops.hpp"
#include "engine/algorithm.hpp"
#include "engine/op_invoker/algorithm_step.hpp"
#include "engine/spec/step_contract.hpp"
#include "policy/traversal/base_traversal.hpp"
#include "policy/traversal/global_traversal.hpp"
#include "policy/traversal/pair_traversal.hpp"
#include "policy/traversal/row_traversal.hpp"
#include "util/tmp.hpp"

namespace achilles::algorithms::verlet {

// Velocity Verlet's two stages. One tick is the step list
// algorithms::VelocityVerlet (registry.hpp): ABA at (q, qd) -> half kick ->
// full drift from the half-kicked velocity -> ABA again at the new position
// -> the other half kick.
//
// Local truncation error is one order better than semi-implicit Euler's
// (O(dt^3) per step vs O(dt^2)) for the *position-dependent* part of the
// dynamics. ABA's Coriolis/gyroscopic bias terms are velocity-dependent, and
// the second ABA evaluates them at the half-kicked velocity rather than the
// not-yet-known qd_{n+1}, so that piece doesn't get the full second-order
// benefit -- see algorithms/midpoint for the time-symmetric scheme that
// does.
//
// Like vi/pi, each pass is independent per row, so it runs as a
// LinearTraversal over view.Size().
using ForwardBatched = policy::traversal::
    LinearTraversal<policy::traversal::Direction::kForward, kLanes>;

using VerletAlgorithm = engine::Algorithm<VerletField, VerletFieldTraits>;
static_assert(engine::AlgorithmLike<VerletAlgorithm>);

// qd += qdd * (DtFraction * dt).
template <typename DtFraction>
struct KickStep : base::EveryRowStep<KickStep<DtFraction>, VerletAlgorithm> {
  static auto MakeOps(const SimConfig& /*sim_config*/, ScalarOperationT dt) {
    using engine::op_invoker::OpWalk;
    return engine::op_invoker::Ops<OpWalk<KickVelocityOp, ForwardBatched>>{
        KickVelocityOp(dt * util::kRatioValueV<ScalarOperationT, DtFraction>)
    };
  }
};

// q = q * Exp(S qd dt).
struct DriftStep : base::EveryRowStep<DriftStep, VerletAlgorithm> {
  static auto MakeOps(const SimConfig& /*sim_config*/, ScalarOperationT dt) {
    using engine::op_invoker::OpWalk;
    return engine::op_invoker::Ops<OpWalk<DriftPositionOp, ForwardBatched>>{
        DriftPositionOp(dt)
    };
  }
};

}  // namespace achilles::algorithms::verlet
