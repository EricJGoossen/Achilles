#pragma once

#include "algorithms/conventions.hpp"
#include "algorithms/sim_config.hpp"
#include "algorithms/verlet/verlet_data.hpp"
#include "algorithms/verlet/verlet_ops.hpp"
#include "engine/algorithm_contract.hpp"
#include "engine/pass/algorithm_step.hpp"
#include "engine/pass/traversals.hpp"
#include "engine/pass/view_step.hpp"
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
using ForwardBatched =
    engine::pass::LinearTraversal<engine::pass::Direction::kForward, kLanes>;

using VerletAlgorithm = engine::Algorithm<VerletField, VerletFieldTraits>;
static_assert(engine::AlgorithmLike<VerletAlgorithm>);

// qd += qdd * (DtFraction * dt).
template <typename DtFraction>
struct KickStep
    : engine::pass::ViewStep<KickStep<DtFraction>, VerletAlgorithm> {
  static auto MakeOps(const SimConfig& /*sim_config*/, ScalarOperationT dt) {
    using engine::pass::Pass;
    return engine::pass::Ops<Pass<KickVelocityOp, ForwardBatched>>{
        KickVelocityOp(dt * util::kRatioValueV<ScalarOperationT, DtFraction>)
    };
  }
};

// q = q * Exp(S qd dt).
struct DriftStep : engine::pass::ViewStep<DriftStep, VerletAlgorithm> {
  static auto MakeOps(const SimConfig& /*sim_config*/, ScalarOperationT dt) {
    using engine::pass::Pass;
    return engine::pass::Ops<Pass<DriftPositionOp, ForwardBatched>>{
        DriftPositionOp(dt)
    };
  }
};

}  // namespace achilles::algorithms::verlet
