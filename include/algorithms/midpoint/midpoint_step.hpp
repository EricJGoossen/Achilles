#pragma once

#include "algorithms/base/every_row_step.hpp"
#include "algorithms/conventions.hpp"
#include "algorithms/midpoint/midpoint_data.hpp"
#include "algorithms/midpoint/midpoint_ops.hpp"
#include "algorithms/sim_config.hpp"
#include "engine/algorithm.hpp"
#include "engine/op_invoker/algorithm_step.hpp"
#include "engine/spec/step_contract.hpp"
#include "policy/traversal/base_traversal.hpp"
#include "policy/traversal/global_traversal.hpp"
#include "policy/traversal/pair_traversal.hpp"
#include "policy/traversal/row_traversal.hpp"
#include "util/tmp.hpp"

namespace achilles::algorithms::midpoint {

// Implicit midpoint's stages. One tick is the step list
// algorithms::ImplicitMidpoint (registry.hpp): BeginStep, then a fixed
// number of (TrialStep, ABA, UpdateStep) fixed-point iterations, then
// CommitStep.
//
// Implicit midpoint evaluates ABA's forward dynamics at the time-symmetric
// AVERAGE of the old and new state, rather than at the old state alone
// (semi-implicit Euler) or an extrapolated half-kicked velocity
// (algorithms/verlet). For a system with ABA's velocity-dependent
// Coriolis/gyroscopic bias terms, those plain schemes lose their
// bounded-energy-error guarantee; the midpoint rule doesn't:
// qd_mid = (qd_n + qd_{n+1})/2 and q_mid = q_n * Exp(S qd_mid dt/2) are
// exactly the state the bias terms should be evaluated at for a
// time-centered (and, for a constant mass matrix, exactly
// energy-conserving) step.
//
// The cost is that qd_{n+1} appears on both sides of that definition, so
// this is a genuine implicit method: each fixed-point iteration is one full
// ABA evaluation. TrialStep writes the trial into kJointPosition/
// kJointVelocity, because that's the only way ABA receives a joint's state,
// and UpdateStep folds ABA's qdd back into the estimate. The tick's starting
// state lives in this algorithm's own scratch fields (BeginStep) so
// CommitStep can build the final state from it.
//
// Like vi/pi, each pass is independent per row, so it runs as a
// LinearTraversal over view.Size().
using ForwardBatched = policy::traversal::
    LinearTraversal<policy::traversal::Direction::kForward, kLanes>;

using MidpointAlgorithm = engine::Algorithm<MidpointField, MidpointFieldTraits>;
static_assert(engine::AlgorithmLike<MidpointAlgorithm>);

// Snapshots (q_n, qd_n) and seeds the estimate qd_{n+1} = qd_n.
struct BeginStep : base::EveryRowStep<BeginStep, MidpointAlgorithm> {
  static auto
  MakeOps(const SimConfig& /*sim_config*/, ScalarOperationT /*dt*/) {
    using engine::op_invoker::OpWalk;
    return engine::op_invoker::Ops<OpWalk<BeginStepOp, ForwardBatched>>{
        BeginStepOp()
    };
  }
};

// Writes the midpoint trial for the current estimate where ABA reads it.
struct TrialStep : base::EveryRowStep<TrialStep, MidpointAlgorithm> {
  static auto MakeOps(const SimConfig& /*sim_config*/, ScalarOperationT dt) {
    using engine::op_invoker::OpWalk;
    return engine::op_invoker::Ops<OpWalk<MidpointTrialOp, ForwardBatched>>{
        MidpointTrialOp(dt)
    };
  }
};

// Folds ABA's qdd at the trial back into the estimate of qd_{n+1}.
struct UpdateStep : base::EveryRowStep<UpdateStep, MidpointAlgorithm> {
  static auto MakeOps(const SimConfig& /*sim_config*/, ScalarOperationT dt) {
    using engine::op_invoker::OpWalk;
    return engine::op_invoker::Ops<
        OpWalk<UpdateNextVelocityOp, ForwardBatched>>{UpdateNextVelocityOp(dt)};
  }
};

// Commits (q_{n+1}, qd_{n+1}) from the converged estimate.
struct CommitStep : base::EveryRowStep<CommitStep, MidpointAlgorithm> {
  static auto MakeOps(const SimConfig& /*sim_config*/, ScalarOperationT dt) {
    using engine::op_invoker::OpWalk;
    return engine::op_invoker::Ops<OpWalk<CommitOp, ForwardBatched>>{CommitOp(dt
    )};
  }
};

}  // namespace achilles::algorithms::midpoint
