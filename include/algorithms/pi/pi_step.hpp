#pragma once

#include "algorithms/base/every_row_step.hpp"
#include "algorithms/conventions.hpp"
#include "algorithms/pi/pi_data.hpp"
#include "algorithms/pi/pi_ops.hpp"
#include "algorithms/sim_config.hpp"
#include "engine/algorithm.hpp"
#include "engine/op_invoker/algorithm_step.hpp"
#include "engine/spec/step_contract.hpp"
#include "policy/traversal/base_traversal.hpp"
#include "policy/traversal/global_traversal.hpp"
#include "policy/traversal/pair_traversal.hpp"
#include "policy/traversal/row_traversal.hpp"
#include "util/tmp.hpp"

namespace achilles::algorithms::pi {

using PIAlgorithm = engine::Algorithm<PIField, PIFieldTraits>;
static_assert(engine::AlgorithmLike<PIAlgorithm>);

// Runs one position-integration step over every row of PIView (see
// base::EveryRowStep). Like VI, PI has no JointTopology of its own --
// IntegratePositionOp reads/writes each row independently (composing each
// joint's own pose forward by its own velocity, with no parent/child
// dependency), so ForwardBatched below is a LinearTraversal driven by
// view.Size() rather than a TreeTraversal driven by a topology.
struct PIStep : base::EveryRowStep<PIStep, PIAlgorithm> {
  using ForwardBatched = policy::traversal::
      LinearTraversal<policy::traversal::Direction::kForward, kLanes>;

  static auto MakeOps(const SimConfig& /*sim_config*/, ScalarOperationT dt) {
    using engine::op_invoker::OpWalk;
    return engine::op_invoker::Ops<OpWalk<IntegratePositionOp, ForwardBatched>>{
        IntegratePositionOp(dt)
    };
  }
};

}  // namespace achilles::algorithms::pi
