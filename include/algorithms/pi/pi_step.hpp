#pragma once

#include "algorithms/conventions.hpp"
#include "algorithms/pi/pi_data.hpp"
#include "algorithms/pi/pi_ops.hpp"
#include "algorithms/sim_config.hpp"
#include "engine/algorithm_contract.hpp"
#include "engine/pass/algorithm_step.hpp"
#include "engine/pass/traversals.hpp"
#include "engine/pass/view_step.hpp"
#include "util/tmp.hpp"

namespace achilles::algorithms::pi {

using PIAlgorithm = engine::Algorithm<PIField, PIFieldTraits>;
static_assert(engine::AlgorithmLike<PIAlgorithm>);

// Runs one position-integration step over every row of PIView (see
// engine::pass::ViewStep). Like VI, PI has no JointTopology of its own --
// IntegratePositionOp reads/writes each row independently (composing each
// joint's own pose forward by its own velocity, with no parent/child
// dependency), so ForwardBatched below is a LinearTraversal driven by
// view.Size() rather than a TreeTraversal driven by a topology.
struct PIStep : engine::pass::ViewStep<PIStep, PIAlgorithm> {
  using ForwardBatched =
      engine::pass::LinearTraversal<engine::pass::Direction::kForward, kLanes>;

  static auto MakeOps(const SimConfig& /*sim_config*/, ScalarOperationT dt) {
    using engine::pass::Pass;
    return engine::pass::Ops<Pass<IntegratePositionOp, ForwardBatched>>{
        IntegratePositionOp(dt)
    };
  }
};

}  // namespace achilles::algorithms::pi
