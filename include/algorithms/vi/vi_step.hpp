#pragma once

#include "algorithms/base/every_row_step.hpp"
#include "algorithms/conventions.hpp"
#include "algorithms/sim_config.hpp"
#include "algorithms/vi/vi_data.hpp"
#include "algorithms/vi/vi_ops.hpp"
#include "engine/algorithm.hpp"
#include "engine/op_invoker/algorithm_step.hpp"
#include "engine/spec/step_contract.hpp"
#include "policy/traversal/base_traversal.hpp"
#include "policy/traversal/global_traversal.hpp"
#include "policy/traversal/pair_traversal.hpp"
#include "policy/traversal/row_traversal.hpp"
#include "util/tmp.hpp"

namespace achilles::algorithms::vi {

using VIAlgorithm = engine::Algorithm<VIField, VIFieldTraits>;
static_assert(engine::AlgorithmLike<VIAlgorithm>);

// Runs one velocity-integration step over every row of VIView (see
// base::EveryRowStep). Unlike ABA, VI has no JointTopology to look up --
// IntegrateVelocityOp reads/writes each row independently, so
// ForwardBatched below is a LinearTraversal driven by view.Size() rather
// than a TreeTraversal driven by a topology.
struct VIStep : base::EveryRowStep<VIStep, VIAlgorithm> {
  using ForwardBatched = policy::traversal::
      LinearTraversal<policy::traversal::Direction::kForward, kLanes>;

  static auto MakeOps(const SimConfig& /*sim_config*/, ScalarOperationT dt) {
    using engine::op_invoker::OpWalk;
    return engine::op_invoker::Ops<OpWalk<IntegrateVelocityOp, ForwardBatched>>{
        IntegrateVelocityOp(dt)
    };
  }
};

}  // namespace achilles::algorithms::vi
