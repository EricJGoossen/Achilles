#pragma once

#include "algorithms/conventions.hpp"
#include "algorithms/sim_config.hpp"
#include "algorithms/vi/vi_data.hpp"
#include "algorithms/vi/vi_ops.hpp"
#include "engine/algorithm_contract.hpp"
#include "engine/pass/algorithm_step.hpp"
#include "engine/pass/traversals.hpp"
#include "engine/pass/view_step.hpp"
#include "util/tmp.hpp"

namespace achilles::algorithms::vi {

using VIAlgorithm = engine::Algorithm<VIField, VIFieldTraits>;
static_assert(engine::AlgorithmLike<VIAlgorithm>);

// Runs one velocity-integration step over every row of VIView (see
// engine::pass::ViewStep). Unlike ABA, VI has no JointTopology to look up --
// IntegrateVelocityOp reads/writes each row independently, so
// ForwardBatched below is a LinearTraversal driven by view.Size() rather
// than a TreeTraversal driven by a topology.
struct VIStep : engine::pass::ViewStep<VIStep, VIAlgorithm> {
  using ForwardBatched = engine::pass::LinearTraversal<
      engine::pass::Direction::kForward,
      util::LaneCountOf<MathematicalT>()>;

  static auto MakeOps(const SimConfig& /*sim_config*/, ScalarOperationT dt) {
    using engine::pass::Pass;
    return engine::pass::Ops<Pass<IntegrateVelocityOp, ForwardBatched>>{
        IntegrateVelocityOp(dt)
    };
  }
};

}  // namespace achilles::algorithms::vi
