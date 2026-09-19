#pragma once

#include "algorithms/conventions.hpp"
#include "algorithms/sim_config.hpp"
#include "algorithms/vi/vi_data.hpp"
#include "algorithms/vi/vi_ops.hpp"
#include "engine/algorithm_contract.hpp"
#include "engine/pass/algorithm_step.hpp"
#include "engine/pass/sim_context.hpp"
#include "engine/pass/traversals.hpp"
#include "util/tmp.hpp"

namespace achilles::algorithms::vi {

// Runs one velocity-integration step for a single archetype. `sim_state`
// supplies this Step's own VIView, looked up via ViewFor<VIView>() the same
// way ABAStep looks up its own ABAView (see aba_step.hpp's comment,
// including why SimStateT stays constrained by engine::pass::SimContextLike
// rather than naming SimContext<...> directly). Unlike ABA, VI has no
// JointTopology to look up -- IntegrateVelocityOp reads/writes each row
// independently, so ForwardBatched below is a LinearTraversal driven by
// view.Size() rather than a TreeTraversal driven by a topology.
struct VIStep {
  using FieldEnum = VIField;

  using ForwardBatched = engine::pass::LinearTraversal<
      engine::pass::Direction::kForward,
      util::LaneCountOf<MathematicalT>()>;

  template <engine::pass::SimContextLike SimStateT>
  static void Step(
      const SimStateT& sim_state, const SimConfig& sim_config, float dt
  ) {
    (void)sim_config;
    using engine::pass::Pass;

    const auto view = sim_state.template ViewFor<VIView>();

    engine::pass::Step(
        engine::pass::Ops<Pass<IntegrateVelocityOp, ForwardBatched>>{
            IntegrateVelocityOp(dt)
        },
        view,
        view.Size()
    );
  }
};

using VIAlgorithm = engine::Algorithm<VIField, VIFieldTraits, VIStep>;
static_assert(engine::AlgorithmLike<VIAlgorithm>);

}  // namespace achilles::algorithms::vi
