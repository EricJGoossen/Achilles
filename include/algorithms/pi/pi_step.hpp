#pragma once

#include "algorithms/conventions.hpp"
#include "algorithms/pi/pi_data.hpp"
#include "algorithms/pi/pi_ops.hpp"
#include "algorithms/sim_config.hpp"
#include "engine/algorithm_contract.hpp"
#include "engine/pass/algorithm_step.hpp"
#include "engine/pass/sim_context.hpp"
#include "engine/pass/traversals.hpp"
#include "util/tmp.hpp"

namespace achilles::algorithms::pi {

// Runs one position-integration step for a single archetype. `sim_state`
// supplies this Step's own PIView, looked up via ViewFor<PIView>() the same
// way VIStep looks up its own VIView (see vi_step.hpp's header comment).
// Like VI, PI has no JointTopology of its own -- IntegratePositionOp reads/
// writes each row independently (composing each joint's own pose forward
// by its own velocity, with no parent/child dependency), so ForwardBatched
// below is a LinearTraversal driven by view.Size() rather than a
// TreeTraversal driven by a topology.
struct PIStep {
  using FieldEnum = PIField;

  using ForwardBatched = engine::pass::LinearTraversal<
      engine::pass::Direction::kForward,
      util::LaneCountOf<MathematicalT>()>;

  template <engine::pass::SimContextLike SimStateT>
  static void Step(
      const SimStateT& sim_state, const SimConfig& sim_config, float dt
  ) {
    (void)sim_config;
    using engine::pass::Pass;

    const auto view = sim_state.template ViewFor<PIView>();

    engine::pass::Step(
        engine::pass::Ops<Pass<IntegratePositionOp, ForwardBatched>>{
            IntegratePositionOp(dt)
        },
        view,
        view.Size()
    );
  }
};

using PIAlgorithm = engine::Algorithm<PIField, PIFieldTraits, PIStep>;
static_assert(engine::AlgorithmLike<PIAlgorithm>);

}  // namespace achilles::algorithms::pi
