#pragma once

#include "algorithms/collision/collision_data.hpp"
#include "algorithms/collision/collision_ops.hpp"
#include "algorithms/conventions.hpp"
#include "algorithms/sim_config.hpp"
#include "engine/algorithm_contract.hpp"
#include "engine/pass/algorithm_step.hpp"
#include "engine/pass/sim_context.hpp"
#include "engine/pass/traversals.hpp"
#include "util/tmp.hpp"

namespace achilles::algorithms::collision {

// Runs the broad phase's per-body pass for a single archetype. `sim_state`
// supplies this Step's own CollisionView, looked up via
// ViewFor<CollisionView>() the same way VIStep looks up its own VIView (see
// vi_step.hpp's header comment). Like VI/PI, ComputeWorldBoundsOp reads/
// writes each row independently -- a body's world bound depends only on its
// own already-propagated kWorldTransform, never on its parent's -- so
// ForwardBatched below is a LinearTraversal driven by view.Size() rather
// than a TreeTraversal driven by a topology. `sim_config` and `dt` are both
// unused: the bound is a pure function of this tick's pose.
struct CollisionStep {
  using FieldEnum = CollisionField;

  using ForwardBatched = engine::pass::LinearTraversal<
      engine::pass::Direction::kForward,
      util::LaneCountOf<MathematicalT>()>;

  template <engine::pass::SimContextLike SimStateT>
  static void Step(
      const SimStateT& sim_state, const SimConfig& sim_config, float dt
  ) {
    (void)sim_config;
    (void)dt;
    using engine::pass::Pass;

    const auto view = sim_state.template ViewFor<CollisionView>();

    engine::pass::Step(
        engine::pass::Ops<Pass<ComputeWorldBoundsOp, ForwardBatched>>{
            ComputeWorldBoundsOp{}
        },
        view,
        view.Size()
    );
  }
};

using CollisionAlgorithm =
    engine::Algorithm<CollisionField, CollisionFieldTraits, CollisionStep>;
static_assert(engine::AlgorithmLike<CollisionAlgorithm>);

}  // namespace achilles::algorithms::collision
