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

// Runs the broad phase for a single archetype: two passes over this Step's
// own CollisionView, looked up via ViewFor<CollisionView>() the same way
// VIStep looks up its own VIView (see vi_step.hpp's header comment).
//
// ComputeWorldBoundsOp first: like VI/PI, it reads/writes each row
// independently -- a body's world bound depends only on its own already-
// propagated kWorldTransform, never on its parent's -- so ForwardBatched is
// a LinearTraversal driven by view.Size() rather than a TreeTraversal driven
// by a topology. Then TestBoundsOverlapOp, over PairBatched: every row
// against every lane group of rows from its own group on (see
// engine::pass::PairTraversal), reading the bounds the first pass just
// wrote. Both are driven by the same view.Size(), which includes the
// reserved base row and lane filler -- those read as Zero() groups, so the
// filter drops them like any other padding row. `sim_config` and `dt` are
// both unused: the broad phase is a pure function of this tick's pose.
struct CollisionStep {
  using FieldEnum = CollisionField;

  using ForwardBatched = engine::pass::LinearTraversal<
      engine::pass::Direction::kForward,
      util::LaneCountOf<MathematicalT>()>;
  using PairBatched = engine::pass::
      PairTraversal<engine::pass::Direction::kForward, kPairLanes>;

  template <engine::pass::SimContextLike SimStateT>
  static void Step(
      const SimStateT& sim_state, const SimConfig& sim_config, float dt
  ) {
    (void)sim_config;
    (void)dt;
    using engine::pass::Pass;

    const auto view = sim_state.template ViewFor<CollisionView>();

    engine::pass::Step(
        engine::pass::Ops<
            Pass<ComputeWorldBoundsOp, ForwardBatched>,
            Pass<TestBoundsOverlapOp, PairBatched>>{
            ComputeWorldBoundsOp{}, TestBoundsOverlapOp{}
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
