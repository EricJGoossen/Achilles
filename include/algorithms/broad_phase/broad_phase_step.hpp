#pragma once

#include "algorithms/broad_phase/broad_phase_data.hpp"
#include "algorithms/broad_phase/broad_phase_ops.hpp"
#include "algorithms/conventions.hpp"
#include "algorithms/sim_config.hpp"
#include "engine/algorithm_contract.hpp"
#include "engine/pass/algorithm_step.hpp"
#include "engine/pass/sim_context.hpp"
#include "engine/pass/traversals.hpp"
#include "engine/topology/ordering_policy.hpp"
#include "util/tmp.hpp"

namespace achilles::algorithms::broad_phase {

// Runs the broad phase for a single archetype: two passes over this Step's
// own BroadPhaseView, looked up via ViewFor<BroadPhaseView>() the same way
// VIStep looks up its own VIView (see vi_step.hpp's header comment).
//
// ComputeSweptBoxBoundsOp first: like VI/PI, it reads/writes each row
// independently -- a body's world bound depends only on its own already-
// propagated kWorldTransform, never on its parent's -- so ForwardBatched is
// a LinearTraversal rather than a TreeTraversal. Then TestBoundsOverlapOp,
// over PairBatched: every row against every lane group of rows from its own
// group on (see engine::pass::PairTraversal), reading the bounds the first
// pass just wrote, and handed which lanes are distinct, non-parent/child
// pairs by the traversal itself. Both are driven by the same
// TopologicalOrdering topology every collision field is ordered by -- its
// Size() for the linear pass, and its parent rows too for the pair pass.
// Padding rows inside it read as Zero() groups, so the filter drops them.
// `sim_config` and `dt` are both unused: the broad phase is a pure function of
// this tick's pose.
struct BroadPhaseStep {
  using FieldEnum = BroadPhaseField;

  using ForwardBatched =
      engine::pass::LinearTraversal<engine::pass::Direction::kForward, kLanes>;
  using PairBatched =
      engine::pass::PairTraversal<engine::pass::Direction::kForward, kLanes>;

  template <engine::pass::SimContextLike SimStateT>
  static void Step(
      const SimStateT& sim_state, const SimConfig& sim_config, float dt
  ) {
    (void)sim_config;
    (void)dt;
    using engine::pass::Pass;

    const auto view = sim_state.template ViewFor<BroadPhaseView>();
    const BroadPhaseTopology& topology =
        sim_state.template TopologyFor<engine::topology::TopologicalOrdering>();

    engine::pass::Step(
        engine::pass::Ops<
            Pass<ComputeSweptBoxBoundsOp, ForwardBatched>,
            Pass<TestBoundsOverlapOp, PairBatched>>{
            ComputeSweptBoxBoundsOp{}, TestBoundsOverlapOp{}
        },
        view,
        topology
    );
  }
};

using BroadPhaseAlgorithm =
    engine::Algorithm<BroadPhaseField, BroadPhaseFieldTraits, BroadPhaseStep>;
static_assert(engine::AlgorithmLike<BroadPhaseAlgorithm>);

}  // namespace achilles::algorithms::broad_phase
