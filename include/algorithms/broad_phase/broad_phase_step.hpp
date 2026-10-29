#pragma once

#include "algorithms/broad_phase/broad_phase_data.hpp"
#include "algorithms/broad_phase/broad_phase_ops.hpp"
#include "algorithms/conventions.hpp"
#include "algorithms/orderings/collision_pairs.hpp"
#include "algorithms/sim_config.hpp"
#include "domain/joint_topology.hpp"
#include "engine/algorithm_contract.hpp"
#include "engine/pass/algorithm_step.hpp"
#include "engine/pass/sim_context.hpp"
#include "engine/pass/traversals.hpp"
#include "engine/topology/orderings.hpp"
#include "util/tmp.hpp"

namespace achilles::algorithms::broad_phase {

// Runs the broad phase for a single archetype: two passes over this Step's
// own BroadPhaseView, looked up via ViewFor<BroadPhaseView>(). A hand-written
// Step like aba::ABAStep rather than an engine::pass::ViewStep, since its
// pair pass is driven by the topology, not by view.Size().
//
// ComputeSweptBoxBoundsOp first: like VI/PI, it reads/writes each row
// independently -- a body's world bound depends only on its own already-
// propagated kWorldTransform, never on its parent's -- so ForwardBatched is
// a LinearTraversal rather than a TreeTraversal. Then TestBoundsOverlapOp,
// over PairBatched: every lane group of CollisionPairs
// (orderings/collision_pairs.hpp, see engine::pass::PairTraversal), reading the
// bounds the first pass just wrote at each pair's two bodies, and handed which
// lanes are pairs at all. The linear pass walks the TopologicalOrdering
// topology every body field is ordered by, the pair pass the pairs' own
// topology. `sim_config` and `dt` are both unused: the broad phase is a pure
// function of this tick's pose.
struct BroadPhaseStep {
  using FieldEnum = BroadPhaseField;

  using ForwardBatched =
      engine::pass::LinearTraversal<engine::pass::Direction::kForward, kLanes>;
  using PairBatched =
      engine::pass::PairTraversal<engine::pass::Direction::kForward, kLanes>;

  template <engine::pass::SimContextLike SimStateT>
  static void Step(
      const SimStateT& sim_state,
      const SimConfig& sim_config,
      ScalarOperationT dt
  ) {
    (void)sim_config;
    (void)dt;
    using engine::pass::Pass;

    const auto view = sim_state.template ViewFor<BroadPhaseView>();
    const BroadPhaseTopology& topology =
        sim_state.template TopologyFor<engine::topology::TopologicalOrdering>();
    const domain::PairTopology& pairs =
        sim_state.template TopologyFor<CollisionPairs>();

    engine::pass::Step(
        engine::pass::Ops<
            Pass<ComputeSweptBoxBoundsOp, ForwardBatched>,
            Pass<TestBoundsOverlapOp, PairBatched>>{
            ComputeSweptBoxBoundsOp{}, TestBoundsOverlapOp{}
        },
        view,
        topology,
        pairs
    );
  }
};

using BroadPhaseAlgorithm =
    engine::Algorithm<BroadPhaseField, BroadPhaseFieldTraits>;
static_assert(engine::AlgorithmLike<BroadPhaseAlgorithm>);

}  // namespace achilles::algorithms::broad_phase
