#pragma once

#include "algorithms/broad_phase/broad_phase_data.hpp"
#include "algorithms/broad_phase/broad_phase_ops.hpp"
#include "algorithms/conventions.hpp"
#include "algorithms/row_sets/collision_pairs.hpp"
#include "algorithms/sim_config.hpp"
#include "domain/topology/joint_topology.hpp"
#include "domain/topology/pair_topology.hpp"
#include "engine/algorithm.hpp"
#include "engine/context/sim_context.hpp"
#include "engine/op_invoker/algorithm_step.hpp"
#include "engine/spec/step_contract.hpp"
#include "policy/ordering/base_ordering.hpp"
#include "policy/ordering/linear_ordering.hpp"
#include "policy/ordering/list_ordering.hpp"
#include "policy/ordering/pair_ordering.hpp"
#include "policy/ordering/topological_ordering.hpp"
#include "policy/traversal/base_traversal.hpp"
#include "policy/traversal/global_traversal.hpp"
#include "policy/traversal/pair_traversal.hpp"
#include "policy/traversal/row_traversal.hpp"
#include "util/tmp.hpp"

namespace achilles::algorithms::broad_phase {

// Runs the broad phase for a single archetype: two passes over this Step's
// own BroadPhaseView, looked up via ViewFor<BroadPhaseView>(). A hand-written
// Step like aba::ABAStep rather than an base::EveryRowStep, since its
// pair pass is driven by the topology, not by view.Size().
//
// ComputeSweptBoxBoundsOp first: like VI/PI, it reads/writes each row
// independently -- a body's world bound depends only on its own already-
// propagated kWorldTransform, never on its parent's -- so ForwardBatched is
// a LinearTraversal rather than a TreeTraversal. Then TestBoundsOverlapOp,
// over PairBatched: every lane group of CollisionPairs
// (row_sets/collision_pairs.hpp, see policy::traversal::PairTraversal), reading
// the bounds the first pass just wrote at each pair's two bodies, and handed
// which lanes are pairs at all. The linear pass walks the TopologicalOrdering
// topology every body field is ordered by, the pair pass the pairs' own
// topology. `sim_config` and `dt` are both unused: the broad phase is a pure
// function of this tick's pose.
struct BroadPhaseStep {
  using FieldEnum = BroadPhaseField;

  using ForwardBatched = policy::traversal::
      LinearTraversal<policy::traversal::Direction::kForward, kLanes>;
  using PairBatched = policy::traversal::
      PairTraversal<policy::traversal::Direction::kForward, kLanes>;

  template <engine::context::SimContextLike SimStateT>
  static void Step(
      const SimStateT& sim_state,
      const SimConfig& sim_config,
      ScalarOperationT dt
  ) {
    (void)sim_config;
    (void)dt;
    using engine::op_invoker::OpWalk;

    const auto view = sim_state.template ViewFor<BroadPhaseView>();
    const BroadPhaseTopology& topology =
        sim_state.template TopologyFor<policy::ordering::TopologicalOrdering>();
    const domain::topology::PairTopology& pairs =
        sim_state.template TopologyFor<row_sets::CollisionPairs>();

    engine::op_invoker::Step(
        engine::op_invoker::Ops<
            OpWalk<ComputeSweptBoxBoundsOp, ForwardBatched>,
            OpWalk<TestBoundsOverlapOp, PairBatched>>{
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
