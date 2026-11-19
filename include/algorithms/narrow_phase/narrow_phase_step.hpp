#pragma once

#include "algorithms/conventions.hpp"
#include "algorithms/narrow_phase/narrow_phase_data.hpp"
#include "algorithms/narrow_phase/narrow_phase_ops.hpp"
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

namespace achilles::algorithms::narrow_phase {

// Runs the narrow phase for a single archetype: one pass over this Step's
// own NarrowPhaseView, looked up via ViewFor<NarrowPhaseView>() the same way
// BroadPhaseStep looks up its own (see broad_phase_step.hpp).
//
// ComputeSweptBoxContactOp over PairBatched -- the same PairTraversal, with
// the same Stride over the same pairs (CollisionPairs,
// row_sets/collision_pairs.hpp) the broad phase's own pair test ran over, so
// each call lines up with exactly the kBoundsOverlap word that test wrote for
// it. Every lane is computed and kBoundsOverlap masks the result; the
// traversal's own PairStructure isn't read again, since every candidate bit
// already has it ANDed in.
//
// The one shape type pair today has the one Pass. Another type pair would
// add its own contact Op here, writing the same contact fields (see
// narrow_phase_data.hpp).
//
// `sim_config` and `dt` are both unused: the narrow phase is a pure function
// of this tick's pose, like the broad phase.
struct NarrowPhaseStep {
  using FieldEnum = NarrowPhaseField;

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

    const auto view = sim_state.template ViewFor<NarrowPhaseView>();
    const NarrowPhaseTopology& topology =
        sim_state.template TopologyFor<policy::ordering::TopologicalOrdering>();
    const domain::topology::PairTopology& pairs =
        sim_state.template TopologyFor<row_sets::CollisionPairs>();

    engine::op_invoker::Step(
        engine::op_invoker::Ops<OpWalk<ComputeSweptBoxContactOp, PairBatched>>{
            ComputeSweptBoxContactOp{}
        },
        view,
        topology,
        pairs
    );
  }
};

using NarrowPhaseAlgorithm =
    engine::Algorithm<NarrowPhaseField, NarrowPhaseFieldTraits>;
static_assert(engine::AlgorithmLike<NarrowPhaseAlgorithm>);

}  // namespace achilles::algorithms::narrow_phase
