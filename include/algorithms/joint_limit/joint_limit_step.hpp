#pragma once

#include "algorithms/conventions.hpp"
#include "algorithms/joint_limit/joint_limit_data.hpp"
#include "algorithms/joint_limit/joint_limit_ops.hpp"
#include "algorithms/sim_config.hpp"
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

namespace achilles::algorithms::joint_limit {

// Runs the joint-limit detector for a single archetype: one pass of
// DetectJointLimitOp over this Step's own JointLimitView. Each joint's rows
// depend only on its own q, so a LinearTraversal, like the broad phase's
// bounds pass, over the same TopologicalOrdering topology.
//
// `sim_config` and `dt` are both unused: like the rest of the collision
// pipeline, it's a pure function of this tick's pose.
struct JointLimitStep {
  using FieldEnum = JointLimitField;

  using LinearBatched = policy::traversal::
      LinearTraversal<policy::traversal::Direction::kForward, kLanes>;

  template <engine::context::SimContextLike SimStateT>
  static void Step(
      const SimStateT& sim_state,
      const SimConfig& sim_config,
      ScalarOperationT dt
  ) {
    (void)sim_config;
    (void)dt;
    using engine::op_invoker::OpWalk;

    const auto view = sim_state.template ViewFor<JointLimitView>();
    const JointLimitTopology& topology =
        sim_state.template TopologyFor<policy::ordering::TopologicalOrdering>();

    engine::op_invoker::Step(
        engine::op_invoker::Ops<OpWalk<DetectJointLimitOp, LinearBatched>>{
            DetectJointLimitOp{}
        },
        view,
        topology
    );
  }
};

using JointLimitAlgorithm =
    engine::Algorithm<JointLimitField, JointLimitFieldTraits>;
static_assert(engine::AlgorithmLike<JointLimitAlgorithm>);

}  // namespace achilles::algorithms::joint_limit
