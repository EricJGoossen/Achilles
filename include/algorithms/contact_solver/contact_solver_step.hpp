// The contact solver's stages, run as step lists.
// Doc: docs/code/algorithms_contact_solver_contact_solver_step.md

#pragma once

#include "algorithms/contact_solver/contact_solver_data.hpp"
#include "algorithms/contact_solver/contact_solver_ops.hpp"
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

namespace achilles::algorithms::contact_solver {

using ForwardBatched = policy::traversal::
    TreeTraversal<policy::traversal::Direction::kForward, kLanes>;
using BackwardBatched = policy::traversal::
    TreeTraversal<policy::traversal::Direction::kBackward, kLanes>;
using LinearBatched = policy::traversal::
    LinearTraversal<policy::traversal::Direction::kForward, kLanes>;
using PairBatched = policy::traversal::
    PairTraversal<policy::traversal::Direction::kForward, kLanes>;
using Global = policy::traversal::GlobalTraversal;

// Every solver Step: one engine::op_invoker::Step over the ContactSolverView,
// the TopologicalOrdering topology and CollisionPairs'
// (row_sets/collision_pairs.hpp), of whatever Ops Derived::MakeOps returns --
// tree, linear, pair and global passes alike: the body passes walk the first,
// the pair passes the second. The Steps below are the solver's own stages; how
// often each runs is the tick's step list (algorithms/integrators.hpp), never a
// loop in here.
template <typename Derived>
struct SolverStep {
  using FieldEnum = ContactSolverField;

  template <engine::context::SimContextLike SimStateT>
  static void Step(
      const SimStateT& sim_state,
      const SimConfig& sim_config,
      ScalarOperationT dt
  ) {
    const auto view = sim_state.template ViewFor<ContactSolverView>();
    const ContactSolverTopology& topology =
        sim_state.template TopologyFor<policy::ordering::TopologicalOrdering>();
    const domain::topology::PairTopology& pairs =
        sim_state.template TopologyFor<row_sets::CollisionPairs>();
    engine::op_invoker::Step(
        Derived::MakeOps(sim_config, dt), view, topology, pairs
    );
  }
};

using engine::op_invoker::OpWalk;

// Once per sim, after an ABAStep at the initial pose (mj_setConst's
// setInertia0): every body's inverse inertia, body_invweight0 and
// dof_invweight0.
struct InvWeightStep : SolverStep<InvWeightStep> {
  static auto
  MakeOps(const SimConfig& /*sim_config*/, ScalarOperationT /*dt*/) {
    return engine::op_invoker::Ops<
        OpWalk<ComputeInverseInertiaOp, ForwardBatched>>{
        ComputeInverseInertiaOp{}
    };
  }
};

// Starts a tick's solve, after the collision pipeline: qacc = qacc_smooth,
// every contact and limit row's aref, D and residual, and the warmstart
// choice -- the cost
// at the previous tick's qacc against the cost at qacc_smooth, along
// search = qacc_warm - qacc_smooth, for the UpdateStep that follows to step
// to.
struct SetupStep : SolverStep<SetupStep> {
  static auto MakeOps(const SimConfig& /*sim_config*/, ScalarOperationT dt) {
    return engine::op_invoker::Ops<
        OpWalk<BeginSolveOp, LinearBatched>,
        OpWalk<
            ApplyInertiaForwardOp<ContactSolverField::kQaccSmooth>,
            ForwardBatched>,
        OpWalk<SetupRowsOp, PairBatched>,
        OpWalk<SetupLimitRowsOp, LinearBatched>,
        OpWalk<
            ApplyInertiaForwardOp<ContactSolverField::kSearch>,
            ForwardBatched>,
        OpWalk<ApplyInertiaBackwardOp, BackwardBatched>,
        OpWalk<ProjectSearchOp, PairBatched>,
        OpWalk<ProjectLimitSearchOp, LinearBatched>,
        OpWalk<GaussTermsOp, LinearBatched>,
        OpWalk<WarmstartCostOp, PairBatched>,
        OpWalk<WarmstartLimitCostOp, LinearBatched>,
        OpWalk<ChooseWarmstartOp, Global>>{
        BeginSolveOp{},
        ApplyInertiaForwardOp<ContactSolverField::kQaccSmooth>{},
        SetupRowsOp(dt),
        SetupLimitRowsOp(dt),
        ApplyInertiaForwardOp<ContactSolverField::kSearch>{},
        ApplyInertiaBackwardOp{},
        ProjectSearchOp{},
        ProjectLimitSearchOp{},
        GaussTermsOp{},
        WarmstartCostOp{},
        WarmstartLimitCostOp{},
        ChooseWarmstartOp{},
    };
  }
};

// The search direction's M search and J search, and the Gauss half of the
// line search's quadratic, then a fresh line search from alpha = 0.
struct DirectionStep : SolverStep<DirectionStep> {
  static auto
  MakeOps(const SimConfig& /*sim_config*/, ScalarOperationT /*dt*/) {
    return engine::op_invoker::Ops<
        OpWalk<
            ApplyInertiaForwardOp<ContactSolverField::kSearch>,
            ForwardBatched>,
        OpWalk<ApplyInertiaBackwardOp, BackwardBatched>,
        OpWalk<ProjectSearchOp, PairBatched>,
        OpWalk<ProjectLimitSearchOp, LinearBatched>,
        OpWalk<GaussTermsOp, LinearBatched>,
        OpWalk<BeginLineSearchOp, Global>>{
        ApplyInertiaForwardOp<ContactSolverField::kSearch>{},
        ApplyInertiaBackwardOp{},
        ProjectSearchOp{},
        ProjectLimitSearchOp{},
        GaussTermsOp{},
        BeginLineSearchOp{},
    };
  }
};

// One Newton step of the line search: the rows' slope and curvature at the
// current alpha, then alpha's update.
struct LineSearchStep : SolverStep<LineSearchStep> {
  static auto
  MakeOps(const SimConfig& /*sim_config*/, ScalarOperationT /*dt*/) {
    return engine::op_invoker::Ops<
        OpWalk<LineSearchRowsOp, PairBatched>,
        OpWalk<LineSearchLimitsOp, LinearBatched>,
        OpWalk<NewtonStepOp, Global>>{
        LineSearchRowsOp{}, LineSearchLimitsOp{}, NewtonStepOp{}
    };
  }
};

// Steps qacc (and, with it, every body's acceleration) to alpha, then
// everything the next direction needs: each
// contact row's force and the wrench it puts on both bodies, J^T f of
// those, then each limit row's force added onto it, the gradient,
// M^-1 of it, and the conjugate direction (steepest descent right after
// SetupStep).
struct UpdateStep : SolverStep<UpdateStep> {
  static auto
  MakeOps(const SimConfig& /*sim_config*/, ScalarOperationT /*dt*/) {
    return engine::op_invoker::Ops<
        OpWalk<StepJointOp, LinearBatched>,
        OpWalk<StepBodyAccelerationOp, LinearBatched>,
        OpWalk<ContactForceOp, PairBatched>,
        OpWalk<ConstraintForceOp, BackwardBatched>,
        OpWalk<LimitForceOp, LinearBatched>,
        OpWalk<GradientOp, LinearBatched>,
        OpWalk<InverseInertiaBackwardOp, BackwardBatched>,
        OpWalk<InverseInertiaForwardOp, ForwardBatched>,
        OpWalk<ConjugacyTermsOp, LinearBatched>,
        OpWalk<ConjugateOp, Global>,
        OpWalk<UpdateSearchOp, LinearBatched>>{
        StepJointOp{},
        StepBodyAccelerationOp{},
        ContactForceOp{},
        ConstraintForceOp{},
        LimitForceOp{},
        GradientOp{},
        InverseInertiaBackwardOp{},
        InverseInertiaForwardOp{},
        ConjugacyTermsOp{},
        ConjugateOp{},
        UpdateSearchOp{},
    };
  }
};

// Hands the constrained qacc back to ABA's qdd.
struct FinishStep : SolverStep<FinishStep> {
  static auto
  MakeOps(const SimConfig& /*sim_config*/, ScalarOperationT /*dt*/) {
    return engine::op_invoker::Ops<OpWalk<FinishSolveOp, LinearBatched>>{
        FinishSolveOp{}
    };
  }
};

// One CG iteration (mj_solPrimal's loop body): the direction, the line
// search along it, and the step.
using Iteration = util::ConcatT<
    util::TypeList<DirectionStep>,
    util::RepeatListT<util::TypeList<LineSearchStep>, kLineSearchIterations>,
    util::TypeList<UpdateStep>>;

// The whole solve, for a tick's step list: set up, take the warmstart (or
// not), kContactSolverIterations iterations, hand qacc back.
using Solve = util::ConcatT<
    util::TypeList<SetupStep, UpdateStep>,
    util::RepeatListT<Iteration, kContactSolverIterations>,
    util::TypeList<FinishStep>>;

using ContactSolverAlgorithm =
    engine::Algorithm<ContactSolverField, ContactSolverFieldTraits>;
static_assert(engine::AlgorithmLike<ContactSolverAlgorithm>);

}  // namespace achilles::algorithms::contact_solver
