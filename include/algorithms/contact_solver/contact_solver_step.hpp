#pragma once

#include "algorithms/contact_solver/contact_solver_data.hpp"
#include "algorithms/contact_solver/contact_solver_ops.hpp"
#include "algorithms/conventions.hpp"
#include "algorithms/sim_config.hpp"
#include "engine/algorithm_contract.hpp"
#include "engine/pass/algorithm_step.hpp"
#include "engine/pass/sim_context.hpp"
#include "engine/pass/traversals.hpp"
#include "engine/topology/ordering_policy.hpp"
#include "util/tmp.hpp"

namespace achilles::algorithms::contact_solver {

using ForwardBatched =
    engine::pass::TreeTraversal<engine::pass::Direction::kForward, kLanes>;
using BackwardBatched =
    engine::pass::TreeTraversal<engine::pass::Direction::kBackward, kLanes>;
using LinearBatched =
    engine::pass::LinearTraversal<engine::pass::Direction::kForward, kLanes>;
using PairBatched =
    engine::pass::PairTraversal<engine::pass::Direction::kForward, kLanes>;
using Global = engine::pass::GlobalTraversal;

// Every solver Step: one engine::pass::Step over the ContactSolverView and
// the TopologicalOrdering topology, of whatever Ops Derived::MakeOps
// returns -- tree, linear, pair and global passes alike, since each
// traversal takes the topology. The Steps below are the solver's own stages;
// how often each runs is the tick's step list (algorithms/integrators.hpp),
// never a loop in here.
template <typename Derived>
struct SolverStep {
  using FieldEnum = ContactSolverField;

  template <engine::pass::SimContextLike SimStateT>
  static void Step(
      const SimStateT& sim_state,
      const SimConfig& sim_config,
      ScalarOperationT dt
  ) {
    const auto view = sim_state.template ViewFor<ContactSolverView>();
    const ContactSolverTopology& topology =
        sim_state.template TopologyFor<engine::topology::TopologicalOrdering>();
    engine::pass::Step(Derived::MakeOps(sim_config, dt), view, topology);
  }
};

using engine::pass::Pass;

// Once per sim, after an ABAStep at the initial pose (mj_setConst's
// setInertia0): every body's inverse inertia, body_invweight0 and
// dof_invweight0.
struct InvWeightStep : SolverStep<InvWeightStep> {
  static auto
  MakeOps(const SimConfig& /*sim_config*/, ScalarOperationT /*dt*/) {
    return engine::pass::Ops<Pass<ComputeInverseInertiaOp, ForwardBatched>>{
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
    return engine::pass::Ops<
        Pass<BeginSolveOp, LinearBatched>,
        Pass<
            ApplyInertiaForwardOp<ContactSolverField::kQaccSmooth>,
            ForwardBatched>,
        Pass<SetupRowsOp, PairBatched>,
        Pass<SetupLimitRowsOp, LinearBatched>,
        Pass<
            ApplyInertiaForwardOp<ContactSolverField::kSearch>,
            ForwardBatched>,
        Pass<ApplyInertiaBackwardOp, BackwardBatched>,
        Pass<ProjectSearchOp, PairBatched>,
        Pass<ProjectLimitSearchOp, LinearBatched>,
        Pass<GaussTermsOp, LinearBatched>,
        Pass<WarmstartCostOp, PairBatched>,
        Pass<WarmstartLimitCostOp, LinearBatched>,
        Pass<ChooseWarmstartOp, Global>>{
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
    return engine::pass::Ops<
        Pass<
            ApplyInertiaForwardOp<ContactSolverField::kSearch>,
            ForwardBatched>,
        Pass<ApplyInertiaBackwardOp, BackwardBatched>,
        Pass<ProjectSearchOp, PairBatched>,
        Pass<ProjectLimitSearchOp, LinearBatched>,
        Pass<GaussTermsOp, LinearBatched>,
        Pass<BeginLineSearchOp, Global>>{
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
    return engine::pass::Ops<
        Pass<LineSearchRowsOp, PairBatched>,
        Pass<LineSearchLimitsOp, LinearBatched>,
        Pass<NewtonStepOp, Global>>{
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
    return engine::pass::Ops<
        Pass<StepJointOp, LinearBatched>,
        Pass<StepBodyAccelerationOp, LinearBatched>,
        Pass<ContactForceOp, PairBatched>,
        Pass<ConstraintForceOp, BackwardBatched>,
        Pass<LimitForceOp, LinearBatched>,
        Pass<GradientOp, LinearBatched>,
        Pass<InverseInertiaBackwardOp, BackwardBatched>,
        Pass<InverseInertiaForwardOp, ForwardBatched>,
        Pass<ConjugacyTermsOp, LinearBatched>,
        Pass<ConjugateOp, Global>,
        Pass<UpdateSearchOp, LinearBatched>>{
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
    return engine::pass::Ops<Pass<FinishSolveOp, LinearBatched>>{FinishSolveOp{}
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
