#pragma once

#include <cstddef>

#include "algorithms/contact_solver/contact_solver_data.hpp"
#include "algorithms/contact_solver/contact_solver_ops.hpp"
#include "algorithms/conventions.hpp"
#include "algorithms/orderings/collision_pairs.hpp"
#include "algorithms/sim_config.hpp"
#include "domain/joint_topology.hpp"
#include "engine/algorithm_contract.hpp"
#include "engine/pass/algorithm_step.hpp"
#include "engine/pass/repeat_step.hpp"
#include "engine/pass/sim_context.hpp"
#include "engine/pass/traversals.hpp"
#include "engine/topology/orderings.hpp"
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

// Every solver Step: one engine::pass::Step over the ContactSolverView, the
// TopologicalOrdering topology and CollisionPairs'
// (orderings/collision_pairs.hpp), of whatever Ops Derived::MakeOps returns --
// tree, linear, pair and global passes alike: the body passes walk the first,
// the pair passes the second. The Steps below are the solver's own stages; how
// often each runs is the tick's step list (algorithms/integrators.hpp), never a
// loop in here.
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
    const domain::PairTopology& pairs =
        sim_state.template TopologyFor<CollisionPairs>();
    engine::pass::Step(Derived::MakeOps(sim_config, dt), view, topology, pairs);
  }
};

using engine::pass::Pass;

// Once per sim, after an ABAStep at the initial pose (mj_setConst's
// setInertia0): every body's inverse inertia, body_invweight0 and
// dof_invweight0.
// Then trace(M), the scale the solvers' tolerance test uses.
struct InvWeightStep : SolverStep<InvWeightStep> {
  static auto
  MakeOps(const SimConfig& /*sim_config*/, ScalarOperationT /*dt*/) {
    return engine::pass::Ops<
        Pass<ComputeInverseInertiaOp, ForwardBatched>,
        Pass<InertiaTraceOp, LinearBatched>>{
        ComputeInverseInertiaOp{}, InertiaTraceOp{}
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

// Steps qacc (and, with it, every body's acceleration) to alpha, then each
// contact row's force and the wrench it puts on both bodies, J^T f of those,
// then each limit row's force added onto it, and the gradient: what both
// solvers' updates start with.
template <typename... Tail>
auto GradientOps(const Tail&... tail) {
  return engine::pass::JoinOps(
      engine::pass::Ops<
          Pass<StepJointOp, LinearBatched>,
          Pass<StepBodyAccelerationOp, LinearBatched>,
          Pass<ContactForceOp, PairBatched>,
          Pass<ConstraintForceOp, BackwardBatched>,
          Pass<LimitForceOp, LinearBatched>,
          Pass<GradientOp, LinearBatched>>{
          StepJointOp{},
          StepBodyAccelerationOp{},
          ContactForceOp{},
          ConstraintForceOp{},
          LimitForceOp{},
          GradientOp{},
      },
      tail...
  );
}

// CG's update: GradientOps, then M^-1 grad, the conjugate direction
// (steepest descent right after SetupStep), and the tolerance test.
struct UpdateStep : SolverStep<UpdateStep> {
  static auto MakeOps(const SimConfig& sim_config, ScalarOperationT /*dt*/) {
    return GradientOps(engine::pass::Ops<
                       Pass<InverseInertiaBackwardOp<>, BackwardBatched>,
                       Pass<InverseInertiaForwardOp<>, ForwardBatched>,
                       Pass<ConjugacyTermsOp, LinearBatched>,
                       Pass<ConjugateOp, Global>,
                       Pass<UpdateSearchOp, LinearBatched>,
                       Pass<GradDotMgradOp, LinearBatched>,
                       Pass<ConvergenceOp, Global>>{
        InverseInertiaBackwardOp<>{},
        InverseInertiaForwardOp<>{},
        ConjugacyTermsOp{},
        ConjugateOp{},
        UpdateSearchOp{},
        GradDotMgradOp{},
        ConvergenceOp(sim_config.contact_solver.tolerance),
    });
  }
};

// Newton's update, first half: GradientOps alone, so the Hessian can be
// factorized at the new qacc (NewtonFactorStep) before the direction is.
struct NewtonGradientStep : SolverStep<NewtonGradientStep> {
  static auto
  MakeOps(const SimConfig& /*sim_config*/, ScalarOperationT /*dt*/) {
    return GradientOps();
  }
};

// H = M + J^T D J's factors, over the rows active at the current qacc (see
// contact_solver_data.hpp): every body's K_b from its pairs' rows, I + K_b
// in its own frame, then the articulated-body recursion over that.
struct NewtonFactorStep : SolverStep<NewtonFactorStep> {
  static auto
  MakeOps(const SimConfig& /*sim_config*/, ScalarOperationT /*dt*/) {
    return engine::pass::Ops<
        Pass<ClearStiffnessOp, LinearBatched>,
        Pass<ContactHessianOp, PairBatched>,
        Pass<BeginHessianOp, LinearBatched>,
        Pass<FactorHessianOp, BackwardBatched>>{
        ClearStiffnessOp{},
        ContactHessianOp{},
        BeginHessianOp{},
        FactorHessianOp{},
    };
  }
};

// Newton's update, second half: H^-1 grad over H's factors, the direction
// -H^-1 grad, and the tolerance test.
struct NewtonDirectionStep : SolverStep<NewtonDirectionStep> {
  static auto MakeOps(const SimConfig& sim_config, ScalarOperationT /*dt*/) {
    using HessianBackward = InverseInertiaBackwardOp<
        ContactSolverField::kHessianU,
        ContactSolverField::kHessianDInv>;
    using HessianForward = InverseInertiaForwardOp<
        ContactSolverField::kHessianU,
        ContactSolverField::kHessianDInv>;
    return engine::pass::Ops<
        Pass<HessianBackward, BackwardBatched>,
        Pass<HessianForward, ForwardBatched>,
        Pass<NewtonSearchOp, LinearBatched>,
        Pass<GradDotMgradOp, LinearBatched>,
        Pass<ConvergenceOp, Global>>{
        HessianBackward{},
        HessianForward{},
        NewtonSearchOp{},
        GradDotMgradOp{},
        ConvergenceOp(sim_config.contact_solver.tolerance),
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

// kContinue, as RepeatWhileStep reads it: 1 while the solve hasn't
// converged.
struct SolveContinues {
  template <engine::pass::SimContextLike SimStateT>
  static bool Get(const SimStateT& sim_state) {
    const auto view = sim_state.template ViewFor<ContactSolverView>();
    return view.template Load<ContactSolverField::kContinue, ScalarOperationT>(0
           )
               .Value() > ScalarOperationT(0.5);
  }
};

// The most iterations a tick runs (SimConfig::contact_solver).
struct SolveIterations {
  static std::size_t Get(const SimConfig& sim_config) {
    return sim_config.contact_solver.iterations;
  }
};

// One iteration of each solver (mj_solPrimal's loop body): the direction's
// M search and J search, the line search along it, and the update.
using CGIteration = util::ConcatT<
    util::TypeList<DirectionStep>,
    util::RepeatListT<util::TypeList<LineSearchStep>, kLineSearchIterations>,
    util::TypeList<UpdateStep>>;
using NewtonIteration = util::ConcatT<
    util::TypeList<DirectionStep>,
    util::RepeatListT<util::TypeList<LineSearchStep>, kLineSearchIterations>,
    util::TypeList<NewtonGradientStep, NewtonDirectionStep>>;
using NewtonRefactorIteration = util::ConcatT<
    util::TypeList<DirectionStep>,
    util::RepeatListT<util::TypeList<LineSearchStep>, kLineSearchIterations>,
    util::TypeList<NewtonGradientStep, NewtonFactorStep, NewtonDirectionStep>>;

template <typename Iteration>
using Iterate = engine::pass::RepeatWhileStep<
    Iteration,
    SolveContinues,
    SolveIterations,
    ContactSolverField>;

// Each whole solve: set up and take the warmstart (or not), the first
// update there, iterations until converged or SimConfig's limit, then hand
// qacc back. Newton factorizes H after its first gradient -- at the
// warmstart, so over the rows the solve starts with.
using CGSolve =
    util::TypeList<SetupStep, UpdateStep, Iterate<CGIteration>, FinishStep>;
using NewtonSolve = util::TypeList<
    SetupStep,
    NewtonGradientStep,
    NewtonFactorStep,
    NewtonDirectionStep,
    Iterate<NewtonIteration>,
    FinishStep>;
using NewtonRefactorSolve = util::TypeList<
    SetupStep,
    NewtonGradientStep,
    NewtonFactorStep,
    NewtonDirectionStep,
    Iterate<NewtonRefactorIteration>,
    FinishStep>;

// The solve a tick's step list runs: whichever SimConfig::contact_solver
// picks, chosen per tick (one comparison, outside every pass).
struct SolveStep {
  using FieldEnum = ContactSolverField;

  template <engine::pass::SimContextLike SimStateT>
  static void Step(
      const SimStateT& sim_state,
      const SimConfig& sim_config,
      ScalarOperationT dt
  ) {
    const ContactSolverConfig& config = sim_config.contact_solver;
    if (config.solver == ContactSolver::kCG) {
      sim_state.template Step<CGSolve>(dt, sim_config);
    } else if (config.refactor_every_iteration) {
      sim_state.template Step<NewtonRefactorSolve>(dt, sim_config);
    } else {
      sim_state.template Step<NewtonSolve>(dt, sim_config);
    }
  }
};

using Solve = util::TypeList<SolveStep>;

using ContactSolverAlgorithm =
    engine::Algorithm<ContactSolverField, ContactSolverFieldTraits>;
static_assert(engine::AlgorithmLike<ContactSolverAlgorithm>);

}  // namespace achilles::algorithms::contact_solver
