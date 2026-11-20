// The contact solver's stages, run as step lists.
// Doc: docs/code/algorithms_contact_solver_contact_solver_step.md

#pragma once

#include "algorithms/contact_solver/contact_solver_data.hpp"
#include "algorithms/contact_solver/contact_solver_ops.hpp"
#include "algorithms/conventions.hpp"
#include "algorithms/orderings/collision_pairs.hpp"
#include "algorithms/sim_config.hpp"
#include "domain/joint_topology.hpp"
#include "engine/algorithm_contract.hpp"
#include "engine/pass/algorithm_step.hpp"
#include "engine/pass/repeat_while_step.hpp"
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
// The pair passes visit only the lane groups with a contact this tick
// (kContactActive, the narrow phase's). Skipping the rest changes nothing:
// a row with no contact has D = 0 and a zero residual (SetupRowsOp), so it
// adds nothing to any body or whole-sim sum, and a skipped group's per-pair
// values are read only by these same passes, which skip it too.
using PairBatched =
    engine::pass::GatedPairTraversal<engine::pass::Direction::kForward, kLanes>;
using Global = engine::pass::GlobalTraversal;

// Every solver Step: one engine::pass::Step over the ContactSolverView, the
// TopologicalOrdering topology and CollisionPairs'
// (orderings/collision_pairs.hpp), of whatever Ops Derived::MakeOps returns --
// tree, linear, pair and global passes alike: the body passes walk the first,
// the pair passes the second, gated on kContactActive. The Steps below are the
// solver's own stages; how often each runs is the tick's step list
// (algorithms/integrators.hpp), never a loop in here.
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
    engine::pass::Step(
        Derived::MakeOps(sim_config, dt),
        view,
        topology,
        pairs,
        engine::pass::MakePairGate<ContactSolverField::kContactActive>(view)
    );
  }
};

using engine::pass::Pass;

// Once per sim, after an ABAStep at the initial pose (mj_setConst's
// setInertia0): every body's inverse inertia, body_invweight0 and
// dof_invweight0, then trace(M) and the tolerance's scale from it.
struct InvWeightStep : SolverStep<InvWeightStep> {
  static auto
  MakeOps(const SimConfig& /*sim_config*/, ScalarOperationT /*dt*/) {
    return engine::pass::Ops<
        Pass<ComputeInverseInertiaOp, ForwardBatched>,
        Pass<SeedCompositeInertiaOp, LinearBatched>,
        Pass<CompositeInertiaOp, BackwardBatched>,
        Pass<ImprovementScaleOp, Global>>{
        ComputeInverseInertiaOp{},
        SeedCompositeInertiaOp{},
        CompositeInertiaOp{},
        ImprovementScaleOp{},
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

// CG's update. Steps qacc (and, with it, every body's acceleration) to
// alpha, then everything the next direction needs: each
// contact row's force and the wrench it puts on both bodies, J^T f of
// those, then each limit row's force added onto it, the gradient,
// M^-1 of it, and the conjugate direction (steepest descent right after
// SetupStep). Last, whether another iteration would still improve enough.
struct UpdateStep : SolverStep<UpdateStep> {
  static auto MakeOps(const SimConfig& sim_config, ScalarOperationT /*dt*/) {
    return engine::pass::Ops<
        Pass<StepJointOp, LinearBatched>,
        Pass<StepBodyAccelerationOp, LinearBatched>,
        Pass<ContactForceOp, PairBatched>,
        Pass<ConstraintForceOp, BackwardBatched>,
        Pass<LimitForceOp, LinearBatched>,
        Pass<GradientOp, LinearBatched>,
        Pass<InverseInertiaBackwardOp<>, BackwardBatched>,
        Pass<InverseInertiaForwardOp<>, ForwardBatched>,
        Pass<ConjugacyTermsOp, LinearBatched>,
        Pass<ConjugateOp, Global>,
        Pass<UpdateSearchOp, LinearBatched>,
        Pass<ConvergenceTermsOp, LinearBatched>,
        Pass<ConvergedOp, Global>>{
        StepJointOp{},
        StepBodyAccelerationOp{},
        ContactForceOp{},
        ConstraintForceOp{},
        LimitForceOp{},
        GradientOp{},
        InverseInertiaBackwardOp<>{},
        InverseInertiaForwardOp<>{},
        ConjugacyTermsOp{},
        ConjugateOp{},
        UpdateSearchOp{},
        ConvergenceTermsOp{},
        ConvergedOp(sim_config.contact_solver.tolerance),
    };
  }
};

// Newton's update: UpdateStep's step to alpha and its gradient, then
// search = -H^-1 grad through H's factors, and the same convergence test.
// With Factor it first factors H at the qacc just reached (each body's
// active contact stiffness, then FactorHessianOp's backward pass), which
// the solve does once, right after the warmstart; the iterations after it
// reuse those factors.
template <bool Factor>
struct NewtonUpdateStep : SolverStep<NewtonUpdateStep<Factor>> {
  static auto MakeOps(const SimConfig& sim_config, ScalarOperationT /*dt*/) {
    using HessianBackward = InverseInertiaBackwardOp<
        ContactSolverField::kHessianU,
        ContactSolverField::kHessianDInv>;
    using HessianForward = InverseInertiaForwardOp<
        ContactSolverField::kHessianU,
        ContactSolverField::kHessianDInv>;
    const ConvergedOp converged(sim_config.contact_solver.tolerance);
    if constexpr (Factor) {
      return engine::pass::Ops<
          Pass<StepJointOp, LinearBatched>,
          Pass<StepBodyAccelerationOp, LinearBatched>,
          Pass<ClearStiffnessOp, LinearBatched>,
          Pass<ContactForceOp, PairBatched>,
          Pass<ContactStiffnessOp, PairBatched>,
          Pass<ConstraintForceOp, BackwardBatched>,
          Pass<LimitForceOp, LinearBatched>,
          Pass<GradientOp, LinearBatched>,
          Pass<SeedHessianOp, LinearBatched>,
          Pass<FactorHessianOp, BackwardBatched>,
          Pass<HessianBackward, BackwardBatched>,
          Pass<HessianForward, ForwardBatched>,
          Pass<NewtonSearchOp, LinearBatched>,
          Pass<ConvergedOp, Global>>{
          StepJointOp{},
          StepBodyAccelerationOp{},
          ClearStiffnessOp{},
          ContactForceOp{},
          ContactStiffnessOp{},
          ConstraintForceOp{},
          LimitForceOp{},
          GradientOp{},
          SeedHessianOp{},
          FactorHessianOp{},
          HessianBackward{},
          HessianForward{},
          NewtonSearchOp{},
          converged,
      };
    } else {
      return engine::pass::Ops<
          Pass<StepJointOp, LinearBatched>,
          Pass<StepBodyAccelerationOp, LinearBatched>,
          Pass<ContactForceOp, PairBatched>,
          Pass<ConstraintForceOp, BackwardBatched>,
          Pass<LimitForceOp, LinearBatched>,
          Pass<GradientOp, LinearBatched>,
          Pass<HessianBackward, BackwardBatched>,
          Pass<HessianForward, ForwardBatched>,
          Pass<NewtonSearchOp, LinearBatched>,
          Pass<ConvergedOp, Global>>{
          StepJointOp{},
          StepBodyAccelerationOp{},
          ContactForceOp{},
          ConstraintForceOp{},
          LimitForceOp{},
          GradientOp{},
          HessianBackward{},
          HessianForward{},
          NewtonSearchOp{},
          converged,
      };
    }
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

using ContactSolverAlgorithm =
    engine::Algorithm<ContactSolverField, ContactSolverFieldTraits>;
static_assert(engine::AlgorithmLike<ContactSolverAlgorithm>);

// One iteration (mj_solPrimal's loop body): the direction, the line search
// along it, and the step, ending in the convergence test.
template <typename Update>
using Iteration = util::ConcatT<
    util::TypeList<DirectionStep>,
    util::RepeatListT<util::TypeList<LineSearchStep>, kLineSearchIterations>,
    util::TypeList<Update>>;

// The iterations: the first always runs, as in mj_solPrimal, whose test
// follows each iteration; the rest only until converged, up to
// kContactSolverIterations in all.
template <typename Update>
using Iterate = util::ConcatT<
    Iteration<Update>,
    util::TypeList<engine::pass::RepeatWhileStep<
        Iteration<Update>,
        ContactSolverAlgorithm,
        ContactSolverField::kContinue,
        kContactSolverIterations - 1>>>;

// The whole solve, for a tick's step list: set up, take the warmstart (or
// not), iterate, hand qacc back. Newton factors H in its first update.
template <ContactSolver Solver>
struct SolveList;
template <>
struct SolveList<ContactSolver::kNewton> {
  using Type = util::ConcatT<
      util::TypeList<SetupStep, NewtonUpdateStep<true>>,
      Iterate<NewtonUpdateStep<false>>,
      util::TypeList<FinishStep>>;
};
template <>
struct SolveList<ContactSolver::kCG> {
  using Type = util::ConcatT<
      util::TypeList<SetupStep, UpdateStep>,
      Iterate<UpdateStep>,
      util::TypeList<FinishStep>>;
};
template <ContactSolver Solver>
using SolveFor = typename SolveList<Solver>::Type;

// The default solver's solve (SimConfig's default, MuJoCo's: Newton).
using Solve = SolveFor<ContactSolver::kNewton>;

}  // namespace achilles::algorithms::contact_solver
