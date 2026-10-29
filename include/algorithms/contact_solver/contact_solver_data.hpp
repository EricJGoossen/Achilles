#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>

#include "algorithms/conventions.hpp"
#include "algorithms/joint_limit/joint_limit_data.hpp"
#include "algorithms/narrow_phase/narrow_phase_data.hpp"
#include "algorithms/orderings/collision_pairs.hpp"
#include "algorithms/shared_slots.hpp"
#include "domain/joint_topology.hpp"
#include "domain/math/activation_mask.hpp"
#include "domain/math/matrix.hpp"
#include "domain/math/scalar.hpp"
#include "domain/math/vector3.hpp"
#include "domain/spatial/dual.hpp"
#include "domain/spatial/inertia.hpp"
#include "domain/spatial/transform.hpp"
#include "engine/field_contract.hpp"
#include "engine/topology/indexing_policy.hpp"
#include "engine/topology/layout_policy.hpp"
#include "engine/topology/orderings.hpp"
#include "engine/view/view_contract.hpp"
#include "engine/view/view_factory.hpp"

namespace achilles::algorithms::contact_solver {

// MuJoCo's soft-constraint contact solver (engine_core_constraint.c,
// engine_solver.c), primal CG flavor (opt.solver = CG): the constrained
// joint acceleration qacc minimizing Gauss's principle plus each contact
// row's soft penalty,
//
//   1/2 (qacc - qacc_smooth)^T M (qacc - qacc_smooth)
//     + sum over active rows of 1/2 D (J qacc - aref)^2,  J qacc - aref < 0,
//
// where qacc_smooth is ABA's own unconstrained qdd. Solved after the
// collision pipeline and before VI, as in mj_step: VI then integrates the
// constrained qacc exactly the way it integrates any other.
//
// Contacts are pyramidal, condim 3: each of a pair's kSweptBoxContactSlots
// contacts is kPyramidEdges rows, J_edge = J_n +- mu J_t along the pair's
// two tangents (mju_makeFrame's), every row sharing its contact's distance,
// so its impedance and aref position term. A row is active while its
// contact is (kContactActive) and r = distance - margin < 0.
//
// Joint limits (algorithms/joint_limit) are rows in the same cost, one per
// DOF, mj_instantiateLimit's: J = side e_k on the joint's own DOF k, so
// J x is side x_k and J^T f is side f on that DOF alone -- every limit pass
// is a linear one, never a tree pass. A limit row is active while the
// detector says so (kLimitActive: limited, and r = distance - margin < 0),
// and its R is mj_makeImpedance's (1 - d) / d dof_invweight0.
//
// M is never formed. CG needs M x, M^-1 y, J x and J^T f, and each of those
// is an O(rows) pass over the tree reusing ABA's velocity-independent
// factors (x_up, I, U, D^-1): M x is RNEA at zero velocity, M^-1 y is ABA at
// zero velocity, J x and J^T f read a forward pass's body motion and feed a
// backward pass's body forces. The same M^-1 as MuJoCo's LDL solve, a
// different route to it.
//
// Fixed iteration counts, MJX-style: no early exit, so every tick runs the
// same passes and nothing branches on data. kContactSolverIterations CG
// iterations, each with kLineSearchIterations 1-D Newton steps on the
// exact (piecewise quadratic) cost along the search direction.
inline constexpr std::size_t kContactSolverIterations = 20;
inline constexpr std::size_t kLineSearchIterations = 4;

// Pyramid edges per condim-3 contact: (n + mu t1, n - mu t1, n + mu t2,
// n - mu t2), mj_instantiateContact's own order.
inline constexpr std::size_t kPyramidEdges = 4;

// One value per (edge, slot) row of a pair: row e is edge e, column k is
// contact slot k -- the same slot k as narrow_phase's kContactDistance.
using ContactRows = math::
    Matrix<MathematicalT, kPyramidEdges, narrow_phase::kSweptBoxContactSlots>;
// A pair's pyramid edge directions, column e is edge e (world).
using EdgeDirections = math::Matrix<MathematicalT, 3, kPyramidEdges>;
// MuJoCo's solref (timeconst, dampratio) and solimp (d0, dwidth, width,
// midpoint, power), and body_invweight0 (translational, rotational).
using Solref = math::Matrix<MathematicalT, 2, 1>;
using Solimp = math::Matrix<MathematicalT, 5, 1>;
using InvWeight = math::Matrix<MathematicalT, 2, 1>;

enum class ContactSolverField : uint8_t {
  // --- ABA's factors and state (shared, read-only here) ---
  kJointSubspace,          // S
  kRigidBodyInertia,       // I
  kParentToBodyTransform,  // x_up
  kWorldTransform,         // x_world
  kSpatialVelocity,        // v (body frame)
  kUTerm,                  // U
  kDInvTerm,               // D^-1
  kJointVelocity,          // qd
  kJointAcceleration,      // qdd: qacc_smooth in, constrained qacc out
  kSpatialAcceleration,    // a: ABA's (smooth) in, constrained out

  // --- Narrow-phase contacts (shared, read-only here) ---
  kCollisionMargin,  // m
  kContactNormal,    // n (world, source -> target)
  kContactDistance,  // per slot
  kContactPoint,     // per slot (world)
  kContactActive,    // per (slot, lane)

  // --- Joint-limit rows (shared, read-only here) ---
  kJointLimitMargin,  // m
  kLimitDistance,     // per DOF
  kLimitSide,         // per DOF, +1 lower / -1 upper
  kLimitActive,       // per DOF bits

  // --- Per-body contact parameters (loaded) ---
  kContactFriction,  // sliding mu
  kContactSolref,    // (timeconst, dampratio)
  kContactSolimp,    // (d0, dwidth, width, midpoint, power)

  // --- Per-joint limit parameters (loaded) ---
  kJointLimitSolref,  // (timeconst, dampratio)
  kJointLimitSolimp,  // (d0, dwidth, width, midpoint, power)

  // --- Set up once per sim (mj_setConst) ---
  kInverseInertia,  // Omega: body acceleration per unit body wrench (body)
  kInvWeight,       // body_invweight0 (translational, rotational)
  kDofInvWeight,    // dof_invweight0, per DOF

  // --- Joint-space CG state ---
  kQaccSmooth,       // qacc_smooth
  kQacc,             // qacc (persists across ticks: the warmstart)
  kMa,               // M (qacc - qacc_smooth)
  kSearch,           // search direction
  kMSearch,          // M search
  kConstraintForce,  // J^T f
  kGrad,             // M (qacc - qacc_smooth) - J^T f
  kGradPrev,         // last iteration's kGrad
  kMgrad,            // M^-1 grad
  kMgradPrev,        // last iteration's kMgrad

  // --- Per-body pass scratch ---
  kBodyAcceleration,   // body motion of a joint-space vector (body)
  kBodyForce,          // subtree force accumulator (body)
  kBodyWrenchTarget,   // contact wrench, as a pair's target (world)
  kBodyWrenchSource,   // contact wrench, as a pair's source (world)
  kInverseBiasForce,   // M^-1's articulated bias p (body)
  kInverseJointForce,  // M^-1's u

  // --- Per-pair contact rows ---
  kEdgeDirections,  // pyramid edges (world)
  kRowPoint,        // contact points, zeroed where inactive (world)
  kRowReference,    // aref
  kRowWeight,       // D = 1/R, zero where inactive
  kRowResidual,     // J qacc - aref
  kRowSearch,       // J search

  // --- Per-joint limit rows, one per DOF ---
  kLimitWeight,    // D = 1/R, zero where inactive
  kLimitResidual,  // J qacc - aref
  kLimitSearch,    // J search

  // --- Whole-sim scalars ---
  kStep,           // line-search alpha
  kStepLow,        // alpha bracket, cost decreasing below it
  kStepHigh,       // alpha bracket, cost increasing above it
  kRowSlope,       // rows' d cost / d alpha at alpha
  kRowCurvature,   // rows' d^2 cost / d alpha^2 at alpha
  kRowCostSmooth,  // rows' cost at qacc_smooth
  kRowCostWarm,    // rows' cost at the warmstart
  kSearchMSearch,  // search . M search
  kSearchMa,       // search . Ma
  kSearchDotDiff,  // d . y   (Hager-Zhang terms, y = grad - grad_prev,
  kDiffDotMDiff,   // y . My   My = Mgrad - Mgrad_prev, d = search)
  kDiffDotMgrad,   // y . Mgrad
  kSearchDotGrad,  // d . grad
  kSearchNormSq,   // |d|^2
  kGradNormSq,     // |grad|^2
  kBeta,           // CG beta
  kRestart,        // 1 while the next direction must be steepest descent

  kCount
};
static_assert(engine::FieldEnumLike<ContactSolverField>);

template <ContactSolverField F>
struct ContactSolverFieldTraits;

// Row and global fields share ABA's TopologicalOrdering, for the same reason
// the collision fields do: rows are joints, numbered once for every
// algorithm. Row fields hold one value per joint; pair fields one per pair
// (CollisionPairs, orderings/collision_pairs.hpp -- so ordered by it instead),
// a lane group at a time, lined up with the narrow phase's own; global fields
// one value for the whole sim (topology::GlobalIndexing).
template <typename A>
struct RowField {
  using Assembler = A;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
};
template <typename A>
struct PairField {
  using Assembler = A;
  using Ordering = CollisionPairs;
  using Layout = engine::topology::PlanarLayout;
};
template <typename A>
struct GlobalField : RowField<A> {
  using Indexing = engine::topology::GlobalIndexing;
};

template <typename A, typename Slot>
struct SharedRowField : RowField<A> {
  using SharedAs = Slot;
};
template <typename A, typename Slot>
struct SharedPairField : PairField<A> {
  using SharedAs = Slot;
};

using ScalarAssembler = math::ScalarAssembler<ScalarOperationT>;
using ForceAssembler = spatial::SpatialForceAssembler<ScalarOperationT>;
using AccelerationAssembler =
    spatial::SpatialAccelerationAssembler<ScalarOperationT>;
using RowsAssembler = math::MatrixAssembler<
    ScalarOperationT,
    kPyramidEdges,
    narrow_phase::kSweptBoxContactSlots>;

// --- ABA (shared) ---
template <>
struct ContactSolverFieldTraits<ContactSolverField::kJointSubspace>
    : SharedRowField<
          math::Matrix6x6Assembler<ScalarOperationT>,
          JointSubspaceSlot> {
  static constexpr std::string_view kName = "joint_subspace";
};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kRigidBodyInertia>
    : SharedRowField<
          spatial::InertiaAssembler<ScalarOperationT>,
          RigidBodyInertiaSlot> {
  static constexpr std::string_view kName = "rigid_body_inertia";
};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kParentToBodyTransform>
    : SharedRowField<
          spatial::TransformAssembler<ScalarOperationT>,
          ParentToBodyTransformSlot> {};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kWorldTransform>
    : SharedRowField<
          spatial::TransformAssembler<ScalarOperationT>,
          WorldTransformSlot> {};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kSpatialVelocity>
    : SharedRowField<
          spatial::SpatialVelocityAssembler<ScalarOperationT>,
          SpatialVelocitySlot> {};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kUTerm>
    : SharedRowField<
          spatial::InertiaOperatorAssembler<ScalarOperationT>,
          UTermSlot> {};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kDInvTerm>
    : SharedRowField<
          spatial::InvertedInertiaOperatorAssembler<ScalarOperationT>,
          DInvTermSlot> {};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kJointVelocity>
    : SharedRowField<
          spatial::SpatialVelocityAssembler<ScalarOperationT>,
          JointVelocitySlot> {
  static constexpr std::string_view kName = "joint_velocity";
};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kJointAcceleration>
    : SharedRowField<AccelerationAssembler, JointAccelerationSlot> {};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kSpatialAcceleration>
    : SharedRowField<AccelerationAssembler, SpatialAccelerationSlot> {};

// --- Narrow phase (shared) ---
template <>
struct ContactSolverFieldTraits<ContactSolverField::kCollisionMargin>
    : SharedRowField<ScalarAssembler, CollisionMarginSlot> {
  static constexpr std::string_view kName = "collision_margin";
};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kContactNormal>
    : SharedPairField<
          math::Vector3Assembler<ScalarOperationT>,
          ContactNormalSlot> {};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kContactDistance>
    : SharedPairField<
          math::MatrixAssembler<
              ScalarOperationT,
              narrow_phase::kSweptBoxContactSlots,
              1>,
          ContactDistanceSlot> {};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kContactPoint>
    : SharedPairField<
          math::MatrixAssembler<
              ScalarOperationT,
              3,
              narrow_phase::kSweptBoxContactSlots>,
          ContactPointSlot> {};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kContactActive>
    : RowField<math::ActivationMaskAssembler<
          kLanes * narrow_phase::kSweptBoxContactSlots>> {
  using SharedAs = ContactActiveSlot;
  using Ordering = CollisionPairs;
  using Indexing = engine::topology::LaneGroupIndexing<kLanes>;
};

// --- Joint limits (shared) ---
template <>
struct ContactSolverFieldTraits<ContactSolverField::kJointLimitMargin>
    : SharedRowField<ScalarAssembler, JointLimitMarginSlot> {
  static constexpr std::string_view kName = "joint_limit_margin";
};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kLimitDistance>
    : SharedRowField<
          joint_limit::CoordinatesAssembler,
          JointLimitDistanceSlot> {};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kLimitSide>
    : SharedRowField<joint_limit::CoordinatesAssembler, JointLimitSideSlot> {};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kLimitActive>
    : SharedRowField<joint_limit::MaskAssembler, JointLimitActiveSlot> {};

// --- Per-body contact parameters ---
// MuJoCo's geom friction (sliding), solref and solimp. A body that gives
// none reads the zero-fill, taken as MuJoCo's defaults: friction 1, solref
// (0.02, 1), solimp (0.9, 0.95, 0.001, 0.5, 2).
template <>
struct ContactSolverFieldTraits<ContactSolverField::kContactFriction>
    : RowField<ScalarAssembler> {
  static constexpr std::string_view kName = "contact_friction";
};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kContactSolref>
    : RowField<math::MatrixAssembler<ScalarOperationT, 2, 1>> {
  static constexpr std::string_view kName = "contact_solref";
};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kContactSolimp>
    : RowField<math::MatrixAssembler<ScalarOperationT, 5, 1>> {
  static constexpr std::string_view kName = "contact_solimp";
};

// --- Per-joint limit parameters ---
// MuJoCo's jnt_solref and jnt_solimp, read the same way: a joint that gives
// none reads the zero-fill, taken as the same defaults as a contact's.
template <>
struct ContactSolverFieldTraits<ContactSolverField::kJointLimitSolref>
    : RowField<math::MatrixAssembler<ScalarOperationT, 2, 1>> {
  static constexpr std::string_view kName = "joint_limit_solref";
};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kJointLimitSolimp>
    : RowField<math::MatrixAssembler<ScalarOperationT, 5, 1>> {
  static constexpr std::string_view kName = "joint_limit_solimp";
};

// --- Set up once ---
template <>
struct ContactSolverFieldTraits<ContactSolverField::kInverseInertia>
    : RowField<math::Matrix6x6Assembler<ScalarOperationT>> {};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kInvWeight>
    : RowField<math::MatrixAssembler<ScalarOperationT, 2, 1>> {};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kDofInvWeight>
    : RowField<joint_limit::CoordinatesAssembler> {};

// --- Joint-space CG state ---
template <>
struct ContactSolverFieldTraits<ContactSolverField::kQaccSmooth>
    : RowField<AccelerationAssembler> {};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kQacc>
    : RowField<AccelerationAssembler> {};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kMa>
    : RowField<ForceAssembler> {};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kSearch>
    : RowField<AccelerationAssembler> {};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kMSearch>
    : RowField<ForceAssembler> {};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kConstraintForce>
    : RowField<ForceAssembler> {};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kGrad>
    : RowField<ForceAssembler> {};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kGradPrev>
    : RowField<ForceAssembler> {};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kMgrad>
    : RowField<AccelerationAssembler> {};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kMgradPrev>
    : RowField<AccelerationAssembler> {};

// --- Per-body pass scratch ---
template <>
struct ContactSolverFieldTraits<ContactSolverField::kBodyAcceleration>
    : RowField<AccelerationAssembler> {};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kBodyForce>
    : RowField<ForceAssembler> {};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kBodyWrenchTarget>
    : RowField<ForceAssembler> {};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kBodyWrenchSource>
    : RowField<ForceAssembler> {};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kInverseBiasForce>
    : RowField<ForceAssembler> {};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kInverseJointForce>
    : RowField<ForceAssembler> {};

// --- Per-pair contact rows ---
template <>
struct ContactSolverFieldTraits<ContactSolverField::kEdgeDirections>
    : PairField<math::MatrixAssembler<ScalarOperationT, 3, kPyramidEdges>> {};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kRowPoint>
    : PairField<math::MatrixAssembler<
          ScalarOperationT,
          3,
          narrow_phase::kSweptBoxContactSlots>> {};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kRowReference>
    : PairField<RowsAssembler> {};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kRowWeight>
    : PairField<RowsAssembler> {};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kRowResidual>
    : PairField<RowsAssembler> {};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kRowSearch>
    : PairField<RowsAssembler> {};

// --- Per-joint limit rows ---
template <>
struct ContactSolverFieldTraits<ContactSolverField::kLimitWeight>
    : RowField<joint_limit::CoordinatesAssembler> {};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kLimitResidual>
    : RowField<joint_limit::CoordinatesAssembler> {};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kLimitSearch>
    : RowField<joint_limit::CoordinatesAssembler> {};

// --- Whole-sim scalars ---
template <>
struct ContactSolverFieldTraits<ContactSolverField::kStep>
    : GlobalField<ScalarAssembler> {};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kStepLow>
    : GlobalField<ScalarAssembler> {};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kStepHigh>
    : GlobalField<ScalarAssembler> {};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kRowSlope>
    : GlobalField<ScalarAssembler> {};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kRowCurvature>
    : GlobalField<ScalarAssembler> {};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kRowCostSmooth>
    : GlobalField<ScalarAssembler> {};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kRowCostWarm>
    : GlobalField<ScalarAssembler> {};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kSearchMSearch>
    : GlobalField<ScalarAssembler> {};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kSearchMa>
    : GlobalField<ScalarAssembler> {};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kSearchDotDiff>
    : GlobalField<ScalarAssembler> {};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kDiffDotMDiff>
    : GlobalField<ScalarAssembler> {};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kDiffDotMgrad>
    : GlobalField<ScalarAssembler> {};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kSearchDotGrad>
    : GlobalField<ScalarAssembler> {};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kSearchNormSq>
    : GlobalField<ScalarAssembler> {};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kGradNormSq>
    : GlobalField<ScalarAssembler> {};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kBeta>
    : GlobalField<ScalarAssembler> {};
template <>
struct ContactSolverFieldTraits<ContactSolverField::kRestart>
    : GlobalField<ScalarAssembler> {};
static_assert(engine::FieldTraitsLike<
              ContactSolverField,
              ContactSolverFieldTraits>);

using ContactSolverView =
    engine::view::ViewFactory<ContactSolverField, ContactSolverFieldTraits>;
static_assert(engine::view::ViewLike<ContactSolverView, ContactSolverField>);

using ContactSolverTopology = domain::JointTopology;
static_assert(domain::TopologyLike<ContactSolverTopology>);

}  // namespace achilles::algorithms::contact_solver
