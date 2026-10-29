#pragma once

#include <cstdint>
#include <string_view>

#include "algorithms/conventions.hpp"
#include "algorithms/shared_slots.hpp"
#include "domain/joint_topology.hpp"
#include "domain/math/activation_mask.hpp"
#include "domain/math/matrix.hpp"
#include "domain/math/scalar.hpp"
#include "domain/spatial/transform.hpp"
#include "engine/field_contract.hpp"
#include "engine/topology/layout_policy.hpp"
#include "engine/topology/orderings.hpp"
#include "engine/view/view_contract.hpp"
#include "engine/view/view_factory.hpp"

namespace achilles::algorithms::joint_limit {

// One value per joint DOF: row k is DOF k, column k of the joint's subspace
// S (and component k of its qd).
using JointCoordinates = math::Matrix<MathematicalT, 6, 1>;

// The joint-limit detector: the collision pipeline's per-joint stage, run
// alongside the broad and narrow phases (algorithms/integrators.hpp's
// Collision). Where the narrow phase finds where two bodies' shapes touch,
// this finds where a joint touches the end of its range -- MuJoCo's
// mj_instantiateLimit for slide and hinge joints, MJX-style: one row per
// DOF, dense and masked, every DOF of every joint computed and only
// kLimitActive saying which rows are real.
//
// A joint's coordinate along DOF k is theta_k = s_k . Log(q) -- s_k being
// S's column k and Log Transform::Exp's inverse, so for a joint PI only ever
// moves along s_k (a hinge, a slide) it's exactly the angle or offset PI
// accumulated. For a joint with several rotational DOFs (a ball, a free
// joint) it's that DOF's component of the rotation vector, so the limits
// are a box in rotation-vector space rather than MuJoCo's ball-joint cone.
// A rotation vector's angle is at most pi, so a rotational range must lie
// inside (-pi, pi) to ever be reached.
//
// Per DOF, as MJX's _instantiate_limit_slide_hinge does it -- only the
// nearer end counts, so a range narrower than twice the margin still gives
// one row per DOF, never two:
//
//   - kLimitDistance: min(theta - lower, upper - theta), positive while
//     inside the range, negative once past an end, by how far.
//   - kLimitSide: +1 when the lower end is the nearer one, -1 the upper --
//     the row's Jacobian is side * e_k, the direction that moves the joint
//     back inside.
//   - kLimitActive: bit k set when DOF k is limited (kJointLimited) and its
//     distance is within kJointLimitMargin, MuJoCo's jnt_margin -- a limit a
//     solver should consider before it's reached. Rows whose bit is clear
//     still hold whatever the Op computed and mean nothing.
//
// A joint that gives none of the limit fields reads the zero-fill:
// kJointLimited clear, so no DOF is limited and nothing changes for it.
enum class JointLimitField : uint8_t {
  // --- ABA inputs (shared, read-only here) ---
  kJointSubspace,  // S
  kJointPosition,  // q

  // --- Per-joint limits (loaded) ---
  kJointLimited,      // which DOFs have a range
  kJointRangeLower,   // per DOF
  kJointRangeUpper,   // per DOF
  kJointLimitMargin,  // m

  // --- Outputs ---
  kLimitDistance,  // per DOF, to the nearer end
  kLimitSide,      // per DOF, +1 lower / -1 upper
  kLimitActive,    // per DOF bits

  kCount
};
static_assert(engine::FieldEnumLike<JointLimitField>);

template <JointLimitField F>
struct JointLimitFieldTraits;

// Every field is per joint, sharing ABA's TopologicalOrdering: two of them
// are ABA's own blocks, and the rest have to line up row for row with them.
template <typename A>
struct RowField {
  using Assembler = A;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
};
template <typename A, typename Slot>
struct SharedRowField : RowField<A> {
  using SharedAs = Slot;
};

using CoordinatesAssembler = math::MatrixAssembler<ScalarOperationT, 6, 1>;
using MaskAssembler = math::ActivationMaskAssembler<6, ScalarOperationT>;

// --- ABA (shared) ---
template <>
struct JointLimitFieldTraits<JointLimitField::kJointSubspace>
    : SharedRowField<
          math::Matrix6x6Assembler<ScalarOperationT>,
          JointSubspaceSlot> {
  static constexpr std::string_view kName = "joint_subspace";
};
template <>
struct JointLimitFieldTraits<JointLimitField::kJointPosition>
    : SharedRowField<
          spatial::TransformAssembler<ScalarOperationT>,
          JointPositionSlot> {
  static constexpr std::string_view kName = "joint_position";
};

// --- Per-joint limits ---
template <>
struct JointLimitFieldTraits<JointLimitField::kJointLimited>
    : RowField<MaskAssembler> {
  static constexpr std::string_view kName = "joint_limited";
};
template <>
struct JointLimitFieldTraits<JointLimitField::kJointRangeLower>
    : RowField<CoordinatesAssembler> {
  static constexpr std::string_view kName = "joint_range_lower";
};
template <>
struct JointLimitFieldTraits<JointLimitField::kJointRangeUpper>
    : RowField<CoordinatesAssembler> {
  static constexpr std::string_view kName = "joint_range_upper";
};
template <>
struct JointLimitFieldTraits<JointLimitField::kJointLimitMargin>
    : SharedRowField<
          math::ScalarAssembler<ScalarOperationT>,
          JointLimitMarginSlot> {
  static constexpr std::string_view kName = "joint_limit_margin";
};

// --- Outputs ---
template <>
struct JointLimitFieldTraits<JointLimitField::kLimitDistance>
    : SharedRowField<CoordinatesAssembler, JointLimitDistanceSlot> {};
template <>
struct JointLimitFieldTraits<JointLimitField::kLimitSide>
    : SharedRowField<CoordinatesAssembler, JointLimitSideSlot> {};
template <>
struct JointLimitFieldTraits<JointLimitField::kLimitActive>
    : SharedRowField<MaskAssembler, JointLimitActiveSlot> {};
static_assert(engine::FieldTraitsLike<JointLimitField, JointLimitFieldTraits>);

using JointLimitView =
    engine::view::ViewFactory<JointLimitField, JointLimitFieldTraits>;
static_assert(engine::view::ViewLike<JointLimitView, JointLimitField>);

using JointLimitTopology = domain::JointTopology;
static_assert(domain::TopologyLike<JointLimitTopology>);

}  // namespace achilles::algorithms::joint_limit
