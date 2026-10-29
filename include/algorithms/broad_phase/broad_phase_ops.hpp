#pragma once

#include <array>

#include "algorithms/broad_phase/broad_phase_data.hpp"
#include "algorithms/conventions.hpp"
#include "engine/pass/op_invoker.hpp"
#include "engine/pass/traversals.hpp"

namespace achilles::algorithms::broad_phase {

// Derives each body's world-axis-aligned bound from its sphere-swept box
// (see broad_phase_data.hpp) -- the bound Op for this one shape type; another
// shape type would bring its own, writing the same bound outputs. The
// shape's world pose is the body's composed with the shape's own, x_world
// x_shape -- written out whole as kWorldShapePose for the narrow phase to
// read back rather than recompose -- so its orientation is R = R_world
// R_shape and its center rides along with the body (x_world applied to the
// shape's own offset). An unset (zero) kCollisionPose rotation is read as
// identity first, by adding 1 to w exactly when the quaternion is zero, so
// composing it can't zero the body's own rotation out. The inner box's extents
// use Arvo's bound -- each world axis's half-extent is the sum of every shape
// axis's half-extent projected onto it, |R| h -- which is exact for a box (the
// tightest world AABB that still contains it under any rotation). The
// sweep radius and the margin are then added on every axis unrotated: a
// sphere's extent is the same along every direction. All multiply/abs/add,
// so every lane runs the same instructions whatever its orientation.
struct ComputeSweptBoxBoundsOp {
  using FieldEnum = BroadPhaseField;
  using ArgData = engine::ArgData<FieldEnum>;
  using ArgIndex = engine::ArgIndex;

  static constexpr std::array<ArgData, 5> kInputs = {
      ArgData{FieldEnum::kWorldTransform, ArgIndex::kTarget},
      ArgData{FieldEnum::kCollisionPose, ArgIndex::kTarget},
      ArgData{FieldEnum::kCollisionExtents, ArgIndex::kTarget},
      ArgData{FieldEnum::kCollisionRadius, ArgIndex::kTarget},
      ArgData{FieldEnum::kCollisionMargin, ArgIndex::kTarget},
  };
  static constexpr std::array<ArgData, 3> kOutputs = {
      ArgData{FieldEnum::kWorldShapePose, ArgIndex::kTarget},
      ArgData{FieldEnum::kWorldBoundCenter, ArgIndex::kTarget},
      ArgData{FieldEnum::kWorldBoundExtents, ArgIndex::kTarget},
  };

  void operator()(
      const Transform& x_world,
      const Transform& x_shape,
      const Vector3& h,
      const Scalar& radius,
      const Scalar& margin,
      Transform* x_world_shape_out,
      Vector3* c_world_out,
      Vector3* h_world_out
  ) const;
};
static_assert(
    engine::OpLike<ComputeSweptBoxBoundsOp>,
    "ComputeSweptBoxBoundsOp must satisfy OpLike concept"
);
static_assert(
    engine::OpArgsMatchView<ComputeSweptBoxBoundsOp, BroadPhaseView>,
    "ComputeSweptBoxBoundsOp's operator() must line up with kInputs/kOutputs"
);

// Tests one source body's world bound (kSource, the same value in every
// lane) against a whole lane group of target bodies' (kPartner) at once: two
// world-axis-aligned boxes overlap when, on every axis, their centers are no
// further apart than their half-extents summed, |c_t - c_s| <= h_t + h_s.
// A pair also has to pass the group filter both ways (see broad_phase_data.hpp)
// -- the target's group in the source's mask, and the source's group in the
// target's -- and be one the traversal marks as a distinct, non-parent/child
// pair (engine::pass::PairStructure, via kTraversalInputs). The
// per-lane result is packed into one LaneMask word, bit t for lane t,
// the same movemask a GPU port would get from a warp ballot.
struct TestBoundsOverlapOp {
  using FieldEnum = BroadPhaseField;
  using ArgData = engine::ArgData<FieldEnum>;
  using ArgIndex = engine::ArgIndex;

  static constexpr std::array<ArgData, 8> kInputs = {
      ArgData{FieldEnum::kWorldBoundCenter, ArgIndex::kPartner},
      ArgData{FieldEnum::kWorldBoundExtents, ArgIndex::kPartner},
      ArgData{FieldEnum::kCollisionGroup, ArgIndex::kPartner},
      ArgData{FieldEnum::kCollisionMask, ArgIndex::kPartner},
      ArgData{FieldEnum::kWorldBoundCenter, ArgIndex::kSource},
      ArgData{FieldEnum::kWorldBoundExtents, ArgIndex::kSource},
      ArgData{FieldEnum::kCollisionGroup, ArgIndex::kSource},
      ArgData{FieldEnum::kCollisionMask, ArgIndex::kSource},
  };
  static constexpr std::array<ArgIndex, 1> kTraversalInputs = {
      ArgIndex::kStructure,
  };
  static constexpr std::array<ArgData, 1> kOutputs = {
      ArgData{FieldEnum::kBoundsOverlap, ArgIndex::kTarget},
  };

  void operator()(
      const Vector3& c_target,
      const Vector3& h_target,
      const CollisionGroupMask& group_target,
      const CollisionGroupMask& mask_target,
      const Vector3& c_source,
      const Vector3& h_source,
      const CollisionGroupMask& group_source,
      const CollisionGroupMask& mask_source,
      const engine::pass::PairStructure& structure,
      LaneMask* overlap_out
  ) const;
};
static_assert(
    engine::OpLike<TestBoundsOverlapOp>,
    "TestBoundsOverlapOp must satisfy OpLike concept"
);
static_assert(
    engine::OpArgsMatchView<TestBoundsOverlapOp, BroadPhaseView>,
    "TestBoundsOverlapOp's operator() must line up with kInputs/kOutputs"
);

}  // namespace achilles::algorithms::broad_phase
