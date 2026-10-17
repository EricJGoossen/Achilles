#pragma once

#include <array>

#include "algorithms/collision/collision_data.hpp"
#include "algorithms/conventions.hpp"
#include "engine/pass/op_invoker.hpp"
#include "engine/pass/traversals.hpp"

namespace achilles::algorithms::collision {

// Re-expresses each body's own body-frame box as a world-axis-aligned
// bound. The center just rides along with the body (x_world applied to
// c_local); the extents use Arvo's bound -- each world axis's half-extent
// is the sum of every body axis's half-extent projected onto it,
// |R| h_local -- which is exact for a box (the tightest world AABB that
// still contains it under any rotation) and is pure multiply/abs/add, so
// every lane runs the same instructions whatever its orientation. The
// body's margin is then added on every axis (see collision_data.hpp).
struct ComputeWorldBoundsOp {
  using FieldEnum = CollisionField;
  using ArgData = engine::ArgData<FieldEnum>;
  using ArgIndex = engine::ArgIndex;

  static constexpr std::array<ArgData, 4> kInputs = {
      ArgData{FieldEnum::kWorldTransform, ArgIndex::kTarget},
      ArgData{FieldEnum::kCollisionCenter, ArgIndex::kTarget},
      ArgData{FieldEnum::kCollisionExtents, ArgIndex::kTarget},
      ArgData{FieldEnum::kCollisionMargin, ArgIndex::kTarget},
  };
  static constexpr std::array<ArgData, 2> kOutputs = {
      ArgData{FieldEnum::kWorldBoundCenter, ArgIndex::kTarget},
      ArgData{FieldEnum::kWorldBoundExtents, ArgIndex::kTarget},
  };

  void operator()(
      const Transform& x_world,
      const Vector3& c_local,
      const Vector3& h_local,
      const Scalar& margin,
      Vector3* c_world_out,
      Vector3* h_world_out
  ) const;
};
static_assert(
    engine::OpLike<ComputeWorldBoundsOp>,
    "ComputeWorldBoundsOp must satisfy OpLike concept"
);
static_assert(
    engine::OpArgsMatchView<ComputeWorldBoundsOp, CollisionView>,
    "ComputeWorldBoundsOp's operator() must line up with kInputs/kOutputs"
);

// Tests one source body's world bound (kSource, the same value in every
// lane) against a whole lane group of target bodies' (kTarget) at once: two
// world-axis-aligned boxes overlap when, on every axis, their centers are no
// further apart than their half-extents summed, |c_t - c_s| <= h_t + h_s.
// A pair also has to pass the group filter both ways (see collision_data.hpp)
// -- the target's group in the source's mask, and the source's group in the
// target's -- and be one the traversal marks as a distinct, non-parent/child
// pair (engine::pass::PairStructure, via kTraversalInputs). The
// per-lane result is packed into one PairLaneMask word, bit t for lane t,
// the same movemask a GPU port would get from a warp ballot.
struct TestBoundsOverlapOp {
  using FieldEnum = CollisionField;
  using ArgData = engine::ArgData<FieldEnum>;
  using ArgIndex = engine::ArgIndex;

  static constexpr std::array<ArgData, 8> kInputs = {
      ArgData{FieldEnum::kWorldBoundCenter, ArgIndex::kTarget},
      ArgData{FieldEnum::kWorldBoundExtents, ArgIndex::kTarget},
      ArgData{FieldEnum::kCollisionGroup, ArgIndex::kTarget},
      ArgData{FieldEnum::kCollisionMask, ArgIndex::kTarget},
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
      PairLaneMask* overlap_out
  ) const;
};
static_assert(
    engine::OpLike<TestBoundsOverlapOp>,
    "TestBoundsOverlapOp must satisfy OpLike concept"
);
static_assert(
    engine::OpArgsMatchView<TestBoundsOverlapOp, CollisionView>,
    "TestBoundsOverlapOp's operator() must line up with kInputs/kOutputs"
);

}  // namespace achilles::algorithms::collision
