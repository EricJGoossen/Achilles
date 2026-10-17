#pragma once

#include <array>

#include "algorithms/collision/collision_data.hpp"
#include "algorithms/conventions.hpp"
#include "engine/pass/op_invoker.hpp"

namespace achilles::algorithms::collision {

// Re-expresses each body's own body-frame box as a world-axis-aligned
// bound. The center just rides along with the body (x_world applied to
// c_local); the extents use Arvo's bound -- each world axis's half-extent
// is the sum of every body axis's half-extent projected onto it,
// |R| h_local -- which is exact for a box (the tightest world AABB that
// still contains it under any rotation) and is pure multiply/abs/add, so
// every lane runs the same instructions whatever its orientation.
struct ComputeWorldBoundsOp {
  using FieldEnum = CollisionField;
  using ArgData = engine::ArgData<FieldEnum>;

  static constexpr std::array<ArgData, 3> kInputs = {
      ArgData{FieldEnum::kWorldTransform, true},
      ArgData{FieldEnum::kCollisionCenter, true},
      ArgData{FieldEnum::kCollisionExtents, true},
  };
  static constexpr std::array<ArgData, 2> kOutputs = {
      ArgData{FieldEnum::kWorldBoundCenter, true},
      ArgData{FieldEnum::kWorldBoundExtents, true},
  };

  void operator()(
      const Transform& x_world,
      const Vector3& c_local,
      const Vector3& h_local,
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

}  // namespace achilles::algorithms::collision
