#pragma once

#include <cstddef>
#include <cstdint>
#include <xsimd/xsimd.hpp>

#include "domain/geometry/box.hpp"
#include "domain/geometry/rectangle.hpp"
#include "domain/geometry/segment.hpp"
#include "domain/math/activation_mask.hpp"
#include "domain/math/matrix.hpp"
#include "domain/math/quaternion.hpp"
#include "domain/math/scalar.hpp"
#include "domain/math/vector3.hpp"
#include "domain/math/vector6.hpp"
#include "domain/spatial/dual.hpp"
#include "domain/spatial/inertia.hpp"
#include "domain/spatial/transform.hpp"
#include "util/simd_ops.hpp"

namespace achilles::algorithms {
namespace math = achilles::domain::math;
namespace spatial = achilles::domain::spatial;
namespace geometry = achilles::domain::geometry;

using BatchOperationT = xsimd::batch<float>;
using ScalarOperationT = float;
using MathematicalT = BatchOperationT;

// How many instances one MathematicalT holds -- the Stride every batched
// traversal walks at, and so how many targets one PairTraversal call pairs a
// source against.
inline constexpr std::size_t kLanes = util::LaneCountOf<MathematicalT>();

// Math
using Scalar = math::Scalar<MathematicalT>;
using Vector3 = math::Vector3<MathematicalT>;
using Vector6 = math::Vector6<MathematicalT>;
using Matrix3x3 = math::Matrix<MathematicalT, 3, 3>;
using Matrix6x6 = math::Matrix<MathematicalT, 6, 6>;
using Quaternion = math::Quaternion<MathematicalT>;
using Mat6Mask = math::ActivationMask<util::MaskStorageFor<MathematicalT>, 6>;

// One bit per lane of a MathematicalT, packed into a plain int32 rather than
// a batch -- what a whole batch's worth of per-lane yes/no answers packs into
// (a compare's movemask, a GPU warp ballot), e.g. one word per (source row,
// target lane group) of a pair pass (see topology::PairGroupIndexing).
using LaneMask = math::ActivationMask<std::int32_t, kLanes>;
static_assert(
    util::LaneCountOf<util::MaskStorageFor<MathematicalT>>() == kLanes,
    "Int32 mask lanes must line up one-to-one with MathematicalT's lanes, "
    "or their movemask bits won't AND with a float compare's."
);

// Spatial
using Transform = spatial::Transform<MathematicalT>;
using Position = spatial::SpatialPosition<MathematicalT>;
using Velocity = spatial::SpatialVelocity<MathematicalT>;
using Acceleration = spatial::SpatialAcceleration<MathematicalT>;
using Momentum = spatial::SpatialMomentum<MathematicalT>;
using Force = spatial::SpatialForce<MathematicalT>;
using Inertia = spatial::Inertia<MathematicalT>;

template <bool Inverted>
using InertiaOperator = spatial::InertiaOperator<MathematicalT, Inverted>;

// Geometry
using Box = geometry::Box<MathematicalT>;
using Rectangle = geometry::Rectangle<MathematicalT>;
using Segment = geometry::Segment<MathematicalT>;

// Additional
using MotionSubspace = Matrix6x6;

}  // namespace achilles::algorithms