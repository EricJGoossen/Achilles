#pragma once

#include <xsimd/xsimd.hpp>

#include "domain/math/activation_mask.hpp"
#include "domain/math/matrix.hpp"
#include "domain/math/quaternion.hpp"
#include "domain/math/vector3.hpp"
#include "domain/math/vector6.hpp"
#include "domain/spatial/dual.hpp"
#include "domain/spatial/inertia.hpp"
#include "domain/spatial/transform.hpp"
#include "util/simd_ops.hpp"

namespace achilles::algorithms {
namespace math = achilles::domain::math;
namespace spatial = achilles::domain::spatial;

using BatchOperationT = xsimd::batch<float>;
using ScalarOperationT = float;
using MathematicalT = BatchOperationT;

// Math
using Vector3 = math::Vector3<MathematicalT>;
using Vector6 = math::Vector6<MathematicalT>;
using Matrix3x3 = math::Matrix<MathematicalT, 3, 3>;
using Matrix6x6 = math::Matrix<MathematicalT, 6, 6>;
using Quaternion = math::Quaternion<MathematicalT>;
using Mat6Mask = math::ActivationMask<util::MaskStorageFor<MathematicalT>, 6>;

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

// Additional
using MotionSubspace = Matrix6x6;

}  // namespace achilles::algorithms