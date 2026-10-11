#pragma once

#include <type_traits>
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

// float unless built with ACHILLES_DOUBLE_PRECISION (the CMake option of
// the same name): double halves the lanes per SIMD register.
#ifndef ACHILLES_DOUBLE_PRECISION
#define ACHILLES_DOUBLE_PRECISION 0
#endif
using ScalarOperationT =
    std::conditional_t<ACHILLES_DOUBLE_PRECISION != 0, double, float>;
using BatchOperationT = xsimd::batch<ScalarOperationT>;
using MathematicalT = BatchOperationT;

// Compile-time switch for field logging (see algorithms/log/log_step.hpp
// and interface::Simulation::EnableLog). Set by the ACHILLES_ENABLE_LOGGING
// CMake option; when false, LogStep compiles to nothing and Simulation logs
// into a util::NullLogger.
#ifndef ACHILLES_ENABLE_LOGGING
#define ACHILLES_ENABLE_LOGGING 1
#endif
inline constexpr bool kLoggingEnabled = ACHILLES_ENABLE_LOGGING != 0;

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