/**
 * Tests for math and spatial helper objects.
 *
 * Covers:
 *   - Math:    Vector, UnitVector, Quaternion, Id
 *   - Spatial: Pose, Twist, Surge, Wrench, Inertia
 */

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>

#include "math/id.hpp"
#include "math/quaternion.hpp"
#include "math/unit_vector.hpp"
#include "math/vector.hpp"
#include "spatial/inertia.hpp"
#include "spatial/pose.hpp"
#include "spatial/surge.hpp"
#include "spatial/twist.hpp"
#include "spatial/wrench.hpp"

// ─────────────────────────────────────────────────────────────
//  Helpers
// ─────────────────────────────────────────────────────────────
namespace {

namespace math = achilles::math;
namespace spatial = achilles::spatial;

constexpr double EPS = 1e-9;
constexpr double LOOSE = 1e-6;

spatial::Inertia sphereInertia(double mass, double r) {
    double I = 0.4 * mass * r * r;
    return achilles::spatial::Inertia{
        I * Eigen::Matrix3d::Identity(), achilles::math::Vector::zero(), mass};
}

spatial::Inertia sphereInertia() { return sphereInertia(1.0, 0.1); }

}  // namespace

// ══════════════════════════════════════════════════════════════
//  1. Vector
// ══════════════════════════════════════════════════════════════

TEST(Vector, DefaultIsZeroVector) {
    math::Vector v;
    EXPECT_NEAR(v.mag(), 0.0, EPS);
}

TEST(Vector, ArithmeticRoundTrip) {
    math::Vector a(1, 2, 3);
    math::Vector b(4, 5, 6);
    math::Vector sum = a + b;
    EXPECT_NEAR(sum.x(), 5.0, EPS);
    EXPECT_NEAR(sum.y(), 7.0, EPS);
    EXPECT_NEAR(sum.z(), 9.0, EPS);

    math::Vector diff = b - a;
    EXPECT_NEAR(diff.x(), 3.0, EPS);
}

TEST(Vector, ScalarMultiplyAndDivide) {
    math::Vector v(2, 4, 6);
    EXPECT_NEAR((v * 0.5).x(), 1.0, EPS);
    EXPECT_NEAR((v * 0.5).y(), 2.0, EPS);
    EXPECT_NEAR((v / 2.0).z(), 3.0, EPS);
}

TEST(Vector, DotProduct) {
    EXPECT_NEAR(math::Vector(1, 0, 0).dot(math::Vector(0, 1, 0)), 0.0, EPS);
    EXPECT_NEAR(math::Vector(1, 2, 3).dot(math::Vector(1, 2, 3)), 14.0, EPS);
}

TEST(Vector, CrossProduct) {
    math::Vector z = math::Vector(1, 0, 0).cross(math::Vector(0, 1, 0));
    EXPECT_NEAR(z.x(), 0.0, EPS);
    EXPECT_NEAR(z.y(), 0.0, EPS);
    EXPECT_NEAR(z.z(), 1.0, EPS);
}

TEST(Vector, NormalizedHasUnitLength) {
    EXPECT_NEAR(math::Vector(3, 4, 0).normalized().mag(), 1.0, EPS);
}

TEST(Vector, SkewMatrixAntiSymmetric) {
    math::Vector v(1, 2, 3);
    Eigen::Matrix3d s = v.skew();
    EXPECT_NEAR((s + s.transpose()).norm(), 0.0, EPS);
    EXPECT_NEAR((s * v.mat()).norm(), 0.0, EPS);
}

TEST(Vector, PropagateIntegration) {
    math::Vector pos(0, 0, 0);
    pos.propagate(math::Vector(1, 2, 3), 0.5);
    EXPECT_NEAR(pos.x(), 0.5, EPS);
    EXPECT_NEAR(pos.y(), 1.0, EPS);
    EXPECT_NEAR(pos.z(), 1.5, EPS);
}

// ══════════════════════════════════════════════════════════════
//  2. UnitVector
// ══════════════════════════════════════════════════════════════

TEST(UnitVector, AlwaysUnitLength) {
    EXPECT_NEAR(math::UnitVector(3.0, 4.0, 0.0).mag(), 1.0, EPS);
}

TEST(UnitVector, AxisAligned) {
    math::UnitVector x(1, 0, 0);
    EXPECT_NEAR(x.x(), 1.0, EPS);
    EXPECT_NEAR(x.y(), 0.0, EPS);
    EXPECT_NEAR(x.z(), 0.0, EPS);
}

// ══════════════════════════════════════════════════════════════
//  3. Quaternion
// ══════════════════════════════════════════════════════════════

TEST(Quaternion, IdentityRotatesVectorUnchanged) {
    math::Vector r = math::Quaternion::identity().rotate(math::Vector(1, 2, 3));
    EXPECT_NEAR(r.x(), 1.0, EPS);
    EXPECT_NEAR(r.y(), 2.0, EPS);
    EXPECT_NEAR(r.z(), 3.0, EPS);
}

TEST(Quaternion, 90DegRotationAboutZ) {
    double a = M_PI_4;
    math::Quaternion q(std::cos(a), 0, 0, std::sin(a));
    math::Vector result = q.rotate(math::Vector(1, 0, 0));
    EXPECT_NEAR(result.x(), 0.0, LOOSE);
    EXPECT_NEAR(result.y(), 1.0, LOOSE);
    EXPECT_NEAR(result.z(), 0.0, LOOSE);
}

TEST(Quaternion, InverseRoundTrip) {
    double a = M_PI / 3.0;
    math::Quaternion q(std::cos(a), std::sin(a), 0, 0);
    math::Quaternion prod = q * q.inverse();
    EXPECT_NEAR(prod.w(), 1.0, LOOSE);
    EXPECT_NEAR(prod.x(), 0.0, LOOSE);
    EXPECT_NEAR(prod.y(), 0.0, LOOSE);
    EXPECT_NEAR(prod.z(), 0.0, LOOSE);
}

TEST(Quaternion, PropagateZeroAngularVelocityIsIdentity) {
    math::Quaternion q = math::Quaternion::identity();
    q.propagate(math::Vector(0, 0, 0), 0.1);
    EXPECT_NEAR(q.w(), 1.0, LOOSE);
}

TEST(Quaternion, PropagateNormPreserved) {
    math::Quaternion q = math::Quaternion::identity();
    math::Vector omega(0.5, 0.2, 0.1);
    for (int i = 0; i < 100; ++i) {
        q.propagate(omega, 0.01);
    }
    EXPECT_NEAR(q.mag(), 1.0, LOOSE);
}

// ══════════════════════════════════════════════════════════════
//  4. Id
// ══════════════════════════════════════════════════════════════

TEST(Id, SequentialIds) {
    math::Id<int>::IdManager mgr;
    auto id0 = math::Id<int>(mgr);
    auto id1 = math::Id<int>(mgr);
    EXPECT_NE(id0, id1);
    EXPECT_EQ(id0.id(), 0U);
    EXPECT_EQ(id1.id(), 1U);
}

TEST(Id, EqualityOnCopy) {
    math::Id<int>::IdManager mgr;
    auto id = math::Id<int>(mgr);
    auto copy = id;
    EXPECT_EQ(id, copy);
}

// ══════════════════════════════════════════════════════════════
//  5. Pose
// ══════════════════════════════════════════════════════════════

TEST(Pose, IdentityCompose) {
    spatial::Pose p(math::Vector(1, 2, 3), math::Quaternion::identity());
    spatial::Pose result = spatial::Pose::identity() * p;
    EXPECT_NEAR(result.position().x(), 1.0, EPS);
    EXPECT_NEAR(result.position().y(), 2.0, EPS);
    EXPECT_NEAR(result.position().z(), 3.0, EPS);
}

TEST(Pose, InverseRoundTrip) {
    double a = M_PI / 4;
    math::Quaternion q(std::cos(a), std::sin(a), 0, 0);
    spatial::Pose p(math::Vector(1, 0, 0), q);
    spatial::Pose identity = p * p.inverse();
    EXPECT_NEAR(identity.position().mag(), 0.0, LOOSE);
    EXPECT_NEAR(identity.orientation().w(), 1.0, LOOSE);
}

TEST(Pose, TransformPoint) {
    spatial::Pose p(math::Vector(1, 0, 0), math::Quaternion::identity());
    math::Vector transformed = p * math::Vector(0, 0, 0);
    EXPECT_NEAR(transformed.x(), 1.0, EPS);
}

TEST(Pose, PropagateZeroTwistNoChange) {
    spatial::Pose p(math::Vector(1, 2, 3), math::Quaternion::identity());
    p.propagate(spatial::Twist::identity(), 0.1);
    EXPECT_NEAR(p.position().x(), 1.0, LOOSE);
}

TEST(Pose, PropagateLinearVelocity) {
    spatial::Pose p = spatial::Pose::identity();
    p.propagate(
        spatial::Twist(math::Vector(1, 0, 0), math::Vector(0, 0, 0)), 1.0
    );
    EXPECT_NEAR(p.position().x(), 1.0, LOOSE);
}

// ══════════════════════════════════════════════════════════════
//  6. Twist
// ══════════════════════════════════════════════════════════════

TEST(Twist, Addition) {
    spatial::Twist a(math::Vector(1, 0, 0), math::Vector(0, 0, 1));
    spatial::Twist b(math::Vector(0, 1, 0), math::Vector(0, 0, 0));
    spatial::Twist c = a + b;
    EXPECT_NEAR(c.linear().x(), 1.0, EPS);
    EXPECT_NEAR(c.linear().y(), 1.0, EPS);
    EXPECT_NEAR(c.angular().z(), 1.0, EPS);
}

TEST(Twist, ScalarMultiply) {
    spatial::Twist t(math::Vector(2, 4, 6), math::Vector(1, 2, 3));
    EXPECT_NEAR((t * 0.5).linear().x(), 1.0, EPS);
}

TEST(Twist, MatRoundTrip) {
    Eigen::Matrix<double, 6, 1> v;
    v << 1, 2, 3, 4, 5, 6;
    EXPECT_NEAR((v - spatial::Twist(v).mat()).norm(), 0.0, EPS);
}

// ══════════════════════════════════════════════════════════════
//  7. Surge
// ══════════════════════════════════════════════════════════════

TEST(Surge, ZeroIsZero) {
    EXPECT_NEAR(spatial::Surge::zero().norm(), 0.0, EPS);
}

TEST(Surge, FromWrenchIdentityInertia) {
    spatial::Surge j = spatial::Surge::fromWrench(
        spatial::Wrench(math::Vector(1, 0, 0), math::Vector(0, 0, 0)),
        spatial::Inertia::identity()
    );
    EXPECT_NEAR(j.linear().x(), 1.0, EPS);
}

// ══════════════════════════════════════════════════════════════
//  8. Inertia
// ══════════════════════════════════════════════════════════════

TEST(Inertia, ZeroMatrixIsZero) {
    EXPECT_NEAR(spatial::Inertia::zero().mat().norm(), 0.0, EPS);
}

TEST(Inertia, AdditionIsCommutative) {
    spatial::Inertia a = spatial::Inertia::identity();
    spatial::Inertia b = sphereInertia(2.0, 0.2);
    EXPECT_NEAR(((a + b).mat() - (b + a).mat()).norm(), 0.0, EPS);
}

TEST(Inertia, SphereIsPositiveDefinite) {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> es(
        sphereInertia().mat()
    );
    EXPECT_GT(es.eigenvalues().minCoeff(), 0.0);
}

// ══════════════════════════════════════════════════════════════
//  9. Wrench
// ══════════════════════════════════════════════════════════════

TEST(Wrench, NormalizedHasUnitNorm) {
    spatial::Wrench w(math::Vector(3, 4, 0), math::Vector(0, 0, 0));
    EXPECT_NEAR(w.normalized().norm(), 1.0, EPS);
}
