#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

#include "domain/math/quaternion.hpp"
#include "domain/spatial/transform.hpp"
#include "render/mat4.hpp"

// render::mat4.hpp is the one part of the visualization module with no GL/
// window/GPU dependency at all -- pure matrix math, so it's the part this
// headless test environment can actually exercise directly (see its own
// header comment). The rest of the renderer (shader/mesh/window/
// scene_renderer) needs a live GL context and is verified by hand on a
// machine with a display, not gtest.

using achilles::domain::math::Quaternion;
using achilles::render::FromTransformAndScale;
using achilles::render::Identity;
using achilles::render::LookAt;
using achilles::render::Mat4;
using achilles::render::Perspective;
using achilles::render::Vec3;
using Transform = achilles::domain::spatial::Transform<float>;

namespace {

// Multiplies a 4x4 by a homogeneous column point, returning the result as
// plain (x, y, z, w) rather than routing through Vec4Column/a 4x1
// Matrix -- keeps every call site below a single readable line.
struct Vec4 {
  float x, y, z, w;
};

Vec4 Transform4(const Mat4& m, float x, float y, float z, float w) {
  Vec4 out{};
  out.x = m(0, 0) * x + m(0, 1) * y + m(0, 2) * z + m(0, 3) * w;
  out.y = m(1, 0) * x + m(1, 1) * y + m(1, 2) * z + m(1, 3) * w;
  out.z = m(2, 0) * x + m(2, 1) * y + m(2, 2) * z + m(2, 3) * w;
  out.w = m(3, 0) * x + m(3, 1) * y + m(3, 2) * z + m(3, 3) * w;
  return out;
}

constexpr float kEpsilon = 1e-5F;

}  // namespace

TEST(Mat4, IdentityLeavesAPointUnchanged) {
  Mat4 m = Identity();
  Vec4 p = Transform4(m, 1.0F, 2.0F, 3.0F, 1.0F);
  EXPECT_NEAR(p.x, 1.0F, kEpsilon);
  EXPECT_NEAR(p.y, 2.0F, kEpsilon);
  EXPECT_NEAR(p.z, 3.0F, kEpsilon);
  EXPECT_NEAR(p.w, 1.0F, kEpsilon);
}

// A point sitting exactly on the near plane, on the view axis, must land
// at NDC z == -1 after the perspective divide -- the defining property of
// OpenGL's own clip-space convention (see Perspective's own comment).
TEST(Mat4, PerspectiveMapsNearPlaneCenterToNdcZMinusOne) {
  float znear = 0.5F;
  float zfar = 100.0F;
  Mat4 proj = Perspective(1.2F, 16.0F / 9.0F, znear, zfar);

  Vec4 clip = Transform4(proj, 0.0F, 0.0F, -znear, 1.0F);
  ASSERT_GT(clip.w, 0.0F);
  EXPECT_NEAR(clip.z / clip.w, -1.0F, kEpsilon);
}

// Same, at the far plane -- NDC z == +1.
TEST(Mat4, PerspectiveMapsFarPlaneCenterToNdcZPlusOne) {
  float znear = 0.5F;
  float zfar = 100.0F;
  Mat4 proj = Perspective(1.2F, 16.0F / 9.0F, znear, zfar);

  Vec4 clip = Transform4(proj, 0.0F, 0.0F, -zfar, 1.0F);
  ASSERT_GT(clip.w, 0.0F);
  EXPECT_NEAR(clip.z / clip.w, 1.0F, kEpsilon);
}

// LookAt must map its own `eye` argument to the view-space origin --
// exactly what "the camera is at eye" means.
TEST(Mat4, LookAtMapsEyeToOrigin) {
  Vec3 eye(3.0F, 4.0F, 5.0F);
  Vec3 target(0.0F, 0.0F, 0.0F);
  Vec3 up(0.0F, 1.0F, 0.0F);
  Mat4 view = LookAt(eye, target, up);

  Vec4 p = Transform4(view, eye.X(), eye.Y(), eye.Z(), 1.0F);
  EXPECT_NEAR(p.x, 0.0F, kEpsilon);
  EXPECT_NEAR(p.y, 0.0F, kEpsilon);
  EXPECT_NEAR(p.z, 0.0F, kEpsilon);
}

// LookAt must map a point straight ahead of the camera onto the view
// space's own -Z axis (OpenGL's right-handed camera-looks-down--Z
// convention), at a distance equal to how far ahead it actually is.
TEST(Mat4, LookAtMapsForwardPointOntoNegativeViewZ) {
  Vec3 eye(0.0F, 0.0F, 5.0F);
  Vec3 target(0.0F, 0.0F, 0.0F);
  Vec3 up(0.0F, 1.0F, 0.0F);
  Mat4 view = LookAt(eye, target, up);

  Vec4 p = Transform4(view, 0.0F, 0.0F, 0.0F, 1.0F);
  EXPECT_NEAR(p.x, 0.0F, kEpsilon);
  EXPECT_NEAR(p.y, 0.0F, kEpsilon);
  EXPECT_NEAR(p.z, -5.0F, kEpsilon);
}

// A pure-translation transform (identity rotation) must carry a local
// origin point to exactly that world translation, unscaled (scale =
// Ones()).
TEST(Mat4, FromTransformAndScaleAppliesTranslation) {
  Transform t(Vec3(1.0F, 2.0F, 3.0F), Quaternion<float>::Identity());
  Mat4 m = FromTransformAndScale(t, Vec3::Ones());

  Vec4 p = Transform4(m, 0.0F, 0.0F, 0.0F, 1.0F);
  EXPECT_NEAR(p.x, 1.0F, kEpsilon);
  EXPECT_NEAR(p.y, 2.0F, kEpsilon);
  EXPECT_NEAR(p.z, 3.0F, kEpsilon);
}

// A 90-degree yaw (about Z) must rotate the local +X axis onto world +Y --
// proves the rotation block, not just the translation column, actually
// makes it into the model matrix.
TEST(Mat4, FromTransformAndScaleAppliesRotation) {
  float half_angle = std::numbers::pi_v<float> / 4.0F;
  Quaternion<float> yaw90(
      std::cos(half_angle), 0.0F, 0.0F, std::sin(half_angle)
  );
  Transform t(Vec3::Zero(), yaw90);
  Mat4 m = FromTransformAndScale(t, Vec3::Ones());

  Vec4 p = Transform4(m, 1.0F, 0.0F, 0.0F, 1.0F);
  EXPECT_NEAR(p.x, 0.0F, kEpsilon);
  EXPECT_NEAR(p.y, 1.0F, kEpsilon);
  EXPECT_NEAR(p.z, 0.0F, kEpsilon);
}

// Scale is applied in the pose's own local frame: with identity rotation,
// a local-axis point must simply be multiplied component-wise.
TEST(Mat4, FromTransformAndScaleAppliesLocalScale) {
  Transform t(Vec3::Zero(), Quaternion<float>::Identity());
  Mat4 m = FromTransformAndScale(t, Vec3(2.0F, 3.0F, 4.0F));

  Vec4 p = Transform4(m, 1.0F, 1.0F, 1.0F, 1.0F);
  EXPECT_NEAR(p.x, 2.0F, kEpsilon);
  EXPECT_NEAR(p.y, 3.0F, kEpsilon);
  EXPECT_NEAR(p.z, 4.0F, kEpsilon);
}
