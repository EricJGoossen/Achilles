#include <gtest/gtest.h>

#include "domain/geometry/ray.hpp"
#include "domain/geometry/swept_box.hpp"
#include "domain/math/vector3.hpp"

using achilles::domain::geometry::Ray;
using achilles::domain::geometry::RayHit;
using achilles::domain::geometry::SweptBox;
using Vector3 = achilles::domain::math::Vector3<float>;

namespace {

constexpr float kTolerance = 1e-3F;

}  // namespace

TEST(Ray, AtWalksAlongTheDirection) {
  const Ray<float> ray(Vector3(1.0F, 2.0F, 3.0F), Vector3(0.0F, 0.0F, 1.0F));
  const Vector3 point = ray.At(2.0F);
  EXPECT_FLOAT_EQ(point.X(), 1.0F);
  EXPECT_FLOAT_EQ(point.Y(), 2.0F);
  EXPECT_FLOAT_EQ(point.Z(), 5.0F);
}

TEST(SweptBox, DistanceIsSignedFromTheSurface) {
  const SweptBox<float> box(Vector3(1.0F, 1.0F, 1.0F), 0.5F);
  EXPECT_NEAR(box.Distance(Vector3(3.0F, 0.0F, 0.0F)), 1.5F, kTolerance);
  EXPECT_NEAR(box.Distance(Vector3(0.0F, 0.0F, 0.0F)), -1.5F, kTolerance);
  // Off a corner, the sweep rounds it: |(1, 1, 1)| - 0.5.
  EXPECT_NEAR(
      box.Distance(Vector3(2.0F, 2.0F, 2.0F)),
      Vector3(1.0F, 1.0F, 1.0F).Norm() - 0.5F,
      kTolerance
  );
}

TEST(SweptBox, CastHitsAFaceAtItsDistance) {
  const SweptBox<float> box(Vector3(1.0F, 2.0F, 3.0F), 0.0F);
  const Ray<float> ray(Vector3(-5.0F, 0.0F, 0.0F), Vector3(1.0F, 0.0F, 0.0F));
  const RayHit<float> hit = box.Cast(ray, 0.0F, 10.0F);
  ASSERT_TRUE(hit.hit);
  EXPECT_NEAR(hit.distance, 4.0F, kTolerance);
}

TEST(SweptBox, CastHitsASphereAtItsRadius) {
  const SweptBox<float> sphere(Vector3(0.0F, 0.0F, 0.0F), 0.5F);
  const Ray<float> ray(Vector3(0.0F, -3.0F, 0.0F), Vector3(0.0F, 1.0F, 0.0F));
  const RayHit<float> hit = sphere.Cast(ray, 0.0F, 10.0F);
  ASSERT_TRUE(hit.hit);
  EXPECT_NEAR(hit.distance, 2.5F, kTolerance);
}

TEST(SweptBox, CastHitsACapsulesSide) {
  // A capsule along y: inner half-length 1, radius 0.25.
  const SweptBox<float> capsule(Vector3(0.0F, 1.0F, 0.0F), 0.25F);
  const Ray<float> ray(Vector3(0.0F, 0.5F, 4.0F), Vector3(0.0F, 0.0F, -1.0F));
  const RayHit<float> hit = capsule.Cast(ray, 0.0F, 10.0F);
  ASSERT_TRUE(hit.hit);
  EXPECT_NEAR(hit.distance, 3.75F, kTolerance);
}

TEST(SweptBox, CastMissesPastTheShapeAndBeyondTheRange) {
  const SweptBox<float> box(Vector3(1.0F, 1.0F, 1.0F), 0.0F);
  const Ray<float> past(Vector3(-5.0F, 3.0F, 0.0F), Vector3(1.0F, 0.0F, 0.0F));
  EXPECT_FALSE(box.Cast(past, 0.0F, 10.0F).hit);

  const Ray<float> toward(
      Vector3(-5.0F, 0.0F, 0.0F), Vector3(1.0F, 0.0F, 0.0F)
  );
  const RayHit<float> short_range = box.Cast(toward, 0.0F, 3.0F);
  EXPECT_FALSE(short_range.hit);
  EXPECT_FLOAT_EQ(short_range.distance, 3.0F);
}

TEST(SweptBox, CastFromInsideDoesntSeeTheShape) {
  const SweptBox<float> box(Vector3(1.0F, 1.0F, 1.0F), 0.0F);
  const Ray<float> ray(Vector3(0.0F, 0.0F, 0.0F), Vector3(1.0F, 0.0F, 0.0F));
  EXPECT_FALSE(box.Cast(ray, 0.0F, 10.0F).hit);
}

TEST(SweptBox, CastSkipsWhatsCloserThanMinRange) {
  const SweptBox<float> box(Vector3(1.0F, 1.0F, 1.0F), 0.0F);
  const Ray<float> ray(Vector3(-2.0F, 0.0F, 0.0F), Vector3(1.0F, 0.0F, 0.0F));
  // The face is 1 away; starting the search at 1.5 lands inside the box.
  EXPECT_FALSE(box.Cast(ray, 1.5F, 10.0F).hit);
  EXPECT_TRUE(box.Cast(ray, 0.5F, 10.0F).hit);
}

TEST(SweptBox, AnEmptyShapeIsNeverHit) {
  const SweptBox<float> nothing(Vector3(0.0F, 0.0F, 0.0F), 0.0F);
  const Ray<float> ray(Vector3(-1.0F, 0.0F, 0.0F), Vector3(1.0F, 0.0F, 0.0F));
  EXPECT_FALSE(nothing.Cast(ray, 0.0F, 10.0F).hit);
}
