#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <xsimd/xsimd.hpp>

#include "algorithms/conventions.hpp"
#include "algorithms/joint_limit/joint_limit_data.hpp"
#include "algorithms/joint_limit/joint_limit_ops.hpp"
#include "support/simd_test_helpers.hpp"
#include "util/simd_ops.hpp"

// DetectJointLimitOp, directly: theta = S^T Log(q), and per DOF the
// distance to the nearer end of [lower, upper], which end that is, and
// whether the row is limited and within the margin (joint_limit_data.hpp).
// Every value is at MathematicalT, the one shape the Op runs at; lanes are
// uniform unless a test is about per-lane independence.

using namespace achilles::algorithms;
using namespace achilles::algorithms::joint_limit;
using achilles::test_support::MakeBatch;

namespace {

using B = MathematicalT;
using MaskBits = achilles::util::MaskStorageFor<B>;

// One limit row's outputs, as a test reads them.
struct Rows {
  JointCoordinates distance;
  JointCoordinates side;
  Mat6Mask active;
};

// Bit `dof` of lane `lane`'s active word.
bool Active(const Rows& rows, std::size_t dof, std::size_t lane = 0) {
  return (rows.active.AsStorage().get(lane) & (std::int32_t{1} << dof)) != 0;
}

// A hinge about z driven by DOF 0 (S's column 0 = angular z).
MotionSubspace HingeZ() {
  MotionSubspace s = MotionSubspace::Zero();
  s(2, 0) = B(1.0F);
  return s;
}

Transform RotationAboutZ(const B& angle) {
  return {Vector3::Zero(), Quaternion::Exp(Vector3(B(0.0F), B(0.0F), angle))};
}

// [lower, upper] on DOF `dof`, every other DOF's range zero.
struct Range {
  JointCoordinates lower = JointCoordinates::Zero();
  JointCoordinates upper = JointCoordinates::Zero();
};
Range RangeOn(std::size_t dof, float lower, float upper) {
  Range range;
  range.lower[dof] = B(lower);
  range.upper[dof] = B(upper);
  return range;
}

Mat6Mask LimitedOn(std::size_t dof) {
  return Mat6Mask(MaskBits(std::int32_t{1} << dof));
}

Rows Detect(
    const MotionSubspace& s,
    const Transform& q,
    const Mat6Mask& limited,
    const Range& range,
    float margin
) {
  Rows rows;
  DetectJointLimitOp{}(
      s,
      q,
      limited,
      range.lower,
      range.upper,
      Scalar(B(margin)),
      &rows.distance,
      &rows.side,
      &rows.active
  );
  return rows;
}

Rows DetectHinge(float angle, float margin = 0.0F) {
  return Detect(
      HingeZ(), RotationAboutZ(B(angle)), LimitedOn(0), RangeOn(0, -1.0F, 1.0F),
      margin
  );
}

}  // namespace

// Well inside the range: a positive distance and no active row.
TEST(DetectJointLimitOp, InsideTheRangeIsInactive) {
  Rows rows = DetectHinge(0.2F);
  EXPECT_NEAR(rows.distance[0].get(0), 0.8F, 1e-5F);
  EXPECT_FALSE(Active(rows, 0));
}

// Only the nearer end counts, and side says which: +1 at the lower end
// (J = +e_k pushes theta back up), -1 at the upper.
TEST(DetectJointLimitOp, NearerEndSetsDistanceAndSide) {
  Rows near_upper = DetectHinge(0.7F);
  EXPECT_NEAR(near_upper.distance[0].get(0), 0.3F, 1e-5F);
  EXPECT_FLOAT_EQ(near_upper.side[0].get(0), -1.0F);

  Rows near_lower = DetectHinge(-0.6F);
  EXPECT_NEAR(near_lower.distance[0].get(0), 0.4F, 1e-5F);
  EXPECT_FLOAT_EQ(near_lower.side[0].get(0), 1.0F);
}

// Past either end the distance goes negative by how far, and the row is
// active even with no margin.
TEST(DetectJointLimitOp, PastAnEndIsNegativeAndActive) {
  Rows past_upper = DetectHinge(1.2F);
  EXPECT_NEAR(past_upper.distance[0].get(0), -0.2F, 1e-5F);
  EXPECT_FLOAT_EQ(past_upper.side[0].get(0), -1.0F);
  EXPECT_TRUE(Active(past_upper, 0));

  Rows past_lower = DetectHinge(-1.3F);
  EXPECT_NEAR(past_lower.distance[0].get(0), -0.3F, 1e-5F);
  EXPECT_FLOAT_EQ(past_lower.side[0].get(0), 1.0F);
  EXPECT_TRUE(Active(past_lower, 0));
}

// The margin activates a row before the limit is reached (MuJoCo's
// jnt_margin): distance 0.05 < margin 0.1 is active though still inside.
TEST(DetectJointLimitOp, MarginActivatesBeforeTheEndIsReached) {
  Rows rows = DetectHinge(0.95F, 0.1F);
  EXPECT_GT(rows.distance[0].get(0), 0.0F);
  EXPECT_TRUE(Active(rows, 0));
  EXPECT_FALSE(Active(DetectHinge(0.85F, 0.1F), 0));
}

// distance < margin is strict, as mj_instantiateLimit's: a joint resting
// exactly on its end with no margin isn't yet a row.
TEST(DetectJointLimitOp, ExactlyAtTheEndWithNoMarginIsInactive) {
  Rows rows = Detect(
      HingeZ(), RotationAboutZ(B(0.0F)), LimitedOn(0), RangeOn(0, 0.0F, 1.0F),
      0.0F
  );
  EXPECT_FLOAT_EQ(rows.distance[0].get(0), 0.0F);
  EXPECT_FALSE(Active(rows, 0));
}

// A DOF not marked limited never activates, however far past its range --
// the zero-fill default (no joint_limited) must mean "no limits".
TEST(DetectJointLimitOp, UnlimitedDofIsNeverActive) {
  Rows rows = Detect(
      HingeZ(), RotationAboutZ(B(2.0F)), Mat6Mask::Zero(),
      RangeOn(0, -1.0F, 1.0F), 0.1F
  );
  EXPECT_LT(rows.distance[0].get(0), 0.0F);
  EXPECT_EQ(rows.active.AsStorage().get(0), 0);
}

// theta is S^T Log(q): DOF k reads S's column k, wherever that column
// points. Here DOF 1 drives rotation about z, so DOF 1's row sees the
// angle and DOF 0's (a zero column) sees 0.
TEST(DetectJointLimitOp, DofIndexIsTheSubspaceColumn) {
  MotionSubspace s = MotionSubspace::Zero();
  s(2, 1) = B(1.0F);
  Range range;
  range.lower[0] = B(-1.0F);
  range.upper[0] = B(1.0F);
  range.lower[1] = B(-0.5F);
  range.upper[1] = B(0.5F);
  Rows rows = Detect(
      s, RotationAboutZ(B(0.6F)),
      Mat6Mask(MaskBits(std::int32_t{0b11})), range, 0.0F
  );

  EXPECT_NEAR(rows.distance[1].get(0), -0.1F, 1e-5F);
  EXPECT_TRUE(Active(rows, 1));
  EXPECT_NEAR(rows.distance[0].get(0), 1.0F, 1e-5F);
  EXPECT_FALSE(Active(rows, 0));
}

// A slide joint's coordinate is its translation along its axis: S's
// column 0 = linear y (row 4), q translated 0.3 along y.
TEST(DetectJointLimitOp, SlideJointLimitsItsTranslation) {
  MotionSubspace s = MotionSubspace::Zero();
  s(4, 0) = B(1.0F);
  Transform q(Vector3(B(0.0F), B(0.3F), B(0.0F)), Quaternion::Identity());
  Rows rows = Detect(s, q, LimitedOn(0), RangeOn(0, -0.2F, 0.2F), 0.0F);

  EXPECT_NEAR(rows.distance[0].get(0), -0.1F, 1e-5F);
  EXPECT_FLOAT_EQ(rows.side[0].get(0), -1.0F);
  EXPECT_TRUE(Active(rows, 0));
}

// Each lane is its own joint: lanes inside, near, and past their ranges
// get their own distances and their own active bits.
TEST(DetectJointLimitOp, EachLaneIsDetectedIndependently) {
  Rows rows = Detect(
      HingeZ(), RotationAboutZ(MakeBatch({0.0F, 1.5F, -1.5F, 0.95F})),
      LimitedOn(0), RangeOn(0, -1.0F, 1.0F), 0.1F
  );

  EXPECT_FALSE(Active(rows, 0, 0));
  EXPECT_TRUE(Active(rows, 0, 1));
  EXPECT_FLOAT_EQ(rows.side[0].get(1), -1.0F);
  if (B::size > 3) {
    EXPECT_TRUE(Active(rows, 0, 2));
    EXPECT_FLOAT_EQ(rows.side[0].get(2), 1.0F);
    EXPECT_TRUE(Active(rows, 0, 3));
  }
}

// A zero-filled padding row (zero S, zero quaternion, no limits) must come
// out finite and inactive: the pass is dense, so padding lanes run too.
TEST(DetectJointLimitOp, ZeroFilledPaddingRowIsFiniteAndInactive) {
  Transform zero_q(
      Vector3::Zero(), Quaternion(B(0.0F), B(0.0F), B(0.0F), B(0.0F))
  );
  Rows rows = Detect(MotionSubspace::Zero(), zero_q, Mat6Mask::Zero(), Range{}, 0.0F);

  for (std::size_t k = 0; k < 6; ++k) {
    EXPECT_FALSE(std::isnan(rows.distance[k].get(0)));
    EXPECT_FALSE(std::isnan(rows.side[k].get(0)));
  }
  EXPECT_EQ(rows.active.AsStorage().get(0), 0);
}
