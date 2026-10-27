#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>

#include "algorithms/conventions.hpp"
#include "algorithms/narrow_phase/narrow_phase_data.hpp"
#include "algorithms/narrow_phase/narrow_phase_ops.hpp"

// ComputeSweptBoxContactOp's activation margin: a pair's contact counts
// while its distance is within m_s + m_t (mj_assignMargin's sum), not the
// larger of the two. Two spheres (zero extents, radius 0.1) a known gap
// apart along y, every lane the same pair.

using namespace achilles::algorithms;
using namespace achilles::algorithms::narrow_phase;

namespace {

using B = MathematicalT;

constexpr float kRadius = 0.1F;

struct SpherePair {
  Vector3 normal;
  ContactDistances distance;
  ContactPoints point;
  ContactActiveMask active;
};

// Spheres at the origin and at (0, 2 r + gap, 0), with margins m_t, m_s.
SpherePair Collide(float gap, float m_target, float m_source) {
  SpherePair out;
  LaneMask candidate(static_cast<std::int32_t>((std::uint32_t{1} << kLanes) - 1)
  );
  ComputeSweptBoxContactOp{
  }(Transform::Identity(),
    Vector3::Zero(),
    Scalar(B(kRadius)),
    Scalar(B(m_target)),
    Transform(
        Vector3(B(0.0F), B((2.0F * kRadius) + gap), B(0.0F)),
        Quaternion::Identity()
    ),
    Vector3::Zero(),
    Scalar(B(kRadius)),
    Scalar(B(m_source)),
    candidate,
    &out.normal,
    &out.distance,
    &out.point,
    &out.active);
  return out;
}

// Lane 0's active bit in any slot (bit k * kLanes for slot k).
bool AnySlotActiveInLaneZero(const ContactActiveMask& active) {
  auto bits = static_cast<std::uint32_t>(active.AsStorage());
  for (std::size_t k = 0; k < kSweptBoxContactSlots; ++k) {
    if (((bits >> (k * kLanes)) & 1U) != 0U) {
      return true;
    }
  }
  return false;
}

}  // namespace

// A 0.004 gap with margins 0.003 and 0.002: inside their sum (0.005) but
// outside the larger one (0.003) -- active only under the sum rule.
TEST(SweptBoxContactMargin, GapWithinTheSumOfBothMarginsIsActive) {
  SpherePair out = Collide(0.004F, 0.003F, 0.002F);
  EXPECT_TRUE(AnySlotActiveInLaneZero(out.active));
  EXPECT_NEAR(out.distance(0, 0).get(0), 0.004F, 1e-5F);
}

// The same gap with margins 0.003 and 0.0005 (sum 0.0035) is beyond it.
TEST(SweptBoxContactMargin, GapBeyondTheSumIsInactive) {
  EXPECT_FALSE(AnySlotActiveInLaneZero(Collide(0.004F, 0.003F, 0.0005F).active)
  );
}

// The sum is symmetric in which body is the target: swapping the two
// margins doesn't change the answer.
TEST(SweptBoxContactMargin, EitherBodysMarginCounts) {
  EXPECT_TRUE(AnySlotActiveInLaneZero(Collide(0.004F, 0.002F, 0.003F).active));
}
