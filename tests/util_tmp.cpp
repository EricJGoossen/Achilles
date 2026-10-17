#include <gtest/gtest.h>

#include <ratio>

#include "util/tmp.hpp"

// Covers kIsOneOfV and kRatioValueV -- the two util::tmp helpers the step
// lists depend on (SimContext::Step<StepList>'s "is every Step hosted"
// check, and the std::ratio stage weights/dt fractions in
// algorithms/integrators.hpp). Both are compile-time constants, so the
// static_asserts below are the real checks; the TESTs exist so a failure
// names which check broke, and so the runtime-visible float values get an
// EXPECT_FLOAT_EQ too.

using achilles::util::kIsOneOfV;
using achilles::util::kRatioValueV;

namespace {

struct A {};
struct B {};
struct C {};

// Present at either end of the pack, and as the only element.
static_assert(kIsOneOfV<A, A, B, C>);
static_assert(kIsOneOfV<C, A, B, C>);
static_assert(kIsOneOfV<A, A>);
// Absent, and the empty pack -- a fold over || with no operands is false,
// which is exactly "T is not one of nothing".
static_assert(!kIsOneOfV<C, A, B>);
static_assert(!kIsOneOfV<A>);
// Exact type identity, not convertibility or cv-stripping.
static_assert(!kIsOneOfV<const A, A, B>);
static_assert(!kIsOneOfV<int, float, double>);

// Whole numbers, fractions, and std::ratio's own reduction (2/4 == 1/2).
static_assert(kRatioValueV<float, std::ratio<1>> == 1.0F);
static_assert(kRatioValueV<float, std::ratio<2>> == 2.0F);
static_assert(kRatioValueV<float, std::ratio<1, 2>> == 0.5F);
static_assert(kRatioValueV<float, std::ratio<2, 4>> == 0.5F);
static_assert(kRatioValueV<double, std::ratio<-3, 4>> == -0.75);

}  // namespace

TEST(IsOneOf, AcceptsMembersAndRejectsNonMembers) { SUCCEED(); }

// The value is computed as num/den in T, not in integer arithmetic first --
// 1/3 in integer division would be 0.
TEST(RatioValue, DividesInTheTargetTypeRatherThanInIntegers) {
  EXPECT_FLOAT_EQ((kRatioValueV<float, std::ratio<1, 3>>), 1.0F / 3.0F);
  EXPECT_DOUBLE_EQ((kRatioValueV<double, std::ratio<1, 6>>), 1.0 / 6.0);
}
