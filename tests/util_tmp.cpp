#include <gtest/gtest.h>

#include <cstdint>
#include <ratio>
#include <type_traits>

#include "util/tmp.hpp"

// Covers kIsOneOfV and kRatioValueV -- the two util::tmp helpers the step
// lists depend on (SimContext::Step<StepList>'s "is every Step hosted"
// check, and the std::ratio stage weights/dt fractions in
// algorithms/integrators.hpp) -- and DetectedOrT, which resolves every
// optional per-field policy (engine/memory/binding.hpp). All are
// compile-time, so the static_asserts below are the real checks; the TESTs
// exist so a failure names which check broke, and so the runtime-visible
// float values get an EXPECT_FLOAT_EQ too.

using achilles::util::DetectedOrT;
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

// DetectedOrT's probe: the member type `Policy`, if T declares one.
template <typename T>
using PolicyOf = typename T::Policy;

struct DeclaresPolicy {
  using Policy = B;
};
struct DeclaresNothing {};
// A member with the right name that isn't a type -- must read as absent,
// not as a hard error.
struct DeclaresPolicyAsFunction {
  static void Policy() {}
};

static_assert(std::is_same_v<DetectedOrT<A, PolicyOf, DeclaresPolicy>, B>);
static_assert(std::is_same_v<DetectedOrT<A, PolicyOf, DeclaresNothing>, A>);
static_assert(std::is_same_v<
              DetectedOrT<A, PolicyOf, DeclaresPolicyAsFunction>,
              A>);
// Not just for member probes: any alias template, any arity.
template <typename T, typename U>
using CommonOf = std::common_type_t<T, U>;
static_assert(std::is_same_v<
              DetectedOrT<A, CommonOf, std::int32_t, std::int64_t>,
              std::int64_t>);
static_assert(std::is_same_v<DetectedOrT<C, CommonOf, A, B>, C>);

}  // namespace

TEST(IsOneOf, AcceptsMembersAndRejectsNonMembers) { SUCCEED(); }

TEST(DetectedOr, YieldsTheProbedTypeWhenValidAndTheDefaultOtherwise) {
  SUCCEED();
}

// The value is computed as num/den in T, not in integer arithmetic first --
// 1/3 in integer division would be 0.
TEST(RatioValue, DividesInTheTargetTypeRatherThanInIntegers) {
  EXPECT_FLOAT_EQ((kRatioValueV<float, std::ratio<1, 3>>), 1.0F / 3.0F);
  EXPECT_DOUBLE_EQ((kRatioValueV<double, std::ratio<1, 6>>), 1.0 / 6.0);
}
