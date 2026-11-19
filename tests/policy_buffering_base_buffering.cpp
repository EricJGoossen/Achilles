#include <gtest/gtest.h>

#include <array>
#include <bit>
#include <cstddef>

#include "engine/memory/buffered_scalar.hpp"
#include "policy/buffering/base_buffering.hpp"
#include "policy/buffering/double_buffering.hpp"
#include "policy/buffering/single_buffering.hpp"

// Covers the buffering policies and BufferedScalar on raw memory -- the
// offset arithmetic every double-buffered View read and every actuator write
// is built on. The policies are exercised directly here, not through a
// SimAllocator, so a wrong offset fails here by name rather than as a
// mysteriously stale value several layers up (engine_view_view.cpp and
// engine_pass_sim_context.cpp cover the same policies through the real
// allocator).

using achilles::engine::memory::BufferedScalar;
using achilles::policy::buffering::BufferingPolicyLike;
using achilles::policy::buffering::BufferParity;
using achilles::policy::buffering::BufferSide;
using achilles::policy::buffering::DoubleBuffering;
using achilles::policy::buffering::GroupBufferedLike;
using achilles::policy::buffering::SingleBuffering;

namespace {

struct Group;
using Reader = DoubleBuffering<Group, BufferSide::kRead>;
using Writer = DoubleBuffering<Group, BufferSide::kWrite>;

// Exactly what BufferingPolicyLike requires. Nothing else.
struct BufferingPolicyArchetype {
  static constexpr std::size_t kBlockCount = 1;
  static std::size_t
  ReadOffset(const BufferParity* /*parity*/, std::size_t /*n*/) {
    return 0;
  }
  static void Replicate(std::byte* /*region*/, std::size_t /*n*/) {}
};
static_assert(BufferingPolicyLike<BufferingPolicyArchetype>);
static_assert(BufferingPolicyLike<SingleBuffering>);
static_assert(BufferingPolicyLike<Reader>);
static_assert(BufferingPolicyLike<Writer>);
static_assert(!BufferingPolicyLike<int>);
static_assert(GroupBufferedLike<Reader>);
static_assert(!GroupBufferedLike<SingleBuffering>);

// One float per block, so a region of DoubleBuffering's kBlockCount floats
// is [half 0 | half 1] and each half's value is directly addressable.
constexpr std::size_t kBlockBytes = sizeof(float);
using Region = std::array<float, Reader::kBlockCount>;

std::byte* Bytes(Region& region) {
  return std::bit_cast<std::byte*>(region.data());
}

}  // namespace

// A single-buffered field reads in place and must never dereference the
// parity -- a View with no double-buffered field is allowed to hold null.
TEST(SingleBuffering, ReadsInPlaceWithoutTouchingTheParity) {
  EXPECT_EQ(SingleBuffering::kBlockCount, 1U);
  EXPECT_EQ(SingleBuffering::ReadOffset(nullptr, kBlockBytes), 0U);
}

// The read side addresses half `parity` and the write side the other one --
// if the two offsets ever coincided, an actuator would scribble on the tick
// in progress.
TEST(DoubleBuffering, ReadAndWriteSidesAreTheTwoHalvesForEitherParity) {
  BufferParity even{0};
  BufferParity odd{1};
  EXPECT_EQ(Reader::ReadOffset(&even, kBlockBytes), 0 * kBlockBytes);
  EXPECT_EQ(Writer::ReadOffset(&even, kBlockBytes), 1 * kBlockBytes);
  EXPECT_EQ(Reader::ReadOffset(&odd, kBlockBytes), 1 * kBlockBytes);
  EXPECT_EQ(Writer::ReadOffset(&odd, kBlockBytes), 0 * kBlockBytes);
}

// Both halves must start as the populated first half, so the first tick
// reads archetype data whichever half it starts on.
TEST(DoubleBuffering, ReplicateCopiesTheFirstHalfIntoTheSecond) {
  Region region = {7.0F, 0.0F};
  Reader::Replicate(Bytes(region), kBlockBytes);
  EXPECT_FLOAT_EQ(region[0], 7.0F);
  EXPECT_FLOAT_EQ(region[1], 7.0F);
}

// Add accumulates into the write half and nowhere else -- not the read half
// the tick in progress is using.
TEST(BufferedScalar, AddAccumulatesIntoTheWriteHalfOnly) {
  Region region = {7.0F, 7.0F};
  BufferParity parity{0};
  BufferedScalar<float> slot(Bytes(region), kBlockBytes, &parity);
  slot.Add(1.5F);
  slot.Add(2.0F);
  EXPECT_FLOAT_EQ(region[0], 7.0F);
  EXPECT_FLOAT_EQ(region[1], 10.5F);
}

// A handle is made once and kept (an actuator lives across many ticks), so
// it must follow the parity each time it writes rather than caching a half.
TEST(BufferedScalar, FollowsAParityFlipWithoutBeingRebuilt) {
  Region region = {0.0F, 0.0F};
  BufferParity parity{0};
  BufferedScalar<float> slot(Bytes(region), kBlockBytes, &parity);
  slot.Add(1.0F);
  parity.value = 1;
  slot.Add(2.0F);
  EXPECT_FLOAT_EQ(region[0], 2.0F);
  EXPECT_FLOAT_EQ(region[1], 1.0F);
}

// Proves ReadOffset's precondition fires: a View over a double-buffered
// field built without its group's parity would otherwise read garbage.
TEST(DoubleBuffering, DiesReadingWithoutAParity) {
  EXPECT_DEATH(
      (void)Reader::ReadOffset(nullptr, kBlockBytes),
      "a View over a double-buffered field"
  );
}
