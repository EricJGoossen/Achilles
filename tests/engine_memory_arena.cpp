#include <gtest/gtest.h>

#include <bit>
#include <cstddef>
#include <cstdint>

#include "engine/memory/arena.hpp"

using achilles::engine::memory::Arena;

TEST(ArenaConstruction, AccessorsMatchArguments) {
  Arena arena(128, 16);

  EXPECT_EQ(arena.Used(), 0U);
  EXPECT_EQ(arena.Capacity(), 128U);
  EXPECT_EQ(arena.BaseAlignment(), 16U);
}

TEST(ArenaAllocate, BumpsCursorByExactlyTheRequestedBytes) {
  Arena arena(128, 16);

  arena.Allocate(16, 16);
  EXPECT_EQ(arena.Used(), 16U);
  arena.Allocate(8, 8);
  EXPECT_EQ(arena.Used(), 24U);
}

// A second allocation whose alignment doesn't already divide the current
// cursor must round the cursor up first -- the returned pointer, not just
// Used(), is what a caller actually dereferences, so this checks the
// pointer's own alignment, not just the bookkeeping.
TEST(ArenaAllocate, RoundsCursorUpToRequestedAlignment) {
  Arena arena(128, 32);

  std::byte* first = arena.Allocate(3, 1);  // cursor now at 3, unaligned
  std::byte* second = arena.Allocate(8, 8);  // must round 3 up to 8

  EXPECT_EQ(second - first, 8);
  EXPECT_EQ(std::bit_cast<std::uintptr_t>(second) % 8, 0U);
}

TEST(ArenaAllocate, SequentialAllocationsNeverOverlap) {
  Arena arena(64, 16);

  std::byte* first = arena.Allocate(10, 1);
  std::byte* second = arena.Allocate(10, 1);

  EXPECT_GE(second, first + 10);
}

TEST(ArenaZeroFill, WritesZeroToEveryByteOfTheGivenBlock) {
  Arena arena(16, 8);
  std::byte* block = arena.Allocate(16, 8);
  for (std::size_t i = 0; i < 16; ++i) {
    block[i] = std::byte{0xFF};
  }

  arena.ZeroFill(block, 16);

  for (std::size_t i = 0; i < 16; ++i) {
    EXPECT_EQ(block[i], std::byte{0});
  }
}

TEST(ArenaAllocatePrecondition, DiesWhenAlignmentExceedsBaseAlignment) {
  Arena arena(128, 8);

  EXPECT_DEATH(arena.Allocate(16, 16), "not constructed with enough base alignment");
}

TEST(ArenaAllocatePrecondition, DiesOnNonPowerOfTwoAlignment) {
  Arena arena(128, 16);

  // 3 <= base_alignment (16), so this isolates the power-of-two check from
  // the base-alignment check above rather than tripping both.
  EXPECT_DEATH(arena.Allocate(8, 3), "power of two");
}

TEST(ArenaAllocatePrecondition, DiesOnZeroBytes) {
  Arena arena(128, 16);

  EXPECT_DEATH(arena.Allocate(0, 8), "bytes must be > 0");
}

TEST(ArenaAllocatePrecondition, DiesOnOverflow) {
  Arena arena(8, 8);

  EXPECT_DEATH(arena.Allocate(16, 8), "Arena overflow");
}
