#include <gtest/gtest.h>

// Emulated batches give this file the 4-, 8- and 32-lane widths TESTING.md
// 9.4 asks for regardless of the build's native SIMD target; they have to
// be switched on before xsimd is first included.
#define XSIMD_WITH_EMULATED 1  // NOLINT(cppcoreguidelines-macro-usage)

#include <array>
#include <cstddef>
#include <xsimd/xsimd.hpp>

#include "util/simd_ops.hpp"

// Covers util::SimdAccumulate: the lane reduction every BroadcastIndex
// output (topology/indexing_policy.hpp) bottoms out in, one leaf at a time
// (Assembler::Accumulate, engine/assembler.hpp).

using achilles::util::SimdAccumulate;

namespace {

template <std::size_t Bits>
using EmulatedBatch = xsimd::batch<float, xsimd::emulated<Bits>>;

// 1, 2, ..., N in lanes 0 .. N-1, so the expected sum N (N + 1) / 2 can only
// come out if every lane was folded in exactly once.
template <typename Batch>
Batch Ascending() {
  std::array<float, Batch::size> lanes{};
  for (std::size_t i = 0; i < Batch::size; ++i) {
    lanes[i] = static_cast<float>(i + 1);
  }
  return Batch::load_unaligned(lanes.data());
}

template <typename Batch>
float AscendingSum() {
  constexpr auto kN = static_cast<float>(Batch::size);
  return kN * (kN + 1.0F) * 0.5F;
}

}  // namespace

// A scalar value is its own one-lane sum: ptr[0] += val, nothing more.
TEST(SimdAccumulate, ScalarAddsOntoTheStoredValue) {
  float stored = 10.0F;
  SimdAccumulate(&stored, 2.5F);
  EXPECT_FLOAT_EQ(stored, 12.5F);
}

// The stored value survives: += onto it, never = (TESTING.md 9.2) -- a
// reduction landing on a row an earlier pass already reduced into must add
// on top of it.
TEST(SimdAccumulate, BatchAddsItsLaneSumOntoTheStoredValue) {
  using Batch = xsimd::batch<float>;
  float stored = 100.0F;
  SimdAccumulate(&stored, Ascending<Batch>());
  EXPECT_FLOAT_EQ(stored, 100.0F + AscendingSum<Batch>());
}

// Every lane counts once, at each width -- a reduction that dropped or
// doubled a lane past the native width would only show up here.
TEST(SimdAccumulate, FoldsEveryLaneAtFourLanes) {
  using Batch = EmulatedBatch<128>;
  static_assert(Batch::size == 4);
  float stored = 0.0F;
  SimdAccumulate(&stored, Ascending<Batch>());
  EXPECT_FLOAT_EQ(stored, AscendingSum<Batch>());
}

TEST(SimdAccumulate, FoldsEveryLaneAtEightLanes) {
  using Batch = EmulatedBatch<256>;
  static_assert(Batch::size == 8);
  float stored = 0.0F;
  SimdAccumulate(&stored, Ascending<Batch>());
  EXPECT_FLOAT_EQ(stored, AscendingSum<Batch>());
}

TEST(SimdAccumulate, FoldsEveryLaneAtThirtyTwoLanes) {
  using Batch = EmulatedBatch<1024>;
  static_assert(Batch::size == 32);
  float stored = 0.0F;
  SimdAccumulate(&stored, Ascending<Batch>());
  EXPECT_FLOAT_EQ(stored, AscendingSum<Batch>());
}

// Only the one element at ptr is touched -- a reduction into one row must
// never spill into the next.
TEST(SimdAccumulate, TouchesOnlyTheFirstElement) {
  std::array<float, 3> stored = {1.0F, 2.0F, 3.0F};
  SimdAccumulate(stored.data(), Ascending<EmulatedBatch<256>>());
  EXPECT_FLOAT_EQ(stored[0], 1.0F + AscendingSum<EmulatedBatch<256>>());
  EXPECT_FLOAT_EQ(stored[1], 2.0F);
  EXPECT_FLOAT_EQ(stored[2], 3.0F);
}
