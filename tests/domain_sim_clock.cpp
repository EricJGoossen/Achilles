// SimClock: the sim's one clock.
#include <gtest/gtest.h>

#include <cstdint>

#include "domain/sim_clock.hpp"

using achilles::domain::Advance;
using achilles::domain::SimClock;

TEST(SimClock, StartsAtZero) {
  const SimClock clock;
  EXPECT_EQ(clock.time, 0.0);
  EXPECT_EQ(clock.tick, 0U);
}

TEST(SimClock, AdvancesByEachDtAndCountsTicks) {
  SimClock clock;
  Advance(&clock, 0.5F);
  Advance(&clock, 0.25F);
  EXPECT_DOUBLE_EQ(clock.time, 0.75);
  EXPECT_EQ(clock.tick, 2U);
}

// A float would stop gaining at this size; a long run at a fine dt can't lose
// ticks to rounding.
TEST(SimClock, DoesNotLoseFineTicksOnALongRun) {
  SimClock clock;
  clock.time = 100000.0;
  const double start = clock.time;
  constexpr std::uint64_t kTicks = 1000;
  for (std::uint64_t i = 0; i < kTicks; ++i) {
    Advance(&clock, 0.0009765625F);  // 2^-10 s: exact in both float and double
  }
  EXPECT_DOUBLE_EQ(clock.time - start, 0.9765625);
}
