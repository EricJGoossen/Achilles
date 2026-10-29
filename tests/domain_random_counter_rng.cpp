#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

#include "domain/random/counter_rng.hpp"

using achilles::domain::random::CounterRng;

namespace {

constexpr int kSamples = 20000;

}  // namespace

TEST(CounterRng, SameStreamAndCounterGiveTheSameDraws) {
  CounterRng a(7, 42);
  CounterRng b(7, 42);
  for (int i = 0; i < 16; ++i) {
    EXPECT_EQ(a.Bits(), b.Bits());
  }
}

TEST(CounterRng, ADifferentStreamOrCounterGivesDifferentDraws) {
  CounterRng base(7, 42);
  CounterRng other_stream(8, 42);
  CounterRng other_counter(7, 43);
  const std::uint64_t first = base.Bits();
  EXPECT_NE(first, other_stream.Bits());
  EXPECT_NE(first, other_counter.Bits());
}

TEST(CounterRng, SuccessiveDrawsDiffer) {
  CounterRng rng(1, 1);
  const std::uint64_t first = rng.Bits();
  EXPECT_NE(first, rng.Bits());
}

TEST(CounterRng, UniformStaysInTheUnitIntervalWithMeanOneHalf) {
  double sum = 0.0;
  for (int i = 0; i < kSamples; ++i) {
    CounterRng rng(3, static_cast<std::uint64_t>(i));
    const float u = rng.Uniform();
    ASSERT_GE(u, 0.0F);
    ASSERT_LT(u, 1.0F);
    sum += u;
  }
  EXPECT_NEAR(sum / kSamples, 0.5, 0.01);
}

TEST(CounterRng, NormalHasZeroMeanAndUnitVariance) {
  double sum = 0.0;
  double sum_squares = 0.0;
  for (int i = 0; i < kSamples; ++i) {
    CounterRng rng(5, static_cast<std::uint64_t>(i));
    const double x = rng.Normal();
    ASSERT_TRUE(std::isfinite(x));
    sum += x;
    sum_squares += x * x;
  }
  const double mean = sum / kSamples;
  EXPECT_NEAR(mean, 0.0, 0.03);
  EXPECT_NEAR((sum_squares / kSamples) - (mean * mean), 1.0, 0.04);
}
