#pragma once

#include <cmath>
#include <cstdint>
#include <numbers>

namespace achilles::domain::random {

// A counter-based random generator: every draw is a pure function of
// (stream, counter, draw index), with no state carried from one tick to the
// next. Two generators built from the same (stream, counter) produce the
// same draws, whatever ran before them -- so a sensor instance's noise is
// reproducible no matter which thread runs it, or in what order instances
// run. stream picks the instance (its index, mixed with the sim's seed),
// counter the moment (its tick).
//
// The mixing function is SplitMix64's finalizer -- not cryptographic, but
// well distributed bit-for-bit, which is all noise models need.
class CounterRng {
 public:
  CounterRng(std::uint64_t stream, std::uint64_t counter)
      : key_(Mix(stream ^ Mix(counter + kGolden))) {}

  // The next 64 random bits.
  std::uint64_t Bits() {
    ++draw_;
    return Mix(key_ + (kGolden * draw_));
  }

  // Uniform in [0, 1).
  float Uniform() {
    return static_cast<float>(Bits() >> kFloatShift) * kFloatUnit;
  }

  // Standard normal (mean 0, variance 1), by Box-Muller -- one of the pair,
  // so each call costs two uniforms.
  float Normal() {
    // (0, 1], so the log is finite.
    float u1 = static_cast<float>((Bits() >> kFloatShift) + 1) * kFloatUnit;
    float u2 = Uniform();
    return std::sqrt(-2.0F * std::log(u1)) *
           std::cos(2.0F * std::numbers::pi_v<float> * u2);
  }

 private:
  static constexpr std::uint64_t kGolden = 0x9E3779B97F4A7C15ULL;
  // A float's 24 bits of mantissa: the top 24 bits of a draw, scaled by
  // 2^-24.
  static constexpr int kFloatShift = 40;
  static constexpr float kFloatUnit = 1.0F / 16777216.0F;

  static constexpr std::uint64_t Mix(std::uint64_t x) {
    x = (x ^ (x >> 30U)) * 0xBF58476D1CE4E5B9ULL;
    x = (x ^ (x >> 27U)) * 0x94D049BB133111EBULL;
    return x ^ (x >> 31U);
  }

  std::uint64_t key_;
  std::uint64_t draw_ = 0;
};

}  // namespace achilles::domain::random
