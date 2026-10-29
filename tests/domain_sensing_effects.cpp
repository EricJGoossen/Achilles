#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

#include "domain/random/counter_rng.hpp"
#include "domain/sensing/effects.hpp"

using achilles::domain::random::CounterRng;
using achilles::domain::sensing::ConstantBias;
using achilles::domain::sensing::Delay;
using achilles::domain::sensing::ErrorChain;
using achilles::domain::sensing::GaussianNoise;
using achilles::domain::sensing::Quantize;
using achilles::domain::sensing::RandomWalk;
using achilles::domain::sensing::Saturate;
using achilles::domain::sensing::ScaleError;
using achilles::domain::sensing::Vec3;

namespace {

constexpr float kDt = 0.01F;

// One application of effect E to `value`, at tick `tick`.
template <typename E, typename V>
V ApplyOnce(
    const typename E::Params& params,
    typename E::State& state,
    const V& value,
    std::uint64_t tick = 0
) {
  CounterRng rng(1, tick);
  return E::Apply(params, state, rng, kDt, value);
}

}  // namespace

// ---------------------------------------------------------------------------
// Each stage on its own
// ---------------------------------------------------------------------------

TEST(ScaleError, MultipliesEachAxisByOnePlusItsError) {
  ScaleError<Vec3>::Params params{Vec3(0.1F, -0.2F, 0.0F)};
  ScaleError<Vec3>::State state;
  const Vec3 out =
      ApplyOnce<ScaleError<Vec3>>(params, state, Vec3(2.0F, 2.0F, 2.0F));
  EXPECT_FLOAT_EQ(out.X(), 2.2F);
  EXPECT_FLOAT_EQ(out.Y(), 1.6F);
  EXPECT_FLOAT_EQ(out.Z(), 2.0F);
}

TEST(ConstantBias, AddsTheBias) {
  ConstantBias<float>::Params params{0.25F};
  ConstantBias<float>::State state;
  EXPECT_FLOAT_EQ(ApplyOnce<ConstantBias<float>>(params, state, 1.0F), 1.25F);
}

TEST(RandomWalk, StartsAtItsInitialBiasAndThenWanders) {
  RandomWalk<float>::Params still{0.0F, 0.5F};
  RandomWalk<float>::State still_state;
  EXPECT_FLOAT_EQ(ApplyOnce<RandomWalk<float>>(still, still_state, 1.0F), 1.5F);

  RandomWalk<float>::Params walking{1.0F, 0.0F};
  RandomWalk<float>::State state;
  float last = 0.0F;
  bool moved = false;
  for (std::uint64_t tick = 0; tick < 10; ++tick) {
    const float out = ApplyOnce<RandomWalk<float>>(walking, state, 0.0F, tick);
    moved = moved || out != last;
    last = out;
  }
  EXPECT_TRUE(moved);
  EXPECT_FLOAT_EQ(last, state.bias);
}

TEST(GaussianNoise, ZeroDensityIsIdentity) {
  GaussianNoise<float>::Params params{0.0F};
  GaussianNoise<float>::State state;
  EXPECT_FLOAT_EQ(ApplyOnce<GaussianNoise<float>>(params, state, 3.0F), 3.0F);
}

TEST(GaussianNoise, PerSampleSigmaIsDensityOverRootDt) {
  constexpr int kSamples = 20000;
  constexpr float kDensity = 0.02F;
  GaussianNoise<float>::Params params{kDensity};
  GaussianNoise<float>::State state;
  double sum_squares = 0.0;
  for (int i = 0; i < kSamples; ++i) {
    const float out = ApplyOnce<GaussianNoise<float>>(
        params, state, 0.0F, static_cast<std::uint64_t>(i)
    );
    sum_squares += static_cast<double>(out) * out;
  }
  const double sigma = std::sqrt(sum_squares / kSamples);
  EXPECT_NEAR(sigma / (kDensity / std::sqrt(kDt)), 1.0, 0.03);
}

TEST(Saturate, ClampsEachComponentToTheLimitAndZeroIsUnlimited) {
  Saturate<Vec3>::Params params{1.0F};
  Saturate<Vec3>::State state;
  const Vec3 out =
      ApplyOnce<Saturate<Vec3>>(params, state, Vec3(2.0F, -3.0F, 0.5F));
  EXPECT_FLOAT_EQ(out.X(), 1.0F);
  EXPECT_FLOAT_EQ(out.Y(), -1.0F);
  EXPECT_FLOAT_EQ(out.Z(), 0.5F);

  Saturate<float>::Params unlimited{0.0F};
  Saturate<float>::State unlimited_state;
  EXPECT_FLOAT_EQ(
      ApplyOnce<Saturate<float>>(unlimited, unlimited_state, 100.0F), 100.0F
  );
}

TEST(Quantize, RoundsToTheNearestStep) {
  Quantize<float>::Params params{0.25F};
  Quantize<float>::State state;
  EXPECT_FLOAT_EQ(ApplyOnce<Quantize<float>>(params, state, 1.1F), 1.0F);
  EXPECT_FLOAT_EQ(ApplyOnce<Quantize<float>>(params, state, 1.2F), 1.25F);
}

TEST(Delay, ReportsTheValueFromThatManySamplesAgo) {
  Delay<float>::Params params{2};
  Delay<float>::State state;
  // Until two samples have been taken, the oldest one it has.
  EXPECT_FLOAT_EQ(ApplyOnce<Delay<float>>(params, state, 1.0F), 1.0F);
  EXPECT_FLOAT_EQ(ApplyOnce<Delay<float>>(params, state, 2.0F), 1.0F);
  EXPECT_FLOAT_EQ(ApplyOnce<Delay<float>>(params, state, 3.0F), 1.0F);
  EXPECT_FLOAT_EQ(ApplyOnce<Delay<float>>(params, state, 4.0F), 2.0F);
  EXPECT_FLOAT_EQ(ApplyOnce<Delay<float>>(params, state, 5.0F), 3.0F);
}

TEST(Delay, ZeroSamplesIsIdentity) {
  Delay<float>::Params params{0};
  Delay<float>::State state;
  EXPECT_FLOAT_EQ(ApplyOnce<Delay<float>>(params, state, 1.0F), 1.0F);
  EXPECT_FLOAT_EQ(ApplyOnce<Delay<float>>(params, state, 2.0F), 2.0F);
}

// ---------------------------------------------------------------------------
// ErrorChain
// ---------------------------------------------------------------------------

TEST(ErrorChain, DefaultParamsAreAnIdealSensor) {
  ErrorChain<Vec3>::Params params;
  ErrorChain<Vec3>::State state;
  const Vec3 out =
      ApplyOnce<ErrorChain<Vec3>>(params, state, Vec3(1.0F, -2.0F, 3.0F));
  EXPECT_FLOAT_EQ(out.X(), 1.0F);
  EXPECT_FLOAT_EQ(out.Y(), -2.0F);
  EXPECT_FLOAT_EQ(out.Z(), 3.0F);
}

// Gain, then bias, then range, then resolution: (2 * 1.5 + 1) clamped to 3,
// then rounded to a step of 2 -- 4 if the range came after resolution.
TEST(ErrorChain, AppliesItsStagesInTheDatasheetOrder) {
  ErrorChain<float>::Params params;
  params.scale.error = 0.5F;
  params.bias.bias = 1.0F;
  params.range.limit = 3.0F;
  params.resolution.resolution = 2.0F;
  ErrorChain<float>::State state;
  EXPECT_FLOAT_EQ(ApplyOnce<ErrorChain<float>>(params, state, 2.0F), 4.0F);

  params.range.limit = 2.5F;
  ErrorChain<float>::State clamped;
  EXPECT_FLOAT_EQ(ApplyOnce<ErrorChain<float>>(params, clamped, 2.0F), 2.0F);
}

TEST(ErrorChain, TheSameTickGivesTheSameNoise) {
  ErrorChain<float>::Params params;
  params.noise.density = 0.1F;
  ErrorChain<float>::State a;
  ErrorChain<float>::State b;
  EXPECT_FLOAT_EQ(
      ApplyOnce<ErrorChain<float>>(params, a, 0.0F, 9),
      ApplyOnce<ErrorChain<float>>(params, b, 0.0F, 9)
  );
}
