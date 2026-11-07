#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

#include "domain/error/effects.hpp"
#include "domain/random/counter_rng.hpp"

using achilles::domain::error::ConstantBias;
using achilles::domain::error::CoulombFriction;
using achilles::domain::error::Delay;
using achilles::domain::error::ErrorChain;
using achilles::domain::error::GaussianNoise;
using achilles::domain::error::LossChain;
using achilles::domain::error::Quantize;
using achilles::domain::error::RandomWalk;
using achilles::domain::error::SampleNoise;
using achilles::domain::error::Saturate;
using achilles::domain::error::ScaleError;
using achilles::domain::error::SpeedCurve;
using achilles::domain::error::TransmissionLoss;
using achilles::domain::error::Vec3;
using achilles::domain::error::ViscousFriction;
using achilles::domain::random::CounterRng;

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

// One application of rate effect E to `value`, at `rate`.
template <typename E>
float ApplyAt(const typename E::Params& params, float rate, float value) {
  typename E::State state;
  CounterRng rng(1, 0);
  return E::Apply(params, state, rng, kDt, rate, value);
}

// The standard deviation of `sigma` SampleNoise around 0, sampled every `dt`.
float SampleNoiseDeviation(float sigma, float dt) {
  constexpr int kSamples = 20000;
  SampleNoise<float>::Params params{sigma};
  SampleNoise<float>::State state;
  double sum_squares = 0.0;
  for (int i = 0; i < kSamples; ++i) {
    CounterRng rng(1, static_cast<std::uint64_t>(i));
    const float sample =
        SampleNoise<float>::Apply(params, state, rng, dt, 0.0F);
    sum_squares += static_cast<double>(sample) * sample;
  }
  return static_cast<float>(std::sqrt(sum_squares / kSamples));
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

TEST(ErrorChain, PerSampleNoiseIsOneOfItsStages) {
  ErrorChain<float>::Params params;
  params.sample_noise.sigma = 0.5F;
  ErrorChain<float>::State state;
  EXPECT_NE(ApplyOnce<ErrorChain<float>>(params, state, 1.0F), 1.0F);
}

// ---------------------------------------------------------------------------
// Per-sample noise
// ---------------------------------------------------------------------------

TEST(SampleNoise, ZeroSigmaIsIdentity) {
  SampleNoise<float>::Params params;
  SampleNoise<float>::State state;
  EXPECT_FLOAT_EQ(ApplyOnce<SampleNoise<float>>(params, state, 2.0F), 2.0F);
}

// Unlike GaussianNoise's density, sigma doesn't depend on the sample rate.
TEST(SampleNoise, SigmaIsTheDeviationWhateverTheRate) {
  EXPECT_NEAR(SampleNoiseDeviation(0.5F, 0.001F), 0.5F, 0.02F);
  EXPECT_NEAR(SampleNoiseDeviation(0.5F, 0.1F), 0.5F, 0.02F);
}

TEST(SampleNoise, ActsOnEachComponent) {
  SampleNoise<Vec3>::Params params{0.5F};
  SampleNoise<Vec3>::State state;
  const Vec3 out = ApplyOnce<SampleNoise<Vec3>>(params, state, Vec3());
  EXPECT_NE(out.X(), out.Y());
  EXPECT_NE(out.Y(), out.Z());
}

// ---------------------------------------------------------------------------
// The effects that depend on the rate
// ---------------------------------------------------------------------------

TEST(SpeedCurve, ZeroStallIsNoCurve) {
  EXPECT_FLOAT_EQ(ApplyAt<SpeedCurve>({}, 100.0F, 50.0F), 50.0F);
}

TEST(SpeedCurve, AtRestTheLimitIsTheStallEitherWay) {
  const SpeedCurve::Params params{.stall = 10.0F, .free_rate = 5.0F};
  EXPECT_FLOAT_EQ(ApplyAt<SpeedCurve>(params, 0.0F, 20.0F), 10.0F);
  EXPECT_FLOAT_EQ(ApplyAt<SpeedCurve>(params, 0.0F, -20.0F), -10.0F);
  EXPECT_FLOAT_EQ(ApplyAt<SpeedCurve>(params, 0.0F, 3.0F), 3.0F);
}

// Less is available with the motion, and more against it.
TEST(SpeedCurve, TheLimitsShiftWithTheRate) {
  const SpeedCurve::Params params{.stall = 10.0F, .free_rate = 5.0F};
  EXPECT_FLOAT_EQ(ApplyAt<SpeedCurve>(params, 2.5F, 20.0F), 5.0F);
  EXPECT_FLOAT_EQ(ApplyAt<SpeedCurve>(params, 2.5F, -20.0F), -15.0F);
}

TEST(SpeedCurve, NoEffortIsLeftAtTheFreeRate) {
  const SpeedCurve::Params params{.stall = 10.0F, .free_rate = 5.0F};
  EXPECT_FLOAT_EQ(ApplyAt<SpeedCurve>(params, 5.0F, 20.0F), 0.0F);
}

// Past the free rate even a forward command is pushed backward.
TEST(SpeedCurve, PastTheFreeRateTheEffortReverses) {
  const SpeedCurve::Params params{.stall = 10.0F, .free_rate = 5.0F};
  EXPECT_FLOAT_EQ(ApplyAt<SpeedCurve>(params, 10.0F, 3.0F), -10.0F);
}

TEST(ViscousFriction, OpposesTheRateInProportion) {
  const ViscousFriction::Params params{2.0F};
  EXPECT_FLOAT_EQ(ApplyAt<ViscousFriction>(params, 3.0F, 10.0F), 4.0F);
  EXPECT_FLOAT_EQ(ApplyAt<ViscousFriction>(params, -3.0F, 10.0F), 16.0F);
}

TEST(CoulombFriction, ZeroLevelIsIdentity) {
  EXPECT_FLOAT_EQ(ApplyAt<CoulombFriction>({}, 3.0F, 1.0F), 1.0F);
}

// Smoothed through zero: no drag at rest, and its full size once moving.
TEST(CoulombFriction, RampsUpFromNothingAtRest) {
  const CoulombFriction::Params params{.level = 2.0F, .smoothing = 1.0F};
  EXPECT_FLOAT_EQ(ApplyAt<CoulombFriction>(params, 0.0F, 5.0F), 5.0F);
  EXPECT_NEAR(ApplyAt<CoulombFriction>(params, 100.0F, 5.0F), 3.0F, 1e-5F);
  EXPECT_NEAR(ApplyAt<CoulombFriction>(params, -100.0F, 5.0F), 7.0F, 1e-5F);
}

TEST(TransmissionLoss, LosesEffortOnlyWhenDrivingTheLoad) {
  const TransmissionLoss::Params params{0.9F};
  EXPECT_FLOAT_EQ(ApplyAt<TransmissionLoss>(params, 1.0F, 10.0F), 9.0F);
  EXPECT_FLOAT_EQ(ApplyAt<TransmissionLoss>(params, -1.0F, -10.0F), -9.0F);
  EXPECT_FLOAT_EQ(ApplyAt<TransmissionLoss>(params, -1.0F, 10.0F), 10.0F);
}

TEST(LossChain, DefaultParamsLoseNothing) {
  EXPECT_FLOAT_EQ(ApplyAt<LossChain>({}, 3.0F, 10.0F), 10.0F);
}

// Friction comes off first, and the transmission loses a share of the rest.
TEST(LossChain, FrictionComesOffBeforeTheTransmission) {
  LossChain::Params params;
  params.viscous.coefficient = 1.0F;
  params.transmission.efficiency = 0.5F;
  EXPECT_FLOAT_EQ(ApplyAt<LossChain>(params, 2.0F, 10.0F), 4.0F);
}
