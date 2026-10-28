#include <gtest/gtest.h>

#include "domain/math/vector3.hpp"
#include "engine/algorithm_contract.hpp"
#include "engine/memory/sim_allocator.hpp"
#include "engine/pass/sim_context.hpp"
#include "engine/topology/ordering_policy.hpp"
#include "support/toy_buffered_field.hpp"
#include "support/toy_field.hpp"
#include "util/tmp.hpp"

// Covers SimContext::Step<StepList> -- the one engine entry point every
// integrator tick goes through (algorithms/integrators.hpp) -- and that a
// buffer swap (SimContext::FlipParity, engine/memory/buffering_policy.hpp)
// reaches every copy of the context. What Step
// promises: every Step in the list runs exactly once per appearance, left to
// right, each handed the same context, config and dt. The other half of its
// contract -- a Step whose FieldEnum isn't hosted fails to compile -- is a
// static_assert, so it has no runtime test here.
//
// The Steps below are instrumented fakes (TESTING.md 5): each records what it
// saw into row 0 of a real ToyView, so the test can observe call order and
// arguments without any real algorithm's math in the way.

using achilles::domain::math::Vector3;
using achilles::engine::StepLike;
using achilles::engine::memory::SimAllocator;
using achilles::engine::pass::SimContext;
using achilles::engine::pass::SimContextLike;
using achilles::engine::pass::TopologySlot;
using achilles::engine::topology::LinearOrdering;
using achilles::test_support::MakeToyBufferedSim;
using achilles::test_support::MakeToySim;
using achilles::test_support::SwapToyBuffers;
using achilles::test_support::ToyAlgorithm;
using achilles::test_support::ToyBufferedAlgorithm;
using achilles::test_support::ToyBufferedField;
using achilles::test_support::ToyCommandRest;
using achilles::test_support::ToyField;
using achilles::test_support::ToyView;
using achilles::util::TypeList;

namespace {

// Whatever a caller passes as Config -- SimContext::Step is generic over it.
struct ToyConfig {
  float value;
};

// kPosition[0] = kPosition[0] * 10 + Digit at row 0 -- so after a list of
// these runs, the decimal digits of kPosition[0] spell out exactly which
// Steps ran, in which order, how many times.
template <int Digit>
struct AppendDigitStep {
  using FieldEnum = ToyField;

  template <SimContextLike SimStateT, typename Config>
  static void Step(const SimStateT& sim_state, const Config&, float) {
    ToyView view = sim_state.template ViewFor<ToyAlgorithm>();
    float digits = view.Load<ToyField::kPosition, float>(0)[0];
    view.Store<ToyField::kPosition, float>(
        0, Vector3<float>(digits * 10.0F + Digit, 0.0F, 0.0F)
    );
  }
};
static_assert(StepLike<AppendDigitStep<1>>);

// Records the dt and config it was called with into kVelocity at row 0.
struct RecordArgumentsStep {
  using FieldEnum = ToyField;

  template <SimContextLike SimStateT>
  static void Step(
      const SimStateT& sim_state, const ToyConfig& config, float dt
  ) {
    ToyView view = sim_state.template ViewFor<ToyAlgorithm>();
    view.Store<ToyField::kVelocity, float>(
        0, Vector3<float>(dt, config.value, 0.0F)
    );
  }
};
static_assert(StepLike<RecordArgumentsStep>);

float Digits(SimAllocator<ToyAlgorithm>& sim) {
  return sim.ViewFor<ToyAlgorithm>().Load<ToyField::kPosition, float>(0)[0];
}

}  // namespace

// The fold must run left to right -- an integrator's whole meaning is its
// order (ABA before the kick that reads its qdd).
TEST(SimContextStep, RunsStepsInListOrder) {
  SimAllocator<ToyAlgorithm> sim = MakeToySim(1);

  sim.SimContext()
      .Step<
          TypeList<AppendDigitStep<1>, AppendDigitStep<2>, AppendDigitStep<3>>>(
          0.0F, ToyConfig{0.0F}
      );

  EXPECT_FLOAT_EQ(Digits(sim), 123.0F);
}

// A Step listed twice must run twice -- implicit midpoint and RK4 list
// aba::ABAStep several times per tick, so de-duplicating the list (the way
// UsedOrderingsFor de-duplicates orderings) would silently drop
// evaluations.
TEST(SimContextStep, RunsARepeatedStepEveryTimeItIsListed) {
  SimAllocator<ToyAlgorithm> sim = MakeToySim(1);

  sim.SimContext()
      .Step<
          TypeList<AppendDigitStep<1>, AppendDigitStep<1>, AppendDigitStep<2>>>(
          0.0F, ToyConfig{0.0F}
      );

  EXPECT_FLOAT_EQ(Digits(sim), 112.0F);
}

// The zero-length list (TESTING.md 9.2): must compile and leave state
// untouched, rather than e.g. falling back to stepping everything hosted.
TEST(SimContextStep, EmptyListLeavesStateUntouched) {
  SimAllocator<ToyAlgorithm> sim = MakeToySim(1);
  sim.ViewFor<ToyAlgorithm>().Store<ToyField::kPosition, float>(
      0, Vector3<float>(7.0F, 0.0F, 0.0F)
  );

  sim.SimContext().Step<TypeList<>>(0.5F, ToyConfig{1.0F});

  EXPECT_FLOAT_EQ(Digits(sim), 7.0F);
}

// Every Step receives the caller's own dt and config, unchanged.
TEST(SimContextStep, ForwardsDtAndConfigToEachStep) {
  SimAllocator<ToyAlgorithm> sim = MakeToySim(1);

  sim.SimContext().Step<TypeList<RecordArgumentsStep>>(0.25F, ToyConfig{3.0F});

  Vector3<float> recorded =
      sim.ViewFor<ToyAlgorithm>().Load<ToyField::kVelocity, float>(0);
  EXPECT_FLOAT_EQ(recorded[0], 0.25F);
  EXPECT_FLOAT_EQ(recorded[1], 3.0F);
}

// The parity lives in the Arena, not in any one View: a context or View
// copied before the swap (every Step takes a fresh ViewFor copy;
// Simulation::ViewFor hands copies out) must read the new half too.
TEST(SimContextBufferSwap, CopiesTakenBeforeTheSwapSeeIt) {
  auto sim = MakeToyBufferedSim(1);
  auto early_context = sim.SimContext();
  auto early_view = early_context.ViewFor<ToyBufferedAlgorithm>();
  early_view.WriteSlot<ToyBufferedField::kCommand>(0, 0).Add(2.0F);

  SwapToyBuffers(sim);
  const float expected = ToyCommandRest(0).X() + 2.0F;
  EXPECT_FLOAT_EQ(
      (early_view.Load<ToyBufferedField::kCommand, float>(0).X()), expected
  );
  EXPECT_FLOAT_EQ(
      (early_context.ViewFor<ToyBufferedAlgorithm>()
           .Load<ToyBufferedField::kCommand, float>(0)
           .X()),
      expected
  );
}
