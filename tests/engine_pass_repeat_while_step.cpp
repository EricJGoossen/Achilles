#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <type_traits>

#include "domain/math/scalar.hpp"
#include "engine/algorithm_contract.hpp"
#include "engine/memory/sim_allocator.hpp"
#include "engine/op_contract.hpp"
#include "engine/pass/algorithm_step.hpp"
#include "engine/pass/repeat_while_step.hpp"
#include "engine/pass/traversals.hpp"
#include "engine/pass/view_step.hpp"
#include "support/toy_reduce_field.hpp"
#include "util/tmp.hpp"

// Covers engine::pass::RepeatWhileStep: it runs its body while the flag is
// nonzero, checks the flag before every run (so a flag already clear runs
// nothing), and never runs more than its cap. The body counts the flag
// down by one per run, so the flag's final value says how many runs
// happened.

using achilles::domain::math::Scalar;
using achilles::engine::ArgData;
using achilles::engine::ArgIndex;
using achilles::engine::OpArgsMatchView;
using achilles::engine::OpLike;
using achilles::engine::StepLike;
using achilles::engine::memory::SimAllocator;
using achilles::engine::pass::GlobalTraversal;
using achilles::engine::pass::Ops;
using achilles::engine::pass::Pass;
using achilles::engine::pass::RepeatWhileStep;
using achilles::engine::pass::ViewStep;
using achilles::test_support::MakeToyReduceSim;
using achilles::test_support::ToyReduceAlgorithm;
using achilles::test_support::ToyReduceField;
using achilles::test_support::ToyReduceView;
using achilles::util::TypeList;

namespace {

// kTotal -= 1.
struct CountDownOp {
  using FieldEnum = ToyReduceField;

  static constexpr std::array<ArgData<FieldEnum>, 0> kInputs = {};
  static constexpr std::array<ArgData<FieldEnum>, 1> kOutputs = {
      ArgData<FieldEnum>{FieldEnum::kTotal, ArgIndex::kTarget},
  };

  void operator()(Scalar<float>* total_out) const {
    *total_out = Scalar<float>(total_out->Value() - 1.0F);
  }
};
static_assert(OpLike<CountDownOp>);
static_assert(OpArgsMatchView<CountDownOp, ToyReduceView>);

// One CountDownOp over the whole sim.
struct CountDownStep : ViewStep<CountDownStep, ToyReduceAlgorithm> {
  static auto MakeOps(int /*config*/, float /*dt*/) {
    return Ops<Pass<CountDownOp, GlobalTraversal>>{CountDownOp{}};
  }
};

template <std::size_t Max>
using CountDown = RepeatWhileStep<
    TypeList<CountDownStep>,
    ToyReduceAlgorithm,
    ToyReduceField::kTotal,
    Max>;
static_assert(StepLike<CountDown<1>>);
static_assert(std::is_same_v<CountDown<1>::FieldEnum, ToyReduceField>);

// The flag's value after running CountDown<Max> from `start`.
template <std::size_t Max>
float RunFrom(float start) {
  SimAllocator<ToyReduceAlgorithm> sim = MakeToyReduceSim(1);
  ToyReduceView view = sim.ViewFor<ToyReduceAlgorithm>();
  view.Store<ToyReduceField::kTotal, float>(0, Scalar<float>(start));
  CountDown<Max>::Step(sim.SimContext(), 0, 0.0F);
  return view.Load<ToyReduceField::kTotal, float>(0).Value();
}

}  // namespace

// Runs until the body clears the flag, well under the cap.
TEST(RepeatWhileStep, StopsWhenTheFlagClears) {
  EXPECT_FLOAT_EQ(RunFrom<10>(3.0F), 0.0F);
}

// A flag already clear runs nothing: the check comes before each run.
TEST(RepeatWhileStep, AClearFlagRunsNothing) {
  EXPECT_FLOAT_EQ(RunFrom<10>(0.0F), 0.0F);
}

// A flag that never clears in time stops at the cap.
TEST(RepeatWhileStep, NeverRunsMoreThanTheCap) {
  EXPECT_FLOAT_EQ(RunFrom<2>(5.0F), 3.0F);
}
