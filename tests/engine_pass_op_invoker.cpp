#include <gtest/gtest.h>

#include <array>

#include "domain/math/vector3.hpp"
#include "engine/memory/sim_allocator.hpp"
#include "engine/op_contract.hpp"
#include "engine/pass/op_invoker.hpp"
#include "support/toy_field.hpp"

using achilles::domain::math::Vector3;
using achilles::engine::ArgData;
using achilles::engine::OpArgsMatchView;
using achilles::engine::OpLike;
using achilles::engine::memory::SimAllocator;
using achilles::engine::pass::OpInvoker;
using achilles::engine::pass::SingleOpInvoker;
using achilles::test_support::MakeToySim;
using achilles::test_support::ToyAlgorithm;
using achilles::test_support::ToyField;
using achilles::test_support::ToyView;

namespace {

// Reads kPosition at the target index and kVelocity at the parent index
// (use_target mixed true/false, per ArgData), writes the elementwise sum
// to kVelocity at the target index -- exercises the (target_index,
// parent_index) split OpInvoker::operator() implements.
struct CombineOp {
  using FieldEnum = ToyField;

  static constexpr std::array<ArgData<FieldEnum>, 2> kInputs = {
      ArgData<FieldEnum>{FieldEnum::kPosition, true},
      ArgData<FieldEnum>{FieldEnum::kVelocity, false},
  };
  static constexpr std::array<ArgData<FieldEnum>, 1> kOutputs = {
      ArgData<FieldEnum>{FieldEnum::kVelocity, true},
  };

  void operator()(
      const Vector3<float>& target_position,
      const Vector3<float>& parent_velocity,
      Vector3<float>* velocity_out
  ) const {
    *velocity_out = target_position + parent_velocity;
  }
};
static_assert(OpLike<CombineOp>);
static_assert(OpArgsMatchView<CombineOp, ToyView>);

// Empty kInputs, one output -- the shape RunSeed/SingleOpInvoker exist
// for (see algorithm_step.hpp: "a SeedBaseOp carrying real gravity/base-
// velocity state").
struct SeedPositionOp {
  using FieldEnum = ToyField;

  static constexpr std::array<ArgData<FieldEnum>, 0> kInputs = {};
  static constexpr std::array<ArgData<FieldEnum>, 1> kOutputs = {
      ArgData<FieldEnum>{FieldEnum::kPosition, true},
  };

  Vector3<float> value = Vector3<float>(9.0F, 9.0F, 9.0F);

  void operator()(Vector3<float>* position_out) const { *position_out = value; }
};
static_assert(OpLike<SeedPositionOp>);
static_assert(OpArgsMatchView<SeedPositionOp, ToyView>);

// Writes kVelocity at the parent index via `+=` -- the shape
// PropagateInertiaOp (algorithms/aba/aba_ops.hpp) uses for real to fold
// each child's contribution into a shared parent row. Invoke must load
// the output's current value from the view before calling operator(),
// not hand it a fresh zero, or a second call targeting the same parent
// clobbers the first instead of accumulating onto it.
struct AccumulateIntoParentOp {
  using FieldEnum = ToyField;

  static constexpr std::array<ArgData<FieldEnum>, 1> kInputs = {
      ArgData<FieldEnum>{FieldEnum::kPosition, true},
  };
  static constexpr std::array<ArgData<FieldEnum>, 1> kOutputs = {
      ArgData<FieldEnum>{FieldEnum::kVelocity, false},
  };

  void operator()(
      const Vector3<float>& contribution, Vector3<float>* velocity_parent_out
  ) const {
    *velocity_parent_out += contribution;
  }
};
static_assert(OpLike<AccumulateIntoParentOp>);
static_assert(OpArgsMatchView<AccumulateIntoParentOp, ToyView>);

// Same `+=`-onto-the-view shape as AccumulateIntoParentOp, but for
// SingleOpInvoker's single-index Invoke overload (use_target=true, no
// parent_index at all) -- that overload has its own separate output-tuple
// construction in OpInvokerBase::Invoke, so it needed the same
// load-before-store fix and deserves its own regression test.
struct AccumulateAtTargetOp {
  using FieldEnum = ToyField;

  static constexpr std::array<ArgData<FieldEnum>, 1> kInputs = {
      ArgData<FieldEnum>{FieldEnum::kPosition, true},
  };
  static constexpr std::array<ArgData<FieldEnum>, 1> kOutputs = {
      ArgData<FieldEnum>{FieldEnum::kVelocity, true},
  };

  void operator()(
      const Vector3<float>& contribution, Vector3<float>* velocity_out
  ) const {
    *velocity_out += contribution;
  }
};
static_assert(OpLike<AccumulateAtTargetOp>);
static_assert(OpArgsMatchView<AccumulateAtTargetOp, ToyView>);

}  // namespace

TEST(OpInvoker, ReadsTargetAndParentWritesTarget) {
  SimAllocator<ToyAlgorithm> sim = MakeToySim(4);
  ToyView view = sim.ViewFor<ToyAlgorithm>();

  view.Store<ToyField::kPosition, float>(2, Vector3<float>(1.0F, 2.0F, 3.0F));
  view.Store<ToyField::kVelocity, float>(
      0, Vector3<float>(10.0F, 20.0F, 30.0F)
  );

  CombineOp op;
  OpInvoker<CombineOp, ToyView> invoker(view, op);
  invoker(/*target_index=*/2, /*parent_index=*/0);

  EXPECT_TRUE((view.Load<ToyField::kVelocity, float>(2).IsApprox(
      Vector3<float>(11.0F, 22.0F, 33.0F)
  )));
  // Untouched: kVelocity at the parent index itself must be unchanged.
  EXPECT_TRUE((view.Load<ToyField::kVelocity, float>(0).IsApprox(
      Vector3<float>(10.0F, 20.0F, 30.0F)
  )));
}

TEST(OpInvoker, TargetAndParentSameIndexBothReadsHitTheSameRow) {
  SimAllocator<ToyAlgorithm> sim = MakeToySim(4);
  ToyView view = sim.ViewFor<ToyAlgorithm>();

  view.Store<ToyField::kPosition, float>(1, Vector3<float>(1.0F, 0.0F, 0.0F));
  view.Store<ToyField::kVelocity, float>(1, Vector3<float>(0.0F, 1.0F, 0.0F));

  CombineOp op;
  OpInvoker<CombineOp, ToyView> invoker(view, op);
  invoker(1, 1);

  EXPECT_TRUE((view.Load<ToyField::kVelocity, float>(1).IsApprox(
      Vector3<float>(1.0F, 1.0F, 0.0F)
  )));
}

// Regression test for the bug where Invoke default-constructed (zeroed)
// the output locals on every call instead of loading the view's current
// value: two different targets (1 and 2) both write into parent index 0
// via `+=`, and the parent's own pre-existing value must survive both
// calls, with each contribution summed on top of it rather than the
// second call's fresh zero overwriting the first call's result.
TEST(OpInvoker, AccumulatesWhenMultipleCallsTargetTheSameParent) {
  SimAllocator<ToyAlgorithm> sim = MakeToySim(4);
  ToyView view = sim.ViewFor<ToyAlgorithm>();

  view.Store<ToyField::kVelocity, float>(0, Vector3<float>(100.0F, 0.0F, 0.0F));
  view.Store<ToyField::kPosition, float>(1, Vector3<float>(1.0F, 0.0F, 0.0F));
  view.Store<ToyField::kPosition, float>(2, Vector3<float>(10.0F, 0.0F, 0.0F));

  AccumulateIntoParentOp op;
  OpInvoker<AccumulateIntoParentOp, ToyView> invoker(view, op);
  invoker(/*target_index=*/1, /*parent_index=*/0);
  invoker(/*target_index=*/2, /*parent_index=*/0);

  EXPECT_TRUE((view.Load<ToyField::kVelocity, float>(0).IsApprox(
      Vector3<float>(111.0F, 0.0F, 0.0F)
  )));
}

TEST(SingleOpInvokerTest, WritesOnlyTheGivenTargetIndex) {
  SimAllocator<ToyAlgorithm> sim = MakeToySim(4);
  ToyView view = sim.ViewFor<ToyAlgorithm>();

  SeedPositionOp op;
  SingleOpInvoker<SeedPositionOp, ToyView> invoker(view, op);
  invoker(2);

  EXPECT_TRUE((view.Load<ToyField::kPosition, float>(2).IsApprox(
      Vector3<float>(9.0F, 9.0F, 9.0F)
  )));
  EXPECT_TRUE((view.Load<ToyField::kPosition, float>(0).IsZero()));
  EXPECT_TRUE((view.Load<ToyField::kPosition, float>(1).IsZero()));
  EXPECT_TRUE((view.Load<ToyField::kPosition, float>(3).IsZero()));
}

// Regression test for the same zero-init bug as
// OpInvoker.AccumulatesWhenMultipleCallsTargetTheSameParent, but through
// SingleOpInvoker's single-index Invoke overload: two calls to the same
// target index with a `+=` op must sum, not have the second call's fresh
// zero clobber the first call's result.
TEST(SingleOpInvokerTest, AccumulatesAcrossMultipleCallsToTheSameTarget) {
  SimAllocator<ToyAlgorithm> sim = MakeToySim(4);
  ToyView view = sim.ViewFor<ToyAlgorithm>();

  view.Store<ToyField::kVelocity, float>(2, Vector3<float>(100.0F, 0.0F, 0.0F));
  view.Store<ToyField::kPosition, float>(2, Vector3<float>(1.0F, 0.0F, 0.0F));

  AccumulateAtTargetOp op;
  SingleOpInvoker<AccumulateAtTargetOp, ToyView> invoker(view, op);
  invoker(2);
  invoker(2);

  EXPECT_TRUE((view.Load<ToyField::kVelocity, float>(2).IsApprox(
      Vector3<float>(102.0F, 0.0F, 0.0F)
  )));
}
