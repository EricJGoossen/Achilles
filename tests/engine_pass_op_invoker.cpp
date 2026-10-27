#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <xsimd/xsimd.hpp>

#include "domain/math/scalar.hpp"
#include "domain/math/vector3.hpp"
#include "engine/memory/sim_allocator.hpp"
#include "engine/op_contract.hpp"
#include "engine/pass/op_invoker.hpp"
#include "engine/topology/indexing_policy.hpp"
#include "support/toy_field.hpp"
#include "support/toy_reduce_field.hpp"

using achilles::domain::math::Scalar;
using achilles::domain::math::Vector3;
using achilles::engine::topology::BroadcastIndex;
using achilles::engine::ArgData;
using achilles::engine::ArgIndex;
using achilles::engine::OpArgsMatchView;
using achilles::engine::OpHasInit;
using achilles::engine::OpLike;
using achilles::engine::memory::SimAllocator;
using achilles::engine::pass::OpInvoker;
using achilles::engine::pass::SingleOpInvoker;
using achilles::test_support::MakeToySim;
using achilles::test_support::ToyAlgorithm;
using achilles::test_support::ToyField;
using achilles::test_support::MakeToyReduceSim;
using achilles::test_support::ToyReduceAlgorithm;
using achilles::test_support::ToyReduceField;
using achilles::test_support::ToyReduceView;
using achilles::test_support::ToyView;

namespace {

// Reads kPosition at the target index and kVelocity at the parent index
// (ArgIndex::kTarget/kParent mixed, per ArgData), writes the elementwise sum
// to kVelocity at the target index -- exercises the (target_index,
// parent_index) split OpInvoker::operator() implements.
struct CombineOp {
  using FieldEnum = ToyField;

  static constexpr std::array<ArgData<FieldEnum>, 2> kInputs = {
      ArgData<FieldEnum>{FieldEnum::kPosition, ArgIndex::kTarget},
      ArgData<FieldEnum>{FieldEnum::kVelocity, ArgIndex::kParent},
  };
  static constexpr std::array<ArgData<FieldEnum>, 1> kOutputs = {
      ArgData<FieldEnum>{FieldEnum::kVelocity, ArgIndex::kTarget},
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
      ArgData<FieldEnum>{FieldEnum::kPosition, ArgIndex::kTarget},
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
      ArgData<FieldEnum>{FieldEnum::kPosition, ArgIndex::kTarget},
  };
  static constexpr std::array<ArgData<FieldEnum>, 1> kOutputs = {
      ArgData<FieldEnum>{FieldEnum::kVelocity, ArgIndex::kParent},
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
// SingleOpInvoker's single-index operator() (ArgIndex::kTarget, the
// target doubling as its own source) -- a separate entry point from
// OpInvoker::operator(), so it needed the same
// load-before-store fix and deserves its own regression test.
struct AccumulateAtTargetOp {
  using FieldEnum = ToyField;

  static constexpr std::array<ArgData<FieldEnum>, 1> kInputs = {
      ArgData<FieldEnum>{FieldEnum::kPosition, ArgIndex::kTarget},
  };
  static constexpr std::array<ArgData<FieldEnum>, 1> kOutputs = {
      ArgData<FieldEnum>{FieldEnum::kVelocity, ArgIndex::kTarget},
  };

  void operator()(
      const Vector3<float>& contribution, Vector3<float>* velocity_out
  ) const {
    *velocity_out += contribution;
  }
};
static_assert(OpLike<AccumulateAtTargetOp>);
static_assert(OpArgsMatchView<AccumulateAtTargetOp, ToyView>);

// The same `+=`-onto-the-view shape again, but through Initialize/
// kInitOutputs -- OpInitInvoker builds its own output tuple, separate from
// OpInvokerBase's, so it needs the same load-before-store contract (outputs
// are always read/write, see op_invoker.hpp) and its own regression tests.
struct AccumulateOnInitializeOp {
  using FieldEnum = ToyField;

  static constexpr std::array<ArgData<FieldEnum>, 0> kInitInputs = {};
  static constexpr std::array<ArgData<FieldEnum>, 1> kInitOutputs = {
      ArgData<FieldEnum>{FieldEnum::kVelocity, ArgIndex::kTarget},
  };
  static constexpr std::array<ArgData<FieldEnum>, 0> kInputs = {};
  static constexpr std::array<ArgData<FieldEnum>, 0> kOutputs = {};

  void Initialize(Vector3<float>* velocity_out) const {
    *velocity_out += Vector3<float>(1.0F, 0.0F, 0.0F);
  }
  void operator()() const {}
};
static_assert(OpLike<AccumulateOnInitializeOp>);
static_assert(OpHasInit<AccumulateOnInitializeOp>);
static_assert(OpArgsMatchView<AccumulateOnInitializeOp, ToyView>);

using Batch = xsimd::batch<float>;

// A batched read of kValue's target group, added into kTotal -- a
// GlobalIndexing field, so a batched output of it resolves to
// BroadcastIndex{0}: the invoker must hand it in zeroed and store it as
// kTotal += (sum of lanes). `+=` onto the incoming value, so a non-zero
// start (a splat of the stored total) would show up as that total counted
// once per lane.
struct ReduceIntoTotalOp {
  using FieldEnum = ToyReduceField;

  static constexpr std::array<ArgData<FieldEnum>, 1> kInputs = {
      ArgData<FieldEnum>{FieldEnum::kValue, ArgIndex::kTarget},
  };
  static constexpr std::array<ArgData<FieldEnum>, 1> kOutputs = {
      ArgData<FieldEnum>{FieldEnum::kTotal, ArgIndex::kTarget},
  };

  void operator()(const Scalar<Batch>& value, Scalar<Batch>* total_out)
      const {
    *total_out += value;
  }
};
static_assert(OpLike<ReduceIntoTotalOp>);
static_assert(OpArgsMatchView<ReduceIntoTotalOp, ToyReduceView>);

// A pair pass's shape: kValue's target group read lane for lane, added into
// the source row -- a row-indexed field resolved through the traversal's
// BroadcastIndex source, so a reduction into that one row.
struct ReduceIntoSourceOp {
  using FieldEnum = ToyReduceField;

  static constexpr std::array<ArgData<FieldEnum>, 1> kInputs = {
      ArgData<FieldEnum>{FieldEnum::kValue, ArgIndex::kTarget},
  };
  static constexpr std::array<ArgData<FieldEnum>, 1> kOutputs = {
      ArgData<FieldEnum>{FieldEnum::kValue, ArgIndex::kSource},
  };

  void operator()(const Scalar<Batch>& target, Scalar<Batch>* source_out)
      const {
    *source_out += target;
  }
};
static_assert(OpLike<ReduceIntoSourceOp>);
static_assert(OpArgsMatchView<ReduceIntoSourceOp, ToyReduceView>);

// Overwrites the whole target group with 100 and reduces 1 per lane into
// the source row -- in that kOutputs order, the order the contact solver's
// ContactForceOp relies on (target wrench, then source wrench). When the
// source row sits inside the target group both cover the same memory, and
// the reduction must land on top of the group store, not under it.
struct GroupThenSourceOp {
  using FieldEnum = ToyReduceField;

  static constexpr std::array<ArgData<FieldEnum>, 0> kInputs = {};
  static constexpr std::array<ArgData<FieldEnum>, 2> kOutputs = {
      ArgData<FieldEnum>{FieldEnum::kValue, ArgIndex::kTarget},
      ArgData<FieldEnum>{FieldEnum::kValue, ArgIndex::kSource},
  };

  void operator()(Scalar<Batch>* target_out, Scalar<Batch>* source_out) const {
    *target_out = Scalar<Batch>(Batch(100.0F));
    *source_out += Scalar<Batch>(Batch(1.0F));
  }
};
static_assert(OpLike<GroupThenSourceOp>);
static_assert(OpArgsMatchView<GroupThenSourceOp, ToyReduceView>);

// A scalar read and write of kTotal -- what a GlobalTraversal Op does: a
// one-lane load resolves to plain index 0, so the output starts from the
// stored value (an ordinary read/write output), not from zero.
struct DoubleTotalOp {
  using FieldEnum = ToyReduceField;

  static constexpr std::array<ArgData<FieldEnum>, 0> kInputs = {};
  static constexpr std::array<ArgData<FieldEnum>, 1> kOutputs = {
      ArgData<FieldEnum>{FieldEnum::kTotal, ArgIndex::kTarget},
  };

  void operator()(Scalar<float>* total_out) const {
    *total_out = Scalar<float>(total_out->Value() * 2.0F);
  }
};
static_assert(OpLike<DoubleTotalOp>);
static_assert(OpArgsMatchView<DoubleTotalOp, ToyReduceView>);

// Fills row r of kValue with r + 1, for the reductions below to sum.
void FillAscending(const ToyReduceView& view, std::size_t rows) {
  for (std::size_t row = 0; row < rows; ++row) {
    view.Store<ToyReduceField::kValue, float>(
        row, Scalar<float>(static_cast<float>(row + 1))
    );
  }
}

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

// Regression test for the same zero-init bug, through OpInvoker::Initialize:
// the base row's stored value must reach Initialize as its output's
// incoming value, not be replaced by a fresh default-constructed one --
// two calls must sum onto it.
TEST(OpInvoker, InitializeAccumulatesOntoTheStoredValue) {
  SimAllocator<ToyAlgorithm> sim = MakeToySim(4);
  ToyView view = sim.ViewFor<ToyAlgorithm>();

  view.Store<ToyField::kVelocity, float>(0, Vector3<float>(100.0F, 0.0F, 0.0F));

  AccumulateOnInitializeOp op;
  OpInvoker<AccumulateOnInitializeOp, ToyView> invoker(view, op);
  invoker.Initialize(0);
  invoker.Initialize(0);

  EXPECT_TRUE((view.Load<ToyField::kVelocity, float>(0).IsApprox(
      Vector3<float>(102.0F, 0.0F, 0.0F)
  )));
}

// Same as above through SingleOpInvoker::Initialize -- a separate class
// from OpInvoker, so a separate shape per TESTING.md 9.2, even though both
// reach the same OpInitInvoker underneath today.
TEST(SingleOpInvokerTest, InitializeAccumulatesOntoTheStoredValue) {
  SimAllocator<ToyAlgorithm> sim = MakeToySim(4);
  ToyView view = sim.ViewFor<ToyAlgorithm>();

  view.Store<ToyField::kVelocity, float>(1, Vector3<float>(100.0F, 0.0F, 0.0F));

  AccumulateOnInitializeOp op;
  SingleOpInvoker<AccumulateOnInitializeOp, ToyView> invoker(view, op);
  invoker.Initialize(1);
  invoker.Initialize(1);

  EXPECT_TRUE((view.Load<ToyField::kVelocity, float>(1).IsApprox(
      Vector3<float>(102.0F, 0.0F, 0.0F)
  )));
  // Initialize only touches the base index it was given.
  EXPECT_TRUE((view.Load<ToyField::kVelocity, float>(0).IsZero()));
}

// A batched output of a GlobalIndexing field is a reduction: each call
// adds the sum of its lanes onto the stored total, and two calls (two lane
// groups of a linear pass) both land. The stored 5 must be counted once,
// not once per lane -- proving the output started from zero rather than a
// splat of the total.
TEST(OpInvoker, BatchedGlobalOutputReducesEveryCallOntoTheStoredValue) {
  SimAllocator<ToyReduceAlgorithm> sim = MakeToyReduceSim(2 * Batch::size);
  ToyReduceView view = sim.ViewFor<ToyReduceAlgorithm>();
  FillAscending(view, 2 * Batch::size);
  view.Store<ToyReduceField::kTotal, float>(0, Scalar<float>(5.0F));

  ReduceIntoTotalOp op;
  OpInvoker<ReduceIntoTotalOp, ToyReduceView> invoker(view, op);
  invoker(/*target_index=*/0, /*source_index=*/std::size_t{0});
  invoker(/*target_index=*/1, /*source_index=*/std::size_t{1});

  auto n = static_cast<float>(2 * Batch::size);
  EXPECT_FLOAT_EQ(
      (view.Load<ToyReduceField::kTotal, float>(0).Value()),
      5.0F + (n * (n + 1.0F) * 0.5F)
  );
}

// The pair-pass shape of the same reduction: a row-indexed output named by
// a BroadcastIndex source sums the target group's lanes into that one
// source row, and leaves every other row alone.
TEST(OpInvoker, OutputAtABroadcastSourceReducesIntoThatRow) {
  SimAllocator<ToyReduceAlgorithm> sim = MakeToyReduceSim(2 * Batch::size);
  ToyReduceView view = sim.ViewFor<ToyReduceAlgorithm>();
  FillAscending(view, 2 * Batch::size);

  ReduceIntoSourceOp op;
  OpInvoker<ReduceIntoSourceOp, ToyReduceView> invoker(view, op);
  // Source row 0 against target group 1 (rows lane .. 2 lane - 1).
  invoker(/*target_index=*/1, BroadcastIndex{0});

  auto lanes = static_cast<float>(Batch::size);
  // 1 + sum over r in [lane, 2 lane) of (r + 1)
  float group_sum = (lanes * (lanes + 1.0F) * 0.5F) + (lanes * lanes);
  EXPECT_FLOAT_EQ(
      (view.Load<ToyReduceField::kValue, float>(0).Value()), 1.0F + group_sum
  );
  EXPECT_FLOAT_EQ((view.Load<ToyReduceField::kValue, float>(1).Value()), 2.0F);
}

// Outputs store in kOutputs order, and a reduction reads its row at store
// time: with the source row inside the target group, the group's 100 is
// stored first and the source's lane sum lands on top of it.
TEST(OpInvoker, BroadcastOutputStoresOnTopOfAnEarlierGroupOutput) {
  SimAllocator<ToyReduceAlgorithm> sim = MakeToyReduceSim(Batch::size);
  ToyReduceView view = sim.ViewFor<ToyReduceAlgorithm>();

  GroupThenSourceOp op;
  OpInvoker<GroupThenSourceOp, ToyReduceView> invoker(view, op);
  invoker(/*target_index=*/0, BroadcastIndex{0});

  auto lanes = static_cast<float>(Batch::size);
  EXPECT_FLOAT_EQ(
      (view.Load<ToyReduceField::kValue, float>(0).Value()), 100.0F + lanes
  );
  // The rest of the group holds just the group store.
  if (Batch::size > 1) {
    EXPECT_FLOAT_EQ(
        (view.Load<ToyReduceField::kValue, float>(1).Value()), 100.0F
    );
  }
}

// A scalar output of a GlobalIndexing field is an ordinary read/write
// output at index 0: it starts from the stored value, and is overwritten
// with whatever the Op leaves there.
TEST(SingleOpInvokerTest, ScalarGlobalOutputReadsAndWritesTheValuePlainly) {
  SimAllocator<ToyReduceAlgorithm> sim = MakeToyReduceSim(4);
  ToyReduceView view = sim.ViewFor<ToyReduceAlgorithm>();
  view.Store<ToyReduceField::kTotal, float>(0, Scalar<float>(3.0F));

  DoubleTotalOp op;
  SingleOpInvoker<DoubleTotalOp, ToyReduceView> invoker(view, op);
  invoker(0);
  invoker(0);

  EXPECT_FLOAT_EQ((view.Load<ToyReduceField::kTotal, float>(0).Value()), 12.0F);
}
