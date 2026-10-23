#include <gtest/gtest.h>

#include <cstddef>
#include <tuple>
#include <xsimd/xsimd.hpp>

#include "domain/math/scalar.hpp"
#include "domain/math/vector3.hpp"
#include "engine/memory/sim_allocator.hpp"
#include "engine/topology/indexing_policy.hpp"
#include "engine/topology/ordering_policy.hpp"
#include "support/toy_field.hpp"
#include "support/toy_reduce_field.hpp"

using achilles::domain::math::Scalar;
using achilles::domain::math::Vector3;
using achilles::engine::topology::BroadcastIndex;
using achilles::engine::memory::SimAllocator;
using achilles::engine::topology::LinearOrdering;
using achilles::test_support::MakeToySim;
using achilles::test_support::ToyAlgorithm;
using achilles::test_support::ToyField;
using achilles::test_support::MakeToyReduceSim;
using achilles::test_support::ToyReduceAlgorithm;
using achilles::test_support::ToyReduceField;
using achilles::test_support::ToyReduceView;
using achilles::test_support::ToyView;
using Batch = xsimd::batch<float>;

namespace {

// Every padded row plus the reserved base row, rounded up to the lane
// width -- what OrderingPlan::ViewInstanceCount() sized the View to.
std::size_t ExpectedViewSize(const SimAllocator<ToyAlgorithm>& sim) {
  std::size_t lane = SimAllocator<ToyAlgorithm>::RequiredLaneSize();
  std::size_t needed = sim.LayoutFor<LinearOrdering>().BaseRowIndex() + 1;
  return (needed + lane - 1) / lane * lane;
}

}  // namespace

// ToyField never names an Ordering (see toy_field.hpp), so it resolves to
// the default topology::LinearOrdering -- the real row placement SimAllocator
// built is the one source of truth these tests compare View::Size()/
// NumBatches() against, rather than assuming a requested instance count
// passes through unchanged: a real View always carries the ordering
// policy's own lane padding plus one reserved base row on top of whatever
// instance count was asked for (see
// topology::OrderingPlan::ViewInstanceCount()).

TEST(ViewSize, SizeAndLaneSize) {
  SimAllocator<ToyAlgorithm> sim = MakeToySim(8);
  ToyView view = sim.ViewFor<ToyAlgorithm>();

  EXPECT_EQ(view.Size(), ExpectedViewSize(sim));
  EXPECT_EQ((view.LaneSize<float>()), 1U);
  EXPECT_EQ((view.LaneSize<xsimd::batch<float>>()), xsimd::batch<float>::size);
}

TEST(ViewSize, NumBatchesDividesSizeByLaneSize) {
  std::size_t lane = xsimd::batch<float>::size;
  SimAllocator<ToyAlgorithm> sim = MakeToySim(4 * lane);
  ToyView view = sim.ViewFor<ToyAlgorithm>();
  std::size_t size = ExpectedViewSize(sim);

  EXPECT_EQ((view.NumBatches<ToyField::kPosition, float>()), size);
  EXPECT_EQ(
      (view.NumBatches<ToyField::kPosition, xsimd::batch<float>>()), size / lane
  );
}

// Load/Store round trip through the view directly (not via a cached
// FieldCursor), scalar and batched.
TEST(ViewLoadStore, ScalarRoundTrip) {
  SimAllocator<ToyAlgorithm> sim = MakeToySim(4);
  ToyView view = sim.ViewFor<ToyAlgorithm>();

  Vector3<float> value(1.0F, 2.0F, 3.0F);
  view.Store<ToyField::kPosition, float>(2, value);
  Vector3<float> read_back = view.Load<ToyField::kPosition, float>(2);

  EXPECT_TRUE(read_back.IsApprox(value));
}

TEST(ViewLoadStore, BatchedRoundTrip) {
  std::size_t lane = xsimd::batch<float>::size;
  SimAllocator<ToyAlgorithm> sim = MakeToySim(2 * lane);
  ToyView view = sim.ViewFor<ToyAlgorithm>();
  using Batch = xsimd::batch<float>;

  Vector3<Batch> value(Batch(1.0F), Batch(2.0F), Batch(3.0F));
  view.Store<ToyField::kPosition, Batch>(1, value);
  Vector3<Batch> read_back = view.Load<ToyField::kPosition, Batch>(1);

  EXPECT_FLOAT_EQ(read_back.X().get(0), 1.0F);
  EXPECT_FLOAT_EQ(read_back.Y().get(0), 2.0F);
  EXPECT_FLOAT_EQ(read_back.Z().get(0), 3.0F);
}

// Two fields backed by independent allocations (see View's own comment:
// "each type's array is its own contiguous allocation... not assumed to be
// laid out relative to one another") -- writing one must never bleed into
// the other.
TEST(ViewLoadStore, FieldsDoNotAliasEachOther) {
  SimAllocator<ToyAlgorithm> sim = MakeToySim(4);
  ToyView view = sim.ViewFor<ToyAlgorithm>();

  view.Store<ToyField::kPosition, float>(0, Vector3<float>(1.0F, 2.0F, 3.0F));
  view.Store<ToyField::kVelocity, float>(0, Vector3<float>(4.0F, 5.0F, 6.0F));

  EXPECT_TRUE((view.Load<ToyField::kPosition, float>(0).IsApprox(
      Vector3<float>(1.0F, 2.0F, 3.0F)
  )));
  EXPECT_TRUE((view.Load<ToyField::kVelocity, float>(0).IsApprox(
      Vector3<float>(4.0F, 5.0F, 6.0F)
  )));
}

// Different instance indices within the same field are independent too.
TEST(ViewLoadStore, InstancesDoNotAliasEachOther) {
  SimAllocator<ToyAlgorithm> sim = MakeToySim(4);
  ToyView view = sim.ViewFor<ToyAlgorithm>();

  view.Store<ToyField::kPosition, float>(0, Vector3<float>(1.0F, 0.0F, 0.0F));
  view.Store<ToyField::kPosition, float>(1, Vector3<float>(0.0F, 1.0F, 0.0F));

  EXPECT_TRUE((view.Load<ToyField::kPosition, float>(0).IsApprox(
      Vector3<float>(1.0F, 0.0F, 0.0F)
  )));
  EXPECT_TRUE((view.Load<ToyField::kPosition, float>(1).IsApprox(
      Vector3<float>(0.0F, 1.0F, 0.0F)
  )));
}

// FieldCursor<F> (Field<F>()) is documented as a cached-lookup equivalent
// to repeated Load<F>/Store<F> calls -- must read/write the exact same
// storage.
TEST(ViewFieldCursor, LoadStoreMatchDirectViewAccess) {
  SimAllocator<ToyAlgorithm> sim = MakeToySim(4);
  ToyView view = sim.ViewFor<ToyAlgorithm>();

  auto cursor = view.Field<ToyField::kPosition>();
  Vector3<float> value(7.0F, 8.0F, 9.0F);
  cursor.Store<float>(3, value);

  EXPECT_TRUE((view.Load<ToyField::kPosition, float>(3).IsApprox(value)));
  EXPECT_TRUE(cursor.Load<float>(3).IsApprox(value));

  view.Store<ToyField::kPosition, float>(1, Vector3<float>(0.5F, 0.5F, 0.5F));
  EXPECT_TRUE(cursor.Load<float>(1).IsApprox(Vector3<float>(0.5F, 0.5F, 0.5F)));
}

// A GlobalIndexing field's block is one element, whatever the row count --
// sized by its own Indexing policy's Count, not by the View's rows.
TEST(ViewSize, GlobalFieldHoldsOneElement) {
  SimAllocator<ToyReduceAlgorithm> sim = MakeToyReduceSim(8);
  ToyReduceView view = sim.ViewFor<ToyReduceAlgorithm>();

  EXPECT_EQ((view.NumBatches<ToyReduceField::kTotal, float>()), 1U);
  EXPECT_GT((view.NumBatches<ToyReduceField::kValue, float>()), 1U);
}

// A BroadcastIndex input reads one row, splatted into every lane -- the
// value a reduction's own Load (as opposed to its LoadOutput) must see.
TEST(ViewFieldCursor, BroadcastLoadSplatsOneRowAcrossEveryLane) {
  SimAllocator<ToyReduceAlgorithm> sim = MakeToyReduceSim(2 * Batch::size);
  ToyReduceView view = sim.ViewFor<ToyReduceAlgorithm>();
  view.Store<ToyReduceField::kValue, float>(3, Scalar<float>(4.5F));

  auto cursor = view.Field<ToyReduceField::kValue>();
  Scalar<Batch> splat = cursor.Load<Batch>(BroadcastIndex{3});
  for (std::size_t lane = 0; lane < Batch::size; ++lane) {
    EXPECT_FLOAT_EQ(splat.Value().get(lane), 4.5F);
  }
}

// At a group index an output starts from whatever the field holds (outputs
// are read/write) -- LoadOutput there is exactly Load.
TEST(ViewFieldCursor, LoadOutputAtAGroupIndexIsTheStoredValue) {
  SimAllocator<ToyReduceAlgorithm> sim = MakeToyReduceSim(4);
  ToyReduceView view = sim.ViewFor<ToyReduceAlgorithm>();
  view.Store<ToyReduceField::kValue, float>(2, Scalar<float>(7.0F));

  auto cursor = view.Field<ToyReduceField::kValue>();
  EXPECT_FLOAT_EQ(cursor.LoadOutput<float>(2).Value(), 7.0F);
}

// At a BroadcastIndex an output starts from zero, not from a splat of the
// row: every lane's contribution is summed back into the row on Store, so
// a splatted start would add the row's own value back in once per lane.
TEST(ViewFieldCursor, LoadOutputAtABroadcastIndexIsZero) {
  SimAllocator<ToyReduceAlgorithm> sim = MakeToyReduceSim(4);
  ToyReduceView view = sim.ViewFor<ToyReduceAlgorithm>();
  view.Store<ToyReduceField::kTotal, float>(0, Scalar<float>(7.0F));

  auto cursor = view.Field<ToyReduceField::kTotal>();
  Scalar<Batch> start = cursor.LoadOutput<Batch>(BroadcastIndex{0});
  for (std::size_t lane = 0; lane < Batch::size; ++lane) {
    EXPECT_FLOAT_EQ(start.Value().get(lane), 0.0F);
  }
}

// Store at a BroadcastIndex is row += (sum of the lanes): onto the stored
// value, never over it, and into that one row only.
TEST(ViewFieldCursor, BroadcastStoreAddsTheLaneSumOntoTheRow) {
  SimAllocator<ToyReduceAlgorithm> sim = MakeToyReduceSim(2 * Batch::size);
  ToyReduceView view = sim.ViewFor<ToyReduceAlgorithm>();
  view.Store<ToyReduceField::kValue, float>(1, Scalar<float>(10.0F));
  view.Store<ToyReduceField::kValue, float>(2, Scalar<float>(20.0F));

  auto cursor = view.Field<ToyReduceField::kValue>();
  cursor.Store<Batch>(BroadcastIndex{1}, Scalar<Batch>(Batch(0.5F)));
  cursor.Store<Batch>(BroadcastIndex{1}, Scalar<Batch>(Batch(0.25F)));

  auto lanes = static_cast<float>(Batch::size);
  EXPECT_FLOAT_EQ(
      (view.Load<ToyReduceField::kValue, float>(1).Value()),
      10.0F + (0.75F * lanes)
  );
  EXPECT_FLOAT_EQ((view.Load<ToyReduceField::kValue, float>(2).Value()), 20.0F);
}

// Which overload an argument gets is its field's Indexing policy's call: a
// batched read of a GlobalIndexing field resolves to BroadcastIndex{0}, a
// scalar one to plain index 0, and a row field to the traversal's index
// unchanged.
TEST(ViewFieldCursor, ResolveFollowsEachFieldsOwnIndexingPolicy) {
  SimAllocator<ToyReduceAlgorithm> sim = MakeToyReduceSim(4);
  ToyReduceView view = sim.ViewFor<ToyReduceAlgorithm>();
  std::tuple<std::size_t, std::size_t> indices{3, 1};

  auto total = view.Field<ToyReduceField::kTotal>();
  EXPECT_EQ((total.Resolve<Batch, 0>(indices).row), 0U);
  EXPECT_EQ((total.Resolve<float, 0>(indices)), 0U);

  auto value = view.Field<ToyReduceField::kValue>();
  EXPECT_EQ((value.Resolve<float, 0>(indices)), 3U);
  EXPECT_EQ((value.Resolve<float, 1>(indices)), 1U);
}
