#include <gtest/gtest.h>

#include <cstddef>
#include <xsimd/xsimd.hpp>

#include "domain/math/vector3.hpp"
#include "engine/memory/sim_allocator.hpp"
#include "engine/topology/ordering_policy.hpp"
#include "support/toy_field.hpp"

using achilles::domain::math::Vector3;
using achilles::engine::memory::SimAllocator;
using achilles::engine::topology::LinearOrdering;
using achilles::test_support::MakeToySim;
using achilles::test_support::ToyAlgorithm;
using achilles::test_support::ToyField;
using achilles::test_support::ToyView;

// ToyField never names an Ordering (see toy_field.hpp), so it resolves to
// the default topology::LinearOrdering -- the real Layout SimAllocator
// built is the one source of truth these tests compare View::Size()/
// NumBatches() against, rather than assuming a requested instance count
// passes through unchanged: a real View always carries the ordering
// policy's own lane padding plus one reserved base row on top of whatever
// instance count was asked for (see topology::Layout::ViewInstanceCount()).

TEST(ViewSize, SizeAndLaneSize) {
  SimAllocator<ToyAlgorithm> sim = MakeToySim(8);
  ToyView view = sim.ViewFor<ToyAlgorithm>();

  EXPECT_EQ(view.Size(), sim.LayoutFor<LinearOrdering>().ViewInstanceCount());
  EXPECT_EQ((view.LaneSize<float>()), 1U);
  EXPECT_EQ((view.LaneSize<xsimd::batch<float>>()), xsimd::batch<float>::size);
}

TEST(ViewSize, NumBatchesDividesSizeByLaneSize) {
  std::size_t lane = xsimd::batch<float>::size;
  SimAllocator<ToyAlgorithm> sim = MakeToySim(4 * lane);
  ToyView view = sim.ViewFor<ToyAlgorithm>();
  std::size_t size = sim.LayoutFor<LinearOrdering>().ViewInstanceCount();

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
