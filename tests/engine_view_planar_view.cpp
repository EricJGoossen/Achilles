#include <gtest/gtest.h>

#include <cstddef>
#include <xsimd/xsimd.hpp>

#include "domain/math/vector3.hpp"
#include "support/planar_view_fixture.hpp"
#include "support/toy_field.hpp"

using achilles::domain::math::Vector3;
using achilles::domain::math::Vector3Assembler;
using achilles::test_support::PlanarViewFixture;
using achilles::test_support::ToyField;
using achilles::test_support::ToyView;

namespace {

using Fixture = PlanarViewFixture<
    ToyField,
    Vector3Assembler<float>,
    Vector3Assembler<float>>;

}  // namespace

TEST(PlanarViewSize, SizeAndLaneSize) {
  Fixture fixture(8);
  ToyView view = fixture.MakeView();

  EXPECT_EQ(view.Size(), 8U);
  EXPECT_EQ((view.LaneSize<float>()), 1U);
  EXPECT_EQ((view.LaneSize<xsimd::batch<float>>()), xsimd::batch<float>::size);
}

TEST(PlanarViewSize, NumBatchesDividesSizeByLaneSize) {
  std::size_t lane = xsimd::batch<float>::size;
  Fixture fixture(4 * lane);
  ToyView view = fixture.MakeView();

  EXPECT_EQ((view.NumBatches<ToyField::kPosition, float>()), 4 * lane);
  EXPECT_EQ((view.NumBatches<ToyField::kPosition, xsimd::batch<float>>()), 4U);
}

// Load/Store round trip through the view directly (not via a cached
// FieldCursor), scalar and batched.
TEST(PlanarViewLoadStore, ScalarRoundTrip) {
  Fixture fixture(4);
  ToyView view = fixture.MakeView();

  Vector3<float> value(1.0F, 2.0F, 3.0F);
  view.Store<ToyField::kPosition, float>(2, value);
  Vector3<float> read_back = view.Load<ToyField::kPosition, float>(2);

  EXPECT_TRUE(read_back.IsApprox(value));
}

TEST(PlanarViewLoadStore, BatchedRoundTrip) {
  std::size_t lane = xsimd::batch<float>::size;
  Fixture fixture(2 * lane);
  ToyView view = fixture.MakeView();
  using Batch = xsimd::batch<float>;

  Vector3<Batch> value(Batch(1.0F), Batch(2.0F), Batch(3.0F));
  view.Store<ToyField::kPosition, Batch>(1, value);
  Vector3<Batch> read_back = view.Load<ToyField::kPosition, Batch>(1);

  EXPECT_FLOAT_EQ(read_back.X().get(0), 1.0F);
  EXPECT_FLOAT_EQ(read_back.Y().get(0), 2.0F);
  EXPECT_FLOAT_EQ(read_back.Z().get(0), 3.0F);
}

// Two fields backed by independent allocations (see PlanarView's own
// comment: "each type's array is its own contiguous allocation... not
// assumed to be laid out relative to one another") -- writing one must
// never bleed into the other.
TEST(PlanarViewLoadStore, FieldsDoNotAliasEachOther) {
  Fixture fixture(4);
  ToyView view = fixture.MakeView();

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
TEST(PlanarViewLoadStore, InstancesDoNotAliasEachOther) {
  Fixture fixture(4);
  ToyView view = fixture.MakeView();

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
TEST(PlanarViewFieldCursor, LoadStoreMatchDirectViewAccess) {
  Fixture fixture(4);
  ToyView view = fixture.MakeView();

  auto cursor = view.Field<ToyField::kPosition>();
  Vector3<float> value(7.0F, 8.0F, 9.0F);
  cursor.Store<float>(3, value);

  EXPECT_TRUE((view.Load<ToyField::kPosition, float>(3).IsApprox(value)));
  EXPECT_TRUE(cursor.Load<float>(3).IsApprox(value));

  view.Store<ToyField::kPosition, float>(1, Vector3<float>(0.5F, 0.5F, 0.5F));
  EXPECT_TRUE(cursor.Load<float>(1).IsApprox(Vector3<float>(0.5F, 0.5F, 0.5F)));
}
