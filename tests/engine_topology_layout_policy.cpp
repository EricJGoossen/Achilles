#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <xsimd/xsimd.hpp>

#include "domain/math/activation_mask.hpp"
#include "domain/math/vector3.hpp"
#include "engine/topology/layout_policy.hpp"

using achilles::domain::math::ActivationMaskAssembler;
using achilles::domain::math::Vector3Assembler;
using achilles::engine::topology::AoSoALayout;
using achilles::engine::topology::LayoutPolicyLike;
using achilles::engine::topology::PlanarLayout;

static_assert(LayoutPolicyLike<PlanarLayout>);

TEST(
    PlanarLayoutLaneCount, MatchesXsimdBatchSizeForTheAssemblersOwnScalarType
) {
  EXPECT_EQ(
      PlanarLayout::LaneCount<Vector3Assembler<float>>(),
      xsimd::batch<float>::size
  );
  EXPECT_EQ(
      PlanarLayout::LaneCount<ActivationMaskAssembler<8>>(),
      xsimd::batch<std::int32_t>::size
  );
}

TEST(
    PlanarLayoutAlignment, MatchesAlignofXsimdBatchForTheAssemblersOwnScalarType
) {
  EXPECT_EQ(
      PlanarLayout::Alignment<Vector3Assembler<float>>(),
      alignof(xsimd::batch<float>)
  );
  EXPECT_EQ(
      PlanarLayout::Alignment<ActivationMaskAssembler<8>>(),
      alignof(xsimd::batch<std::int32_t>)
  );
}

TEST(PlanarLayoutElementStrideBytes, EqualsSizeofScalarType) {
  EXPECT_EQ(
      PlanarLayout::ElementStrideBytes<Vector3Assembler<float>>(), sizeof(float)
  );
  EXPECT_EQ(
      PlanarLayout::ElementStrideBytes<ActivationMaskAssembler<8>>(),
      sizeof(std::int32_t)
  );
}

// StrideBytes is the raw padded_instances*sizeof(ScalarType) byte count,
// rounded UP to Alignment<A>() -- pick a padded_instances that leaves a
// remainder under float's own alignment, to actually exercise the
// rounding rather than coincidentally landing on a multiple already.
TEST(PlanarLayoutStrideBytes, RoundsUpToAlignment) {
  std::size_t alignment = PlanarLayout::Alignment<Vector3Assembler<float>>();
  std::size_t padded_instances = alignment / sizeof(float) + 1;
  std::size_t raw_bytes = padded_instances * sizeof(float);
  std::size_t expected = (raw_bytes + alignment - 1) / alignment * alignment;

  EXPECT_EQ(
      PlanarLayout::StrideBytes<Vector3Assembler<float>>(padded_instances),
      expected
  );
  EXPECT_GT(
      expected, raw_bytes
  );  // otherwise this test isn't exercising rounding
}

TEST(PlanarLayoutStrideBytes, ExactMultipleOfAlignmentIsUnchanged) {
  std::size_t alignment = PlanarLayout::Alignment<Vector3Assembler<float>>();
  std::size_t padded_instances = 2 * alignment / sizeof(float);
  std::size_t raw_bytes = padded_instances * sizeof(float);

  EXPECT_EQ(
      PlanarLayout::StrideBytes<Vector3Assembler<float>>(padded_instances),
      raw_bytes
  );
}

// BlockBytes multiplies StrideBytes by kNumFields -- Vector3Assembler<T>
// flattens to 3 leaves (x, y, z), so its block is 3 independent leaf-arrays
// back to back, not one array of Vector3-sized elements.
TEST(PlanarLayoutBlockBytes, MultipliesStrideBytesByNumFields) {
  std::size_t padded_instances =
      PlanarLayout::LaneCount<Vector3Assembler<float>>();
  std::size_t stride =
      PlanarLayout::StrideBytes<Vector3Assembler<float>>(padded_instances);

  EXPECT_EQ(
      PlanarLayout::BlockBytes<Vector3Assembler<float>>(padded_instances),
      stride * 3
  );
}

// ActivationMaskAssembler<8> flattens to a single leaf (RepeatTypes<1>),
// so its BlockBytes is exactly one leaf's StrideBytes, not multiplied.
TEST(PlanarLayoutBlockBytes, SingleLeafAssemblerBlockBytesEqualsStrideBytes) {
  std::size_t padded_instances =
      PlanarLayout::LaneCount<ActivationMaskAssembler<8>>();
  std::size_t stride =
      PlanarLayout::StrideBytes<ActivationMaskAssembler<8>>(padded_instances);

  EXPECT_EQ(
      PlanarLayout::BlockBytes<ActivationMaskAssembler<8>>(padded_instances),
      stride
  );
}

TEST(PlanarLayoutIsLaneMultiple, TrueForExactMultipleFalseOtherwise) {
  std::size_t lane = PlanarLayout::LaneCount<Vector3Assembler<float>>();

  EXPECT_TRUE(PlanarLayout::IsLaneMultiple<Vector3Assembler<float>>(lane));
  EXPECT_TRUE(PlanarLayout::IsLaneMultiple<Vector3Assembler<float>>(2 * lane));
  EXPECT_TRUE(PlanarLayout::IsLaneMultiple<Vector3Assembler<float>>(0));
  EXPECT_FALSE(PlanarLayout::IsLaneMultiple<Vector3Assembler<float>>(lane + 1));
}

// RowOffsetBytes is the same linear step ElementStrideBytes describes, and
// the stride depends on the instance count, so it isn't fixed.
TEST(PlanarLayoutRowOffsetBytes, IsRowTimesElementStride) {
  EXPECT_EQ(PlanarLayout::RowOffsetBytes<Vector3Assembler<float>>(0), 0U);
  EXPECT_EQ(
      PlanarLayout::RowOffsetBytes<Vector3Assembler<float>>(5),
      5 * PlanarLayout::ElementStrideBytes<Vector3Assembler<float>>()
  );
  EXPECT_FALSE(PlanarLayout::kFixedStride);
}

// ===== AoSoALayout =====

static_assert(LayoutPolicyLike<AoSoALayout>);

TEST(AoSoALayoutLaneCountAndAlignment, MatchPlanarLayout) {
  EXPECT_EQ(
      AoSoALayout::LaneCount<Vector3Assembler<float>>(),
      PlanarLayout::LaneCount<Vector3Assembler<float>>()
  );
  EXPECT_EQ(
      AoSoALayout::Alignment<ActivationMaskAssembler<8>>(),
      PlanarLayout::Alignment<ActivationMaskAssembler<8>>()
  );
}

// The leaf stride is one batch, whatever the instance count -- which is
// what lets View fold it into each load at compile time.
TEST(AoSoALayoutStrideBytes, IsOneBatchIndependentOfInstanceCount) {
  constexpr std::size_t kBatchBytes = sizeof(xsimd::batch<float>);
  EXPECT_TRUE(AoSoALayout::kFixedStride);
  EXPECT_EQ(AoSoALayout::StrideBytes<Vector3Assembler<float>>(0), kBatchBytes);
  EXPECT_EQ(
      AoSoALayout::StrideBytes<Vector3Assembler<float>>(1000), kBatchBytes
  );
}

// A batch group holds every leaf back to back: Vector3's three batches.
// Blocks round up to whole groups.
TEST(AoSoALayoutBlockBytes, WholeGroupsOfEveryLeaf) {
  constexpr std::size_t kBatchBytes = sizeof(xsimd::batch<float>);
  constexpr std::size_t kLanes = xsimd::batch<float>::size;
  EXPECT_EQ(
      AoSoALayout::GroupBytes<Vector3Assembler<float>>(), 3 * kBatchBytes
  );
  EXPECT_EQ(
      AoSoALayout::BlockBytes<Vector3Assembler<float>>(2 * kLanes),
      2 * 3 * kBatchBytes
  );
  EXPECT_EQ(
      AoSoALayout::BlockBytes<Vector3Assembler<float>>(kLanes + 1),
      2 * 3 * kBatchBytes
  );
}

// Within a group a row is one scalar into each leaf; the next group starts
// a whole group later -- so every group, and every leaf in it, stays
// batch-aligned.
TEST(AoSoALayoutRowOffsetBytes, StepsWithinThenAcrossGroups) {
  constexpr std::size_t kLanes = xsimd::batch<float>::size;
  constexpr std::size_t kGroup =
      AoSoALayout::GroupBytes<Vector3Assembler<float>>();
  EXPECT_EQ(AoSoALayout::RowOffsetBytes<Vector3Assembler<float>>(0), 0U);
  EXPECT_EQ(
      AoSoALayout::RowOffsetBytes<Vector3Assembler<float>>(kLanes - 1),
      (kLanes - 1) * sizeof(float)
  );
  EXPECT_EQ(
      AoSoALayout::RowOffsetBytes<Vector3Assembler<float>>(kLanes), kGroup
  );
  EXPECT_EQ(
      AoSoALayout::RowOffsetBytes<Vector3Assembler<float>>(2 * kLanes + 1),
      2 * kGroup + sizeof(float)
  );
  EXPECT_EQ(kGroup % AoSoALayout::Alignment<Vector3Assembler<float>>(), 0U);
}
