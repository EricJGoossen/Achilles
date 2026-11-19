#include <gtest/gtest.h>

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <xsimd/xsimd.hpp>

#include "domain/math/activation_mask.hpp"
#include "domain/math/vector3.hpp"
#include "engine/topology/layout_policy.hpp"

using achilles::domain::math::ActivationMaskAssembler;
using achilles::domain::math::Vector3;
using achilles::domain::math::Vector3Assembler;
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

// ---------------------------------------------------------------------------
// BlockedLayout: PlanarLayout's leaves a lane group at a time.
// ---------------------------------------------------------------------------

namespace {

using achilles::engine::topology::BlockedLayout;
using achilles::engine::topology::NoRowTable;
using V3A = Vector3Assembler<float>;
constexpr std::size_t kL = xsimd::batch<float>::size;

}  // namespace

// One instance's leaves are a lane group's run apart, and a block holds
// every leaf of every padded instance with nothing between.
TEST(BlockedLayoutSizes, LeafStrideIsOneLaneGroupAndBlocksArePacked) {
  EXPECT_EQ(BlockedLayout::StrideBytes<V3A>(4 * kL), kL * sizeof(float));
  EXPECT_EQ(BlockedLayout::BlockBytes<V3A>(4 * kL), 4 * kL * 3 * sizeof(float));
}

// Row r sits in lane group r / L, at lane r % L of that group's run.
TEST(BlockedLayoutRowOffset, PutsARowInItsLaneGroupsRunAtItsLane) {
  const std::size_t row = (2 * kL) + 3;
  EXPECT_EQ(
      (BlockedLayout::RowOffset<V3A, float>(row, NoRowTable{})),
      ((2 * kL * 3) + 3) * sizeof(float)
  );
}

// What makes in-place batches possible: rows written one scalar at a time
// read back as a lane group whose memory is exactly a Vector3 of batches,
// leaf after leaf.
TEST(BlockedLayoutMemory, ALaneGroupIsItsBatchedValueInMemory) {
  using B = xsimd::batch<float>;
  constexpr std::size_t kRows = 2 * kL;
  alignas(64) std::array<float, kRows * 3> block{};
  const std::size_t stride = BlockedLayout::StrideBytes<V3A>(kRows);
  auto* base = std::bit_cast<std::byte*>(block.data());
  for (std::size_t row = 0; row < kRows; ++row) {
    V3A::Write<float, BlockedLayout>(
        base + BlockedLayout::RowOffset<V3A, float>(row, NoRowTable{}),
        stride,
        Vector3<float>(
            static_cast<float>(row),
            10.0F + static_cast<float>(row),
            20.0F + static_cast<float>(row)
        )
    );
  }

  const auto* group1 = std::bit_cast<const Vector3<B>*>(
      base + BlockedLayout::RowOffset<V3A, float>(kL, NoRowTable{})
  );
  for (std::size_t lane = 0; lane < kL; ++lane) {
    EXPECT_FLOAT_EQ(group1->X().get(lane), static_cast<float>(kL + lane));
    EXPECT_FLOAT_EQ(
        group1->Z().get(lane), 20.0F + static_cast<float>(kL + lane)
    );
  }
}
