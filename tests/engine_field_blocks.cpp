#include <gtest/gtest.h>

#include <cstdint>
#include <string_view>
#include <type_traits>

#include "domain/math/vector3.hpp"
#include "engine/algorithm_contract.hpp"
#include "engine/field_blocks.hpp"
#include "engine/topology/layout_policy.hpp"
#include "util/tmp.hpp"

using achilles::domain::math::Vector3Assembler;
using achilles::engine::Algorithm;
using achilles::engine::BlockIndex;
using achilles::engine::BlockInfos;
using achilles::engine::FieldBlocksT;
using achilles::engine::FieldRef;
using achilles::engine::FindBlock;
using achilles::engine::topology::PlanarLayout;

namespace {

struct SharedSlot;

// Owner: one private named field, one shared named field, one private
// unnamed field.
enum class OwnerField : std::uint8_t { kPosition, kShared, kScratch, kCount };
template <OwnerField F>
struct OwnerTraits {
  using Assembler = Vector3Assembler<float>;
  using Layout = PlanarLayout;
};
template <>
struct OwnerTraits<OwnerField::kPosition> {
  using Assembler = Vector3Assembler<float>;
  using Layout = PlanarLayout;
  static constexpr std::string_view kName = "position";
};
template <>
struct OwnerTraits<OwnerField::kShared> {
  using Assembler = Vector3Assembler<float>;
  using Layout = PlanarLayout;
  using SharedAs = SharedSlot;
  static constexpr std::string_view kName = "shared";
};
using Owner = Algorithm<OwnerField, OwnerTraits>;

// Viewer: an unnamed view of the same shared block, plus its own field.
enum class ViewerField : std::uint8_t { kOwn, kShared, kCount };
template <ViewerField F>
struct ViewerTraits {
  using Assembler = Vector3Assembler<float>;
  using Layout = PlanarLayout;
};
template <>
struct ViewerTraits<ViewerField::kShared> {
  using Assembler = Vector3Assembler<float>;
  using Layout = PlanarLayout;
  using SharedAs = SharedSlot;
};
using Viewer = Algorithm<ViewerField, ViewerTraits>;

using Blocks = FieldBlocksT<Owner, Viewer>;

// Five fields across two Algorithms, but Viewer::kShared views Owner's
// block -- four distinct blocks.
static_assert(std::is_same_v<
              Blocks,
              achilles::util::TypeList<
                  achilles::engine::FieldBlock<
                      FieldRef<Owner, 0>,
                      achilles::util::TypeList<FieldRef<Owner, 0>>>,
                  achilles::engine::FieldBlock<
                      SharedSlot,
                      achilles::util::
                          TypeList<FieldRef<Owner, 1>, FieldRef<Viewer, 1>>>,
                  achilles::engine::FieldBlock<
                      FieldRef<Owner, 2>,
                      achilles::util::TypeList<FieldRef<Owner, 2>>>,
                  achilles::engine::FieldBlock<
                      FieldRef<Viewer, 0>,
                      achilles::util::TypeList<FieldRef<Viewer, 0>>>>>);

// Both views of the shared block map to the same block index.
static_assert(BlockIndex<FieldRef<Owner, 1>, Blocks>::kValue == 1);
static_assert(BlockIndex<FieldRef<Viewer, 1>, Blocks>::kValue == 1);
static_assert(BlockIndex<FieldRef<Viewer, 0>, Blocks>::kValue == 3);

// The unnamed view inherits the owner's name.
constexpr auto kInfos = BlockInfos<Blocks>::kValue;
static_assert(kInfos[1].name == "shared");
static_assert(kInfos[2].name.empty());
static_assert(FindBlock(kInfos, "position")->scalars == 3);
static_assert(!FindBlock(kInfos, "nope").has_value());

// FieldBlocksT itself static_asserts the block rules, so building
// Blocks above already proves this set satisfies them.

}  // namespace

// Everything above is compile-time; this keeps the file a runnable test
// target like every other tests/*.cpp.
TEST(FieldBlocks, CompileTimeChecksHold) { SUCCEED(); }
