// Builds one field's block: carve, seed padding, populate, replicate.

#pragma once

#include <algorithm>
#include <array>
#include <bit>
#include <cassert>
#include <cstddef>
#include <initializer_list>
#include <memory>
#include <span>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "domain/archetype/archetype.hpp"
#include "domain/archetype/archetype_set.hpp"
#include "domain/layout/joint_layout.hpp"
#include "domain/topology/joint_topology.hpp"
#include "domain/topology/ordering_plan.hpp"
#include "domain/topology/pair_topology.hpp"
#include "domain/topology/segmented_topology.hpp"
#include "engine/algorithm.hpp"
#include "engine/allocation/allocation_maps.hpp"
#include "engine/allocation/plan_copy.hpp"
#include "engine/binding.hpp"
#include "engine/context/sim_context.hpp"
#include "engine/memory/arena.hpp"
#include "engine/spec/field_binding.hpp"
#include "engine/spec/field_contract.hpp"
#include "engine/spec/step_contract.hpp"
#include "engine/view/view.hpp"
#include "engine/view/view_factory.hpp"
#include "policy/buffering/base_buffering.hpp"
#include "policy/buffering/double_buffering.hpp"
#include "policy/buffering/single_buffering.hpp"
#include "policy/layout/base_layout.hpp"
#include "policy/layout/planar_layout.hpp"
#include "policy/layout/segmented_layout.hpp"
#include "policy/ordering/base_ordering.hpp"
#include "policy/ordering/linear_ordering.hpp"
#include "policy/ordering/list_ordering.hpp"
#include "policy/ordering/pair_ordering.hpp"
#include "policy/ordering/topological_ordering.hpp"
#include "util/tmp.hpp"

namespace achilles::engine::allocation {

using domain::archetype::Archetype;

namespace detail {

// Per-field operations, parameterized directly on EnumT/Traits/F (no
// deduction needed -- the caller already knows all three).
template <typename EnumT, template <EnumT> class Traits, EnumT F>
struct FieldBuilder {
  using FieldTraits = Traits<F>;
  using FieldAssembler = typename FieldTraits::Assembler;
  using FieldLayout = engine::FieldLayoutT<EnumT, Traits, F>;
  using FieldOrdering = engine::FieldOrderingT<EnumT, Traits, F>;
  using FieldIndexing = engine::FieldIndexingT<EnumT, Traits, F>;
  using FieldBuffering = engine::FieldBufferingT<EnumT, Traits, F>;

  static constexpr std::size_t LaneCount() {
    return FieldLayout::template LaneCount<FieldAssembler>();
  }
  static constexpr std::size_t Alignment() {
    return FieldLayout::template Alignment<FieldAssembler>();
  }
  // How many rows this field's block is sized by: its own ordering's
  // (OrderingPlan::ViewInstanceCount) -- every field sharing that ordering
  // has the same count, and a View holds one per ordering its fields use.
  static std::size_t RowCountFor(const PlanMap& plans) {
    return plans.at(engine::SlotId<FieldOrdering>()).ViewInstanceCount();
  }

  // How many elements this field's block holds for `row_count` rows
  // -- row_count itself for a row-indexed field, fewer or more for any
  // other index space (see topology/indexing_policy.hpp). Every
  // byte-size/stride computation below is taken over this, never over
  // row_count directly.
  static constexpr std::size_t Elements(std::size_t row_count) {
    return FieldIndexing::Count(row_count);
  }
  static std::size_t BlockBytes(
      std::size_t row_count, const std::vector<std::size_t>& segments
  ) {
    return FieldLayout::template BlockBytes<FieldAssembler>(
        Elements(row_count), RowTableOf(segments)
    );
  }
  // Every copy of the block this field's Buffering policy keeps (one for a
  // single-buffered field, see memory/buffering_policy.hpp) -- what Carve
  // actually allocates. SeedPadding/Populate only ever fill the first copy;
  // Replicate fans it out to the rest.
  static std::size_t RegionBytes(
      std::size_t row_count, const std::vector<std::size_t>& segments
  ) {
    return BlockBytes(row_count, segments) * FieldBuffering::kBlockCount;
  }

  // The RowTable a View over this field holds: a segmented field's segment
  // table, nothing for a planar one.
  static auto RowTableOf(const std::vector<std::size_t>& segments) {
    return FieldLayout::template RowTableOf<FieldAssembler>(segments);
  }

  // This field's slot in a View's RowTables (view::detail::RowTableSlot),
  // field index I.
  template <std::size_t I>
  static auto RowSlot(const std::vector<std::size_t>& segments) {
    using Table = decltype(RowTableOf(segments));
    return RowSlot<I, Table>(segments);
  }
  template <std::size_t I, typename Table>
    requires std::is_empty_v<Table>
  static view::detail::RowTableSlot<I, Table> RowSlot(
      const std::vector<std::size_t>& /*segments*/
  ) {
    return {};
  }
  template <std::size_t I, typename Table>
    requires(!std::is_empty_v<Table>)
  static view::detail::RowTableSlot<I, Table> RowSlot(
      const std::vector<std::size_t>& segments
  ) {
    return {RowTableOf(segments)};
  }

  // This field's own row order -- planned once per distinct policy by
  // SimAllocator (see PlanMap above), looked up here by the policy type
  // Traits<F>::Ordering itself resolves to. Every field sharing that same
  // policy type shares this exact row order; two fields with different
  // policies never do, even within the same algorithm.
  static domain::layout::JointLayout RowsFor(const PlanMap& plans) {
    return plans.at(engine::SlotId<FieldOrdering>()).Layout();
  }

  // The table this field's Layout places rows from, built from its own
  // plan's segment boundaries (policy::layout::SegmentedLayout) -- empty for
  // every other layout.
  static std::vector<std::size_t> SegmentsFor(const PlanMap& plans) {
    return FieldLayout::template Segments<FieldAssembler>(
        plans.at(engine::SlotId<FieldOrdering>()).Layout().InstanceOffsets()
    );
  }

  // A zeroed block, or null for an empty one (a segmented field whose
  // ordering has no rows) -- the Arena never hands out zero bytes, and
  // nothing addresses a block with no rows.
  static std::byte* AllocateZeroed(
      Arena& arena, std::size_t bytes, std::size_t alignment
  ) {
    if (bytes == 0) {
      return nullptr;
    }
    std::byte* base = arena.Allocate(bytes, alignment);
    arena.ZeroFill(base, bytes);
    return base;
  }

  // Carves (or, for a shared field already carved by an earlier algorithm,
  // reuses) this field's block. A reused slot is checked for an exact match
  // on shape -- two algorithms naming the same SharedAs tag with
  // incompatible field shapes is unsound and must fail loudly, not
  // silently alias mismatched memory.
  static std::byte* Carve(
      Arena& arena,
      SlotMap& slots,
      std::size_t row_count,
      const std::vector<std::size_t>& segments
  ) {
    std::size_t bytes = RegionBytes(row_count, segments);
    std::size_t alignment = Alignment();
    std::size_t lanes = LaneCount();
    const void* ordering_id = engine::SlotId<FieldOrdering>();
    const void* indexing_id = engine::SlotId<FieldIndexing>();
    const void* buffering_id =
        engine::SlotId<policy::buffering::BufferStorageT<FieldBuffering>>();

    if constexpr (engine::FieldIsShared<EnumT, Traits, F>) {
      using Tag = typename FieldTraits::SharedAs;
      const void* id = engine::SlotId<Tag>();
      auto it = slots.find(id);
      if (it != slots.end()) {
        const SlotRecord& rec = it->second;
        assert(
            rec.block_bytes == bytes && rec.alignment == alignment &&
            rec.lane_count == lanes && rec.ordering_id == ordering_id &&
            rec.indexing_id == indexing_id &&
            rec.buffering_id == buffering_id &&
            "Two algorithms name the same SharedAs tag with incompatible "
            "field shapes (bytes/alignment/lane count/ordering/indexing/"
            "buffering) -- "
            "sharing "
            "an array two algorithms disagree about the layout of is "
            "unsound."
        );
        return rec.base;
      }
      std::byte* base = AllocateZeroed(arena, bytes, alignment);
      slots.emplace(
          id,
          SlotRecord{
              base,
              bytes,
              alignment,
              lanes,
              ordering_id,
              indexing_id,
              buffering_id
          }
      );
      return base;
    } else {
      std::byte* base = AllocateZeroed(arena, bytes, alignment);
      return base;
    }
  }

  // Writes AssembledType::PaddingSeed() into every element of this field's
  // block that doesn't belong to a real (instance, joint) under its OWN
  // row order (RowsFor). *Which* elements those are is this field's own
  // Indexing policy's call (topology/indexing_policy.hpp) -- for a row-
  // indexed field, every padding row up to that order's own BaseRowIndex(),
  // then the lane-rounding filler past it up to row_count. No
  // ordering-specific
  // branch here: every ordering policy can produce padding rows (even
  // LinearOrdering, from lane-rounding an archetype's own instance count), so
  // every field gets seeded the same way regardless of which policy it named.
  static void SeedPadding(
      std::byte* base,
      std::size_t row_count,
      const domain::layout::JointLayout& rows
  )
    requires(!policy::layout::SegmentedLayoutLike<FieldLayout>)
  {
    using ScalarType = typename FieldAssembler::ScalarType;
    const assembly::AssembledT<FieldAssembler> seed =
        FieldAssembler::PaddingSeed();
    const std::size_t stride =
        FieldLayout::template StrideBytes<FieldAssembler>(Elements(row_count));
    const std::size_t element_stride =
        FieldLayout::template ElementStrideBytes<FieldAssembler>();

    auto write_element = [&](std::size_t element) {
      std::byte* element_ptr = base + element * element_stride;
      FieldAssembler::template Write<ScalarType, FieldLayout>(
          element_ptr, stride, seed
      );
    };

    FieldIndexing::SeedPadding(write_element, rows, row_count);
  }

  // A segmented field's padding rows sit in no segment and take no space
  // (see policy::layout::SegmentedLayout): nothing to seed.
  static void SeedPadding(
      std::byte* /*base*/,
      std::size_t /*row_count*/,
      const domain::layout::JointLayout& /*rows*/
  )
    requires policy::layout::SegmentedLayoutLike<FieldLayout>
  {}

  // Copies this field's data out of whichever of `archetypes` declares a
  // field named Traits<F>::kName, into the sorted rows this field's own
  // row order (RowsFor) assigned that archetype's instances -- a field
  // with no kName (an algorithm-internal scratch or output field, e.g.
  // ABA's kJointAcceleration) is never user-supplied, so this is a no-op
  // for it: Carve's own ZeroFill already leaves it deterministically zero,
  // and Op::Initialize/a real pass own filling it in at runtime.
  //
  // Writes raw scalars directly, one leaf at a time, rather than routing
  // through FieldAssembler::Write: doing so would require materializing a
  // real domain value (Transform, Inertia, ...) from the archetype's flat
  // per-leaf doubles first, and there's no generic way to do that -- a
  // domain type's own constructor shape varies per type.
  // domain::JointField's own contract (see archetype.hpp) is that
  // its leaves are already given in the bound field's Assembler's own
  // flattening order, which is exactly what makes writing each one straight to
  // the leaf the field's Layout puts it at (FieldLayout::LeafAt) correct.
  static void Populate(
      std::byte* base,
      std::size_t row_count,
      const domain::layout::JointLayout& rows,
      const domain::archetype::ArchetypeSet& archetypes
  ) {
    if constexpr (engine::FieldHasName<EnumT, Traits, F>) {
      static_assert(
          std::is_same_v<FieldIndexing, policy::indexing::RowIndexing>,
          "SimAllocator: a field with a kName must be row-indexed -- "
          "archetype data supplies one value per (instance, joint), and "
          "there's no row of any other index space to copy it into."
      );
      constexpr std::string_view kFieldName = Traits<F>::kName;
      constexpr std::size_t kLeafCount = FieldAssembler::kNumFields;
      using ScalarType = typename FieldAssembler::ScalarType;
      const std::size_t stride =
          FieldLayout::template StrideBytes<FieldAssembler>(row_count);
      const std::size_t element_stride =
          FieldLayout::template ElementStrideBytes<FieldAssembler>();

      archetypes.ForEachField(
          kFieldName,
          [&](const domain::layout::JointField& field,
              std::size_t first_instance,
              std::size_t instance_count,
              std::size_t joint_count) {
            assert(
                field.scalars_per_leaf == kLeafCount &&
                "SimAllocator: an archetype field's scalar count doesn't match "
                "the bound Assembler's own leaf count -- the archetype data "
                "and "
                "the Algorithm it's being populated against disagree about "
                "this field's shape."
            );

            for (std::size_t i = 0; i < instance_count; ++i) {
              for (std::size_t j = 0; j < joint_count; ++j) {
                std::size_t sorted_row = rows.ToSorted(first_instance + i, j);
                for (std::size_t leaf = 0; leaf < kLeafCount; ++leaf) {
                  std::byte* leaf_ptr = FieldLayout::LeafAt(
                      base + (sorted_row * element_stride), leaf, stride
                  );
                  *std::bit_cast<ScalarType*>(leaf_ptr) =
                      static_cast<ScalarType>(field.At(i, j, leaf, joint_count)
                      );
                }
              }
            }
          }
      );
    }
  }

  // Copies the seeded+populated first block into every other copy this
  // field's Buffering policy keeps -- a no-op for a single-buffered field.
  static void Replicate(
      std::byte* base,
      std::size_t row_count,
      const std::vector<std::size_t>& segments
  ) {
    const std::size_t block_bytes = BlockBytes(row_count, segments);
    if (block_bytes > 0) {
      FieldBuffering::Replicate(base, block_bytes);
    }
  }
};

}  // namespace detail

}  // namespace achilles::engine::allocation
