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

#include "domain/archetype.hpp"
#include "domain/joint_layout.hpp"
#include "domain/joint_topology.hpp"
#include "engine/algorithm_contract.hpp"
#include "engine/field_contract.hpp"
#include "engine/memory/arena.hpp"
#include "engine/memory/binding.hpp"
#include "engine/memory/buffering_policy.hpp"
#include "engine/pass/sim_context.hpp"
#include "engine/topology/layout_policy.hpp"
#include "engine/topology/ordering_plan.hpp"
#include "engine/topology/orderings.hpp"
#include "engine/view/view.hpp"
#include "engine/view/view_factory.hpp"
#include "util/tmp.hpp"

namespace achilles::engine::memory {

using domain::Archetype;
using domain::ArchetypeField;
using domain::ArchetypeFieldAt;
using domain::ArchetypeTreeStructure;

namespace detail {

// One OrderingPlan per distinct ordering policy actually named by some
// field of some hosted Algorithm, keyed by SlotId<Policy>() -- the same
// "distinct link-time address per type" trick SlotId already uses for
// SharedAs tags, reused here since an ordering policy is just another type
// identity that needs a runtime-comparable key. There is no single
// program-wide row order: a field's own row order is entirely a function
// of its own Traits<F>::Ordering (default topology::LinearOrdering), never
// of what any other field in the sim happens to want. Scratch: the plans
// die with the SimAllocator, after BuildTopologies has copied what persists
// into the Arena.
using PlanMap = std::unordered_map<const void*, topology::OrderingPlan>;

struct SlotRecord {
  std::byte* base = nullptr;
  std::size_t block_bytes = 0;
  std::size_t alignment = 0;
  std::size_t lane_count = 0;
  const void* ordering_id = nullptr;
  const void* indexing_id = nullptr;
  const void* buffering_id = nullptr;
};
using SlotMap = std::unordered_map<const void*, SlotRecord>;

// The parity a field's View reads: none for a single-buffered field, its
// group's for a double-buffered one -- looked up by group in the parities
// SimAllocator carved before any View (see pass::ParityTupleFor).
template <BufferingPolicyLike Buffering, typename ParityTuple>
  requires(!GroupBufferedLike<Buffering>)
const BufferParity* ParityFor(const ParityTuple& /*parities*/) {
  return nullptr;
}

template <GroupBufferedLike Buffering, typename ParityTuple>
const BufferParity* ParityFor(const ParityTuple& parities) {
  return std::get<pass::BufferParitySlot<typename Buffering::BufferGroup>>(
             parities
  )
      .parity;
}

// Per-field operations, parameterized directly on EnumT/Traits/F (no
// deduction needed -- the caller already knows all three).
template <typename EnumT, template <EnumT> class Traits, EnumT F>
struct FieldVisitor {
  using FieldTraits = Traits<F>;
  using FieldAssembler = typename FieldTraits::Assembler;
  using FieldLayout = FieldLayoutT<EnumT, Traits, F>;
  using FieldOrdering = FieldOrderingT<EnumT, Traits, F>;
  using FieldIndexing = FieldIndexingT<EnumT, Traits, F>;
  using FieldBuffering = FieldBufferingT<EnumT, Traits, F>;

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
    return plans.at(SlotId<FieldOrdering>()).ViewInstanceCount();
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
  static domain::JointLayout RowsFor(const PlanMap& plans) {
    return plans.at(SlotId<FieldOrdering>()).Layout();
  }

  // The table this field's Layout places rows from, built from its own
  // plan's segment boundaries (topology::SegmentedLayout) -- empty for
  // every other layout.
  static std::vector<std::size_t> SegmentsFor(const PlanMap& plans) {
    return FieldLayout::template Segments<FieldAssembler>(
        plans.at(SlotId<FieldOrdering>()).Layout().InstanceOffsets()
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
    const void* ordering_id = SlotId<FieldOrdering>();
    const void* indexing_id = SlotId<FieldIndexing>();
    const void* buffering_id = SlotId<BufferStorageT<FieldBuffering>>();

    if constexpr (FieldIsShared<EnumT, Traits, F>) {
      using Tag = typename FieldTraits::SharedAs;
      const void* id = SlotId<Tag>();
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
      std::byte* base, std::size_t row_count, const domain::JointLayout& rows
  )
    requires(!topology::SegmentedLayoutLike<FieldLayout>)
  {
    using ScalarType = typename FieldAssembler::ScalarType;
    const AssembledT<FieldAssembler> seed = FieldAssembler::PaddingSeed();
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
  // (see topology::SegmentedLayout): nothing to seed.
  static void SeedPadding(
      std::byte* /*base*/,
      std::size_t /*row_count*/,
      const domain::JointLayout& /*rows*/
  )
    requires topology::SegmentedLayoutLike<FieldLayout>
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
  // domain type's own constructor shape varies per type. ArchetypeField's
  // own contract (see archetype.hpp) is that its leaves are already given
  // in the bound field's Assembler's own flattening order, which is
  // exactly what makes writing each one straight to the leaf the field's
  // Layout puts it at (FieldLayout::LeafAt) correct.
  static void Populate(
      std::byte* base,
      std::size_t row_count,
      const domain::JointLayout& rows,
      std::span<const domain::Archetype> archetypes
  ) {
    if constexpr (FieldHasName<EnumT, Traits, F>) {
      static_assert(
          std::is_same_v<FieldIndexing, topology::RowIndexing>,
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

      // Global index of each archetype's first instance -- archetypes'
      // instances are numbered end to end, in archetype order.
      std::size_t first_instance = 0;
      for (const domain::Archetype& archetype : archetypes) {
        const std::size_t instance_count = archetype.InstanceCount();
        const domain::ArchetypeField* field = archetype.FindField(kFieldName);
        if (field == nullptr) {
          first_instance += instance_count;
          continue;
        }
        assert(
            field->scalars_per_leaf == kLeafCount &&
            "SimAllocator: an archetype field's scalar count doesn't match "
            "the bound Assembler's own leaf count -- the archetype data and "
            "the Algorithm it's being populated against disagree about "
            "this field's shape."
        );

        std::size_t joint_count = archetype.JointCount();
        for (std::size_t i = 0; i < instance_count; ++i) {
          for (std::size_t j = 0; j < joint_count; ++j) {
            std::size_t sorted_row = rows.ToSorted({first_instance + i, j});
            for (std::size_t leaf = 0; leaf < kLeafCount; ++leaf) {
              std::byte* leaf_ptr = FieldLayout::LeafAt(
                  base + (sorted_row * element_stride), leaf, stride
              );
              *std::bit_cast<ScalarType*>(leaf_ptr) = static_cast<ScalarType>(
                  ArchetypeFieldAt(*field, i, j, leaf, joint_count)
              );
            }
          }
        }
        first_instance += instance_count;
      }
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

// Same shared-slot deduplicate Carve performs for real, but pure counting: no
// Arena, no pointers, just how many bytes the real Carve pass will
// actually consume -- an exact (not
// upper-bound) budget, now that SimAllocator's Archetype-taking
// constructor knows real instance counts up front rather than being handed
// an arbitrary count by the caller. `counted` is shared across every
// algorithm being measured, so a SharedAs tag two algorithms both name is
// still only ever counted once, same as a real Carve pass would.
using MeasureSlotSet = std::unordered_set<const void*>;

template <typename EnumT, template <EnumT> class Traits, EnumT F>
std::size_t MeasureField(MeasureSlotSet& counted, const PlanMap& plans) {
  using V = FieldVisitor<EnumT, Traits, F>;
  std::size_t bytes =
      V::RegionBytes(V::RowCountFor(plans), V::SegmentsFor(plans));
  std::size_t alignment = V::Alignment();

  if constexpr (FieldIsShared<EnumT, Traits, F>) {
    using Tag = typename V::FieldTraits::SharedAs;
    if (!counted.insert(SlotId<Tag>()).second) {
      return 0;
    }
  }
  // Arena::Allocate rounds its cursor up to `alignment` before bumping, so
  // a block carved right after a smaller-aligned one can waste up to
  // alignment-1 bytes this exact per-field sum wouldn't otherwise account
  // for. Padding every counted block by that worst case keeps this a safe
  // budget (Arena::Allocate asserts rather than grows on overflow) instead
  // of one that could under-allocate by a few bytes of alignment slop.
  return bytes + (alignment - 1);
}

template <typename EnumT, template <EnumT> class Traits, std::size_t... Is>
std::size_t MeasureAlgorithm(
    MeasureSlotSet& counted, std::index_sequence<Is...>, const PlanMap& plans
) {
  return (
      MeasureField<EnumT, Traits, static_cast<EnumT>(Is)>(counted, plans) + ...
  );
}

// Plans one row order per distinct ordering policy in `List`, keyed the
// same way FieldVisitor::RowsFor looks them back up (SlotId<Policy>()).
template <typename List>
struct BuildPlanMap;

template <typename... Policies>
struct BuildPlanMap<util::TypeList<Policies...>> {
  static_assert(
      sizeof...(Policies) > 0,
      "SimAllocator: no ordering policy found across any hosted "
      "Algorithm's fields -- every FieldTraits must resolve to at least "
      "topology::LinearOrdering (the default)."
  );

  static PlanMap Build(
      std::span<domain::ArchetypeTreeStructure> tree, std::size_t lane_size
  ) {
    PlanMap plans;
    (plans.emplace(SlotId<Policies>(), Policies::Build(tree, lane_size)), ...);
    return plans;
  }
};

// Allocates one JointTopology and JointLayout per distinct ordering policy
// in `List` (the same List BuildPlanMap above was built from), wrapped into
// the exact tuple shape SimContext<Algorithms...> expects
// (pass::TopologyTupleFor, engine/pass/sim_context.hpp) rather than a second,
// independently-shaped tuple this file would otherwise have to keep
// hand-in-sync with it.
template <typename List>
struct BuildTopologyTuple;

template <typename... Policies>
struct BuildTopologyTuple<util::TypeList<Policies...>> {
  static std::tuple<pass::TopologySlot<Policies>...> Build(
      const PlanMap& plans, Arena& arena
  ) {
    return std::tuple<pass::TopologySlot<Policies>...>{
        MakeSlot<Policies>(plans.at(SlotId<Policies>()).Allocate(arena))...
    };
  }

 private:
  template <typename Policy>
    requires std::same_as<topology::TopologyOfT<Policy>, domain::JointTopology>
  static pass::TopologySlot<Policy> MakeSlot(
      topology::OrderingPlan::Allocated allocated
  ) {
    return {allocated.topology, allocated.layout};
  }

  template <typename Policy>
    requires std::
        same_as<topology::TopologyOfT<Policy>, domain::SegmentedTopology>
      static pass::TopologySlot<Policy> MakeSlot(
          topology::OrderingPlan::Allocated allocated
      ) {
    return {
        domain::SegmentedTopology(
            allocated.topology, allocated.layout.InstanceOffsets()
        ),
        allocated.layout
    };
  }

  template <typename Policy>
    requires std::same_as<topology::TopologyOfT<Policy>, domain::PairTopology>
  static pass::TopologySlot<Policy> MakeSlot(
      topology::OrderingPlan::Allocated allocated
  ) {
    return {
        domain::PairTopology(allocated.topology, allocated.partners),
        allocated.layout
    };
  }
};

// Recovers EnumT and the Traits template-template parameter from a concrete
// Algorithm<EnumT, Traits> via partial-specialization pattern matching --
// the same trick FixtureFor (tests/support/view_fixture.hpp) already
// uses to pick a resolved View type's Assemblers... back apart.
template <typename AlgorithmT>
struct AlgorithmVisitor;

template <typename EnumT, template <EnumT> class Traits>
struct AlgorithmVisitor<Algorithm<EnumT, Traits>> {
  using View = typename Algorithm<EnumT, Traits>::View;

  template <std::size_t... Is>
  static constexpr std::size_t MaxLaneCount(std::index_sequence<Is...>) {
    std::size_t result = 1;
    ((result = std::max(
          result,
          FieldVisitor<EnumT, Traits, static_cast<EnumT>(Is)>::LaneCount()
      )),
     ...);
    return result;
  }

  template <std::size_t... Is>
  static constexpr std::size_t MaxAlignment(std::index_sequence<Is...>) {
    std::size_t result = 1;
    ((result = std::max(
          result,
          FieldVisitor<EnumT, Traits, static_cast<EnumT>(Is)>::Alignment()
      )),
     ...);
    return result;
  }

  static constexpr std::size_t RequiredLaneSize() {
    return MaxLaneCount(
        std::make_index_sequence<static_cast<std::size_t>(EnumT::kCount)>{}
    );
  }
  static constexpr std::size_t RequiredAlignment() {
    return MaxAlignment(
        std::make_index_sequence<static_cast<std::size_t>(EnumT::kCount)>{}
    );
  }

  static std::size_t MeasureExact(
      MeasureSlotSet& counted, const PlanMap& plans
  ) {
    return MeasureAlgorithm<EnumT, Traits>(
        counted,
        std::make_index_sequence<static_cast<std::size_t>(EnumT::kCount)>{},
        plans
    );
  }

  // The View's row count per ordering its fields use (View::Orderings).
  template <typename... Orderings>
  static typename View::RowCounts RowCountsOf(
      util::TypeList<Orderings...> /*orderings*/, const PlanMap& plans
  ) {
    return {plans.at(SlotId<Orderings>()).ViewInstanceCount()...};
  }

  template <typename ParityTuple, std::size_t... Is>
  static View CarveView(
      std::index_sequence<Is...>,
      Arena& arena,
      SlotMap& slots,
      const PlanMap& plans,
      std::span<const domain::Archetype> archetypes,
      const ParityTuple& parities
  ) {
    const std::array<std::size_t, sizeof...(Is)> row_counts{
        FieldVisitor<EnumT, Traits, static_cast<EnumT>(Is)>::RowCountFor(plans
        )...
    };
    const std::array<std::vector<std::size_t>, sizeof...(Is)> segments{
        FieldVisitor<EnumT, Traits, static_cast<EnumT>(Is)>::SegmentsFor(plans
        )...
    };
    std::array<std::byte*, sizeof...(Is)> type_bases{
        FieldVisitor<EnumT, Traits, static_cast<EnumT>(Is)>::Carve(
            arena, slots, row_counts[Is], segments[Is]
        )...
    };
    // Each field's RowTable: a segmented field's slot is initialized from
    // its table; an empty one has nothing to initialize.
    typename View::RowTables row_tables{
        FieldVisitor<EnumT, Traits, static_cast<EnumT>(Is)>::template RowSlot<
            Is>(segments[Is])...
    };
    // Seed after every field's block exists, so a shared field carved once
    // by an earlier algorithm still gets seeded exactly once here too (a
    // second SeedPadding call over the same memory is idempotent -- it
    // rewrites the same seed values -- so this is safe even for shared
    // fields visited from more than one Algorithm).
    (FieldVisitor<EnumT, Traits, static_cast<EnumT>(Is)>::SeedPadding(
         type_bases[Is],
         row_counts[Is],
         FieldVisitor<EnumT, Traits, static_cast<EnumT>(Is)>::RowsFor(plans)
     ),
     ...);
    // Populate after seeding, so real archetype data always wins over a
    // padding seed for any row that happens to be both (never possible
    // today -- ToSorted only ever names a real row -- but keeping the
    // order this way costs nothing and stays correct if that ever
    // changes).
    (FieldVisitor<EnumT, Traits, static_cast<EnumT>(Is)>::Populate(
         type_bases[Is],
         row_counts[Is],
         FieldVisitor<EnumT, Traits, static_cast<EnumT>(Is)>::RowsFor(plans),
         archetypes
     ),
     ...);
    // Last, once the first block holds its final initial contents.
    (FieldVisitor<EnumT, Traits, static_cast<EnumT>(Is)>::Replicate(
         type_bases[Is], row_counts[Is], segments[Is]
     ),
     ...);
    std::array<const BufferParity*, sizeof...(Is)> field_parities{
        ParityFor<typename FieldVisitor<EnumT, Traits, static_cast<EnumT>(Is)>::
                      FieldBuffering>(parities)...
    };
    return View(
        type_bases,
        RowCountsOf(typename View::Orderings{}, plans),
        field_parities,
        row_tables
    );
  }

  template <typename ParityTuple>
  static View Carve(
      Arena& arena,
      SlotMap& slots,
      const PlanMap& plans,
      std::span<const domain::Archetype> archetypes,
      const ParityTuple& parities
  ) {
    return CarveView(
        std::make_index_sequence<static_cast<std::size_t>(EnumT::kCount)>{},
        arena,
        slots,
        plans,
        archetypes,
        parities
    );
  }
};

}  // namespace detail

// Owns the Arena for one sim. One JointTopology/JointLayout per distinct
// ordering policy any hosted field names; one View per hosted algorithm.
// Archetypes are runtime, supplied once at construction; to resize,
// destroy this SimAllocator and build a new one -- there's no in-place
// resize.
template <AlgorithmLike... Algorithms>
class SimAllocator {
  // Reuses SimContext's own computation (engine/pass/sim_context.hpp)
  // rather than keeping a second, independent copy here that this file's
  // BuildPlanMap/BuildTopologyTuple and SimContext's own topology tuple
  // could silently drift out of sync against.
  using UsedOrderings = pass::UsedOrderingsFor<Algorithms...>;

 public:
  struct State {
    engine::memory::Arena arena;
    pass::SimContext<Algorithms...> context;
  };

  // Builds every topology/layout/View from archetypes; archetypes need not
  // outlive this call.
  explicit SimAllocator(std::span<const domain::Archetype> archetypes)
      : plans_(BuildPlans(archetypes)),
        state_({{MeasureBytes(), RequiredAlignment()}, BuildContext(archetypes)}
        ) {}

  // Constructs and immediately extracts -- convenience for the common case.
  static State Build(const std::vector<domain::Archetype>& archetypes) {
    return SimAllocator(archetypes).Extract();
  }

  // Cheap fresh copy; doesn't borrow from this SimAllocator, only the Arena.
  pass::SimContext<Algorithms...> SimContext() const { return state_.context; }

  // Rvalue-qualified: can't Extract() a SimAllocator still meant for use.
  State Extract() && { return std::move(state_); }

  // Forwards to the built SimContext's own LayoutFor.
  template <topology::LayoutPolicyLike PolicyT>
  domain::JointLayout LayoutFor() const {
    return state_.context.template LayoutFor<PolicyT>();
  }

  // Forwards to the built SimContext's own TopologyFor.
  template <topology::LayoutPolicyLike PolicyT>
  topology::TopologyOfT<PolicyT> TopologyFor() const {
    return state_.context.template TopologyFor<PolicyT>();
  }

  // Forwards to the built SimContext's own ViewFor.
  template <AlgorithmLike AlgorithmT>
  typename AlgorithmT::View ViewFor() const {
    return state_.context.template ViewFor<AlgorithmT>();
  }

  // Largest lane size any hosted algorithm requires.
  static constexpr std::size_t RequiredLaneSize() {
    return std::max(
        {std::size_t{1},
         detail::AlgorithmVisitor<Algorithms>::RequiredLaneSize()...}
    );
  }

  // Largest alignment any hosted algorithm requires.
  static constexpr std::size_t RequiredAlignment() {
    return std::max(
        {std::size_t{1},
         detail::AlgorithmVisitor<Algorithms>::RequiredAlignment()...}
    );
  }

 private:
  // Plans one row order per distinct ordering policy named by any hosted
  // field.
  static detail::PlanMap BuildPlans(
      std::span<const domain::Archetype> archetypes
  ) {
    std::vector<domain::ArchetypeTreeStructure> tree(archetypes.size());
    for (std::size_t i = 0; i < archetypes.size(); ++i) {
      tree[i] = archetypes[i].TreeStructure();
    }
    return detail::BuildPlanMap<UsedOrderings>::Build(tree, RequiredLaneSize());
  }

  // Exact byte cost per field, deduped across shared slots.
  std::size_t MeasureBytes() const {
    detail::MeasureSlotSet counted;
    std::size_t total =
        (detail::AlgorithmVisitor<Algorithms>::MeasureExact(counted, plans_) +
         ...);
    for (const auto& [id, plan] : plans_) {
      total += plan.AllocatedBytes();
    }
    total += std::tuple_size_v<pass::ParityTupleFor<Algorithms...>> *
             (sizeof(BufferParity) + (alignof(BufferParity) - 1));
    return total;
  }

  // Everything SimContext holds, carved out of state_.arena (already
  // constructed by the time state_'s second member is -- see the
  // constructor): the topologies, one parity per buffer group, then every
  // View, each handed its fields' groups' parities.
  pass::SimContext<Algorithms...> BuildContext(
      std::span<const domain::Archetype> archetypes
  ) {
    auto topologies = BuildTopologies();
    auto parities = std::apply(
        [this](auto... slots) {
          return pass::ParityTupleFor<Algorithms...>{CarveParity(slots)...};
        },
        pass::ParityTupleFor<Algorithms...>{}
    );
    auto views = BuildViews(
        archetypes, parities, std::make_index_sequence<sizeof...(Algorithms)>{}
    );
    return pass::SimContext<Algorithms...>(
        std::move(topologies), std::move(views), parities
    );
  }

  // Value-initialized, so every group starts out reading half 0.
  template <typename Group>
  pass::BufferParitySlot<Group> CarveParity(
      pass::BufferParitySlot<Group> /*group*/
  ) {
    std::byte* bytes =
        state_.arena.Allocate(sizeof(BufferParity), alignof(BufferParity));
    return {std::construct_at(std::bit_cast<BufferParity*>(bytes))};
  }

  // Allocates one JointTopology and JointLayout per distinct ordering
  // policy in the Arena, wrapped into the exact tuple shape
  // SimContext<Algorithms...> expects.
  pass::TopologyTupleFor<Algorithms...> BuildTopologies() {
    return detail::BuildTopologyTuple<UsedOrderings>::Build(
        plans_, state_.arena
    );
  }

  // Pack order is left-to-right: first algorithm naming a SharedAs tag owns it.
  template <std::size_t... Is>
  std::tuple<typename Algorithms::View...> BuildViews(
      std::span<const domain::Archetype> archetypes,
      const pass::ParityTupleFor<Algorithms...>& parities,
      std::index_sequence<Is...> /*algorithms*/
  ) {
    detail::SlotMap slots;
    return std::tuple<typename Algorithms::View...>(
        detail::AlgorithmVisitor<Algorithms>::Carve(
            state_.arena, slots, plans_, archetypes, parities
        )...
    );
  }

  detail::PlanMap plans_;
  State state_;
};

// Builds SimAllocator<Algorithms...> from a util::TypeList<Algorithms...>
// instead of a caller restating the pack -- lets every hosted Algorithm be
// registered exactly once (see e.g. algorithms/registry.hpp) and reused
// here for the Sim type itself. Nothing downstream needs its own separate
// registration: engine::pass::Step's own Algorithms... pack is deduced from
// whichever SimAllocator it's given, so building the Sim type this way is
// the only place the registered list has to be named at all.
template <typename List>
struct SimAllocatorFor;

template <AlgorithmLike... Algorithms>
struct SimAllocatorFor<util::TypeList<Algorithms...>> {
  using Type = SimAllocator<Algorithms...>;
};

template <typename List>
using SimAllocatorForT = typename SimAllocatorFor<List>::Type;

}  // namespace achilles::engine::memory
