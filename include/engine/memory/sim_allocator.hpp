#pragma once

#include <algorithm>
#include <array>
#include <bit>
#include <cassert>
#include <cstddef>
#include <initializer_list>
#include <span>
#include <string_view>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "domain/archetype.hpp"
#include "domain/joint_topology.hpp"
#include "engine/algorithm_contract.hpp"
#include "engine/field_blocks.hpp"
#include "engine/field_contract.hpp"
#include "engine/memory/arena.hpp"
#include "engine/memory/binding.hpp"
#include "engine/pass/sim_context.hpp"
#include "engine/topology/layout.hpp"
#include "engine/topology/layout_policy.hpp"
#include "engine/view/view.hpp"
#include "engine/view/view_factory.hpp"
#include "util/tmp.hpp"

namespace achilles::engine::memory {

using domain::Archetype;
using domain::ArchetypeField;
using domain::ArchetypeFieldAt;
using domain::ArchetypeTreeStructure;

namespace detail {

// One Layout per distinct ordering policy actually named by some field of
// some hosted Algorithm, keyed by SlotId<Policy>() -- the same "distinct
// link-time address per type" trick SlotId already uses for SharedAs tags,
// reused here since an ordering policy is just another type identity that
// needs a runtime-comparable key. There is no single program-wide Layout:
// a field's own row order is entirely a function of its own
// Traits<F>::Ordering (default topology::LinearOrdering), never of what
// any other field in the sim happens to want.
using LayoutMap = std::unordered_map<const void*, topology::Layout>;

// Per-block operations for one FieldBlock (engine/field_blocks.hpp) --
// one distinct block of field storage, however many hosted fields view it
// (several only for SharedAs fields). Its Primary member supplies the
// Assembler/Layout/Ordering types; FieldBlock's own rules guarantee every
// other member agrees with them.
template <typename BlockT>
struct BlockOps {
  using FieldAssembler = typename BlockT::Assembler;
  using FieldLayout = typename BlockT::Layout;
  using FieldOrdering = typename BlockT::Ordering;

  static constexpr std::size_t LaneCount() {
    return FieldLayout::template LaneCount<FieldAssembler>();
  }
  static constexpr std::size_t Alignment() {
    return FieldLayout::template Alignment<FieldAssembler>();
  }
  static std::size_t BlockBytes(std::size_t padded_instances) {
    return FieldLayout::template BlockBytes<FieldAssembler>(padded_instances);
  }

  // Exact Arena cost of this block. Arena::Allocate rounds its cursor up
  // to `alignment` before bumping, so a block carved right after a
  // smaller-aligned one can waste up to alignment-1 bytes; padding every
  // block by that worst case keeps the total a safe budget (Arena::Allocate
  // asserts rather than grows on overflow).
  static std::size_t MeasureBytes(std::size_t padded_instances) {
    return BlockBytes(padded_instances) + (Alignment() - 1);
  }

  // This block's own row order -- built once per distinct policy by
  // SimAllocator (see LayoutMap above), looked up by the policy type its
  // Ordering resolves to.
  static const topology::Layout& FieldLayoutFor(const LayoutMap& layouts) {
    return layouts.at(SlotId<FieldOrdering>());
  }

  // Allocates, zero-fills, pads and populates this block -- exactly once,
  // however many hosted fields view it.
  static std::byte* Carve(
      Arena& arena,
      std::size_t padded_instances,
      const LayoutMap& layouts,
      std::span<const domain::Archetype> archetypes
  ) {
    std::size_t bytes = BlockBytes(padded_instances);
    std::byte* base = arena.Allocate(bytes, Alignment());
    arena.ZeroFill(base, bytes);
    const topology::Layout& layout = FieldLayoutFor(layouts);
    SeedPadding(base, padded_instances, layout);
    // Populate after seeding, so real archetype data always wins over a
    // padding seed for any row that happens to be both (never possible
    // today -- ToSorted only ever names a real row -- but keeping the
    // order this way costs nothing and stays correct if that ever
    // changes).
    Populate(base, padded_instances, layout, archetypes);
    return base;
  }

  // Writes AssembledType::PaddingSeed() into every row of this block that
  // isn't a real (instance, joint) under its OWN Layout -- every padding
  // row up to that Layout's own BaseRowIndex(), then everything past it up
  // to padded_instances (the global max over every hosted field's own
  // Layout -- see SimAllocator::padded_instances_), which covers both this
  // block's own lane-rounding tail and the gap between its own row count
  // and whatever wider Layout some other field in the sim needed. No
  // policy-specific branch here: every ordering policy can produce padding
  // rows (even LinearOrdering, from lane-rounding an archetype's own
  // instance count), so every block gets seeded the same way regardless of
  // which policy it named. The reserved base row itself is deliberately
  // NOT touched here: it is seeded by the hosting algorithm's own
  // Op::Initialize with real physically meaningful state (e.g. gravity,
  // world velocity), not a generic safe-neutral value.
  static void SeedPadding(
      std::byte* base,
      std::size_t padded_instances,
      const topology::Layout& field_layout
  ) {
    using ScalarType = typename FieldAssembler::ScalarType;
    using AssembledScalar = decltype(FieldAssembler::template Read<ScalarType>(
        std::declval<const std::byte*>(), std::size_t{0}
    ));
    const AssembledScalar seed = AssembledScalar::PaddingSeed();
    const std::size_t stride =
        FieldLayout::template StrideBytes<FieldAssembler>(padded_instances);
    const std::size_t element_stride =
        FieldLayout::template ElementStrideBytes<FieldAssembler>();

    auto write_row = [&](std::size_t row) {
      std::byte* row_ptr = base + row * element_stride;
      FieldAssembler::template Write<ScalarType>(row_ptr, stride, seed);
    };

    for (std::size_t sorted = 0; sorted < field_layout.PaddedSize(); ++sorted) {
      if (field_layout.IsPadding(sorted)) {
        write_row(sorted);
      }
    }
    for (std::size_t row = field_layout.BaseRowIndex() + 1;
         row < padded_instances;
         ++row) {
      write_row(row);
    }
  }

  // Copies this block's data out of whichever of `archetypes` supplies a
  // field with the block's name, into the sorted rows its Layout assigned
  // that archetype's instances. A no-op for an unnamed block, or one no
  // archetype supplies: Carve's ZeroFill already leaves it
  // deterministically zero, and Op::Initialize/a real pass own filling it
  // in at runtime.
  //
  // Writes raw scalars directly, one leaf at a time, rather than routing
  // through FieldAssembler::Write: doing so would require materializing a
  // real domain value (Transform, Inertia, ...) from the archetype's flat
  // per-leaf doubles first, and there's no generic way to do that -- a
  // domain type's own constructor shape varies per type. ArchetypeField's
  // own contract (see archetype.hpp) is that its leaves are already given
  // in the bound field's Assembler's own flattening order, which is
  // exactly what makes writing them straight into the planar block, one
  // leaf-array at a time, correct.
  static void Populate(
      std::byte* base,
      std::size_t padded_instances,
      const topology::Layout& field_layout,
      std::span<const domain::Archetype> archetypes
  ) {
    if constexpr (!BlockT::kName.empty()) {
      constexpr std::string_view kFieldName = BlockT::kName;
      constexpr std::size_t kLeafCount = FieldAssembler::kNumFields;
      using ScalarType = typename FieldAssembler::ScalarType;
      const std::size_t stride =
          FieldLayout::template StrideBytes<FieldAssembler>(padded_instances);
      const std::size_t element_stride =
          FieldLayout::template ElementStrideBytes<FieldAssembler>();

      for (std::size_t a = 0; a < archetypes.size(); ++a) {
        const domain::Archetype& archetype = archetypes[a];
        const domain::ArchetypeField* field = archetype.FindField(kFieldName);
        if (field == nullptr) {
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
        std::size_t instance_count = archetype.InstanceCount();
        for (std::size_t i = 0; i < instance_count; ++i) {
          for (std::size_t j = 0; j < joint_count; ++j) {
            std::size_t sorted_row =
                field_layout.ToSorted(a, i * joint_count + j);
            for (std::size_t leaf = 0; leaf < kLeafCount; ++leaf) {
              std::byte* leaf_ptr =
                  base + (leaf * stride) + (sorted_row * element_stride);
              *std::bit_cast<ScalarType*>(leaf_ptr) = static_cast<ScalarType>(
                  ArchetypeFieldAt(*field, i, j, leaf, joint_count)
              );
            }
          }
        }
      }
    }
  }
};

// Builds one hosted Algorithm's View over already-carved blocks: field I's
// base pointer is whichever block FieldRef<AlgorithmT, I> views
// (BlockIndex), so a SharedAs field resolves to the same block in every
// Algorithm that names its tag.
template <typename AlgorithmT, typename BlockList, std::size_t NumBlocks>
struct ViewBuilder {
  template <std::size_t... Is>
  static typename AlgorithmT::View
  Build(const std::array<std::byte*, NumBlocks>& blocks, std::size_t padded_instances, std::index_sequence<Is...>) {
    std::array<std::byte*, sizeof...(Is)> type_bases{
        blocks[BlockIndex<FieldRef<AlgorithmT, Is>, BlockList>::kValue]...
    };
    return typename AlgorithmT::View(type_bases, padded_instances);
  }

  static typename AlgorithmT::View Build(
      const std::array<std::byte*, NumBlocks>& blocks,
      std::size_t padded_instances
  ) {
    return Build(
        blocks,
        padded_instances,
        std::make_index_sequence<
            static_cast<std::size_t>(AlgorithmT::Enum::kCount)>{}
    );
  }
};

// Builds one Layout per distinct ordering policy in `List`, keyed the same
// way BlockOps::FieldLayoutFor looks them back up (SlotId<Policy>()).
template <typename List>
struct BuildLayoutMap;

template <typename... Policies>
struct BuildLayoutMap<util::TypeList<Policies...>> {
  static_assert(
      sizeof...(Policies) > 0,
      "SimAllocator: no ordering policy found across any hosted "
      "Algorithm's fields -- every FieldTraits must resolve to at least "
      "topology::LinearOrdering (the default)."
  );

  static LayoutMap Build(
      std::span<domain::ArchetypeTreeStructure> tree, std::size_t lane_size
  ) {
    LayoutMap layouts;
    (layouts.emplace(SlotId<Policies>(), Policies::Build(tree, lane_size)),
     ...);
    return layouts;
  }
};

// Allocates one JointTopology per distinct ordering policy in `List` (the
// same List BuildLayoutMap above was built from), wrapped into the exact
// tuple shape SimContext<Algorithms...> expects (pass::TopologyTupleFor,
// engine/pass/sim_context.hpp) rather than a second, independently-shaped
// tuple this file would otherwise have to keep hand-in-sync with it.
template <typename List>
struct BuildTopologyTuple;

template <typename... Policies>
struct BuildTopologyTuple<util::TypeList<Policies...>> {
  static std::tuple<pass::TopologySlot<Policies>...> Build(
      LayoutMap& layouts, Arena& arena
  ) {
    return std::tuple<pass::TopologySlot<Policies>...>{
        pass::TopologySlot<Policies>{
            layouts.at(SlotId<Policies>()).AllocateTopology(arena),
            layouts.at(SlotId<Policies>())
        }...
    };
  }
};

// The one place every hosted field's block size still agrees. Each field's
// own Layout may (in principle -- see Ordering::Consolidate for why every
// policy actually produces the same row count against the same
// archetypes/lane_size today) disagree on row order, but View still
// needs one uniform instance count for a whole View's array length. Taking
// the max over every distinct policy's own Layout is always safe: a field
// whose own Layout needs fewer rows than this just has its own excess rows
// treated the same as any other padding/filler (see SeedPadding).
inline std::size_t MaxViewInstanceCount(const LayoutMap& layouts) {
  std::size_t result = 0;
  for (const auto& [id, layout] : layouts) {
    result = std::max(result, layout.ViewInstanceCount());
  }
  return result;
}

}  // namespace detail

// Owns the Arena for one sim. One Layout/JointTopology per distinct
// ordering policy any hosted field names; one View per hosted algorithm.
// Archetypes are runtime, supplied once at construction; to resize,
// destroy this SimAllocator and build a new one -- there's no in-place
// resize.
template <AlgorithmLike... Algorithms>
class SimAllocator {
  // Reuses SimContext's own computation (engine/pass/sim_context.hpp)
  // rather than keeping a second, independent copy here that this file's
  // BuildLayoutMap/BuildTopologyTuple and SimContext's own topology tuple
  // could silently drift out of sync against.
  using UsedOrderings = pass::UsedOrderingsFor<Algorithms...>;

  // Every distinct block of field storage across Algorithms... -- what
  // gets carved, padded and populated, once each. Building it enforces
  // the block rules (see engine/field_blocks.hpp).
  using Blocks = FieldBlocksT<Algorithms...>;

 public:
  struct State {
    engine::memory::Arena arena;
    pass::SimContext<Algorithms...> context;
  };

  // Builds every Layout/View from archetypes; archetypes need not outlive this
  // call.
  explicit SimAllocator(std::span<const domain::Archetype> archetypes)
      : layouts_(BuildLayouts(archetypes)),
        padded_instances_(detail::MaxViewInstanceCount(layouts_)),
        state_(
            {{MeasureBytes(), RequiredAlignment()},
             {BuildTopologies(),
              BuildViews(
                  archetypes, std::make_index_sequence<sizeof...(Algorithms)>{}
              )}}
        ) {}

  // Constructs and immediately extracts -- convenience for the common case.
  static State Build(const std::vector<domain::Archetype>& archetypes) {
    return SimAllocator(archetypes).Extract();
  }

  // Cheap fresh copy; doesn't borrow from this SimAllocator, only the Arena.
  pass::SimContext<Algorithms...> SimContext() const { return state_.context; }

  // Rvalue-qualified: can't Extract() a SimAllocator still meant for use.
  State Extract() && { return std::move(state_); }

  // Kept queryable past construction for tests (row placement checks).
  template <topology::LayoutPolicyLike PolicyT>
  topology::Layout LayoutFor() const {
    return layouts_.at(SlotId<PolicyT>());
  }

  // Forwards to the built SimContext's own TopologyFor.
  template <topology::LayoutPolicyLike PolicyT>
  domain::JointTopology TopologyFor() const {
    return state_.context.template TopologyFor<PolicyT>();
  }

  // Forwards to the built SimContext's own ViewFor.
  template <AlgorithmLike AlgorithmT>
  typename AlgorithmT::View ViewFor() const {
    return state_.context.template ViewFor<AlgorithmT>();
  }

  // Largest lane size any hosted field requires.
  static constexpr std::size_t RequiredLaneSize() {
    return MaxOverBlocks(Blocks{}, [](auto ops) { return ops.LaneCount(); });
  }

  // Largest alignment any hosted field requires.
  static constexpr std::size_t RequiredAlignment() {
    return MaxOverBlocks(Blocks{}, [](auto ops) { return ops.Alignment(); });
  }

 private:
  // Builds one Layout per distinct ordering policy named by any hosted field.
  static detail::LayoutMap BuildLayouts(
      std::span<const domain::Archetype> archetypes
  ) {
    std::vector<domain::ArchetypeTreeStructure> tree(archetypes.size());
    for (std::size_t i = 0; i < archetypes.size(); ++i) {
      tree[i] = archetypes[i].TreeStructure();
    }
    return detail::BuildLayoutMap<UsedOrderings>::Build(
        tree, RequiredLaneSize()
    );
  }

  template <typename... BlockTs, typename Fn>
  static constexpr std::size_t MaxOverBlocks(
      util::TypeList<BlockTs...> /*blocks*/, Fn fn
  ) {
    return std::max({std::size_t{1}, fn(detail::BlockOps<BlockTs>{})...});
  }

  // Exact byte cost: every distinct block once, plus every topology.
  std::size_t MeasureBytes() const {
    std::size_t total = MeasureBlocks(Blocks{});
    for (const auto& [id, layout] : layouts_) {
      total += layout.TopologyBytes() + (alignof(std::size_t) - 1);
    }
    return total;
  }

  template <typename... BlockTs>
  std::size_t MeasureBlocks(util::TypeList<BlockTs...> /*blocks*/
  ) const {
    return (
        std::size_t{0} + ... +
        detail::BlockOps<BlockTs>::MeasureBytes(padded_instances_)
    );
  }

  // Allocates one JointTopology per distinct ordering policy in the Arena,
  // wrapped into the exact tuple shape SimContext<Algorithms...> expects.
  pass::TopologyTupleFor<Algorithms...> BuildTopologies() {
    return detail::BuildTopologyTuple<UsedOrderings>::Build(
        layouts_, state_.arena
    );
  }

  // Carves every distinct block once, then hands each Algorithm a View
  // whose fields point at the blocks they name.
  template <std::size_t... Is>
  std::tuple<typename Algorithms::View...>
  BuildViews(std::span<const domain::Archetype> archetypes, std::index_sequence<Is...>) {
    auto blocks = CarveBlocks(archetypes, Blocks{});
    return std::tuple<typename Algorithms::View...>(
        detail::ViewBuilder<Algorithms, Blocks, blocks.size()>::Build(
            blocks, padded_instances_
        )...
    );
  }

  template <typename... BlockTs>
  std::array<std::byte*, sizeof...(BlockTs)> CarveBlocks(
      std::span<const domain::Archetype> archetypes,
      util::TypeList<BlockTs...> /*blocks*/
  ) {
    // Braced init: evaluated left to right, so blocks are carved in
    // FieldBlocksT order.
    return {detail::BlockOps<BlockTs>::Carve(
        state_.arena, padded_instances_, layouts_, archetypes
    )...};
  }

  detail::LayoutMap layouts_;
  std::size_t padded_instances_;
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
