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
#include <unordered_set>
#include <utility>
#include <vector>

#include "domain/archetype.hpp"
#include "domain/joint_topology.hpp"
#include "engine/algorithm_contract.hpp"
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

struct SlotRecord {
  std::byte* base = nullptr;
  std::size_t block_bytes = 0;
  std::size_t alignment = 0;
  std::size_t lane_count = 0;
  const void* ordering_id = nullptr;
};
using SlotMap = std::unordered_map<const void*, SlotRecord>;

// Per-field operations, parameterized directly on EnumT/Traits/F (no
// deduction needed -- the caller already knows all three).
template <typename EnumT, template <EnumT> class Traits, EnumT F>
struct FieldVisitor {
  using FieldTraits = Traits<F>;
  using FieldAssembler = typename FieldTraits::Assembler;
  using FieldLayout = FieldLayoutT<EnumT, Traits, F>;
  using FieldOrdering = FieldOrderingT<EnumT, Traits, F>;

  static constexpr std::size_t LaneCount() {
    return FieldLayout::template LaneCount<FieldAssembler>();
  }
  static constexpr std::size_t Alignment() {
    return FieldLayout::template Alignment<FieldAssembler>();
  }
  static std::size_t BlockBytes(std::size_t padded_instances) {
    return FieldLayout::template BlockBytes<FieldAssembler>(padded_instances);
  }

  // This field's own row order -- built once per distinct policy by
  // SimAllocator (see LayoutMap above), looked up here by the policy type
  // Traits<F>::Ordering itself resolves to. Every field sharing that same
  // policy type shares this exact Layout; two fields with different
  // policies never do, even within the same algorithm.
  static const topology::Layout& FieldLayoutFor(const LayoutMap& layouts) {
    return layouts.at(SlotId<FieldOrdering>());
  }

  // Carves (or, for a shared field already carved by an earlier algorithm,
  // reuses) this field's block. A reused slot is checked for an exact match
  // on shape -- two algorithms naming the same SharedAs tag with
  // incompatible field shapes is unsound and must fail loudly, not
  // silently alias mismatched memory.
  static std::byte* Carve(
      Arena& arena, SlotMap& slots, std::size_t padded_instances
  ) {
    std::size_t bytes = BlockBytes(padded_instances);
    std::size_t alignment = Alignment();
    std::size_t lanes = LaneCount();
    const void* ordering_id = SlotId<FieldOrdering>();

    if constexpr (FieldIsShared<EnumT, Traits, F>) {
      using Tag = typename FieldTraits::SharedAs;
      const void* id = SlotId<Tag>();
      auto it = slots.find(id);
      if (it != slots.end()) {
        const SlotRecord& rec = it->second;
        assert(
            rec.block_bytes == bytes && rec.alignment == alignment &&
            rec.lane_count == lanes && rec.ordering_id == ordering_id &&
            "Two algorithms name the same SharedAs tag with incompatible "
            "field shapes (bytes/alignment/lane count/ordering) -- sharing "
            "an array two algorithms disagree about the layout of is "
            "unsound."
        );
        return rec.base;
      }
      std::byte* base = arena.Allocate(bytes, alignment);
      arena.ZeroFill(base, bytes);
      slots.emplace(id, SlotRecord{base, bytes, alignment, lanes, ordering_id});
      return base;
    } else {
      std::byte* base = arena.Allocate(bytes, alignment);
      arena.ZeroFill(base, bytes);
      return base;
    }
  }

  // Writes AssembledType::PaddingSeed() into every row of this field's
  // block that isn't a real (instance, joint) under its OWN Layout
  // (FieldLayoutFor) -- every padding row up to that Layout's own
  // BaseRowIndex(), then everything past it up to padded_instances (the
  // global max over every hosted field's own Layout -- see
  // SimAllocator::padded_instances_), which covers both this field's own
  // lane-rounding tail and the gap between its own row count and whatever
  // wider Layout some other field in the sim needed. No policy-specific
  // branch here: every ordering policy can produce padding rows (even
  // LinearOrdering, from lane-rounding an archetype's own instance count),
  // so every field gets seeded the same way regardless of which policy it
  // named. The reserved base row itself is deliberately NOT touched here:
  // it is seeded by the hosting algorithm's own Op::Initialize with real
  // physically meaningful state (e.g. gravity, world velocity), not a
  // generic safe-neutral value.
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

  // Copies this field's data out of whichever of `archetypes` declares a
  // field named Traits<F>::kName, into the sorted rows this field's own
  // Layout (FieldLayoutFor) assigned that archetype's instances -- a field
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
  // exactly what makes writing them straight into the planar block, one
  // leaf-array at a time, correct.
  static void Populate(
      std::byte* base,
      std::size_t padded_instances,
      const topology::Layout& field_layout,
      std::span<const domain::Archetype> archetypes
  ) {
    if constexpr (FieldHasName<EnumT, Traits, F>) {
      constexpr std::string_view kFieldName = Traits<F>::kName;
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

// Same shared-slot deduplicate Carve performs for real, but pure counting: no
// Arena, no pointers, just how many bytes the real Carve pass will
// actually consume for a given padded_instances -- an exact (not
// upper-bound) budget, now that SimAllocator's Archetype-taking
// constructor knows real instance counts up front rather than being handed
// an arbitrary count by the caller. `counted` is shared across every
// algorithm being measured, so a SharedAs tag two algorithms both name is
// still only ever counted once, same as a real Carve pass would.
using MeasureSlotSet = std::unordered_set<const void*>;

template <typename EnumT, template <EnumT> class Traits, EnumT F>
std::size_t MeasureField(
    MeasureSlotSet& counted, std::size_t padded_instances
) {
  using V = FieldVisitor<EnumT, Traits, F>;
  std::size_t bytes = V::BlockBytes(padded_instances);
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
    MeasureSlotSet& counted,
    std::index_sequence<Is...>,
    std::size_t padded_instances
) {
  return (
      MeasureField<EnumT, Traits, static_cast<EnumT>(Is)>(
          counted, padded_instances
      ) +
      ...
  );
}

// Collects FieldOrderingT<EnumT, Traits, F> for every field of one
// Algorithm into a TypeList -- SimAllocator concatenates and de-dups these
// across its whole Algorithms... pack to find every distinct ordering
// policy actually in play (see SimAllocator::UsedOrderings).
template <typename EnumT, template <EnumT> class Traits, std::size_t... Is>
auto CollectFieldOrderings(std::index_sequence<Is...>)
    -> util::TypeList<FieldOrderingT<EnumT, Traits, static_cast<EnumT>(Is)>...>;

template <typename AlgorithmT>
struct AlgorithmOrderings;

template <typename EnumT, template <EnumT> class Traits, typename StepT>
struct AlgorithmOrderings<Algorithm<EnumT, Traits, StepT>> {
  using Type = decltype(CollectFieldOrderings<EnumT, Traits>(
      std::make_index_sequence<static_cast<std::size_t>(EnumT::kCount)>{}
  ));
};

// Builds one Layout per distinct ordering policy in `List`, keyed the same
// way FieldVisitor::FieldLayoutFor looks them back up (SlotId<Policy>()).
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

// Recovers EnumT and the Traits template-template parameter from a concrete
// Algorithm<EnumT, Traits> via partial-specialization pattern matching --
// the same trick FixtureFor (tests/support/view_fixture.hpp) already
// uses to pick a resolved View type's Assemblers... back apart.
template <typename AlgorithmT>
struct AlgorithmVisitor;

template <typename EnumT, template <EnumT> class Traits, typename StepT>
struct AlgorithmVisitor<Algorithm<EnumT, Traits, StepT>> {
  using View = typename Algorithm<EnumT, Traits, StepT>::View;

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
      MeasureSlotSet& counted, std::size_t padded_instances
  ) {
    return MeasureAlgorithm<EnumT, Traits>(
        counted,
        std::make_index_sequence<static_cast<std::size_t>(EnumT::kCount)>{},
        padded_instances
    );
  }

  template <std::size_t... Is>
  static View CarveView(
      std::index_sequence<Is...>,
      Arena& arena,
      SlotMap& slots,
      std::size_t padded_instances,
      const LayoutMap& layouts,
      std::span<const domain::Archetype> archetypes
  ) {
    std::array<std::byte*, sizeof...(Is)> type_bases{
        FieldVisitor<EnumT, Traits, static_cast<EnumT>(Is)>::Carve(
            arena, slots, padded_instances
        )...
    };
    // Seed after every field's block exists, so a shared field carved once
    // by an earlier algorithm still gets seeded exactly once here too (a
    // second SeedPadding call over the same memory is idempotent -- it
    // rewrites the same seed values -- so this is safe even for shared
    // fields visited from more than one Algorithm).
    (FieldVisitor<EnumT, Traits, static_cast<EnumT>(Is)>::SeedPadding(
         type_bases[Is],
         padded_instances,
         FieldVisitor<EnumT, Traits, static_cast<EnumT>(Is)>::FieldLayoutFor(
             layouts
         )
     ),
     ...);
    // Populate after seeding, so real archetype data always wins over a
    // padding seed for any row that happens to be both (never possible
    // today -- ToSorted only ever names a real row -- but keeping the
    // order this way costs nothing and stays correct if that ever
    // changes).
    (FieldVisitor<EnumT, Traits, static_cast<EnumT>(Is)>::Populate(
         type_bases[Is],
         padded_instances,
         FieldVisitor<EnumT, Traits, static_cast<EnumT>(Is)>::FieldLayoutFor(
             layouts
         ),
         archetypes
     ),
     ...);
    return View(type_bases, padded_instances);
  }

  static View Carve(
      Arena& arena,
      SlotMap& slots,
      std::size_t padded_instances,
      const LayoutMap& layouts,
      std::span<const domain::Archetype> archetypes
  ) {
    return CarveView(
        std::make_index_sequence<static_cast<std::size_t>(EnumT::kCount)>{},
        arena,
        slots,
        padded_instances,
        layouts,
        archetypes
    );
  }
};

}  // namespace detail

// Owns the Arena for one sim. One Layout/JointTopology per distinct
// ordering policy any hosted field names; one View per hosted algorithm.
// Archetypes are runtime, supplied once at construction; to resize,
// destroy this SimAllocator and build a new one -- there's no in-place
// resize.
template <AlgorithmLike... Algorithms>
class SimAllocator {
  using UsedOrderings = util::UniqueT<
      util::ConcatT<typename detail::AlgorithmOrderings<Algorithms>::Type...>>;

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

  // Exact byte cost per field, deduped across shared slots.
  std::size_t MeasureBytes() const {
    detail::MeasureSlotSet counted;
    std::size_t total =
        (detail::AlgorithmVisitor<Algorithms>::MeasureExact(
             counted, padded_instances_
         ) +
         ...);
    for (const auto& [id, layout] : layouts_) {
      total += layout.TopologyBytes() + (alignof(std::size_t) - 1);
    }
    return total;
  }

  // Allocates one JointTopology per distinct ordering policy in the Arena.
  pass::TopologyMap BuildTopologies() {
    pass::TopologyMap topologies;
    for (auto& [id, layout] : layouts_) {
      topologies.emplace(id, layout.AllocateTopology(state_.arena));
    }
    return topologies;
  }

  // Pack order is left-to-right: first algorithm naming a SharedAs tag owns it.
  template <std::size_t... Is>
  std::tuple<typename Algorithms::View...>
  BuildViews(std::span<const domain::Archetype> archetypes, std::index_sequence<Is...>) {
    detail::SlotMap slots;
    return std::tuple<typename Algorithms::View...>(
        detail::AlgorithmVisitor<Algorithms>::Carve(
            state_.arena, slots, padded_instances_, layouts_, archetypes
        )...
    );
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
