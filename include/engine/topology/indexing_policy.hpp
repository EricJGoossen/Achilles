#pragma once

#include <concepts>
#include <cstddef>
#include <tuple>
#include <type_traits>

#include "domain/joint_layout.hpp"

namespace achilles::engine::topology {

// A traversal's source index (see engine/pass/traversals.hpp) when it names
// one raw row to be splatted across every lane, rather than a batch group to
// gather lane-for-lane. A plain size_t source index (a TreeTraversal's
// parent group, a LinearTraversal's own target again) means "read the group
// at this index"; a BroadcastIndex means "read this one row, and hand every
// lane of the batch the same value" -- what a PairTraversal hands every
// row-indexed field it pairs against a whole lane group of targets.
//
// A distinct type rather than a flag, so which load a source argument gets
// is settled entirely at compile time by which traversal drove it: a field
// read through a BroadcastIndex resolves to FieldCursor's broadcasting Load
// overload (engine/view/view.hpp). One written through it is a reduction
// instead: storing a whole batch into one row has exactly one meaning that
// isn't a last-lane-wins race -- summing the lanes into it -- so such an
// output is handed to its Op zeroed and stored as row += (sum of its
// lanes), FieldCursor's accumulating Store overload. That's how a pair pass
// adds what each lane group contributes to its broadcast source row, or any
// pass adds into a GlobalIndexing field below (a lane-group reduction on a
// CPU, an atomic or segmented one on a GPU).
struct BroadcastIndex {
  std::size_t row;
};

// The index-space seam: which instances a field holds one value per, and
// how a traversal's (target, source) index pair resolves to the one element
// a given argument reads/writes. A stateless policy type, mirroring
// PlanarLayout (layout_policy.hpp) and the ordering policies
// (orderings.hpp): a new index space is a new policy struct here, not
// a new branch inside View, OpInvoker, or SimAllocator.
//
//   - Count(rows): how many elements this field's block holds, given the
//     View's own row count -- what every byte-size/stride computation for
//     the field is taken over in place of `rows` itself.
//   - Resolve<Slot, Lanes>(indices, rows): the index to Load/Store at, for
//     an argument naming position Slot (engine::ArgIndex) of `indices` --
//     the tuple of every index the traversal handed, target first -- read
//     as a T `Lanes` wide. The target is always a group index (in units of
//     Lanes); the source is whatever the traversal handed in its place.
//   - SeedPadding(write, layout, rows): calls write(element) for every
//     element that doesn't belong to a real (instance, joint) under the
//     field's own JointLayout -- see SimAllocator's padding-seed pass
//     (engine/memory/sim_allocator.hpp), which owns everything about the
//     write itself but defers *which* elements are padding to here.
template <typename P>
concept IndexingPolicyLike = std::is_empty_v<P> && requires(std::size_t rows) {
  { P::Count(rows) } -> std::same_as<std::size_t>;
};

// One element per row -- every field that existed before index spaces did,
// and still the default (see memory::FieldIndexingT, binding.hpp). An
// argument reads whichever index its Slot names, unchanged: the target
// group, a TreeTraversal's parent group, a LinearTraversal's own row again,
// or a PairTraversal's BroadcastIndex.
struct RowIndexing {
  static constexpr std::size_t Count(std::size_t rows) { return rows; }

  template <std::size_t Slot, std::size_t Lanes, typename Indices>
  static constexpr auto Resolve(const Indices& indices, std::size_t rows) {
    (void)rows;
    return std::get<Slot>(indices);
  }

  // Every padding row up to the JointLayout's own BaseRowIndex(), then every
  // filler row past it. The reserved base row itself is deliberately left
  // alone: it's seeded by the hosting algorithm's own Op::Initialize with
  // real, physically meaningful state (gravity, world velocity, ...), not a
  // generic safe-neutral value.
  template <typename WriteElement>
  static void SeedPadding(
      const WriteElement& write,
      const domain::JointLayout& layout,
      std::size_t rows
  ) {
    for (std::size_t sorted = 0; sorted < layout.PaddedSize(); ++sorted) {
      if (layout.IsPadding(sorted)) {
        write(sorted);
      }
    }
    for (std::size_t row = layout.BaseRowIndex() + 1; row < rows; ++row) {
      write(row);
    }
  }
};
static_assert(IndexingPolicyLike<RowIndexing>);

// One element per lane group of rows -- a fact about a whole group at once,
// e.g. a Lanes-bit domain::math::ActivationMask with bit t set for row t of
// the group (a compare's movemask, a GPU warp ballot), such as which of a
// pair ordering's lane group of pairs overlap. Read/written as the field's
// plain scalar at the target group, in place of a batch of the group's rows.
// Its padding is whatever a pass writes for a group's padding rows: nothing
// to seed.
template <std::size_t Lanes>
  requires(Lanes > 0)
struct LaneGroupIndexing {
  static constexpr std::size_t Count(std::size_t rows) { return rows / Lanes; }

  template <std::size_t Slot, std::size_t LoadLanes>
    requires(LoadLanes == 1)
  static constexpr std::size_t
  Resolve(const auto& indices, std::size_t /*rows*/) {
    return std::get<Slot>(indices);
  }

  template <typename WriteElement>
  static void SeedPadding(
      const WriteElement& /*write*/,
      const domain::JointLayout& /*layout*/,
      std::size_t /*rows*/
  ) {}
};
static_assert(IndexingPolicyLike<LaneGroupIndexing<4>>);

// One element for the whole field, whatever the row count -- a quantity the
// whole sim shares, e.g. a dot product summed over every row, or a step
// length an iterative solver picks from it. Every index resolves to that
// one element, by load width:
//
//   - a scalar load (LoadLanes = 1) reads/writes it plainly at index 0 --
//     what an Op driven by a GlobalTraversal (engine/pass/traversals.hpp)
//     works on, once per pass, to update the value itself;
//   - a batched load resolves to BroadcastIndex{0}: as an input every lane
//     reads the one value, and as an output each call's lanes are summed
//     into it (see BroadcastIndex above) -- how a tree, linear or pair pass
//     reduces over its rows into it.
//
// Nothing here is ever padding: the one element is always real.
struct GlobalIndexing {
  static constexpr std::size_t Count(std::size_t /*rows*/) { return 1; }

  template <std::size_t Slot, std::size_t LoadLanes>
    requires(LoadLanes == 1)
  static constexpr std::size_t
  Resolve(const auto& /*indices*/, std::size_t /*rows*/) {
    return 0;
  }

  template <std::size_t Slot, std::size_t LoadLanes>
    requires(LoadLanes > 1)
  static constexpr BroadcastIndex
  Resolve(const auto& /*indices*/, std::size_t /*rows*/) {
    return BroadcastIndex{0};
  }

  template <typename WriteElement>
  static void SeedPadding(
      const WriteElement& /*write*/,
      const domain::JointLayout& /*layout*/,
      std::size_t /*rows*/
  ) {}
};
static_assert(IndexingPolicyLike<GlobalIndexing>);

}  // namespace achilles::engine::topology
