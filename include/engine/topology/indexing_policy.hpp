#pragma once

#include <concepts>
#include <cstddef>
#include <tuple>
#include <type_traits>

#include "engine/topology/layout.hpp"

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
// (ordering_policy.hpp): a new index space is a new policy struct here, not
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
//     field's own Layout -- see SimAllocator's padding-seed pass
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

  // Every padding row up to the Layout's own BaseRowIndex(), then every
  // filler row past it. The reserved base row itself is deliberately left
  // alone: it's seeded by the hosting algorithm's own Op::Initialize with
  // real, physically meaningful state (gravity, world velocity, ...), not a
  // generic safe-neutral value.
  template <typename WriteElement>
  static void SeedPadding(
      const WriteElement& write, const Layout& layout, std::size_t rows
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

// ElementLanes elements per (source row, target lane group), source-major:
// the `rows / Lanes` groups of targets paired against source row s occupy
// groups [s * rows / Lanes, (s + 1) * rows / Lanes), each ElementLanes
// elements wide. Lanes is the width of the target groups a PairTraversal
// hands out (its own Stride), not necessarily the width this field is read
// at; ElementLanes is how many elements one group holds, and so the one
// load width Resolve accepts:
//
//   - ElementLanes = 1 (the default): one element summarizes a whole lane
//     group's worth of pairs at once, e.g. a Lanes-bit
//     domain::math::ActivationMask with bit t set when target
//     (group * Lanes + t) overlaps the source. As a whole the field is the
//     rows x (rows / Lanes) matrix of those per-group words, read/written
//     as the field's plain scalar.
//   - ElementLanes = Lanes: one element per pair, lane t of a group holding
//     target (group * Lanes + t) -- a per-pair quantity (a contact
//     distance, normal, ...) read/written as one batch per call, lane for
//     lane with the target group it was computed from. As a whole the
//     field is the rows x rows matrix of those per-pair values.
//
// Either way the address is the same (source row, target group) index, in
// units of the load width -- which is exactly the unit FieldCursor's
// Load/Store index in (engine/view/view.hpp), so one formula serves both.
// Resolve has no overload for any other width: a narrower load would split
// a group, a wider one would span several, never anything one call means.
// Only a traversal that hands a BroadcastIndex source (PairTraversal) can
// address it at all; pairing a field like this with a tree or linear pass
// fails to compile instead of silently reading row-indexed addresses out of
// a pair-sized block.
//
// Both indices always feed the address, whatever Slot the argument names:
// a pair element belongs to the pair, not to either side of it.
//
// Sized rows * (rows / Lanes) * ElementLanes, so its memory is still
// quadratic in the sim's row count -- fine for the tens-to-hundreds of
// bodies a brute-forced pair pass handles, and exactly the dense layout a
// GPU grid over (source, target group) wants.
template <std::size_t Lanes, std::size_t ElementLanes = 1>
  requires(Lanes > 0 && ElementLanes > 0)
struct PairGroupIndexing {
  static constexpr std::size_t Count(std::size_t rows) {
    return rows * (rows / Lanes) * ElementLanes;
  }

  template <std::size_t Slot, std::size_t LoadLanes>
    requires(LoadLanes == ElementLanes)
  static constexpr std::size_t Resolve(const auto& indices, std::size_t rows) {
    const auto& target_group = std::get<0>(indices);
    const BroadcastIndex& source = std::get<1>(indices);
    return source.row * (rows / Lanes) + target_group;
  }

  // Every element of every group whose source isn't a real row (padding,
  // the reserved base row, or filler), or whose target group holds no real
  // row at all. A group pairing a real source with at least one real target
  // is left to Carve's zero-fill, the same as any real row of a row-indexed
  // output: a pass computes it at runtime, and any padding targets inside
  // that group are that pass's to mask out.
  template <typename WriteElement>
  static void SeedPadding(
      const WriteElement& write, const Layout& layout, std::size_t rows
  ) {
    auto is_real = [&](std::size_t row) {
      return row < layout.PaddedSize() && !layout.IsPadding(row);
    };
    auto group_has_real = [&](std::size_t group) {
      for (std::size_t lane = 0; lane < Lanes; ++lane) {
        if (is_real(group * Lanes + lane)) {
          return true;
        }
      }
      return false;
    };
    std::size_t groups = rows / Lanes;
    for (std::size_t source = 0; source < rows; ++source) {
      for (std::size_t group = 0; group < groups; ++group) {
        if (!is_real(source) || !group_has_real(group)) {
          for (std::size_t element = 0; element < ElementLanes; ++element) {
            write(((source * groups + group) * ElementLanes) + element);
          }
        }
      }
    }
  }
};
static_assert(IndexingPolicyLike<PairGroupIndexing<1>>);
static_assert(IndexingPolicyLike<PairGroupIndexing<4, 4>>);

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
      const Layout& /*layout*/,
      std::size_t /*rows*/
  ) {}
};
static_assert(IndexingPolicyLike<GlobalIndexing>);

}  // namespace achilles::engine::topology
