// The base every indexing policy derives from, its concept, and the
// broadcast index.

#pragma once

#include <concepts>
#include <cstddef>
#include <tuple>
#include <type_traits>

#include "domain/layout/joint_layout.hpp"

namespace achilles::policy::indexing {

// What every indexing policy derives from: logic its indexing policys share
// goes here. Never used polymorphically: nothing holds or calls one through it.
struct BaseIndexing {
  // Seeds nothing: only a row-indexed field has padding rows of its own.
  template <typename WriteElement>
  static void SeedPadding(
      const WriteElement& /*write*/,
      const domain::layout::JointLayout& /*layout*/,
      std::size_t /*rows*/
  ) {}
};

// A traversal's source index (see policy/traversal/) when it names
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
//     an argument naming position Slot (engine::spec::ArgIndex) of
//     `indices` -- the tuple of every index the traversal handed, target first
//     -- read as a T `Lanes` wide. The target is always a group index (in units
//     of Lanes); the source is whatever the traversal handed in its place.
//   - SeedPadding(write, layout, rows): calls write(element) for every
//     element that doesn't belong to a real (instance, joint) under the
//     field's own JointLayout -- see SimAllocator's padding-seed pass
//     (engine/allocation/sim_allocator.hpp), which owns everything about the
//     write itself but defers *which* elements are padding to here.
template <typename P>
concept IndexingPolicyLike = std::is_empty_v<P> && requires(std::size_t rows) {
  { P::Count(rows) } -> std::same_as<std::size_t>;
};

}  // namespace achilles::policy::indexing
