#pragma once

#include <cstddef>
#include <cstdint>

#include "domain/joint_topology.hpp"
#include "engine/topology/indexing_policy.hpp"

namespace achilles::engine::pass {

// The policies a RowTraversal (traversals.hpp) is built from: a Source
// (where each row group's second index comes from), a Range (which row
// groups it walks) and a Scan (what each visited group is paired against,
// if anything). Each is a set of static functions resolved at compile
// time, so a traversal built from them costs exactly what its own
// hand-written loop would.

// -- Sources: where a RowTraversal's second index (ArgIndex::kSource) comes
// from for each row group j, and where Initialize runs. Each is a pair of
// static functions resolved at compile time, plus whether it can drive a
// batched (Stride > 1) pass.

// The row itself -- a pass whose fields all live in one row space reads
// and writes each row on its own. Driven by a bare row count or anything
// with a Size(). Initialize runs at row 0.
struct SelfSource {
  static constexpr bool kBatchable = true;

  template <size_t Stride, typename Rows>
  static constexpr size_t Of(const Rows& /*rows*/, size_t group) {
    return group;
  }
  template <size_t Stride, typename Rows>
  static constexpr size_t InitRow(const Rows& /*rows*/) {
    return 0;
  }
};

// The row's parent group, in the same row space (ArgIndex::kParent) -- a
// tree ordering's topology. Batchable because the tree planner guarantees
// no parent shares a lane group with its child (see JointTopology::
// IsBatchSafe). Initialize runs at row 0's parent: the base row a root
// joint reads as its parent, seeded before the walk.
struct ParentSource {
  static constexpr bool kBatchable = true;

  template <size_t Stride, domain::TopologyLike Topology>
  static constexpr size_t Of(const Topology& parents, size_t group) {
    return parents[group * Stride] / Stride;
  }
  template <size_t Stride, domain::TopologyLike Topology>
  static constexpr size_t InitRow(const Topology& parents) {
    return parents[0] / Stride;
  }
};

// The row's *link* (ArgIndex::kLink) -- for a list ordering (topology::
// ListOrdering), whose topology holds, per row, the row in another
// ordering's row space that entry is attached to (e.g. the joint a sensor
// is mounted on). An Op reads that other ordering's fields at kLink and its
// own list-ordered ones at kTarget. Not batchable: links point anywhere, so
// a lane group of entries has no single source group to load. Initialize
// runs at row 0 of the list itself.
struct LinkSource {
  static constexpr bool kBatchable = false;

  template <size_t Stride, domain::TopologyLike Topology>
  static constexpr size_t Of(const Topology& links, size_t row) {
    return links[row];
  }
  template <size_t Stride, domain::TopologyLike Topology>
  static constexpr size_t InitRow(const Topology& /*links*/) {
    return 0;
  }
};

// The row group's *source* (ArgIndex::kSource) -- for a pair ordering
// (topology::PairOrdering), whose lane groups each pair one source row of
// another ordering with a lane group of that ordering's rows (see
// domain::PairTopology). Handed as a topology::BroadcastIndex: an Op reads
// the source's fields once, splatted across every lane, and an output
// written there sums the group's lanes into it. Batchable, since a lane
// group shares its source. Initialize runs at row 0 of the pairs.
struct PairSource {
  static constexpr bool kBatchable = true;

  template <size_t Stride>
  static constexpr topology::BroadcastIndex Of(
      const domain::PairTopology& pairs, size_t group
  ) {
    return {pairs[group * Stride]};
  }
  template <size_t Stride>
  static constexpr size_t InitRow(const domain::PairTopology& /*pairs*/) {
    return 0;
  }
};

namespace detail {

// How many raw rows a RowTraversal walks: a bare count as-is, or a
// topology's Size().
constexpr size_t RowCount(size_t rows) { return rows; }
template <typename Rows>
  requires requires(const Rows& rows) { rows.Size(); }
constexpr size_t RowCount(const Rows& rows) {
  return rows.Size();
}

}  // namespace detail

// Which row groups a RowTraversal walks, [Begin, End) in groups of Stride:
// every one (AllRows), or only segment K of a topology split into segments
// (one per list of a topology::ListOrdering -- see
// domain::JointTopology::SegmentBegin), one row at a time since a segment
// needn't start on a lane boundary.
struct AllRows {
  template <size_t Stride, typename Rows>
  static constexpr size_t Begin(const Rows& /*rows*/) {
    return 0;
  }
  template <size_t Stride, typename Rows>
  static constexpr size_t End(const Rows& rows) {
    return detail::RowCount(rows) / Stride;
  }
};

template <size_t K>
struct Segment {
  template <size_t Stride, typename Rows>
    requires(Stride == 1)
  static constexpr size_t Begin(const Rows& rows) {
    return rows.SegmentBegin(K);
  }
  template <size_t Stride, typename Rows>
    requires(Stride == 1)
  static constexpr size_t End(const Rows& rows) {
    return rows.SegmentEnd(K);
  }
};

// What a RowTraversal does at each row group it visits: call once with
// (group, source) -- NoScan -- or, ScanRows, pair the group with every real
// row of the JointLayout a Step was given second (its padding rows, which
// hold no real entry, skipped), calling once per pair with that row as a
// third index (ArgIndex::kScene). Scanning is for an Op relating each entry
// of one list to every row of another -- a lidar against every body's
// shape, say -- that still reads the entry's own source (its link). Given
// no second argument, a scan has nothing to pair against and does nothing.
struct NoScan {
  static constexpr bool kBatchable = true;

  template <
      size_t Stride,
      typename Callable,
      typename Rows,
      typename Source,
      typename... Rest>
  static constexpr void Visit(
      const Callable& callable,
      const Rows& /*rows*/,
      size_t group,
      const Source& source,
      const Rest&... /*rest*/
  ) {
    callable(group, source);
  }
  template <typename... Rest>
  static constexpr bool HasWork(const Rest&... /*rest*/) {
    return true;
  }
};

struct ScanRows {
  static constexpr bool kBatchable = false;

  template <
      size_t Stride,
      typename Callable,
      typename Rows,
      typename Scene,
      typename... Rest>
  static constexpr void Visit(
      const Callable& callable,
      const Rows& /*rows*/,
      size_t group,
      size_t source,
      const Scene& scene,
      const Rest&... /*rest*/
  ) {
    const size_t rows = scene.PaddedSize();
    for (size_t row = 0; row < rows; ++row) {
      if (!scene.IsPadding(row)) {
        callable(group, source, row);
      }
    }
  }
  template <size_t Stride, typename Callable, typename Rows>
  static constexpr void Visit(
      const Callable& /*callable*/,
      const Rows& /*rows*/,
      size_t /*group*/,
      size_t /*source*/
  ) {}

  template <typename Scene, typename... Rest>
  static constexpr bool HasWork(const Scene& scene, const Rest&... /*rest*/) {
    return scene.PaddedSize() > 0;
  }
  static constexpr bool HasWork() { return false; }
};

// What a pass over pairs knows about one lane group of them, handed to an Op
// through its tuple's ArgIndex::kStructure slot (see kTraversalInputs,
// engine/op_contract.hpp): bit t of distinct_lanes is set when lane t is a
// pair at all, not padding (see topology::PairOrdering for which pairs are:
// a real row above the source, neither row the other's parent, admitted by
// the ordering's filter). Whether the pair matters this tick is the Op's
// own business.
struct PairStructure {
  std::uint64_t distinct_lanes;
};

// Pairs each lane group of a pair ordering with its partners (see
// domain::PairTopology): one call per group, handed the partners' lane
// group as a third index (ArgIndex::kPartner) and which lanes are pairs at
// all as a fourth (ArgIndex::kStructure). A row-indexed argument naming
// kPartner reads that lane group of the other ordering, lane for lane with
// the pairs.
struct PairedRows {
  static constexpr bool kBatchable = true;

  template <size_t Stride, typename Callable, typename... Rest>
  static constexpr void Visit(
      const Callable& callable,
      const domain::PairTopology& pairs,
      size_t group,
      const topology::BroadcastIndex& source,
      const Rest&... /*rest*/
  ) {
    size_t partner_group = 0;
    std::uint64_t lanes = 0;
    for (size_t lane = 0; lane < Stride; ++lane) {
      const size_t partner = pairs.Partner((group * Stride) + lane);
      if (partner != domain::PairTopology::kNoPartner) {
        partner_group = partner / Stride;
        lanes |= std::uint64_t{1} << lane;
      }
    }
    callable(group, source, partner_group, PairStructure{lanes});
  }
  template <typename... Rest>
  static constexpr bool HasWork(const Rest&... /*rest*/) {
    return true;
  }
};

}  // namespace achilles::engine::pass
