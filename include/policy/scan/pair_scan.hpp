// Each pair group against its partners' group, with which lanes are real.

#pragma once

#include <cstddef>
#include <cstdint>

#include "domain/topology/pair_topology.hpp"
#include "policy/indexing/base_indexing.hpp"
#include "policy/scan/base_scan.hpp"

namespace achilles::policy::scan {

// What a pass over pairs knows about one lane group of them, handed to an Op
// through its tuple's ArgIndex::kStructure slot (see kTraversalInputs,
// engine/spec/op_contract.hpp): bit t of distinct_lanes is set when lane t
// is a pair at all, not padding (see policy::ordering::PairOrdering for which
// pairs are: a real row above the source, neither row the other's parent,
// admitted by the ordering's filter). Whether the pair matters this tick is the
// Op's own business.
struct PairStructure {
  std::uint64_t distinct_lanes;
};

// Pairs each lane group of a pair ordering with its partners (see
// domain::PairTopology): one call per group, handed the partners' lane
// group as a third index (ArgIndex::kPartner) and which lanes are pairs at
// all as a fourth (ArgIndex::kStructure). A row-indexed argument naming
// kPartner reads that lane group of the other ordering, lane for lane with
// the pairs.
struct PairScan : BaseScan {
  static constexpr bool kBatchable = true;

  template <size_t Stride, typename Callable, typename... Rest>
  static constexpr void Visit(
      const Callable& callable,
      const domain::topology::PairTopology& pairs,
      size_t group,
      const policy::indexing::BroadcastIndex& source,
      const Rest&... /*rest*/
  ) {
    size_t partner_group = 0;
    std::uint64_t lanes = 0;
    for (size_t lane = 0; lane < Stride; ++lane) {
      const size_t partner = pairs.Partner((group * Stride) + lane);
      if (partner != domain::topology::PairTopology::kNoPartner) {
        partner_group = partner / Stride;
        lanes |= std::uint64_t{1} << lane;
      }
    }
    callable(group, source, partner_group, PairStructure{lanes});
  }
};
static_assert(ScanLike<PairScan>);

}  // namespace achilles::policy::scan
