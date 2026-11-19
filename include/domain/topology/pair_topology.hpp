// A topology whose rows pair two rows of another ordering.

#pragma once

#include <cstddef>
#include <span>

#include "domain/topology/joint_topology.hpp"
#include "domain/topology/topology_contract.hpp"

namespace achilles::domain::topology {

// A JointTopology whose rows are pairs of another ordering's rows: each
// row's entry (operator[]) is its *source*, one row of the other ordering,
// and Partner(row) is the row it's paired with. Only a pair ordering has one
// (policy::ordering::PairOrdering): a lane group of its rows shares one
// source, and lane t's partner is row t of one lane group of the other
// ordering -- or kNoPartner, for a padding lane (a pair that isn't one).
class PairTopology : public JointTopology {
 public:
  static constexpr size_t kNoPartner = static_cast<size_t>(-1);

  PairTopology(JointTopology rows, std::span<const size_t> partners)
      : JointTopology(rows), partners_(partners) {}

  size_t Partner(size_t row) const {
    assert(row < partners_.size());
    return partners_[row];
  }

 private:
  std::span<const size_t> partners_;
};
static_assert(TopologyLike<PairTopology>);

}  // namespace achilles::domain::topology
