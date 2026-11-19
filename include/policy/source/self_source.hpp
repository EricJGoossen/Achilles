// Each row group is its own source.

#pragma once

#include <cstddef>

#include "domain/topology/joint_topology.hpp"
#include "domain/topology/topology_contract.hpp"
#include "policy/source/base_source.hpp"

namespace achilles::policy::source {

// The row itself -- a pass whose fields all live in one row space reads
// and writes each row on its own. Driven by a bare row count or anything
// with a Size(). Initialize runs at row 0.
struct SelfSource : BaseSource {
  static constexpr bool kBatchable = true;

  template <size_t Stride, typename Rows>
  static constexpr size_t Of(const Rows& /*rows*/, size_t group) {
    return group;
  }
};
static_assert(SourceLike<SelfSource>);

}  // namespace achilles::policy::source
