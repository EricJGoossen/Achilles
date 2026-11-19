// RangeLike: what a range of row groups provides.

#pragma once

#include <concepts>
#include <cstddef>
#include <type_traits>

#include "domain/topology/segmented_topology.hpp"

namespace achilles::policy::range {

// Which row groups a traversal walks: Begin and End over the rows it's
// given, in units of Stride-row groups. Checked against a segmented
// topology, which every range can walk.
template <typename R>
concept RangeLike = std::is_empty_v<R> &&
                    requires(const domain::topology::SegmentedTopology& rows) {
                      {
                        R::template Begin<1>(rows)
                      } -> std::convertible_to<std::size_t>;
                      {
                        R::template End<1>(rows)
                      } -> std::convertible_to<std::size_t>;
                    };

}  // namespace achilles::policy::range
