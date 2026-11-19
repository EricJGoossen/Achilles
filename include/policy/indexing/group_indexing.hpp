// One element per lane group of rows.

#pragma once

#include <concepts>
#include <cstddef>
#include <tuple>
#include <type_traits>

#include "domain/layout/joint_layout.hpp"
#include "policy/indexing/base_indexing.hpp"

namespace achilles::policy::indexing {

// One element per lane group of rows -- a fact about a whole group at once,
// e.g. a Lanes-bit domain::math::ActivationMask with bit t set for row t of
// the group (a compare's movemask, a GPU warp ballot), such as which of a
// pair ordering's lane group of pairs overlap. Read/written as the field's
// plain scalar at the target group, in place of a batch of the group's rows.
// Its padding is whatever a pass writes for a group's padding rows: nothing
// to seed.
template <std::size_t Lanes>
  requires(Lanes > 0)
struct GroupIndexing : BaseIndexing {
  static constexpr std::size_t Count(std::size_t rows) { return rows / Lanes; }

  template <std::size_t Slot, std::size_t LoadLanes>
    requires(LoadLanes == 1)
  static constexpr std::size_t
  Resolve(const auto& indices, std::size_t /*rows*/) {
    return std::get<Slot>(indices);
  }
};

static_assert(IndexingPolicyLike<GroupIndexing<4>>);

}  // namespace achilles::policy::indexing
