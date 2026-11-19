// One element per row: a field's default index space.

#pragma once

#include <concepts>
#include <cstddef>
#include <tuple>
#include <type_traits>

#include "domain/layout/joint_layout.hpp"
#include "policy/indexing/base_indexing.hpp"

namespace achilles::policy::indexing {

// One element per row -- every field that existed before index spaces did,
// and still the default (see memory::FieldIndexingT, binding.hpp). An
// argument reads whichever index its Slot names, unchanged: the target
// group, a TreeTraversal's parent group, a LinearTraversal's own row again,
// or a PairTraversal's BroadcastIndex.
struct RowIndexing : BaseIndexing {
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
      const domain::layout::JointLayout& layout,
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

}  // namespace achilles::policy::indexing
