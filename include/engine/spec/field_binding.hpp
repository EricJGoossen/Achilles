// One field's resolved policies and assembler, as a view holds them.

#pragma once

#include <array>
#include <cassert>
#include <concepts>
#include <cstddef>
#include <tuple>
#include <type_traits>
#include <utility>
#include <xsimd/xsimd.hpp>

#include "engine/assembly/assembler.hpp"
#include "engine/binding.hpp"
#include "engine/memory/buffered_scalar.hpp"
#include "engine/spec/field_contract.hpp"
#include "policy/buffering/base_buffering.hpp"
#include "policy/buffering/double_buffering.hpp"
#include "policy/buffering/single_buffering.hpp"
#include "policy/indexing/base_indexing.hpp"
#include "policy/indexing/global_indexing.hpp"
#include "policy/indexing/group_indexing.hpp"
#include "policy/indexing/row_indexing.hpp"
#include "policy/layout/base_layout.hpp"
#include "policy/layout/planar_layout.hpp"
#include "policy/layout/segmented_layout.hpp"
#include "policy/ordering/base_ordering.hpp"
#include "policy/ordering/linear_ordering.hpp"
#include "policy/ordering/list_ordering.hpp"
#include "policy/ordering/pair_ordering.hpp"
#include "policy/ordering/topological_ordering.hpp"
#include "util/simd_ops.hpp"
#include "util/tmp.hpp"

namespace achilles::engine::spec {

// One field's (memory-style, packing, index-space, buffering, row order)
// tuple. Bundled
// rather than kept as parallel template packs, because a View's fields are
// heterogeneous per-field, not uniform: there is no way to zip an
// independent Policies... pack against an Assemblers... pack and know
// Policies[i] belongs to Assemblers[i] specifically. ViewFactory builds this
// pack from Traits (see view_factory.hpp); nothing else needs to construct
// one by hand. IndexingT defaults to policy::indexing::RowIndexing, BufferingT
// to policy::buffering::SingleBuffering and OrderingT to
// policy::ordering::LinearOrdering, the same defaults
// memory::FieldIndexingT/FieldBufferingT/FieldOrderingT resolve a Traits<F>
// naming none of them to. OrderingT says which row count the field's block is
// sized by (see View's row counts).
template <
    policy::layout::LayoutPolicyLike PolicyT,
    assembly::AssemblerLike AssemblerT,
    policy::indexing::IndexingPolicyLike IndexingT =
        policy::indexing::RowIndexing,
    policy::buffering::BufferingPolicyLike BufferingT =
        policy::buffering::SingleBuffering,
    typename OrderingT = policy::ordering::LinearOrdering>
  requires assembly::AssemblesThrough<AssemblerT, PolicyT>
struct FieldBinding {
  using Layout = PolicyT;
  using assembly::Assembler = AssemblerT;
  using Indexing = IndexingT;
  using Buffering = BufferingT;
  using Ordering = OrderingT;
};

template <typename T>
concept FieldBindingLike =
    requires {
      typename T::Layout;
      typename T::Assembler;
      typename T::Indexing;
      typename T::Buffering;
      typename T::Ordering;
    } && policy::layout::LayoutPolicyLike<typename T::Layout> &&
    assembly::AssemblesThrough<typename T::Assembler, typename T::Layout> &&
    policy::indexing::IndexingPolicyLike<typename T::Indexing> &&
    policy::buffering::BufferingPolicyLike<typename T::Buffering>;

}  // namespace achilles::engine::spec
