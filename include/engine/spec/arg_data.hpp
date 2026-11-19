// Which field and which index an op argument reads or writes.

#pragma once

#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <tuple>
#include <type_traits>
#include <utility>

#include "engine/spec/field_contract.hpp"
#include "engine/spec/view_contract.hpp"
#include "util/tmp.hpp"

namespace achilles::engine::spec {

// Which of the indices a traversal hands an Op an argument is read from (or
// written to) -- a position, not a flag: OpInvoker packs every index a
// traversal hands it into one tuple, in this order, and an argument picks
// its own by std::get<ArgIndex>, never by a branch (see
// engine/op_invoker/op_invoker.hpp). kTarget is always the row/group the
// traversal is visiting. kSource is the second index, whatever that
// traversal makes it -- kParent names the same slot under a TreeTraversal,
// which is the only place it means "parent"; kLink names it under a
// LinkedTraversal, where it's the row a list entry is linked to; a
// LinearTraversal hands the target again, and a PairTraversal hands the
// pair's broadcast source row (policy/traversal/). The third slot
// is one only some traversals fill: the row the target is being paired
// against, read through fields like any index. Under a scanning
// RowTraversal (RowScan) it's kScene, a row of the second layout the Step
// was given; under a PairTraversal it's kPartner, the lane group of rows a
// lane group of pairs pairs the source with. The fourth, only a
// PairTraversal's, is kStructure: not an index at all, but a fact the
// traversal derived from its own topology about this one call (its
// PairStructure) -- read through kTraversalInputs, never through a field.
enum class ArgIndex : uint8_t {
  kTarget = 0,
  kSource = 1,
  kParent = kSource,
  kLink = kSource,
  kScene = 2,
  kPartner = kScene,
  kStructure = 3,
};

// One argument to gather (or scatter) for an Op: `field` selects which
// field to read from (or write to) the View, and `index` selects which of
// the traversal's indices to do so at. A field whose own Indexing policy
// isn't per-row (e.g. policy::indexing::GlobalIndexing) may resolve from every
// index regardless; see topology/indexing_policy.hpp.
template <FieldEnumLike EnumT>
struct ArgData {
  EnumT field;
  ArgIndex index;
};

}  // namespace achilles::engine::spec
