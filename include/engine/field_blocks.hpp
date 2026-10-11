#pragma once

#include <array>
#include <cstddef>
#include <optional>
#include <string_view>
#include <type_traits>
#include <utility>

#include "engine/algorithm_contract.hpp"
#include "engine/memory/binding.hpp"
#include "util/tmp.hpp"

namespace achilles::engine {

// One field of one hosted Algorithm: field index I of AlgorithmT's
// FieldEnum, with everything its Traits resolve to.
template <typename AlgorithmT, std::size_t I>
struct FieldRef;

template <
    typename EnumT,
    template <EnumT>
    class Traits,
    typename StepT,
    std::size_t I>
struct FieldRef<Algorithm<EnumT, Traits, StepT>, I> {
  using AlgorithmType = Algorithm<EnumT, Traits, StepT>;
  static constexpr EnumT kField = static_cast<EnumT>(I);
  using FieldTraits = Traits<kField>;
  using Assembler = typename FieldTraits::Assembler;
  using Layout = memory::FieldLayoutT<EnumT, Traits, kField>;
  using Ordering = memory::FieldOrderingT<EnumT, Traits, kField>;
  static constexpr bool kShared = memory::FieldIsShared<EnumT, Traits, kField>;

  static constexpr std::string_view Name() {
    if constexpr (memory::FieldHasName<EnumT, Traits, kField>) {
      return FieldTraits::kName;
    } else {
      return {};
    }
  }
  static constexpr std::string_view kName = Name();
};

namespace detail {

// What makes two FieldRefs the same storage: their SharedAs tag if they
// have one, otherwise the field itself (a private field is its own block).
template <typename Ref, bool = Ref::kShared>
struct FieldKey {
  using Type = Ref;
};
template <typename Ref>
struct FieldKey<Ref, true> {
  using Type = typename Ref::FieldTraits::SharedAs;
};
template <typename Ref>
using FieldKeyT = typename FieldKey<Ref>::Type;

template <typename AlgorithmT, typename Seq>
struct AlgorithmRefsImpl;
template <typename AlgorithmT, std::size_t... Is>
struct AlgorithmRefsImpl<AlgorithmT, std::index_sequence<Is...>> {
  using Type = util::TypeList<FieldRef<AlgorithmT, Is>...>;
};
template <typename AlgorithmT>
using AlgorithmRefsT = typename AlgorithmRefsImpl<
    AlgorithmT,
    std::make_index_sequence<static_cast<std::size_t>(AlgorithmT::Enum::kCount
    )>>::Type;

template <typename Key, typename RefList>
struct RefsWithKey;
template <typename Key, typename... Refs>
struct RefsWithKey<Key, util::TypeList<Refs...>> {
  using Type = util::ConcatT<std::conditional_t<
      std::is_same_v<FieldKeyT<Refs>, Key>,
      util::TypeList<Refs>,
      util::TypeList<>>...>;
};

}  // namespace detail

// One distinct block of field storage, and every hosted field that views
// it (Members, in hosting order -- more than one only for SharedAs
// fields). Primary, the first member, supplies the block's types; Name is
// whichever kName its members give it ("" if none does).
template <typename KeyT, typename MemberList>
struct FieldBlock;

template <typename KeyT, typename PrimaryRef, typename... Rest>
struct FieldBlock<KeyT, util::TypeList<PrimaryRef, Rest...>> {
  using Key = KeyT;
  using Members = util::TypeList<PrimaryRef, Rest...>;
  using Primary = PrimaryRef;
  using Assembler = typename Primary::Assembler;
  using Layout = typename Primary::Layout;
  using Ordering = typename Primary::Ordering;

  static constexpr std::string_view kName = [] {
    std::string_view name = Primary::kName;
    ((name = name.empty() ? Rest::kName : name), ...);
    return name;
  }();

  // Every member must agree on how the block is laid out -- sharing an
  // array two Algorithms disagree about the layout of is unsound.
  static constexpr bool kShapesAgree =
      ((std::is_same_v<typename Rest::Layout, Layout> &&
        std::is_same_v<typename Rest::Ordering, Ordering> &&
        std::is_same_v<
            typename Rest::Assembler::ScalarType,
            typename Assembler::ScalarType> &&
        Rest::Assembler::kNumFields == Assembler::kNumFields) &&
       ...);

  // Members that name the block must all use the same name (an unnamed
  // member, e.g. a read-only view, is fine).
  static constexpr bool kNamesAgree =
      (Primary::kName.empty() || Primary::kName == kName) &&
      ((Rest::kName.empty() || Rest::kName == kName) && ...);
};

namespace detail {

template <typename Keys, typename Refs>
struct MakeBlocks;
template <typename... Keys, typename Refs>
struct MakeBlocks<util::TypeList<Keys...>, Refs> {
  using Type = util::TypeList<
      FieldBlock<Keys, typename RefsWithKey<Keys, Refs>::Type>...>;
};

// The rules a list of blocks must satisfy, kept separate so a failure
// names the rule that broke.
template <typename BlockList>
struct BlockRules;
template <typename... Blocks>
struct BlockRules<util::TypeList<Blocks...>> {
  static constexpr bool kShapesAgree = (Blocks::kShapesAgree && ...);
  static constexpr bool kNamesAgree = (Blocks::kNamesAgree && ...);
  // No two distinct blocks share a name -- a name means one block.
  static constexpr bool kNamesUnique = [] {
    constexpr std::array<std::string_view, sizeof...(Blocks)> kNames = {
        Blocks::kName...
    };
    for (std::size_t a = 0; a < kNames.size(); ++a) {
      for (std::size_t b = a + 1; b < kNames.size(); ++b) {
        if (!kNames[a].empty() && kNames[a] == kNames[b]) {
          return false;
        }
      }
    }
    return true;
  }();
};

}  // namespace detail

// The compile-time, deduplicated list of every field storage block across
// Algorithms...: one FieldBlock per distinct block, in order of first
// appearance (hosting order, then field enum order). The allocator carves
// exactly these blocks; loading and logging address them by name.
//
// Building the list enforces the block rules, so any user of it -- the
// allocator, field logging, Simulation's load checks -- can rely on them.
template <AlgorithmLike... Algorithms>
struct FieldBlocksOf {
  using Refs = util::ConcatT<detail::AlgorithmRefsT<Algorithms>...>;
  using Keys = util::UniqueT<util::TransformT<detail::FieldKeyT, Refs>>;
  using Type = typename detail::MakeBlocks<Keys, Refs>::Type;

 private:
  using Rules = detail::BlockRules<Type>;
  static_assert(
      Rules::kShapesAgree,
      "Two Algorithms name the same SharedAs tag with incompatible field "
      "shapes (Assembler/Layout/Ordering) -- sharing an array two "
      "Algorithms disagree about the layout of is unsound."
  );
  static_assert(
      Rules::kNamesAgree,
      "Fields sharing a SharedAs tag give it different kNames -- one block "
      "of storage has one name."
  );
  static_assert(
      Rules::kNamesUnique,
      "Two distinct storage blocks share a kName -- a name must mean one "
      "block (share a SharedAs tag, or rename one)."
  );
};

template <AlgorithmLike... Algorithms>
using FieldBlocksT = typename FieldBlocksOf<Algorithms...>::Type;

// FieldBlocksT over a util::TypeList of Algorithms (e.g. algorithms::
// RegisteredAlgorithms).
template <typename AlgorithmList>
struct FieldBlocksIn;
template <AlgorithmLike... Algorithms>
struct FieldBlocksIn<util::TypeList<Algorithms...>> {
  using Type = FieldBlocksT<Algorithms...>;
};

// Index of the FieldBlock whose storage FieldRef `Ref` views, within
// `BlockList` (a FieldBlocksT).
template <typename Ref, typename BlockList>
struct BlockIndex;
template <typename Ref, typename... Blocks>
struct BlockIndex<Ref, util::TypeList<Blocks...>> {
  static constexpr std::size_t kValue = [] {
    constexpr std::array<bool, sizeof...(Blocks)> kMatches = {
        std::is_same_v<typename Blocks::Key, detail::FieldKeyT<Ref>>...
    };
    std::size_t index = 0;
    while (!kMatches[index]) {
      ++index;
    }
    return index;
  }();
};

// Runtime-queryable summary of each FieldBlock: its name ("" if unnamed)
// and how many scalars one row holds.
struct BlockInfo {
  std::string_view name;
  std::size_t scalars = 0;
};

template <typename BlockList>
struct BlockInfos;
template <typename... Blocks>
struct BlockInfos<util::TypeList<Blocks...>> {
  static constexpr std::array<BlockInfo, sizeof...(Blocks)> kValue = {
      BlockInfo{Blocks::kName, Blocks::Assembler::kNumFields}...
  };
};

// The block named `name`, if any.
template <std::size_t N>
constexpr std::optional<BlockInfo> FindBlock(
    const std::array<BlockInfo, N>& blocks, std::string_view name
) {
  for (const BlockInfo& block : blocks) {
    if (!block.name.empty() && block.name == name) {
      return block;
    }
  }
  return std::nullopt;
}

}  // namespace achilles::engine
