#pragma once

#include <array>
#include <cstddef>
#include <optional>
#include <string_view>
#include <utility>

#include "engine/algorithm_contract.hpp"
#include "engine/memory/binding.hpp"
#include "util/tmp.hpp"

namespace achilles::engine {

// One named field (Traits<F>::kName) of some hosted Algorithm: its name,
// how many scalars one row of it holds (its Assembler's leaf count), and
// the SharedAs slot its storage belongs to (memory::SlotId; nullptr for a
// private block).
struct NamedField {
  std::string_view name;
  std::size_t scalars = 0;
  const void* slot = nullptr;
};

namespace detail {

template <typename EnumT, template <EnumT> class Traits, EnumT F>
constexpr NamedField DescribeField() {
  if constexpr (memory::FieldHasName<EnumT, Traits, F>) {
    const void* slot = nullptr;
    if constexpr (memory::FieldIsShared<EnumT, Traits, F>) {
      slot = memory::SlotId<typename Traits<F>::SharedAs>();
    }
    return {Traits<F>::kName, Traits<F>::Assembler::kNumFields, slot};
  } else {
    return {};
  }
}

// Recovers EnumT/Traits from a concrete Algorithm<...> the same way
// AlgorithmOrderings (pass/sim_context.hpp) does. Entries for unnamed
// fields are left empty (name == "").
template <typename AlgorithmT>
struct AlgorithmFields;

template <typename EnumT, template <EnumT> class Traits, typename StepT>
struct AlgorithmFields<Algorithm<EnumT, Traits, StepT>> {
  static constexpr std::size_t kCount = static_cast<std::size_t>(EnumT::kCount);

  template <std::size_t... Is>
  static constexpr std::array<NamedField, kCount>
  Describe(std::index_sequence<Is...>) {
    return {DescribeField<EnumT, Traits, static_cast<EnumT>(Is)>()...};
  }
  static constexpr std::array<NamedField, kCount> kFields =
      Describe(std::make_index_sequence<kCount>{});
};

}  // namespace detail

// Every field of every one of Algorithms..., in hosting then enum order;
// unnamed fields appear as empty entries (name == "").
template <AlgorithmLike... Algorithms>
consteval auto AllFields() {
  constexpr std::size_t kTotal =
      (std::size_t{0} + ... + detail::AlgorithmFields<Algorithms>::kCount);
  std::array<NamedField, kTotal> all{};
  std::size_t next = 0;
  (
      [&] {
        for (const NamedField& field :
             detail::AlgorithmFields<Algorithms>::kFields) {
          all[next++] = field;
        }
      }(),
      ...
  );
  return all;
}

// AllFields over a util::TypeList of Algorithms (e.g. algorithms::
// RegisteredAlgorithms).
template <typename AlgorithmList>
struct AllFieldsIn;

template <AlgorithmLike... Algorithms>
struct AllFieldsIn<util::TypeList<Algorithms...>> {
  static constexpr auto kValue = AllFields<Algorithms...>();
};

// The first named field called `name`, if any.
template <std::size_t N>
constexpr std::optional<NamedField> FindField(
    const std::array<NamedField, N>& fields, std::string_view name
) {
  for (const NamedField& field : fields) {
    if (!field.name.empty() && field.name == name) {
      return field;
    }
  }
  return std::nullopt;
}

// True when every name two fields share refers to one block of memory --
// both SharedAs the same slot. A shared name is then safe to load (one
// archetype field fills one block) and to log once; two unrelated fields
// with one name would silently both be filled from the same archetype
// field.
template <std::size_t N>
constexpr bool SameNameMeansSameSlot(const std::array<NamedField, N>& fields) {
  for (std::size_t a = 0; a < N; ++a) {
    for (std::size_t b = a + 1; b < N; ++b) {
      const NamedField& first = fields[a];
      const NamedField& second = fields[b];
      if (!first.name.empty() && first.name == second.name &&
          (first.slot == nullptr || first.slot != second.slot)) {
        return false;
      }
    }
  }
  return true;
}

}  // namespace achilles::engine
