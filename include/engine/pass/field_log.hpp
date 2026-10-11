#pragma once

#include <algorithm>
#include <array>
#include <bitset>
#include <cstddef>
#include <functional>
#include <span>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

#include "domain/archetype.hpp"
#include "engine/algorithm_contract.hpp"
#include "engine/memory/binding.hpp"
#include "engine/named_fields.hpp"
#include "engine/pass/sim_context.hpp"
#include "engine/topology/layout.hpp"

namespace achilles::engine::pass {

namespace detail {

// Recovers EnumT/Traits from a concrete Algorithm<...> the same way
// AlgorithmOrderings (sim_context.hpp) does: every field's name ("" for an
// unnamed field, see engine::AllFields) and ordering policy.
template <typename AlgorithmT>
struct LogInfo;

template <typename EnumT, template <EnumT> class Traits, typename StepT>
struct LogInfo<Algorithm<EnumT, Traits, StepT>> {
  static constexpr std::size_t kCount = static_cast<std::size_t>(EnumT::kCount);

  static constexpr std::array<std::string_view, kCount> kNames = [] {
    std::array<std::string_view, kCount> names{};
    for (std::size_t i = 0; i < kCount; ++i) {
      names[i] = engine::detail::AlgorithmFields<
                     Algorithm<EnumT, Traits, StepT>>::kFields[i]
                     .name;
    }
    return names;
  }();

  template <std::size_t I>
  using Ordering = memory::FieldOrderingT<EnumT, Traits, static_cast<EnumT>(I)>;

  template <std::size_t I>
  static constexpr EnumT kField = static_cast<EnumT>(I);
};

// Per-Algorithm runtime selection: which fields were asked for, and for
// each, the real (non-padding) rows to read in (instance, joint) order.
template <typename AlgorithmT>
struct Selection {
  static constexpr std::size_t kCount = LogInfo<AlgorithmT>::kCount;
  std::bitset<kCount> fields;
  std::array<std::vector<std::size_t>, kCount> rows;
  std::array<std::vector<domain::ArchetypeJointHandle>, kCount> handles;
};

}  // namespace detail

// Runtime-selected per-tick field logging over a SimContext's hosted
// Algorithms. Fields are addressed by kName (e.g. "joint_position"), or
// "*" for every named field -- never by which Algorithm hosts them. A name
// several Algorithms share (one SharedAs block) is logged once, from the
// first hosting Algorithm. Resolution happens once, at construction;
// Gather then copies every selected field's raw scalars (View::LoadFlat)
// for every real joint into one flat row, skipping outright any Algorithm
// with nothing selected.
//
// Row/column order is fixed at construction and matches ColumnNames():
// Algorithm order (as hosted), then field enum order, then joint (instance,
// joint) order, then the field's own scalar (Assembler leaf) order.
template <AlgorithmLike... Algorithms>
class FieldLog {
  static_assert(
      SameNameMeansSameSlot(AllFields<Algorithms...>()),
      "Two hosted fields share a kName without sharing storage -- a name "
      "must mean one block of memory (give both the same SharedAs slot, "
      "or rename one)."
  );

 public:
  using Context = SimContext<Algorithms...>;
  // Names one real joint for column headers, e.g. "arm.elbow".
  using JointLabeler = std::function<std::string(domain::ArchetypeJointHandle)>;

  // Names no hosted field matches are appended to `unknown`, so the caller
  // can report typos rather than silently logging less.
  FieldLog(
      const Context& context,
      std::span<const std::string> names,
      std::vector<std::string>& unknown
  ) {
    // Names already selected from an earlier-hosted Algorithm, so a
    // shared name is only logged once.
    std::vector<std::string_view> taken;
    for (const std::string& name : names) {
      bool claimed = false;
      std::apply(
          [&](auto&... selection) {
            ((claimed = Claim<Algorithms>(selection, name, taken) || claimed),
             ...);
          },
          selections_
      );
      if (!claimed) {
        unknown.push_back(name);
      }
    }
    std::apply(
        [&](auto&... selection) {
          (ResolveRows<Algorithms>(
               context,
               selection,
               std::make_index_sequence<Info<Algorithms>::kCount>{}
           ),
           ...);
        },
        selections_
    );
  }

  // Total scalars Gather writes per tick.
  std::size_t Width() const { return width_; }
  bool Empty() const { return width_ == 0; }

  std::vector<std::string> ColumnNames(const JointLabeler& label) const {
    std::vector<std::string> columns;
    columns.reserve(width_);
    std::apply(
        [&](const auto&... selection) {
          (AppendColumns<Algorithms>(
               selection,
               label,
               columns,
               std::make_index_sequence<Info<Algorithms>::kCount>{}
           ),
           ...);
        },
        selections_
    );
    return columns;
  }

  // `out.size()` must equal Width().
  template <typename OutT>
  void Gather(const Context& context, std::span<OutT> out) const {
    std::size_t offset = 0;
    std::apply(
        [&](const auto&... selection) {
          (GatherAlgorithm<Algorithms>(context, selection, out, offset), ...);
        },
        selections_
    );
  }

 private:
  template <typename AlgorithmT>
  using Info = detail::LogInfo<AlgorithmT>;

  template <typename AlgorithmT>
  static bool Claim(
      detail::Selection<AlgorithmT>& selection,
      std::string_view name,
      std::vector<std::string_view>& taken
  ) {
    bool claimed = false;
    for (std::size_t i = 0; i < Info<AlgorithmT>::kCount; ++i) {
      std::string_view field_name = Info<AlgorithmT>::kNames[i];
      if (field_name.empty() || (name != "*" && name != field_name)) {
        continue;
      }
      claimed = true;
      if (std::ranges::find(taken, field_name) == taken.end()) {
        taken.push_back(field_name);
        selection.fields.set(i);
      }
    }
    return claimed;
  }

  template <typename AlgorithmT, std::size_t... Is>
  void
  ResolveRows(const Context& context, detail::Selection<AlgorithmT>& selection, std::index_sequence<Is...>) {
    using View = typename AlgorithmT::View;
    auto resolve_one = [&]<std::size_t I>() {
      if (!selection.fields.test(I)) {
        return;
      }
      const topology::Layout& layout = context.template LayoutFor<
          typename Info<AlgorithmT>::template Ordering<I>>();
      std::vector<std::pair<domain::ArchetypeJointHandle, std::size_t>> real;
      for (std::size_t row = 0; row < layout.PaddedSize(); ++row) {
        if (!layout.IsPadding(row)) {
          real.emplace_back(layout.ToPhysical(row), row);
        }
      }
      std::ranges::sort(real, [](const auto& a, const auto& b) {
        return std::pair(a.first.instance_index, a.first.joint_index) <
               std::pair(b.first.instance_index, b.first.joint_index);
      });
      for (const auto& [handle, row] : real) {
        selection.handles[I].push_back(handle);
        selection.rows[I].push_back(row);
      }
      width_ +=
          real.size() *
          View::template NumScalars<Info<AlgorithmT>::template kField<I>>();
    };
    (resolve_one.template operator()<Is>(), ...);
  }

  template <typename AlgorithmT, std::size_t... Is>
  static void
  AppendColumns(const detail::Selection<AlgorithmT>& selection, const JointLabeler& label, std::vector<std::string>& columns, std::index_sequence<Is...>) {
    using View = typename AlgorithmT::View;
    auto append_one = [&]<std::size_t I>() {
      if (!selection.fields.test(I)) {
        return;
      }
      constexpr std::size_t kScalars =
          View::template NumScalars<Info<AlgorithmT>::template kField<I>>();
      std::string suffix = "." + std::string(Info<AlgorithmT>::kNames[I]) + ".";
      for (const domain::ArchetypeJointHandle& handle : selection.handles[I]) {
        std::string joint = label(handle);
        for (std::size_t k = 0; k < kScalars; ++k) {
          columns.push_back(joint + suffix + std::to_string(k));
        }
      }
    };
    (append_one.template operator()<Is>(), ...);
  }

  template <typename AlgorithmT, typename OutT>
  static void GatherAlgorithm(
      const Context& context,
      const detail::Selection<AlgorithmT>& selection,
      std::span<OutT> out,
      std::size_t& offset
  ) {
    // The "only call it when there's something to log" check: one bitset
    // test per Algorithm per tick.
    if (selection.fields.none()) {
      return;
    }
    GatherFields<AlgorithmT>(
        context,
        selection,
        out,
        offset,
        std::make_index_sequence<Info<AlgorithmT>::kCount>{}
    );
  }

  template <typename AlgorithmT, typename OutT, std::size_t... Is>
  static void
  GatherFields(const Context& context, const detail::Selection<AlgorithmT>& selection, std::span<OutT> out, std::size_t& offset, std::index_sequence<Is...>) {
    using View = typename AlgorithmT::View;
    const View view = context.template ViewFor<AlgorithmT>();
    auto gather_one = [&]<std::size_t I>() {
      if (!selection.fields.test(I)) {
        return;
      }
      constexpr auto kField = Info<AlgorithmT>::template kField<I>;
      constexpr std::size_t kScalars = View::template NumScalars<kField>();
      for (std::size_t row : selection.rows[I]) {
        view.template LoadFlat<kField>(row, out.subspan(offset, kScalars));
        offset += kScalars;
      }
    };
    (gather_one.template operator()<Is>(), ...);
  }

  std::tuple<detail::Selection<Algorithms>...> selections_;
  std::size_t width_ = 0;
};

// The FieldLog matching a given SimContext's own Algorithms... pack, for a
// caller that only has the context type in hand (e.g. interface::
// Simulation, whose pack comes from algorithms::RegisteredAlgorithms).
template <typename ContextT>
struct FieldLogFor;

template <AlgorithmLike... Algorithms>
struct FieldLogFor<SimContext<Algorithms...>> {
  using Type = FieldLog<Algorithms...>;
};

template <typename ContextT>
using FieldLogForT = typename FieldLogFor<ContextT>::Type;

}  // namespace achilles::engine::pass
