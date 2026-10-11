#pragma once

#include <algorithm>
#include <array>
#include <bitset>
#include <cstddef>
#include <functional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "domain/archetype.hpp"
#include "domain/joint_layout.hpp"
#include "engine/algorithm_contract.hpp"
#include "engine/field_blocks.hpp"
#include "engine/pass/sim_context.hpp"
#include "util/tmp.hpp"

namespace achilles::engine::pass {

// Runtime-selected per-tick field logging over a SimContext's hosted
// Algorithms. Works over FieldBlocksT (engine/field_blocks.hpp) -- one
// entry per distinct block of storage -- so a field is addressed by its
// kName (e.g. "joint_position"), or "*" for every named field, never by
// which Algorithm hosts it, and a block several Algorithms share is
// logged once. Resolution happens once, at construction; Gather then
// copies every selected field's raw scalars (View::LoadFlat, through the
// block's Primary member) for every real joint into one flat row.
//
// Row/column order is fixed at construction and matches ColumnNames():
// field (FieldBlocksT) order, then joint (instance, joint) order, then
// the field's own scalar (Assembler leaf) order.
template <AlgorithmLike... Algorithms>
class FieldLog {
  using Blocks = FieldBlocksT<Algorithms...>;
  static constexpr auto kInfos = BlockInfos<Blocks>::kValue;
  static constexpr std::size_t kNumBlocks = kInfos.size();

 public:
  using Context = SimContext<Algorithms...>;
  // Names one real joint for column headers, e.g. "arm.elbow".
  using JointLabeler = std::function<std::string(domain::ArchetypeJointHandle)>;

  // Names no field matches are appended to `unknown`, so the caller can
  // report typos rather than silently logging less.
  FieldLog(
      const Context& context,
      std::span<const std::string> names,
      std::vector<std::string>& unknown
  ) {
    for (const std::string& name : names) {
      bool claimed = false;
      for (std::size_t i = 0; i < kNumBlocks; ++i) {
        if (!kInfos[i].name.empty() &&
            (name == "*" || name == kInfos[i].name)) {
          selected_.set(i);
          claimed = true;
        }
      }
      if (!claimed) {
        unknown.push_back(name);
      }
    }
    ResolveRows(context, Blocks{}, std::make_index_sequence<kNumBlocks>{});
  }

  // Total scalars Gather writes per tick.
  std::size_t Width() const { return width_; }
  bool Empty() const { return width_ == 0; }

  std::vector<std::string> ColumnNames(const JointLabeler& label) const {
    std::vector<std::string> columns;
    columns.reserve(width_);
    for (std::size_t i = 0; i < kNumBlocks; ++i) {
      if (!selected_.test(i)) {
        continue;
      }
      std::string suffix = "." + std::string(kInfos[i].name) + ".";
      for (const domain::ArchetypeJointHandle& handle : handles_[i]) {
        std::string joint = label(handle);
        for (std::size_t k = 0; k < kInfos[i].scalars; ++k) {
          columns.push_back(joint + suffix + std::to_string(k));
        }
      }
    }
    return columns;
  }

  // `out.size()` must equal Width(). Each unselected field costs one
  // bitset test per tick and nothing else.
  template <typename OutT>
  void Gather(const Context& context, std::span<OutT> out) const {
    std::size_t offset = 0;
    GatherFields(
        context, out, offset, Blocks{}, std::make_index_sequence<kNumBlocks>{}
    );
  }

 private:
  // Records, for each selected field, its real (non-padding) rows under
  // its own ordering's JointLayout, sorted into (instance, joint) order.
  template <typename... BlockTs, std::size_t... Is>
  void ResolveRows(
      const Context& context,
      util::TypeList<BlockTs...> /*blocks*/,
      std::index_sequence<Is...> /*indices*/
  ) {
    auto resolve_one = [&]<typename BlockT, std::size_t I>() {
      if (!selected_.test(I)) {
        return;
      }
      const domain::JointLayout& rows =
          context.template LayoutFor<typename BlockT::Ordering>();
      std::vector<std::pair<domain::ArchetypeJointHandle, std::size_t>> real;
      for (std::size_t row = 0; row < rows.PaddedSize(); ++row) {
        if (!rows.IsPadding(row)) {
          real.emplace_back(rows.ToPhysical(row), row);
        }
      }
      std::ranges::sort(real, [](const auto& a, const auto& b) {
        return std::pair(a.first.instance_index, a.first.joint_index) <
               std::pair(b.first.instance_index, b.first.joint_index);
      });
      for (const auto& [handle, row] : real) {
        handles_[I].push_back(handle);
        rows_[I].push_back(row);
      }
      width_ += real.size() * kInfos[I].scalars;
    };
    (resolve_one.template operator()<BlockTs, Is>(), ...);
  }

  template <typename OutT, typename... BlockTs, std::size_t... Is>
  void GatherFields(
      const Context& context,
      std::span<OutT> out,
      std::size_t& offset,
      util::TypeList<BlockTs...> /*blocks*/,
      std::index_sequence<Is...> /*indices*/
  ) const {
    auto gather_one = [&]<typename BlockT, std::size_t I>() {
      if (!selected_.test(I)) {
        return;
      }
      using Primary = typename BlockT::Primary;
      using View = typename Primary::AlgorithmType::View;
      constexpr std::size_t kScalars = BlockT::Assembler::kNumFields;
      const View view =
          context.template ViewFor<typename Primary::AlgorithmType>();
      for (std::size_t row : rows_[I]) {
        view.template LoadFlat<Primary::kField>(
            row, out.subspan(offset, kScalars)
        );
        offset += kScalars;
      }
    };
    (gather_one.template operator()<BlockTs, Is>(), ...);
  }

  std::bitset<kNumBlocks> selected_;
  std::array<std::vector<std::size_t>, kNumBlocks> rows_;
  std::array<std::vector<domain::ArchetypeJointHandle>, kNumBlocks> handles_;
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
