// Calls an op's Initialize at the base row, for ops that declare one.

#pragma once

#include <algorithm>
#include <array>
#include <concepts>
#include <cstddef>
#include <tuple>
#include <type_traits>
#include <utility>

#include "engine/spec/arg_data.hpp"
#include "engine/spec/op_contract.hpp"
#include "engine/spec/view_contract.hpp"
#include "util/tmp.hpp"

namespace achilles::engine::op_invoker {

// Optional second half of an invoker: builds Initialize-specific cursors
// (from kInitInputs/kInitOutputs, distinct from OpInvokerBase's own
// kInputs/kOutputs cursors -- a field can appear in both arrays with
// different ArgData::index values, e.g. PropagateInertiaOp's
// kArticulatedInertia, and the two calls must still land on the index
// their own ArgData says) and calls Op::Initialize the same
// gather/call/scatter way OpInvokerBase::Invoke calls operator(), just
// always at a single base index instead of a (target, parent) pair.
//
// Split out from OpInvokerBase (rather than folded into it) because
// kInitInputs/kInitOutputs/Initialize don't exist on every Op -- OpHasInit,
// not OpLike, gates them -- so this has to be conditionally present
// without OpInvokerBase ever naming Op::kInitInputs for an Op that never
// declared it. The primary template (HasInit = false) is the empty case;
// only the true specialization below does anything.
template <typename Op, typename View, bool HasInit = spec::OpHasInit<Op>>
class OpInitializer {
 protected:
  explicit OpInitializer(View&, const Op&) {}
};

template <typename Op, typename View>
class OpInitializer<Op, View, true> {
  using InitInputSeq = std::make_index_sequence<Op::kInitInputs.size()>;
  using InitOutputSeq = std::make_index_sequence<Op::kInitOutputs.size()>;

  template <size_t I>
  using InitInputCursor =
      typename View::template FieldCursor<Op::kInitInputs[I].field>;
  template <size_t I>
  using InitOutputCursor =
      typename View::template FieldCursor<Op::kInitOutputs[I].field>;

  template <size_t... Is>
  static auto InitInputCursorsType(std::index_sequence<Is...>)
      -> std::tuple<InitInputCursor<Is>...>;
  template <size_t... Is>
  static auto InitOutputCursorsType(std::index_sequence<Is...>)
      -> std::tuple<InitOutputCursor<Is>...>;

  using InitInputCursors = decltype(InitInputCursorsType(InitInputSeq{}));
  using InitOutputCursors = decltype(InitOutputCursorsType(InitOutputSeq{}));

  template <size_t... Is>
  static InitInputCursors
  MakeInitInputCursors(View& view, std::index_sequence<Is...>) {
    return InitInputCursors(view.template Field<Op::kInitInputs[Is].field>()...
    );
  }
  template <size_t... Is>
  static InitOutputCursors
  MakeInitOutputCursors(View& view, std::index_sequence<Is...>) {
    return InitOutputCursors(view.template Field<Op::kInitOutputs[Is].field>(
    )...);
  }

  template <size_t I>
  using InitInputParam = std::remove_cvref_t<
      std::tuple_element_t<I, util::ArgsOfT<decltype(&Op::Initialize)>>>;
  template <size_t I>
  using InitOutputParam = std::remove_pointer_t<std::tuple_element_t<
      Op::kInitInputs.size() + I,
      util::ArgsOfT<decltype(&Op::Initialize)>>>;

 protected:
  explicit OpInitializer(View& view, const Op& op)
      : init_op_(&op),
        init_input_cursors_(MakeInitInputCursors(view, InitInputSeq{})),
        init_output_cursors_(MakeInitOutputCursors(view, InitOutputSeq{})) {}

  void InitializeAt(size_t base_index) const {
    InitializeImpl(base_index, InitInputSeq{}, InitOutputSeq{});
  }

 private:
  template <size_t... InIs, size_t... OutIs>
  void
  InitializeImpl(size_t base_index, std::index_sequence<InIs...>, std::index_sequence<OutIs...>)
      const {
    std::tuple<InitOutputParam<OutIs>...> outputs{
        std::get<OutIs>(init_output_cursors_)
            .template Load<spec::ParamScalarT<InitOutputParam<OutIs>>>(
                base_index
            )...
    };

    init_op_->Initialize(
        std::get<InIs>(init_input_cursors_)
            .template Load<spec::ParamScalarT<InitInputParam<InIs>>>(base_index
            )...,
        &std::get<OutIs>(outputs)...
    );

    (std::get<OutIs>(init_output_cursors_)
         .template Store<spec::ParamScalarT<InitOutputParam<OutIs>>>(
             base_index, std::get<OutIs>(outputs)
         ),
     ...);
  }

  const Op* init_op_;
  InitInputCursors init_input_cursors_;
  InitOutputCursors init_output_cursors_;
};

}  // namespace achilles::engine::op_invoker
