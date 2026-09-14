#pragma once

#include <array>
#include <concepts>
#include <cstddef>
#include <tuple>
#include <type_traits>
#include <utility>

#include "engine/op_contract.hpp"
#include "engine/view/view_contract.hpp"
#include "util/tmp.hpp"

namespace achilles::engine {

// Wraps an Op -- a callable with `operator()` and two arrays,
// `static constexpr std::array<ArgData<...>, N> kInputs` and
// `static constexpr std::array<ArgData<...>, M> kOutputs` -- together with
// a View. At construction it pulls one FieldCursor per kInputs/kOutputs
// entry out of the View and keeps those, not the View itself, so the hot
// loop never repeats the View's pointer/stride lookup. Invoking it with a
// (target, parent) index pair:
//   1. Reads each of kInputs through its cursor (from `target_index` or
//      `parent_index`, per that argument's `use_target`).
//   2. Calls Op::operator() with those values, followed by a pointer to a
//      freshly default-constructed local for each of kOutputs.
//   3. Writes each local back through its cursor, again at `target_index`
//      or `parent_index` per that output's `use_target`.
// Arguments are strictly one or the other: an Op contract has no room for
// a parameter that is both read from and written back to the View.
//
// Op is borrowed, not copied: operator() is const, so the invoker never
// needs its own copy to call through -- it just binds a reference to
// whatever instance the caller (RunPass/RunSeed) already has, for however
// large an Op's configured state grows.
template <OpLike Op, typename View>
  requires view::ViewLike<View, typename Op::FieldEnum>
class OpInvokerBase {
 protected:
  using InputSeq = std::make_index_sequence<Op::kInputs.size()>;
  using OutputSeq = std::make_index_sequence<Op::kOutputs.size()>;

 private:
  template <size_t I>
  using InputCursor = typename View::template FieldCursor<Op::kInputs[I].field>;
  template <size_t I>
  using OutputCursor =
      typename View::template FieldCursor<Op::kOutputs[I].field>;

  // Declared, never defined: used only inside decltype() below to name
  // the cursor tuple types without restating the index pack.
  template <size_t... Is>
  static auto InputCursorsType(std::index_sequence<Is...>)
      -> std::tuple<InputCursor<Is>...>;
  template <size_t... Is>
  static auto OutputCursorsType(std::index_sequence<Is...>)
      -> std::tuple<OutputCursor<Is>...>;

  using InputCursors = decltype(InputCursorsType(InputSeq{}));
  using OutputCursors = decltype(OutputCursorsType(OutputSeq{}));

  template <size_t... Is>
  static InputCursors MakeInputCursors(View& view, std::index_sequence<Is...>) {
    return InputCursors(view.template Field<Op::kInputs[Is].field>()...);
  }
  template <size_t... Is>
  static OutputCursors
  MakeOutputCursors(View& view, std::index_sequence<Is...>) {
    return OutputCursors(view.template Field<Op::kOutputs[Is].field>()...);
  }

 protected:
  explicit OpInvokerBase(View& view, const Op& op)
      : op_(op),
        input_cursors_(MakeInputCursors(view, InputSeq{})),
        output_cursors_(MakeOutputCursors(view, OutputSeq{})) {}

  // The T to gather/scatter field I as: operator()'s own declared
  // parameter type at that position names it via ScalarType, so a field
  // that's batched needs no different handling here than one that isn't
  // -- Load<T>/Store<T> take whichever T the Op actually asked for.
  template <size_t I>
  using InputParam = std::remove_cvref_t<
      std::tuple_element_t<I, util::ArgsOfT<decltype(&Op::operator())>>>;
  template <size_t I>
  using OutputParam = std::remove_pointer_t<std::tuple_element_t<
      Op::kInputs.size() + I,
      util::ArgsOfT<decltype(&Op::operator())>>>;

  template <size_t... InIs, size_t... OutIs>
  void
  Invoke(size_t target_index, size_t parent_index, std::index_sequence<InIs...>, std::index_sequence<OutIs...>)
      const {
    std::tuple<OutputParam<OutIs>...> outputs;

    this->op_(
        std::get<InIs>(this->input_cursors_)
            .template Load<typename InputParam<InIs>::ScalarType>(
                Op::kInputs[InIs].use_target ? target_index : parent_index
            )...,
        &std::get<OutIs>(outputs)...
    );

    (std::get<OutIs>(this->output_cursors_)
         .template Store<typename OutputParam<OutIs>::ScalarType>(
             Op::kOutputs[OutIs].use_target ? target_index : parent_index,
             std::get<OutIs>(outputs)
         ),
     ...);
  }

  template <size_t... InIs, size_t... OutIs>
  void
  Invoke(size_t target_index, std::index_sequence<InIs...>, std::index_sequence<OutIs...>)
      const {
    std::tuple<OutputParam<OutIs>...> outputs;

    this->op_(
        std::get<InIs>(this->input_cursors_)
            .template Load<typename InputParam<InIs>::ScalarType>(target_index
            )...,
        &std::get<OutIs>(outputs)...
    );

    (std::get<OutIs>(this->output_cursors_)
         .template Store<typename OutputParam<OutIs>::ScalarType>(
             target_index, std::get<OutIs>(outputs)
         ),
     ...);
  }

  const Op& op_;
  InputCursors input_cursors_;
  OutputCursors output_cursors_;
};

// Optional second half of an invoker: builds Initialize-specific cursors
// (from kInitInputs/kInitOutputs, distinct from OpInvokerBase's own
// kInputs/kOutputs cursors -- a field can appear in both arrays with
// different use_target values, e.g. PropagateInertiaOp's
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
template <typename Op, typename View, bool HasInit = OpHasInit<Op>>
class OpInitInvoker {
 protected:
  explicit OpInitInvoker(View&, const Op&) {}
};

template <typename Op, typename View>
class OpInitInvoker<Op, View, true> {
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
  explicit OpInitInvoker(View& view, const Op& op)
      : init_op_(op),
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
    std::tuple<InitOutputParam<OutIs>...> outputs;

    init_op_.Initialize(
        std::get<InIs>(init_input_cursors_)
            .template Load<typename InitInputParam<InIs>::ScalarType>(base_index
            )...,
        &std::get<OutIs>(outputs)...
    );

    (std::get<OutIs>(init_output_cursors_)
         .template Store<typename InitOutputParam<OutIs>::ScalarType>(
             base_index, std::get<OutIs>(outputs)
         ),
     ...);
  }

  const Op& init_op_;
  InitInputCursors init_input_cursors_;
  InitOutputCursors init_output_cursors_;
};

template <OpLike Op, typename View>
  requires view::ViewLike<View, typename Op::FieldEnum>
class OpInvoker : public OpInvokerBase<Op, View>,
                  private OpInitInvoker<Op, View> {
  using Base = OpInvokerBase<Op, View>;
  using InitBase = OpInitInvoker<Op, View>;

 public:
  explicit OpInvoker(View& view, const Op& op)
      : Base(view, op), InitBase(view, op) {}

  void operator()(size_t target_index, size_t parent_index) const {
    this->Invoke(
        target_index,
        parent_index,
        typename Base::InputSeq{},
        typename Base::OutputSeq{}
    );
  }

  void Initialize(size_t base_index) const
    requires OpHasInit<Op>
  {
    this->InitializeAt(base_index);
  }
};

template <OpLike Op, typename View>
  requires view::ViewLike<View, typename Op::FieldEnum>
class SingleOpInvoker : public OpInvokerBase<Op, View> {
  using Base = OpInvokerBase<Op, View>;

 public:
  explicit SingleOpInvoker(View& view, const Op& op) : Base(view, op) {}

  void operator()(size_t target_index) const {
    this->Invoke(
        target_index, typename Base::InputSeq{}, typename Base::OutputSeq{}
    );
  }

  void Initialize(size_t base_index) const
    requires OpHasInit<Op>
  {
    this->InitializeAt(base_index);
  }
};

}  // namespace achilles::engine
