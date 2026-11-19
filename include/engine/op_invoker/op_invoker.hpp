// Calls an op with the field values a traversal points at.
// Doc: docs/code/engine_op_invoker_op_invoker.md

#pragma once

#include <algorithm>
#include <array>
#include <concepts>
#include <cstddef>
#include <tuple>
#include <type_traits>
#include <utility>

#include "engine/op_invoker/op_initializer.hpp"
#include "engine/spec/arg_data.hpp"
#include "engine/spec/op_contract.hpp"
#include "engine/spec/view_contract.hpp"
#include "util/tmp.hpp"

namespace achilles::engine::op_invoker {

// Wraps an Op -- a callable with `operator()` and two arrays,
// `static constexpr std::array<ArgData<...>, N> kInputs` and
// `static constexpr std::array<ArgData<...>, M> kOutputs` -- together with
// a View. At construction it pulls one FieldCursor per kInputs/kOutputs
// entry out of the View and keeps those, not the View itself, so the hot
// loop never repeats the View's pointer/stride lookup. Invoking it with a
// (target, source) index pair -- source being a TreeTraversal's parent
// group, a LinearTraversal's own target again, or a PairTraversal's
// policy::indexing::BroadcastIndex (see policy/traversal/) -- packs both
// into one tuple, ordered the way ArgIndex numbers them, and:
//   1. Reads each of kInputs and kOutputs through its cursor, at whichever
//      element that argument's field's own Indexing policy resolves the
//      tuple to for its ArgData::index -- see FieldCursor::Resolve. An
//      output resolved to a policy::indexing::BroadcastIndex starts from zero
//      instead (FieldCursor::LoadOutput).
//   2. Calls Op::operator() with the inputs by const reference, then each
//      slot of the tuple kTraversalInputs names handed over as-is (if the
//      Op declares any -- see kTraversalInputCountV,
//      engine/spec/op_contract.hpp), then a pointer to the loaded value of
//      each of kOutputs.
//   3. Writes each output back through its cursor, resolved the same way,
//      in kOutputs order.
// That is the whole Op contract: inputs are read-only, outputs are always
// read/write. An output's incoming value is whatever the View holds at that
// index -- an accumulator to add onto (vi's IntegrateVelocityOp, ABA's
// PropagateInertiaOp into its parent), a previous value an Op means to
// read, or stale data an Op simply overwrites. Which of those it is, is the
// Op's own business to document; the invoker treats them all the same.
// The one exception is an output resolved to a BroadcastIndex -- a pair
// pass's kSource row, or a batched read of a GlobalIndexing field: it's a
// reduction, handed in as zero and stored as row += (sum of its lanes)
// (see policy::indexing::BroadcastIndex). Its store reads the row at store
// time, so list it after any group output that might cover the same memory.
// Initialize (OpInitInvoker below) follows the same contract.
//
// Op is borrowed, not copied: operator() is const, so the invoker never
// needs its own copy to call through -- it just binds a reference to
// whatever instance the caller (RunPass/RunSeed) already has, for however
// large an Op's configured state grows.
template <spec::OpLike Op, typename View>
  requires spec::ViewLike<View, typename Op::FieldEnum>
class OpInvoker : private OpInitializer<Op, View> {
 private:
  using InputSeq = std::make_index_sequence<Op::kInputs.size()>;
  using TraversalInputSeq =
      std::make_index_sequence<spec::kTraversalInputCountV<Op>>;
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

 public:
  explicit OpInvoker(View& view, const Op& op)
      : OpInitializer<Op, View>(view, op),
        op_(&op),
        input_cursors_(MakeInputCursors(view, InputSeq{})),
        output_cursors_(MakeOutputCursors(view, OutputSeq{})) {}

 private:
  // The T to gather/scatter field I as: operator()'s own declared
  // parameter type at that position names it (ParamScalarT), so a field
  // that's batched needs no different handling here than one that isn't
  // -- Load<T>/Store<T> take whichever T the Op actually asked for.
  template <size_t I>
  using InputParam = std::remove_cvref_t<
      std::tuple_element_t<I, util::ArgsOfT<decltype(&Op::operator())>>>;
  template <size_t I>
  using OutputParam = std::remove_pointer_t<std::tuple_element_t<
      Op::kInputs.size() + spec::kTraversalInputCountV<Op> + I,
      util::ArgsOfT<decltype(&Op::operator())>>>;

  // The K-th kTraversalInputs value: whatever the traversal put in the
  // tuple slot it names, unchanged -- no cursor, no field.
  template <size_t K, typename Indices>
  static const auto& TraversalInputAt(const Indices& indices) {
    return std::get<static_cast<size_t>(Op::kTraversalInputs[K])>(indices);
  }

  // Gathers/scatters one argument through `cursor` at whatever element its
  // own field's Indexing policy resolves `indices` to for Index -- the only
  // place an argument's ArgData::index is ever consulted. An element that
  // resolves to a policy::indexing::BroadcastIndex picks the cursor's
  // broadcasting Load overload as an input, and its zeroed LoadOutput /
  // accumulating Store overloads as an output.
  template <typename T, spec::ArgIndex Index, typename Cursor, typename Indices>
  static auto LoadAt(const Cursor& cursor, const Indices& indices) {
    return cursor.template Load<T>(
        cursor.template Resolve<T, static_cast<size_t>(Index)>(indices)
    );
  }
  template <typename T, spec::ArgIndex Index, typename Cursor, typename Indices>
  static auto LoadOutputAt(const Cursor& cursor, const Indices& indices) {
    return cursor.template LoadOutput<T>(
        cursor.template Resolve<T, static_cast<size_t>(Index)>(indices)
    );
  }
  template <
      typename T,
      spec::ArgIndex Index,
      typename Cursor,
      typename Indices,
      typename ValueT>
  static void StoreAt(
      const Cursor& cursor, const Indices& indices, const ValueT& value
  ) {
    cursor.template Store<T>(
        cursor.template Resolve<T, static_cast<size_t>(Index)>(indices), value
    );
  }

  // -- In place or copied. An argument whose field holds it as a whole
  // record (FieldCursor::kHoldsRecord), at a plain element index (not a
  // BroadcastIndex), is handed to the Op as the record in memory itself --
  // a const reference for an input, a pointer for an output -- rather than
  // copied in and out, which for a large record (a lidar's beams) would
  // cost far more than the Op's own work. Everything else -- scalars,
  // batches, values assembled from several leaves, broadcasts and
  // reductions -- is gathered and scattered as before. An output whose
  // field the Op also reads keeps the copy, so it never sees its own write
  // through that input mid-call.

  template <typename T, spec::ArgIndex Index, typename Cursor, typename Indices>
  static constexpr bool kInPlace =
      Cursor::template kHoldsRecord<T> &&
      std::same_as<
          decltype(std::declval<const Cursor&>()
                       .template Resolve<T, static_cast<size_t>(Index)>(
                           std::declval<const Indices&>()
                       )),
          size_t>;

  template <size_t I>
  static constexpr bool kOutputAlsoRead =
      std::ranges::any_of(Op::kInputs, [](const auto& input) {
        return input.field == Op::kOutputs[I].field;
      });

  template <size_t I>
  using InputCursorT = std::tuple_element_t<I, InputCursors>;
  template <size_t I>
  using OutputCursorT = std::tuple_element_t<I, OutputCursors>;

  template <size_t I, typename Indices>
  static constexpr bool kInputInPlace = kInPlace<
      spec::ParamScalarT<InputParam<I>>,
      Op::kInputs[I].index,
      InputCursorT<I>,
      Indices>;
  template <size_t I, typename Indices>
  static constexpr bool kOutputInPlace =
      !kOutputAlsoRead<I> && kInPlace<
                                 spec::ParamScalarT<OutputParam<I>>,
                                 Op::kOutputs[I].index,
                                 OutputCursorT<I>,
                                 Indices>;

  template <size_t I, typename Indices>
    requires kInputInPlace<I, Indices>
  const InputParam<I>& InputAt(const Indices& indices) const {
    const auto& cursor = std::get<I>(this->input_cursors_);
    return *cursor.template Ref<InputParam<I>>(
        cursor.template Resolve<
            InputParam<I>,
            static_cast<size_t>(Op::kInputs[I].index)>(indices)
    );
  }
  template <size_t I, typename Indices>
    requires(!kInputInPlace<I, Indices>)
  auto InputAt(const Indices& indices) const {
    return LoadAt<spec::ParamScalarT<InputParam<I>>, Op::kInputs[I].index>(
        std::get<I>(this->input_cursors_), indices
    );
  }

  // An output the Op writes through: its own copy, stored back after the
  // call, or the record in memory.
  template <typename T>
  struct CopiedOutput {
    T value;
  };
  template <typename T>
  struct InPlaceOutput {
    T* record;
  };
  // Where the Op writes either one.
  template <typename T>
  static T* Target(CopiedOutput<T>& output) {
    return &output.value;
  }
  template <typename T>
  static T* Target(InPlaceOutput<T>& output) {
    return output.record;
  }

  template <size_t I, typename Indices>
    requires kOutputInPlace<I, Indices>
  InPlaceOutput<OutputParam<I>> OutputAt(const Indices& indices) const {
    const auto& cursor = std::get<I>(this->output_cursors_);
    return {cursor.template Ref<OutputParam<I>>(
        cursor.template Resolve<
            OutputParam<I>,
            static_cast<size_t>(Op::kOutputs[I].index)>(indices)
    )};
  }
  template <size_t I, typename Indices>
    requires(!kOutputInPlace<I, Indices>)
  CopiedOutput<OutputParam<I>> OutputAt(const Indices& indices) const {
    return {
        LoadOutputAt<spec::ParamScalarT<OutputParam<I>>, Op::kOutputs[I].index>(
            std::get<I>(this->output_cursors_), indices
        )
    };
  }

  template <size_t I, typename Indices>
  void Commit(
      const InPlaceOutput<OutputParam<I>>& /*output*/,
      const Indices& /*indices*/
  ) const {}
  template <size_t I, typename Indices>
  void Commit(
      const CopiedOutput<OutputParam<I>>& output, const Indices& indices
  ) const {
    StoreAt<spec::ParamScalarT<OutputParam<I>>, Op::kOutputs[I].index>(
        std::get<I>(this->output_cursors_), indices, output.value
    );
  }

  template <typename Indices, size_t... InIs, size_t... TrIs, size_t... OutIs>
  void
  Invoke(const Indices& indices, std::index_sequence<InIs...>, std::index_sequence<TrIs...>, std::index_sequence<OutIs...>)
      const {
    auto outputs = std::tuple{OutputAt<OutIs>(indices)...};

    this->op_->operator()(
        InputAt<InIs>(indices)...,
        TraversalInputAt<TrIs>(indices)...,
        Target(std::get<OutIs>(outputs))...
    );

    (Commit<OutIs>(std::get<OutIs>(outputs), indices), ...);
  }

 private:
  const Op* op_;
  InputCursors input_cursors_;
  OutputCursors output_cursors_;

 public:
  template <typename SourceIndex, typename... Extra>
  void operator()(
      size_t target_index, SourceIndex source_index, const Extra&... extra
  ) const {
    this->Invoke(
        std::tuple<size_t, SourceIndex, Extra...>{
            target_index, source_index, extra...
        },
        InputSeq{},
        TraversalInputSeq{},
        OutputSeq{}
    );
  }

  void Initialize(size_t base_index) const
    requires spec::OpHasInit<Op>
  {
    this->InitializeAt(base_index);
  }
};

}  // namespace achilles::engine::op_invoker
