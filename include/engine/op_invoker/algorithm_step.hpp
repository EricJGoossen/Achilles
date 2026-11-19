// Runs a list of passes in order.

#pragma once

#include <tuple>
#include <type_traits>
#include <utility>

#include "engine/op_invoker/op_initializer.hpp"
#include "engine/op_invoker/op_invoker.hpp"
#include "engine/spec/arg_data.hpp"
#include "engine/spec/op_contract.hpp"
#include "engine/spec/view_contract.hpp"
#include "policy/traversal/base_traversal.hpp"
#include "policy/traversal/global_traversal.hpp"
#include "policy/traversal/pair_traversal.hpp"
#include "policy/traversal/row_traversal.hpp"
#include "util/tmp.hpp"

namespace achilles::engine::op_invoker {

// One Op, walked by one Traversal. `Step` below runs a pack of these
// back-to-back, so an algorithm's outer loop is just the ordered list of
// (Op, Traversal) passes it's made of. Traversal is never defaulted: name
// engine::TreeTraversal<Direction::kForward> (or whichever) explicitly at
// every Pass.
template <spec::OpLike Op, policy::traversal::TraversalLike Traversal>
struct Pass {
  using OpType = Op;
  using TraversalType = Traversal;
};

template <typename T>
concept OpWalkLike =
    requires {
      typename T::OpType;
      typename T::TraversalType;
    } && spec::OpLike<typename T::OpType> &&
    policy::traversal::TraversalLike<typename T::TraversalType>;

// `op` is borrowed, not owned: OpInvoker only ever reads through it
// (operator() is const), so there's no reason for RunPass or the invoker
// underneath to hold its own copy of what could be a large,
// per-pass-configured Op.
//
// If OpType declares Initialize (see OpHasInit,
// engine/spec/op_contract.hpp), it's called once at the traversal's base
// row before the main per-target Apply runs -- seeding whatever Apply reads
// from its own parent index on the first (root) iteration, e.g. the world
// transform/velocity a root joint's PropagateVelocityOp reads as
// x_world_parent/v_parent (see algorithms/aba/aba_ops.hpp for the real
// example). Not every Op needs this, so it's gated on OpHasInit rather than
// called unconditionally -- an Op without Initialize simply skips straight to
// Apply.
template <OpWalkLike PassT, typename View, typename... Args>
  requires spec::OpArgsMatchView<typename PassT::OpType, View>
void RunPass(
    View& view, const typename PassT::OpType& op, const Args&... args
) {
  OpInvoker<typename PassT::OpType, View> invoker(view, op);
  if constexpr (spec::OpHasInit<typename PassT::OpType>) {
    PassT::TraversalType::InitOp(invoker, args...);
  }
  PassT::TraversalType::Apply(invoker, args...);
}

// Pairs each of `Passes` with the actual Op instance to run it with -- e.g.
// a PropagateVelocityOp carrying real base-transform/base-velocity state
// for whichever of its passes needs Initialize. No default constructor for
// a non-empty pack: every pass must be given a real instance, there's no
// implicit Op{} to fall back on.
template <OpWalkLike... Passes>
struct Ops {
  using PassList = util::TypeList<Passes...>;

  explicit Ops(typename Passes::OpType... ops)
      : instances_(std::move(ops)...) {}

  // The Op instances, in pass order (see JoinOps).
  const std::tuple<typename Passes::OpType...>& Instances() const {
    return instances_;
  }

 private:
  std::tuple<typename Passes::OpType...> instances_;

  template <OpWalkLike... P, typename View, typename... Args>
    requires(spec::OpArgsMatchView<typename P::OpType, View> && ...)
  friend void Step(const Ops<P...>& ops, View view, const Args&... args);
};

// `ops` is a real value carrying each pass's actual Op instance -- Passes
// is deduced from it, so a call site names no template arguments at all:
// Step(Ops<...>{...}, view, args...).
template <OpWalkLike... Passes, typename View, typename... Args>
  requires(spec::OpArgsMatchView<typename Passes::OpType, View> && ...)
void Step(const Ops<Passes...>& ops, View view, const Args&... args) {
  std::apply(
      [&](const auto&... op_values) {
        (RunPass<Passes>(view, op_values, args...), ...);
      },
      ops.instances_
  );
}

// Every Ops' passes, in order, as one Ops -- so a Step built from several
// lists (one generated per segment, say) is still one Step call.
template <OpWalkLike... Passes, typename... Rest>
auto JoinOps(const Ops<Passes...>& first, const Rest&... rest) {
  using Joined = util::ApplyT<
      Ops,
      util::ConcatT<util::TypeList<Passes...>, typename Rest::PassList...>>;
  return std::make_from_tuple<Joined>(
      std::tuple_cat(first.Instances(), rest.Instances()...)
  );
}

// Keeps every type (SegmentOps' default filter).
template <typename T>
struct KeepAll : std::true_type {};

namespace detail {

// Segment K's pass -- Op<T> over TraversalAt<K> -- or none when Keep
// rejects T. A struct rather than a conditional, so a rejected T's Op is
// never named.
template <
    template <typename>
    class Op,
    template <size_t>
    class TraversalAt,
    typename T,
    size_t K,
    bool Kept>
struct SegmentPass {
  using Type = util::TypeList<>;

  template <typename... Args>
  static std::tuple<> Make(const Args&... /*args*/) {
    return {};
  }
};

template <
    template <typename>
    class Op,
    template <size_t>
    class TraversalAt,
    typename T,
    size_t K>
struct SegmentPass<Op, TraversalAt, T, K, true> {
  using Type = util::TypeList<OpWalk<Op<T>, TraversalAt<K>>>;

  template <typename... Args>
  static std::tuple<Op<T>> Make(const Args&... args) {
    return {Op<T>(args...)};
  }
};

template <
    template <typename>
    class Op,
    template <size_t>
    class TraversalAt,
    template <typename>
    class Keep,
    typename List,
    typename Segments>
struct SegmentOpsFor;

template <
    template <typename>
    class Op,
    template <size_t>
    class TraversalAt,
    template <typename>
    class Keep,
    typename... Ts,
    size_t... Ks>
struct SegmentOpsFor<
    Op,
    TraversalAt,
    Keep,
    util::TypeList<Ts...>,
    std::index_sequence<Ks...>> {
  using Type = util::ApplyT<
      Ops,
      util::ConcatT<
          typename SegmentPass<Op, TraversalAt, Ts, Ks, Keep<Ts>::value>::
              Type...>>;

  template <typename... Args>
  static Type Make(const Args&... args) {
    return std::make_from_tuple<Type>(std::tuple_cat(
        SegmentPass<Op, TraversalAt, Ts, Ks, Keep<Ts>::value>::Make(args...)...
    ));
  }
};

}  // namespace detail

// One pass per type T_K of List that Keep accepts, each running Op<T_K>
// over only segment K of the topology (TraversalAt<K> -- e.g.
// LinkedSegmentTraversal, or LinkedSegmentScanTraversal to pair it against
// a second layout) -- for an algorithm whose rows are a
// policy::ordering::ListOrdering of List, where each list's entries get an Op
// written for that list alone. A type Keep rejects gets no pass, and the
// rest keep their own segment index. Every Op is built from the same
// `args`. Hand the result to Step like any Ops (JoinOps to run several).
template <
    template <typename>
    class Op,
    typename List,
    template <size_t>
    class TraversalAt,
    template <typename> class Keep = KeepAll,
    typename... Args>
auto SegmentOps(const Args&... args) {
  return detail::SegmentOpsFor<
      Op,
      TraversalAt,
      Keep,
      List,
      std::make_index_sequence<util::kSizeV<List>>>::Make(args...);
}

}  // namespace achilles::engine::op_invoker
