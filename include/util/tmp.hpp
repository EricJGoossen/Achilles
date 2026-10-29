#pragma once

#include <cstddef>
#include <tuple>
#include <type_traits>

namespace achilles::util {
template <typename F>
struct arity_of;

template <typename R, typename C, typename... Args>
struct arity_of<R (C::*)(Args...)>
    : std::integral_constant<std::size_t, sizeof...(Args)> {};

template <typename R, typename C, typename... Args>
struct arity_of<R (C::*)(Args...) const>
    : std::integral_constant<std::size_t, sizeof...(Args)> {};

template <typename F>
inline constexpr std::size_t kArityOfV = arity_of<F>::value;

template <typename F>
struct return_type_of;

template <typename R, typename C, typename... Args>
struct return_type_of<R (C::*)(Args...)> {
  using Type = R;
};

template <typename R, typename C, typename... Args>
struct return_type_of<R (C::*)(Args...) const> {
  using Type = R;
};

template <typename F>
using ReturnTypeOfT = typename return_type_of<F>::Type;

template <typename F>
struct args_of;

template <typename R, typename C, typename... Args>
struct args_of<R (C::*)(Args...)> {
  using Type = std::tuple<Args...>;
};

template <typename R, typename C, typename... Args>
struct args_of<R (C::*)(Args...) const> {
  using Type = std::tuple<Args...>;
};

template <typename F>
using ArgsOfT = typename args_of<F>::Type;

template <typename... Ts>
struct TypeList {};

template <typename... Lists>
struct Concat;

template <>
struct Concat<> {
  using Type = TypeList<>;
};

template <typename... As>
struct Concat<TypeList<As...>> {
  using Type = TypeList<As...>;
};

template <typename... As, typename... Bs, typename... Rest>
struct Concat<TypeList<As...>, TypeList<Bs...>, Rest...> {
  using Type = typename Concat<TypeList<As..., Bs...>, Rest...>::Type;
};

template <typename... Lists>
using ConcatT = typename Concat<Lists...>::Type;

template <typename List, std::size_t N>
struct RepeatList {
  using Type = ConcatT<List, typename RepeatList<List, N - 1>::Type>;
};

template <typename List>
struct RepeatList<List, 0> {
  using Type = TypeList<>;
};

template <typename List, std::size_t N>
using RepeatListT = typename RepeatList<List, N>::Type;

template <typename List>
struct ToTuple;

template <typename... Ts>
struct ToTuple<TypeList<Ts...>> {
  using Type = std::tuple<Ts...>;
};

template <typename List>
using ToTupleT = typename ToTuple<List>::Type;

// De-duplicates a TypeList, keeping first-occurrence order. Used wherever a
// pack of per-field declarations (e.g. each field's own ordering policy
// type) needs to be collapsed down to the distinct set of policies actually
// named, without the caller having to hand-write a branch per known policy.
template <typename Seen, typename Rest>
struct UniqueImpl {
  using Type = Seen;
};

template <typename... SeenTs, typename Next, typename... Rest>
struct UniqueImpl<TypeList<SeenTs...>, TypeList<Next, Rest...>> {
  using Type = typename UniqueImpl<
      std::conditional_t<
          (std::is_same_v<SeenTs, Next> || ...),
          TypeList<SeenTs...>,
          TypeList<SeenTs..., Next>>,
      TypeList<Rest...>>::Type;
};

template <typename List>
struct Unique {
  using Type = typename UniqueImpl<TypeList<>, List>::Type;
};

template <typename List>
using UniqueT = typename Unique<List>::Type;

// Applies a unary type-metafunction to every element of a TypeList. Used
// wherever a bare element type can legitimately repeat across a pack (so it
// can't itself be a distinct std::get key -- e.g. two different ordering
// policies both resolving to the exact same domain::JointTopology type) and
// each element needs wrapping in something that's *per-element* distinct
// (e.g. TopologySlot<Policy>, engine/pass/sim_context.hpp) before ToTupleT
// turns the list into a std::tuple a real std::get can key into.
template <template <typename> class F, typename List>
struct Transform;

template <template <typename> class F, typename... Ts>
struct Transform<F, TypeList<Ts...>> {
  using Type = TypeList<F<Ts>...>;
};

template <template <typename> class F, typename List>
using TransformT = typename Transform<F, List>::Type;

// F<Ts...> for a TypeList<Ts...> -- spreads a list into a variadic
// template, e.g. ApplyT<LeafAssembler, Records>.
template <template <typename...> class F, typename List>
struct Apply;

template <template <typename...> class F, typename... Ts>
struct Apply<F, TypeList<Ts...>> {
  using Type = F<Ts...>;
};

template <template <typename...> class F, typename List>
using ApplyT = typename Apply<F, List>::Type;

// How many types a TypeList holds.
template <typename List>
inline constexpr std::size_t kSizeV = 0;
template <typename... Ts>
inline constexpr std::size_t kSizeV<TypeList<Ts...>> = sizeof...(Ts);

// Where T first appears in a TypeList; T must be in it.
template <typename T, typename List>
struct IndexOf;

template <typename T, typename... Rest>
struct IndexOf<T, TypeList<T, Rest...>> {
  static constexpr std::size_t kValue = 0;
};

template <typename T, typename First, typename... Rest>
struct IndexOf<T, TypeList<First, Rest...>> {
  static constexpr std::size_t kValue =
      1 + IndexOf<T, TypeList<Rest...>>::kValue;
};

template <typename T, typename List>
inline constexpr std::size_t kIndexOfV = IndexOf<T, List>::kValue;

// Op<Args...> if that names a valid type, Default otherwise -- an optional
// member type with a fallback (std::experimental::detected_or). Op is usually
// an alias template naming one member, e.g.
//   template <typename T> using OrderingOf = typename T::Ordering;
//   DetectedOrT<LinearOrdering, OrderingOf, Traits<F>>
template <typename Default, template <typename...> class Op, typename... Args>
struct DetectedOr {
  using Type = Default;
};

template <typename Default, template <typename...> class Op, typename... Args>
  requires requires { typename Op<Args...>; }
struct DetectedOr<Default, Op, Args...> {
  using Type = Op<Args...>;
};

template <typename Default, template <typename...> class Op, typename... Args>
using DetectedOrT = typename DetectedOr<Default, Op, Args...>::Type;

// True when T is one of Ts....
template <typename T, typename... Ts>
inline constexpr bool kIsOneOfV = (std::is_same_v<T, Ts> || ...);

// A std::ratio's value as T -- lets a compile-time constant (a stage
// weight, a fraction of dt) be a template argument where a floating-point
// non-type template argument isn't available.
template <typename T, typename Ratio>
inline constexpr T kRatioValueV =
    static_cast<T>(Ratio::num) / static_cast<T>(Ratio::den);
}  // namespace achilles::util