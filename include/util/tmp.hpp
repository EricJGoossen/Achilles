#pragma once

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
}  // namespace achilles::util