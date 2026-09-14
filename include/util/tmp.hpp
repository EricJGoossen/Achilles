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
}  // namespace achilles::util