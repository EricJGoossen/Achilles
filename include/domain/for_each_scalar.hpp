#pragma once

#include <tuple>
#include <type_traits>

namespace achilles::domain {

// Calls `fn` with every scalar of `value`, in the order its ToTuple()
// nests them -- the same order its Assembler stores the leaves in (an
// Assembler writes exactly what ToTuple() returns, recursing into nested
// domain types the same way). A plain arithmetic value is its own one
// scalar.
template <typename ValueT, typename Fn>
constexpr void ForEachScalar(const ValueT& value, const Fn& fn) {
  if constexpr (std::is_arithmetic_v<ValueT>) {
    fn(value);
  } else {
    std::apply(
        [&fn](const auto&... children) { (ForEachScalar(children, fn), ...); },
        value.ToTuple()
    );
  }
}

}  // namespace achilles::domain
