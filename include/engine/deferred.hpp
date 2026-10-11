#pragma once

#include <bit>
#include <cstddef>
#include <type_traits>

namespace achilles::engine {

// An Op input that is read when the Op asks for it, rather than gathered
// before the Op runs.
//
// OpInvoker (engine/pass/op_invoker.hpp) hands an Op every plain `const P&`
// input already loaded. For an input that is large and only needed on one
// branch of the Op, or only late in it, that keeps the whole value live
// across everything before its use -- for a 6x6 matrix of SIMD batches,
// far more values than there are registers, so they are spilled to the
// stack as soon as they're loaded and reloaded where they're used.
// Declaring the parameter `const Deferred<P>&` instead (same kInputs
// entry, same P) gets a handle to that field's storage for this row, plus
// the field's own Assembler::Read to assemble it with, and Load() does the
// read at the point it's called -- as many times as it's called.
//
// Also implicitly constructible from a plain P, so code calling an Op
// directly (e.g. a test) passes values exactly as before; Load() then
// returns a copy of that value, which must outlive the Deferred.
template <typename P>
class Deferred {
 public:
  using ScalarType = typename P::ScalarType;
  using ValueType = P;
  using ReadFn = P (*)(const std::byte*, std::size_t);

  constexpr Deferred(const std::byte* field0, std::size_t stride, ReadFn read)
      : field0_(field0), stride_(stride), read_(read) {}

  // NOLINTNEXTLINE(google-explicit-constructor) -- see the class comment.
  constexpr Deferred(const P& value)
      : field0_(std::bit_cast<const std::byte*>(&value)),
        stride_(0),
        read_(&CopyOf) {}

  [[gnu::always_inline]] P Load() const { return read_(field0_, stride_); }

 private:
  static P CopyOf(const std::byte* value, std::size_t /*stride*/) {
    return *std::bit_cast<const P*>(value);
  }

  const std::byte* field0_;
  std::size_t stride_;
  ReadFn read_;
};

template <typename T>
struct IsDeferred : std::false_type {};
template <typename P>
struct IsDeferred<Deferred<P>> : std::true_type {};

template <typename T>
inline constexpr bool kIsDeferred = IsDeferred<std::remove_cvref_t<T>>::value;

}  // namespace achilles::engine
