// The base every buffering policy derives from, its concepts, and the
// parity a buffer group shares.
// Doc: docs/code/policy_buffering_base_buffering.md

#pragma once

#include <bit>
#include <cassert>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "util/tmp.hpp"

namespace achilles::policy::buffering {

// Which half of a group's buffer its read side addresses -- only ever 0 or
// 1. Carved once per group in the Arena by SimAllocator (see
// pass::BufferParitySlot), so every View and every BufferedScalar handle
// pointing at it sees the same flip, no matter how many copies of a View or
// SimContext are alive.
struct BufferParity {
  std::uint32_t value = 0;
};

// Which half of a group's buffer a field sees (see DoubleBuffering).
enum class BufferSide : std::uint8_t { kWrite, kRead };

// A buffering policy that belongs to a group (DoubleBuffering), and so reads
// that group's parity.
template <typename P>
concept GroupBufferedLike = requires { typename P::BufferGroup; };

namespace detail {
template <typename P>
using DeclaredStorage = typename P::Storage;
}  // namespace detail

// A buffering policy's storage identity -- what two fields sharing a block
// must agree on -- the policy itself unless it declares a Storage.
template <typename P>
using BufferStorageT = util::DetectedOrT<P, detail::DeclaredStorage, P>;

template <typename P>
concept BufferingPolicyLike =
    requires(std::byte* region, const BufferParity* parity, std::size_t n) {
      { P::kBlockCount } -> std::convertible_to<std::size_t>;
      { P::ReadOffset(parity, n) } -> std::convertible_to<std::size_t>;
      P::Replicate(region, n);
    };

// The half a group's write side addresses: always the one its read side
// isn't on.
inline std::size_t WriteHalfOffset(
    const BufferParity* parity, std::size_t block_bytes
) {
  return (parity->value ^ 1U) * block_bytes;
}

}  // namespace achilles::policy::buffering
