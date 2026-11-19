// Two halves, swapped by flipping their group's parity.

#pragma once

#include <bit>
#include <cassert>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "policy/buffering/base_buffering.hpp"
#include "util/tmp.hpp"

namespace achilles::policy::buffering {

// Two halves, swapped by flipping Group's own parity. Both sides name the
// same SharedAs slot, so they share one block; Side only picks which half a
// View addresses: the reader sees half `parity`, the writer the other one.
// Nothing here resets anything on a swap -- the write half keeps whatever
// it last held until an Op writes it.
template <typename Group, BufferSide Side>
struct DoubleBuffering {
  using BufferGroup = Group;
  static constexpr std::size_t kBlockCount = 2;
  // Both sides are one storage, for SharedAs's shape check.
  using Storage = DoubleBuffering<Group, BufferSide::kRead>;

  static std::size_t ReadOffset(
      const BufferParity* parity, std::size_t block_bytes
  ) {
    assert(
        parity != nullptr &&
        "DoubleBuffering: a View over a double-buffered field needs its "
        "group's parity."
    );
    return Side == BufferSide::kRead ? parity->value * block_bytes
                                     : WriteHalfOffset(parity, block_bytes);
  }

  // Copies the populated first half into the second -- run once, after
  // SimAllocator seeds and populates the first.
  static void Replicate(std::byte* region, std::size_t block_bytes) {
    std::memcpy(region + block_bytes, region, block_bytes);
  }
};

}  // namespace achilles::policy::buffering
