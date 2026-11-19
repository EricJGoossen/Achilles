// One block, read in place: every field's default.

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

// One block; reads always at offset 0. The default for every field.
struct SingleBuffering {
  static constexpr std::size_t kBlockCount = 1;

  static constexpr std::size_t ReadOffset(
      const BufferParity* /*parity*/, std::size_t /*block_bytes*/
  ) {
    return 0;
  }

  static void Replicate(std::byte* /*region*/, std::size_t /*block_bytes*/) {}
};

}  // namespace achilles::policy::buffering
