// A handle to one scalar of a double-buffered field's write half.

#pragma once

#include <bit>
#include <cassert>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "policy/buffering/base_buffering.hpp"
#include "util/tmp.hpp"

namespace achilles::engine::memory {

// A handle to one scalar of a double-buffered field's *write* half: always
// the half the read side is not on, whichever that is this tick. Holds only
// pointers into the Arena (the scalar's address within the first half, and
// its group's BufferParity), so it is cheap to copy and stays valid exactly
// as long as the Arena that owns that memory.
template <typename ScalarT>
class BufferedScalar {
 public:
  BufferedScalar(
      std::byte* first_element,
      std::size_t block_bytes,
      const policy::buffering::BufferParity* parity
  )
      : first_element_(first_element),
        block_bytes_(block_bytes),
        parity_(parity) {}

  // Accumulates into the next swap's value.
  void Add(ScalarT value) const { *Address() += value; }

 private:
  ScalarT* Address() const {
    return std::bit_cast<ScalarT*>(
        first_element_ +
        policy::buffering::WriteHalfOffset(parity_, block_bytes_)
    );
  }

  std::byte* first_element_;
  std::size_t block_bytes_;
  const policy::buffering::BufferParity* parity_;
};

}  // namespace achilles::engine::memory
