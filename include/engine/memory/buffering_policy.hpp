#pragma once

#include <bit>
#include <cassert>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace achilles::engine::memory {

// The buffering seam: how many copies of a field's block exist, and which
// one a View reads. A sibling of the Layout/Ordering/Indexing policies (see
// binding.hpp) -- a field opts in with `using Buffering = DoubleBuffered;` in
// its own Traits<F>, and nothing else about it changes: every Op still
// reads/writes it through its View like any other field.
//
// A double-buffered field exists so something outside the sim (an
// actuator, see interface/actuator.hpp) can write the field's *next* value
// while the sim reads, and freely scribbles on, its *current* one. The two
// swap once per tick (pass::SimContext::SwapBuffers).

// Which half of every double-buffered field the sim currently reads -- one
// value for the whole sim, carved once in the Arena by SimAllocator, so every
// View and every BufferedScalar handle pointing at it sees the same flip, no
// matter how many copies of a View or SimContext are alive. Only ever 0 or 1.
struct BufferParity {
  std::uint32_t value = 0;
};

// One block; reads always at offset 0. The default for every field.
struct SingleBuffered {
  static constexpr std::size_t kBlockCount = 1;

  static constexpr std::size_t ReadOffset(
      const BufferParity* /*parity*/, std::size_t /*block_bytes*/
  ) {
    return 0;
  }

  static void Replicate(std::byte* /*region*/, std::size_t /*block_bytes*/) {}

  static void ResetWrite(
      std::byte* /*region*/,
      const BufferParity* /*parity*/,
      std::size_t /*block_bytes*/
  ) {}
};

// Three blocks, laid out [rest | half 0 | half 1]:
//   - rest   -- the field's populated initial contents (archetype data plus
//               padding seeds). Never read or written by a pass; it is only
//               what a write half is reset to.
//   - half p -- what the sim reads/writes this tick (p = parity).
//   - the other half -- what outside writers accumulate into for the next
//               tick.
// SwapBuffers flips the parity, then resets the new write half (the one the
// sim just finished with, scratch and all) back to rest. Resetting to the
// populated contents rather than to zero keeps a field's archetype value as a
// constant baseline that writers add on top of, and keeps padding rows
// seeded with whatever PaddingSeed() the field's own type needs.
struct DoubleBuffered {
  static constexpr std::size_t kBlockCount = 3;

  static std::size_t ReadOffset(
      const BufferParity* parity, std::size_t block_bytes
  ) {
    assert(
        parity != nullptr &&
        "DoubleBuffered: a View over a double-buffered field needs the "
        "sim's BufferParity."
    );
    return (1 + parity->value) * block_bytes;
  }

  static std::size_t WriteOffset(
      const BufferParity* parity, std::size_t block_bytes
  ) {
    return (2 - parity->value) * block_bytes;
  }

  // Copies the populated rest block into both halves -- run once, after
  // SimAllocator seeds and populates the rest block.
  static void Replicate(std::byte* region, std::size_t block_bytes) {
    std::memcpy(region + block_bytes, region, block_bytes);
    std::memcpy(region + (2 * block_bytes), region, block_bytes);
  }

  static void ResetWrite(
      std::byte* region, const BufferParity* parity, std::size_t block_bytes
  ) {
    std::memcpy(region + WriteOffset(parity, block_bytes), region, block_bytes);
  }
};

template <typename P>
concept BufferingPolicyLike =
    requires(std::byte* region, const BufferParity* parity, std::size_t n) {
      { P::kBlockCount } -> std::convertible_to<std::size_t>;
      { P::ReadOffset(parity, n) } -> std::convertible_to<std::size_t>;
      P::Replicate(region, n);
      P::ResetWrite(region, parity, n);
    };

// A handle to one scalar of a double-buffered field's *write* half: always
// the half the sim is not reading, whichever that is this tick. Holds only
// pointers into the Arena (the scalar's address within the rest block, and
// the sim's BufferParity), so it is cheap to copy and stays valid exactly as
// long as the Arena that owns that memory.
template <typename ScalarT>
class BufferedScalar {
 public:
  BufferedScalar(
      std::byte* rest_element,
      std::size_t block_bytes,
      const BufferParity* parity
  )
      : rest_element_(rest_element),
        block_bytes_(block_bytes),
        parity_(parity) {}

  // Accumulates into the next tick's value.
  void Add(ScalarT value) const { *Address() += value; }

 private:
  ScalarT* Address() const {
    return std::bit_cast<ScalarT*>(
        rest_element_ + DoubleBuffered::WriteOffset(parity_, block_bytes_)
    );
  }

  std::byte* rest_element_;
  std::size_t block_bytes_;
  const BufferParity* parity_;
};

}  // namespace achilles::engine::memory
