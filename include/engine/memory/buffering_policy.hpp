#pragma once

#include <bit>
#include <cassert>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "util/tmp.hpp"

namespace achilles::engine::memory {

// The buffering seam: how many copies of a field's block exist, and which
// one a View addresses. A sibling of the Layout/Ordering/Indexing policies
// (see binding.hpp) -- a field opts in with
// `using Buffering = DoubleBuffered<Group, Side>;` in its own Traits<F>,
// and nothing else about it changes: every Op still reads/writes it through
// its View like any other field.
//
// A double-buffered field exists so one side can write a value while the
// other reads the previous one -- an actuator (interface/actuator.hpp)
// writing next tick's torque while ABA reads this tick's. A swap happens
// in the owning Algorithm's Step: it flips the group's parity
// (pass::SimContext::FlipParity), and its passes decide what the new write
// half starts from (ABA's zeroes it; an overwrite-everything writer needs
// nothing).

// Which half of a group's buffer its read side addresses -- only ever 0 or
// 1. Carved once per group in the Arena by SimAllocator (see
// pass::BufferParitySlot), so every View and every BufferedScalar handle
// pointing at it sees the same flip, no matter how many copies of a View or
// SimContext are alive.
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
};

// Which half of a group's buffer a field sees (see DoubleBuffered).
enum class BufferSide : std::uint8_t { kWrite, kRead };

// The half a group's write side addresses: always the one its read side
// isn't on.
inline std::size_t WriteHalfOffset(
    const BufferParity* parity, std::size_t block_bytes
) {
  return (parity->value ^ 1U) * block_bytes;
}

// Two halves, swapped by flipping Group's own parity. Both sides name the
// same SharedAs slot, so they share one block; Side only picks which half a
// View addresses: the reader sees half `parity`, the writer the other one.
// Nothing here resets anything on a swap -- the write half keeps whatever
// it last held until an Op writes it.
template <typename Group, BufferSide Side>
struct DoubleBuffered {
  using BufferGroup = Group;
  static constexpr std::size_t kBlockCount = 2;
  // Both sides are one storage, for SharedAs's shape check.
  using Storage = DoubleBuffered<Group, BufferSide::kRead>;

  static std::size_t ReadOffset(
      const BufferParity* parity, std::size_t block_bytes
  ) {
    assert(
        parity != nullptr &&
        "DoubleBuffered: a View over a double-buffered field needs its "
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

// A buffering policy that belongs to a group (DoubleBuffered), and so reads
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
      const BufferParity* parity
  )
      : first_element_(first_element),
        block_bytes_(block_bytes),
        parity_(parity) {}

  // Accumulates into the next swap's value.
  void Add(ScalarT value) const { *Address() += value; }

 private:
  ScalarT* Address() const {
    return std::bit_cast<ScalarT*>(
        first_element_ + WriteHalfOffset(parity_, block_bytes_)
    );
  }

  std::byte* first_element_;
  std::size_t block_bytes_;
  const BufferParity* parity_;
};

}  // namespace achilles::engine::memory
