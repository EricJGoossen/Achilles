#include "engine/memory/arena.hpp"

#include <cassert>
#include <cstddef>

namespace achilles::engine::memory {

Arena::Arena(std::size_t total_bytes, std::size_t base_alignment)
    : buffer_(total_bytes, base_alignment) {}

std::byte* Arena::Allocate(std::size_t bytes, std::size_t alignment) {
  assert(
      alignment <= buffer_.Alignment() &&
      "Arena was not constructed with enough base alignment for this "
      "block -- SimAllocator's measure pass must size base_alignment to "
      "the max Alignment<A>() over every hosted field before this runs."
  );
  assert(
      (alignment & (alignment - 1)) == 0 &&
      "Allocate() alignment must be a power of two."
  );
  assert(bytes > 0 && "Allocate() bytes must be > 0.");

  std::size_t aligned_cursor =
      (cursor_ + alignment - 1) / alignment * alignment;
  assert(
      aligned_cursor + bytes <= buffer_.Bytes() &&
      "Arena overflow -- SimAllocator's measure pass undercounted "
      "RequiredBytes for the hosted algorithms."
  );

  std::byte* result = buffer_.Data() + aligned_cursor;
  cursor_ = aligned_cursor + bytes;
  return result;
}

void Arena::ZeroFill(std::byte* block, std::size_t bytes) {
  for (std::size_t i = 0; i < bytes; ++i) {
    block[i] = std::byte{0};
  }
}

}  // namespace achilles::engine::memory
