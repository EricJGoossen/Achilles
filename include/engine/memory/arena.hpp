#pragma once

#include <cstddef>

#include "util/buffer.hpp"

namespace achilles::engine::memory {

// The sim's whole fixed backing store: one over-aligned Buffer, bump-
// allocated, no per-block free and no growth. Every field block for every
// hosted algorithm is carved from here at SimAllocator construction and
// lives until this Arena dies. "Resize the sim" means destroying the owning
// SimAllocator (which destroys this) and building a new one with a new
// instance count -- there is deliberately no in-place resize.
class Arena {
 public:
  // base_alignment must be a power of two, >= the max Alignment<A>() over
  // every field that will ever be carved from this Arena (SimAllocator's
  // measure pass computes it before constructing this).
  Arena(std::size_t total_bytes, std::size_t base_alignment);

  // Access
  std::size_t Used() const { return cursor_; }
  std::size_t Capacity() const { return buffer_.Bytes(); }
  std::size_t BaseAlignment() const { return buffer_.Alignment(); }
  bool IsValid() const { return buffer_.IsValid(); }

  // Rounds the cursor up to `alignment` (asserting alignment <= this
  // Arena's own base alignment), then bumps by `bytes`. Asserts on
  // overflow: the budget is computed exactly by SimAllocator's measure
  // pass, so overflow here is a bug in that pass, not an expected runtime
  // condition to be handled gracefully.
  std::byte* Allocate(std::size_t bytes, std::size_t alignment);

  // Zero-fills a just-carved block. Called for every freshly carved block so
  // an unwritten row reads deterministic zero rather than stale memory --
  // matching what ViewFixture already guarantees today via memset.
  // NOTE: zero is not a universally safe value for every field (e.g. a zero
  // Quaternion is degenerate) -- see AssemblerLike's SeedableLike
  // requirement and SimAllocator's padding-seed pass.
  void ZeroFill(std::byte* block, std::size_t bytes);

 private:
  util::Buffer buffer_;
  std::size_t cursor_ = 0;
};

}  // namespace achilles::engine::memory
