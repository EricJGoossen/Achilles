#include "util/buffer.hpp"

#include <bit>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <new>

namespace achilles::util {

std::size_t Buffer::CheckedAlignment(std::size_t alignment) {
  assert(
      (alignment & (alignment - 1)) == 0 &&
      "Buffer alignment must be a power of two."
  );
  return alignment;
}

Buffer::Buffer(std::size_t bytes, std::size_t alignment)
    : data_(static_cast<std::byte*>(
          ::operator new(bytes, std::align_val_t{CheckedAlignment(alignment)})
      )),
      bytes_(bytes),
      alignment_(alignment) {}

Buffer::Buffer(Buffer&& other) noexcept
    : data_(other.data_), bytes_(other.bytes_), alignment_(other.alignment_) {
  other.data_ = nullptr;
  other.bytes_ = 0;
  other.alignment_ = 0;
}

Buffer& Buffer::operator=(Buffer&& other) noexcept {
  if (this != &other) {
    Release();
    data_ = other.data_;
    bytes_ = other.bytes_;
    alignment_ = other.alignment_;
    other.data_ = nullptr;
    other.bytes_ = 0;
    other.alignment_ = 0;
  }
  return *this;
}

Buffer::~Buffer() { Release(); }

void Buffer::Release() {
  if (data_ != nullptr) {
    ::operator delete(data_, std::align_val_t{alignment_});
    data_ = nullptr;
  }
}

bool Buffer::IsAlignedTo(std::size_t alignment) const {
  return (std::bit_cast<std::uintptr_t>(data_) & (alignment - 1)) == 0;
}

}  // namespace achilles::util
