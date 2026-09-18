#pragma once

#include <cstddef>

namespace achilles::util {

// Aligned RAII owner, promoted out of the hand-rolled versions in
// engine_assembler.cpp / view_fixture.hpp. Alignment is a constructor
// parameter, never a default: every SIMD block Arena carves needs
// alignof(xsimd::batch<T>), which exceeds what plain new/delete guarantee --
// hence ::operator new(bytes, align_val_t) rather than unique_ptr<std::byte[]>,
// whose default deleter can't carry the alignment through to delete.
class Buffer {
 public:
  Buffer() = default;

  // alignment must be a power of two. bytes == 0 is allowed -- operator
  // new(0, align_val_t) still returns a distinct, non-null pointer per the
  // standard, so Data() is non-null even then; it just owns zero bytes,
  // matching an Arena that hosts zero fields.
  Buffer(std::size_t bytes, std::size_t alignment);

  Buffer(const Buffer&) = delete;
  Buffer& operator=(const Buffer&) = delete;

  Buffer(Buffer&& other) noexcept;
  Buffer& operator=(Buffer&& other) noexcept;

  ~Buffer();

  std::byte* Data() const { return data_; }
  std::size_t Bytes() const { return bytes_; }
  std::size_t Alignment() const { return alignment_; }
  bool IsValid() const { return data_ != nullptr; }

  // True iff Data() itself satisfies `alignment` -- asserted by Arena when
  // carving blocks, so a misaligned SIMD load fails at construction with a
  // named cause rather than as a segfault inside an xsimd load intrinsic.
  bool IsAlignedTo(std::size_t alignment) const;

 private:
  void Release();

  // Asserts alignment is a power of two and returns it unchanged --
  // called from within the constructor's data_ initializer, before
  // operator new ever runs with a bad alignment. Doing this validation in
  // the constructor *body* instead would run too late: the member
  // initializer list (including the operator new call below) has already
  // executed by the time a constructor body starts, and a non-power-of-two
  // align_val_t is undefined behavior for operator new to receive at all
  // -- in practice, a thrown std::bad_alloc that preempts the assert.
  static std::size_t CheckedAlignment(std::size_t alignment);

  std::byte* data_ = nullptr;
  std::size_t bytes_ = 0;
  std::size_t alignment_ = 0;
};

}  // namespace achilles::util
