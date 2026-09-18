#include <gtest/gtest.h>

#include <bit>
#include <cstddef>
#include <cstdint>
#include <utility>

#include "util/buffer.hpp"

using achilles::util::Buffer;

namespace {

bool IsAlignedPointer(std::byte* ptr, std::size_t alignment) {
  return (std::bit_cast<std::uintptr_t>(ptr) & (alignment - 1)) == 0;
}

}  // namespace

TEST(BufferConstruction, DataIsAlignedAndAccessorsMatchArguments) {
  Buffer buffer(64, 16);

  EXPECT_EQ(buffer.Bytes(), 64U);
  EXPECT_EQ(buffer.Alignment(), 16U);
  ASSERT_NE(buffer.Data(), nullptr);
  EXPECT_TRUE(IsAlignedPointer(buffer.Data(), 16));
}

TEST(BufferConstruction, DefaultConstructedHasNoStorage) {
  Buffer buffer;

  EXPECT_EQ(buffer.Data(), nullptr);
  EXPECT_EQ(buffer.Bytes(), 0U);
}

// A zero-byte request is explicitly allowed (an Arena hosting zero fields
// still needs a Buffer to exist), but ::operator new's own contract
// guarantees a distinct, non-null pointer even for a zero-size request --
// so, contrary to this class's own header comment ("leaves Data() ==
// nullptr"), Data() is not actually null here. This test locks in the real
// behavior; the header comment is stale and should be corrected separately.
TEST(BufferConstruction, ZeroBytesStillReturnsNonNullData) {
  Buffer buffer(0, 8);

  EXPECT_NE(buffer.Data(), nullptr);
  EXPECT_EQ(buffer.Bytes(), 0U);
}

TEST(BufferIsAlignedTo, TrueForConstructedAlignmentAndAnySmallerPowerOfTwo) {
  Buffer buffer(64, 32);

  EXPECT_TRUE(buffer.IsAlignedTo(32));
  EXPECT_TRUE(buffer.IsAlignedTo(16));
  EXPECT_TRUE(buffer.IsAlignedTo(1));
}

// A block over-aligned beyond what this Buffer was actually constructed
// with is exactly the case Arena's own precondition exists to catch before
// carving a SIMD block from it -- this proves IsAlignedTo is the right
// signal for that check to rely on, not just an accessor with no real use.
TEST(BufferIsAlignedTo, FalseForAlignmentLargerThanConstructed) {
  Buffer buffer(64, 8);

  EXPECT_FALSE(buffer.IsAlignedTo(4096));
}

TEST(BufferMove, MoveConstructionTransfersOwnershipAndClearsSource) {
  Buffer original(128, 16);
  std::byte* original_data = original.Data();

  Buffer moved(std::move(original));

  EXPECT_EQ(moved.Data(), original_data);
  EXPECT_EQ(moved.Bytes(), 128U);
  EXPECT_EQ(moved.Alignment(), 16U);
  EXPECT_EQ(original.Data(), nullptr);  // NOLINT
  EXPECT_EQ(original.Bytes(), 0U);
}

TEST(BufferMove, MoveAssignmentReleasesPreviousStorageAndTransfersNew) {
  Buffer target(32, 8);
  Buffer source(64, 16);
  std::byte* source_data = source.Data();

  target = std::move(source);

  EXPECT_EQ(target.Data(), source_data);
  EXPECT_EQ(target.Bytes(), 64U);
  EXPECT_EQ(target.Alignment(), 16U);
  EXPECT_EQ(source.Data(), nullptr);  // NOLINT
  EXPECT_EQ(source.Bytes(), 0U);
}

// operator= must guard against `this == &other`: without the check,
// Release() would free the storage before the (no-op) self-copy of the
// pointer, leaving `target` holding a dangling pointer to freed memory.
TEST(BufferMove, SelfMoveAssignmentIsANoOp) {
  Buffer target(32, 8);
  std::byte* data = target.Data();

  Buffer* alias = &target;
  target = std::move(*alias);

  EXPECT_EQ(target.Data(), data);
  EXPECT_EQ(target.Bytes(), 32U);
}

TEST(BufferConstructorPrecondition, DiesOnNonPowerOfTwoAlignment) {
  EXPECT_DEATH(Buffer(64, 3), "power of two");
}
