#include <gtest/gtest.h>

#include <array>
#include <bit>
#include <cstddef>

#include "domain/math/vector3.hpp"
#include "engine/deferred.hpp"

using achilles::domain::math::Vector3;
using achilles::domain::math::Vector3Assembler;
using achilles::engine::Deferred;
using achilles::engine::kIsDeferred;

static_assert(kIsDeferred<Deferred<Vector3<float>>>);
static_assert(kIsDeferred<const Deferred<Vector3<float>>&>);
static_assert(!kIsDeferred<Vector3<float>>);

// Built from a plain value -- how a test calls an Op that takes a
// Deferred input directly -- Load() returns that value.
TEST(Deferred, FromValueLoadsThatValue) {
  Vector3<float> v(1.0F, 2.0F, 3.0F);
  Deferred<Vector3<float>> deferred = v;
  EXPECT_TRUE(deferred.Load() == v);
}

// Built over storage, Load() is the field's own Assembler::Read of it,
// done when Load() is called rather than when the handle was made.
TEST(Deferred, FromStorageReadsAtLoadTime) {
  using A = Vector3Assembler<float>;
  std::array<float, 3> storage{1.0F, 2.0F, 3.0F};
  Deferred<Vector3<float>>
      deferred(std::bit_cast<const std::byte*>(storage.data()), sizeof(float), &A::Read<float>);
  storage[1] = 5.0F;
  EXPECT_TRUE(deferred.Load() == Vector3<float>(1.0F, 5.0F, 3.0F));
}
