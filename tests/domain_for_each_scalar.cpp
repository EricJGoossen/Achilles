#include <gtest/gtest.h>

#include <vector>

#include "domain/for_each_scalar.hpp"
#include "domain/math/quaternion.hpp"
#include "domain/math/vector3.hpp"
#include "domain/spatial/transform.hpp"

using achilles::domain::ForEachScalar;
using achilles::domain::math::Quaternion;
using achilles::domain::math::Vector3;
using achilles::domain::spatial::Transform;

namespace {

template <typename ValueT>
std::vector<float> Collect(const ValueT& value) {
  std::vector<float> out;
  ForEachScalar(value, [&](float scalar) { out.push_back(scalar); });
  return out;
}

}  // namespace

TEST(ForEachScalar, AScalarIsItself) {
  EXPECT_EQ(Collect(2.5F), (std::vector<float>{2.5F}));
}

TEST(ForEachScalar, VisitsAFlatTypeInToTupleOrder) {
  EXPECT_EQ(
      Collect(Vector3<float>(1.0F, 2.0F, 3.0F)),
      (std::vector<float>{1.0F, 2.0F, 3.0F})
  );
}

// Nested types recurse in ToTuple order: a Transform is its translation,
// then its rotation -- the same order its Assembler stores the leaves in.
TEST(ForEachScalar, RecursesIntoNestedTypes) {
  Transform<float> transform(
      Vector3<float>(1.0F, 2.0F, 3.0F),
      Quaternion<float>(4.0F, 5.0F, 6.0F, 7.0F)
  );
  EXPECT_EQ(
      Collect(transform),
      (std::vector<float>{1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F, 7.0F})
  );
}
