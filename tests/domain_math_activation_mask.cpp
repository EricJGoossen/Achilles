#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <xsimd/xsimd.hpp>

#include "domain/math/activation_mask.hpp"
#include "util/simd_ops.hpp"

using achilles::domain::math::ActivationMask;

namespace {

using Mask4 = ActivationMask<std::int32_t, 4>;

}  // namespace

TEST(ActivationMaskConstruction, DefaultIsAllFalse) {
  Mask4 m;
  EXPECT_TRUE(m.AllFalse());
}

TEST(ActivationMaskConstruction, FromRawStorageInt) {
  // Bits 0 and 2 set.
  Mask4 m(0b0101);
  EXPECT_TRUE(m[0]);
  EXPECT_FALSE(m[1]);
  EXPECT_TRUE(m[2]);
  EXPECT_FALSE(m[3]);
}

TEST(ActivationMaskConstruction, FromBoolArray) {
  Mask4 m(std::array<bool, 4>{true, false, true, false});
  EXPECT_TRUE(m[0]);
  EXPECT_FALSE(m[1]);
  EXPECT_TRUE(m[2]);
  EXPECT_FALSE(m[3]);
}

// The variadic bool-pack constructor is a separate overload from the
// std::array one -- both must produce the same bit pattern for the same
// values.
TEST(ActivationMaskConstruction, FromVariadicBoolsMatchesArrayForm) {
  Mask4 from_pack(true, false, true, false);
  Mask4 from_array(std::array<bool, 4>{true, false, true, false});
  EXPECT_TRUE(from_pack == from_array);
}

TEST(ActivationMaskStaticConstructors, ZeroAndOnes) {
  EXPECT_TRUE(Mask4::Zero().AllFalse());
  EXPECT_TRUE(Mask4::Ones().AllTrue());
}

TEST(ActivationMaskStaticConstructors, SetZeroAndSetOnesInPlace) {
  Mask4 m(true, true, true, true);
  m.SetZero();
  EXPECT_TRUE(m.AllFalse());

  m.SetOnes();
  EXPECT_TRUE(m.AllTrue());
}

// Access: operator[], AsStorage, Size, IsBatched.
TEST(ActivationMaskAccess, IndexingAndAsStorage) {
  Mask4 m(0b1010);
  EXPECT_FALSE(m[0]);
  EXPECT_TRUE(m[1]);
  EXPECT_FALSE(m[2]);
  EXPECT_TRUE(m[3]);
  EXPECT_EQ(m.AsStorage(), 0b1010);
}

TEST(ActivationMaskAccess, ToTupleWrapsStorage) {
  Mask4 m(0b0110);
  EXPECT_EQ(std::get<0>(m.ToTuple()), 0b0110);
}

TEST(ActivationMaskAccess, SizeIsTemplateN) { EXPECT_EQ(Mask4::Size(), 4U); }

TEST(ActivationMaskAccess, IsBatchedFalseForScalarStorage) {
  Mask4 m;
  EXPECT_FALSE(m.IsBatched());
}

TEST(ActivationMaskComparison, EqualityAndInequality) {
  Mask4 a(0b0101);
  Mask4 b(0b0101);
  Mask4 c(0b0111);

  EXPECT_TRUE(a == b);
  EXPECT_FALSE(a != b);
  EXPECT_TRUE(a != c);
  EXPECT_FALSE(a == c);
}

TEST(ActivationMaskComparison, AllTrueAndAllFalse) {
  EXPECT_TRUE(Mask4::Ones().AllTrue());
  EXPECT_FALSE(Mask4::Ones().AllFalse());
  EXPECT_TRUE(Mask4::Zero().AllFalse());
  EXPECT_FALSE(Mask4::Zero().AllTrue());

  Mask4 partial(0b0001);
  EXPECT_FALSE(partial.AllTrue());
  EXPECT_FALSE(partial.AllFalse());
}

TEST(ActivationMaskBitwise, And) {
  Mask4 a(0b1100);
  Mask4 b(0b1010);
  EXPECT_TRUE((a & b) == Mask4(0b1000));

  Mask4 in_place = a;
  in_place &= b;
  EXPECT_TRUE(in_place == Mask4(0b1000));
}

TEST(ActivationMaskBitwise, Or) {
  Mask4 a(0b1100);
  Mask4 b(0b1010);
  EXPECT_TRUE((a | b) == Mask4(0b1110));

  Mask4 in_place = a;
  in_place |= b;
  EXPECT_TRUE(in_place == Mask4(0b1110));
}

TEST(ActivationMaskBitwise, Xor) {
  Mask4 a(0b1100);
  Mask4 b(0b1010);
  EXPECT_TRUE((a ^ b) == Mask4(0b0110));

  Mask4 in_place = a;
  in_place ^= b;
  EXPECT_TRUE(in_place == Mask4(0b0110));
}

// operator~ and NegateInPlace both complement every bit -- including bits
// past N, which is why this compares through operator[] (masked to N bits
// by construction) rather than AsStorage().
TEST(ActivationMaskBitwise, NegationComplementsEachTrackedBit) {
  Mask4 m(0b1010);
  Mask4 negated = ~m;
  for (std::size_t i = 0; i < 4; ++i) {
    EXPECT_EQ(negated[i], !m[i]);
  }

  Mask4 in_place = m;
  in_place.NegateInPlace();
  EXPECT_TRUE(in_place == negated);
}

// Batched smoke test: StorageT itself can be an xsimd::batch<int32_t>,
// which is what IsBatched() distinguishes -- confirms ActivationMask is
// generic over StorageT, not hardcoded to a plain int.
TEST(ActivationMaskBatched, IsBatchedTrueForBatchStorage) {
  using BatchMask4 = ActivationMask<xsimd::batch<std::int32_t>, 4>;
  BatchMask4 m;
  EXPECT_TRUE(m.IsBatched());
  // AllTrue()/operator[] return a batch_bool for batched storage (see
  // BatchMask in activation_mask.hpp), which gtest can't convert to bool
  // directly -- achilles::util::AllTrue collapses it the same way
  // production code does.
  EXPECT_TRUE(achilles::util::AllTrue(BatchMask4::Ones().AllTrue()));
}
