#include <array>

#include <gtest/gtest.h>

#include "domain/math/vector3.hpp"
#include "engine/op_contract.hpp"
#include "support/toy_field.hpp"

using achilles::domain::math::Vector3;
using achilles::engine::ArgData;
using achilles::engine::OpArgsMatchView;
using achilles::engine::OpLike;
using achilles::test_support::ToyField;
using achilles::test_support::ToyView;

// OpLike/OpArgsMatchView are compile-time-only (concepts and consteval
// helpers, no runtime component of their own) -- see
// engine_field_contract.cpp for why these are TESTs with empty bodies
// rather than bare static_asserts.

namespace {

// Minimal type satisfying exactly what OpLike requires: kInputs/kOutputs
// arrays sized to match operator()'s declared arity, one array entry per
// parameter in declaration order. Also satisfies OpArgsMatchView against
// ToyView, since its parameter types are exactly what reading/writing
// those fields as their own ScalarType produces.
struct OpArchetype {
  using FieldEnum = ToyField;

  static constexpr std::array<ArgData<FieldEnum>, 1> kInputs = {
      ArgData<FieldEnum>{FieldEnum::kPosition, true},
  };
  static constexpr std::array<ArgData<FieldEnum>, 1> kOutputs = {
      ArgData<FieldEnum>{FieldEnum::kVelocity, true},
  };

  void operator()(const Vector3<float>& position, Vector3<float>* velocity_out)
      const {
    *velocity_out = position;
  }
};
static_assert(OpLike<OpArchetype>);
static_assert(OpArgsMatchView<OpArchetype, ToyView>);

// Arity mismatch: operator() takes 2 parameters, but kOutputs is declared
// empty -- kInputs.size() + kOutputs.size() (1) != the real arity (2).
// OpLike's own nested requirement (util::kArityOfV<...> ==
// kInputs.size() + kOutputs.size()) must catch this without needing
// OpArgsMatchView at all.
struct WrongArityOp {
  using FieldEnum = ToyField;

  static constexpr std::array<ArgData<FieldEnum>, 1> kInputs = {
      ArgData<FieldEnum>{FieldEnum::kPosition, true},
  };
  static constexpr std::array<ArgData<FieldEnum>, 0> kOutputs = {};

  void operator()(const Vector3<float>& position, Vector3<float>* velocity_out)
      const {
    *velocity_out = position;
  }
};
static_assert(!OpLike<WrongArityOp>);

// Right arity and right shape (OpLike holds), but the input parameter's
// declared type doesn't match what reading kPosition as ScalarType
// produces from ToyView -- here, a plain float instead of a
// Vector3<float>. This is exactly the "field/parameter silently paired
// wrong" case OpArgsMatchView exists to catch (see the comment on it in
// op_contract.hpp): OpLike alone has no opinion on *which* type a
// parameter should be, only that the arity lines up.
struct WrongParamTypeOp {
  using FieldEnum = ToyField;

  static constexpr std::array<ArgData<FieldEnum>, 1> kInputs = {
      ArgData<FieldEnum>{FieldEnum::kPosition, true},
  };
  static constexpr std::array<ArgData<FieldEnum>, 1> kOutputs = {
      ArgData<FieldEnum>{FieldEnum::kVelocity, true},
  };

  void operator()(const float& not_a_vector3, Vector3<float>* velocity_out) const {
    *velocity_out = Vector3<float>(not_a_vector3, not_a_vector3, not_a_vector3);
  }
};
static_assert(OpLike<WrongParamTypeOp>);
static_assert(!OpArgsMatchView<WrongParamTypeOp, ToyView>);

// Output parameter must be a pointer, not a reference or a value --
// caught the same way (OpLike is arity-only, OpArgsMatchView checks the
// actual shape).
struct OutputNotAPointerOp {
  using FieldEnum = ToyField;

  static constexpr std::array<ArgData<FieldEnum>, 1> kInputs = {
      ArgData<FieldEnum>{FieldEnum::kPosition, true},
  };
  static constexpr std::array<ArgData<FieldEnum>, 1> kOutputs = {
      ArgData<FieldEnum>{FieldEnum::kVelocity, true},
  };

  void operator()(const Vector3<float>& position, Vector3<float>& velocity_out)
      const {
    velocity_out = position;
  }
};
static_assert(OpLike<OutputNotAPointerOp>);
static_assert(!OpArgsMatchView<OutputNotAPointerOp, ToyView>);

}  // namespace

TEST(OpLikeConcept, ChecksArityIndependentOfParameterTypes) { SUCCEED(); }
TEST(OpArgsMatchViewConcept, CatchesFieldParameterTypeMismatches) { SUCCEED(); }
