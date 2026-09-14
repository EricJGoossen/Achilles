#include <array>

#include <gtest/gtest.h>

#include "domain/math/vector3.hpp"
#include "engine/op_contract.hpp"
#include "engine/op_invoker.hpp"
#include "support/planar_view_fixture.hpp"
#include "support/toy_field.hpp"

using achilles::domain::math::Vector3;
using achilles::domain::math::Vector3Assembler;
using achilles::engine::ArgData;
using achilles::engine::OpArgsMatchView;
using achilles::engine::OpInvoker;
using achilles::engine::OpLike;
using achilles::engine::SingleOpInvoker;
using achilles::test_support::PlanarViewFixture;
using achilles::test_support::ToyField;
using achilles::test_support::ToyView;

namespace {

using Fixture = PlanarViewFixture<
    ToyField, Vector3Assembler<float>, Vector3Assembler<float>>;

// Reads kPosition at the target index and kVelocity at the parent index
// (use_target mixed true/false, per ArgData), writes the elementwise sum
// to kVelocity at the target index -- exercises the (target_index,
// parent_index) split OpInvoker::operator() implements.
struct CombineOp {
  using FieldEnum = ToyField;

  static constexpr std::array<ArgData<FieldEnum>, 2> kInputs = {
      ArgData<FieldEnum>{FieldEnum::kPosition, true},
      ArgData<FieldEnum>{FieldEnum::kVelocity, false},
  };
  static constexpr std::array<ArgData<FieldEnum>, 1> kOutputs = {
      ArgData<FieldEnum>{FieldEnum::kVelocity, true},
  };

  void operator()(
      const Vector3<float>& target_position,
      const Vector3<float>& parent_velocity,
      Vector3<float>* velocity_out
  ) const {
    *velocity_out = target_position + parent_velocity;
  }
};
static_assert(OpLike<CombineOp>);
static_assert(OpArgsMatchView<CombineOp, ToyView>);

// Empty kInputs, one output -- the shape RunSeed/SingleOpInvoker exist
// for (see algorithm_step.hpp: "a SeedBaseOp carrying real gravity/base-
// velocity state").
struct SeedPositionOp {
  using FieldEnum = ToyField;

  static constexpr std::array<ArgData<FieldEnum>, 0> kInputs = {};
  static constexpr std::array<ArgData<FieldEnum>, 1> kOutputs = {
      ArgData<FieldEnum>{FieldEnum::kPosition, true},
  };

  Vector3<float> value = Vector3<float>(9.0F, 9.0F, 9.0F);

  void operator()(Vector3<float>* position_out) const { *position_out = value; }
};
static_assert(OpLike<SeedPositionOp>);
static_assert(OpArgsMatchView<SeedPositionOp, ToyView>);

}  // namespace

TEST(OpInvoker, ReadsTargetAndParentWritesTarget) {
  Fixture fixture(4);
  ToyView view = fixture.MakeView();

  view.Store<ToyField::kPosition, float>(2, Vector3<float>(1.0F, 2.0F, 3.0F));
  view.Store<ToyField::kVelocity, float>(0, Vector3<float>(10.0F, 20.0F, 30.0F));

  CombineOp op;
  OpInvoker<CombineOp, ToyView> invoker(view, op);
  invoker(/*target_index=*/2, /*parent_index=*/0);

  EXPECT_TRUE(
      (view.Load<ToyField::kVelocity, float>(2).IsApprox(
          Vector3<float>(11.0F, 22.0F, 33.0F)
      ))
  );
  // Untouched: kVelocity at the parent index itself must be unchanged.
  EXPECT_TRUE(
      (view.Load<ToyField::kVelocity, float>(0).IsApprox(
          Vector3<float>(10.0F, 20.0F, 30.0F)
      ))
  );
}

TEST(OpInvoker, TargetAndParentSameIndexBothReadsHitTheSameRow) {
  Fixture fixture(4);
  ToyView view = fixture.MakeView();

  view.Store<ToyField::kPosition, float>(1, Vector3<float>(1.0F, 0.0F, 0.0F));
  view.Store<ToyField::kVelocity, float>(1, Vector3<float>(0.0F, 1.0F, 0.0F));

  CombineOp op;
  OpInvoker<CombineOp, ToyView> invoker(view, op);
  invoker(1, 1);

  EXPECT_TRUE(
      (view.Load<ToyField::kVelocity, float>(1).IsApprox(
          Vector3<float>(1.0F, 1.0F, 0.0F)
      ))
  );
}

TEST(SingleOpInvokerTest, WritesOnlyTheGivenTargetIndex) {
  Fixture fixture(4);
  ToyView view = fixture.MakeView();

  SeedPositionOp op;
  SingleOpInvoker<SeedPositionOp, ToyView> invoker(view, op);
  invoker(2);

  EXPECT_TRUE(
      (view.Load<ToyField::kPosition, float>(2).IsApprox(
          Vector3<float>(9.0F, 9.0F, 9.0F)
      ))
  );
  EXPECT_TRUE((view.Load<ToyField::kPosition, float>(0).IsZero()));
  EXPECT_TRUE((view.Load<ToyField::kPosition, float>(1).IsZero()));
  EXPECT_TRUE((view.Load<ToyField::kPosition, float>(3).IsZero()));
}
