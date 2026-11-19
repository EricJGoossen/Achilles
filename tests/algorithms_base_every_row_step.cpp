#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <type_traits>

#include "algorithms/base/every_row_step.hpp"
#include "domain/math/vector3.hpp"
#include "engine/algorithm.hpp"
#include "engine/allocation/sim_allocator.hpp"
#include "engine/op_invoker/algorithm_step.hpp"
#include "engine/spec/arg_data.hpp"
#include "engine/spec/op_contract.hpp"
#include "engine/spec/step_contract.hpp"
#include "policy/traversal/base_traversal.hpp"
#include "policy/traversal/global_traversal.hpp"
#include "policy/traversal/pair_traversal.hpp"
#include "policy/traversal/row_traversal.hpp"
#include "support/toy_field.hpp"

// Covers achilles::algorithms::base::EveryRowStep -- the optional base for a
// Step that is one engine::op_invoker::Step over every row of its Algorithm's
// View. Its promises: it names the Algorithm's FieldEnum (so the result is
// StepLike), it hands MakeOps the caller's config and dt unchanged, it runs the
// returned Ops over every row (size = view.Size()), and it runs multiple Passes
// in the order MakeOps lists them. Built against ToyAlgorithm with scalar Ops
// and a stride-1 LinearTraversal, so nothing here depends on a real algorithm's
// math or lane width.

using achilles::algorithms::base::EveryRowStep;
using achilles::domain::math::Vector3;
using achilles::engine::allocation::SimAllocator;
using achilles::engine::op_invoker::Ops;
using achilles::engine::op_invoker::OpWalk;
using achilles::engine::spec::ArgData;
using achilles::engine::spec::ArgIndex;
using achilles::engine::spec::OpArgsMatchView;
using achilles::engine::spec::OpLike;
using achilles::engine::spec::StepLike;
using achilles::policy::traversal::Direction;
using achilles::policy::traversal::LinearTraversal;
using achilles::test_support::MakeToySim;
using achilles::test_support::ToyAlgorithm;
using achilles::test_support::ToyField;
using achilles::test_support::ToyView;

namespace {

using Linear = LinearTraversal<Direction::kForward, 1>;

struct ToyConfig {
  float value;
};

// kVelocity = kPosition * scale.
struct ScaleOp {
  using FieldEnum = ToyField;

  explicit ScaleOp(float scale) : scale_(scale) {}

  static constexpr std::array<ArgData<FieldEnum>, 1> kInputs = {
      ArgData<FieldEnum>{FieldEnum::kPosition, ArgIndex::kTarget},
  };
  static constexpr std::array<ArgData<FieldEnum>, 1> kOutputs = {
      ArgData<FieldEnum>{FieldEnum::kVelocity, ArgIndex::kTarget},
  };

  void operator()(const Vector3<float>& position, Vector3<float>* velocity_out)
      const {
    *velocity_out = position * scale_;
  }

 private:
  float scale_;
};
static_assert(OpLike<ScaleOp>);
static_assert(OpArgsMatchView<ScaleOp, ToyView>);

// kPosition = kVelocity -- reads what ScaleOp wrote, so running it after
// ScaleOp vs. before it gives observably different results.
struct CopyVelocityToPositionOp {
  using FieldEnum = ToyField;

  static constexpr std::array<ArgData<FieldEnum>, 1> kInputs = {
      ArgData<FieldEnum>{FieldEnum::kVelocity, ArgIndex::kTarget},
  };
  static constexpr std::array<ArgData<FieldEnum>, 1> kOutputs = {
      ArgData<FieldEnum>{FieldEnum::kPosition, ArgIndex::kTarget},
  };

  void operator()(const Vector3<float>& velocity, Vector3<float>* position_out)
      const {
    *position_out = velocity;
  }
};
static_assert(OpLike<CopyVelocityToPositionOp>);
static_assert(OpArgsMatchView<CopyVelocityToPositionOp, ToyView>);

// The one-Pass shape: scale by config.value * dt, so both arguments are
// observable in the result.
struct ScaleStep : EveryRowStep<ScaleStep, ToyAlgorithm> {
  static auto MakeOps(const ToyConfig& config, float dt) {
    return Ops<OpWalk<ScaleOp, Linear>>{ScaleOp(config.value * dt)};
  }
};
static_assert(StepLike<ScaleStep>);
static_assert(std::is_same_v<ScaleStep::FieldEnum, ToyField>);

// The multi-Pass shape: ScaleOp, then CopyVelocityToPositionOp.
struct ScaleThenCopyStep : EveryRowStep<ScaleThenCopyStep, ToyAlgorithm> {
  static auto MakeOps(const ToyConfig& config, float dt) {
    return Ops<
        OpWalk<ScaleOp, Linear>,
        OpWalk<CopyVelocityToPositionOp, Linear>>{
        ScaleOp(config.value * dt), CopyVelocityToPositionOp{}
    };
  }
};
static_assert(StepLike<ScaleThenCopyStep>);

constexpr std::size_t kRows = 4;

void PopulatePositions(ToyView& view) {
  for (std::size_t row = 0; row < kRows; ++row) {
    view.Store<ToyField::kPosition, float>(
        row, Vector3<float>(static_cast<float>(row + 1), 0.0F, 0.0F)
    );
  }
}

}  // namespace

// Every row, not just the first -- and config/dt reach the Op exactly as
// the caller passed them (scale = 2 * 0.5 = 1 would hide a swapped or
// dropped argument, so the two are chosen to multiply to 1.5).
TEST(EveryRowStep, RunsMakeOpsOverEveryRowWithTheCallersConfigAndDt) {
  SimAllocator<ToyAlgorithm> sim = MakeToySim(kRows);
  ToyView view = sim.ViewFor<ToyAlgorithm>();
  PopulatePositions(view);

  ScaleStep::Step(sim.SimContext(), ToyConfig{3.0F}, 0.5F);

  for (std::size_t row = 0; row < kRows; ++row) {
    EXPECT_FLOAT_EQ(
        (view.Load<ToyField::kVelocity, float>(row)[0]),
        static_cast<float>(row + 1) * 1.5F
    ) << "row "
      << row;
  }
}

// Passes run in MakeOps' order: CopyVelocityToPositionOp must see the
// velocity ScaleOp just wrote. Run the other way round, position would be
// overwritten with the initial (zero) velocity first.
TEST(EveryRowStep, RunsMultiplePassesInTheOrderMakeOpsListsThem) {
  SimAllocator<ToyAlgorithm> sim = MakeToySim(kRows);
  ToyView view = sim.ViewFor<ToyAlgorithm>();
  PopulatePositions(view);

  ScaleThenCopyStep::Step(sim.SimContext(), ToyConfig{2.0F}, 1.0F);

  for (std::size_t row = 0; row < kRows; ++row) {
    EXPECT_FLOAT_EQ(
        (view.Load<ToyField::kPosition, float>(row)[0]),
        static_cast<float>(row + 1) * 2.0F
    ) << "row "
      << row;
  }
}
