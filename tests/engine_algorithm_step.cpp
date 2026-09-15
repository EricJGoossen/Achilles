#include <array>
#include <cstddef>
#include <vector>

#include <gtest/gtest.h>

#include "domain/math/vector3.hpp"
#include "domain/topology/topology_contract.hpp"
#include "engine/algorithm_step.hpp"
#include "engine/op_contract.hpp"
#include "engine/traversals.hpp"
#include "support/planar_view_fixture.hpp"
#include "support/toy_field.hpp"

using achilles::domain::math::Vector3;
using achilles::domain::math::Vector3Assembler;
using achilles::engine::ArgData;
using achilles::engine::ForwardLinearTraversal;
using achilles::engine::ForwardTreeTraversal;
using achilles::engine::OpHasInit;
using achilles::engine::Ops;
using achilles::engine::Pass;
using achilles::engine::PassLike;
using achilles::engine::RunPass;
using achilles::engine::Step;
using achilles::test_support::PlanarViewFixture;
using achilles::test_support::ToyField;
using achilles::test_support::ToyView;

namespace {

using Fixture = PlanarViewFixture<
    ToyField, Vector3Assembler<float>, Vector3Assembler<float>>;

struct TopologyArchetype {
  std::vector<size_t> parents;
  size_t Size() const { return parents.size(); }
  size_t operator[](size_t i) const { return parents[i]; }
};
static_assert(achilles::domain::topology::TopologyLike<TopologyArchetype>);

// velocity[target] = position[target] + velocity[parent]. No
// Initialize/kInitInputs/kInitOutputs -- deliberately, so this doubles as
// the "an Op without Init still works" case (see OpHasInit,
// engine/op_contract.hpp): RunPass must skip straight to Apply for it,
// never attempting to call something that doesn't exist.
struct PropagateOp {
  using FieldEnum = ToyField;
  static constexpr std::array<ArgData<FieldEnum>, 2> kInputs = {
      ArgData<FieldEnum>{FieldEnum::kPosition, true},
      ArgData<FieldEnum>{FieldEnum::kVelocity, false},
  };
  static constexpr std::array<ArgData<FieldEnum>, 1> kOutputs = {
      ArgData<FieldEnum>{FieldEnum::kVelocity, true},
  };
  void operator()(
      const Vector3<float>& position, const Vector3<float>& velocity_parent,
      Vector3<float>* velocity_out
  ) const {
    *velocity_out = position + velocity_parent;
  }
};
static_assert(!OpHasInit<PropagateOp>);

// Same shape as PropagateOp, plus an Initialize that seeds the base row's
// own kVelocity with caller-configured state -- the same pattern
// algorithms/aba/aba_ops.hpp's PropagateVelocityOp uses for real (there,
// seeding x_world_parent/v_parent for a root joint).
struct SeededPropagateOp {
  using FieldEnum = ToyField;

  explicit SeededPropagateOp(Vector3<float> seed_value) : seed_value(seed_value) {}

  static constexpr std::array<ArgData<FieldEnum>, 0> kInitInputs = {};
  static constexpr std::array<ArgData<FieldEnum>, 1> kInitOutputs = {
      ArgData<FieldEnum>{FieldEnum::kVelocity, true},
  };
  void Initialize(Vector3<float>* velocity_out) const { *velocity_out = seed_value; }

  static constexpr std::array<ArgData<FieldEnum>, 2> kInputs = {
      ArgData<FieldEnum>{FieldEnum::kPosition, true},
      ArgData<FieldEnum>{FieldEnum::kVelocity, false},
  };
  static constexpr std::array<ArgData<FieldEnum>, 1> kOutputs = {
      ArgData<FieldEnum>{FieldEnum::kVelocity, true},
  };
  void operator()(
      const Vector3<float>& position, const Vector3<float>& velocity_parent,
      Vector3<float>* velocity_out
  ) const {
    *velocity_out = position + velocity_parent;
  }

  Vector3<float> seed_value;
};
static_assert(OpHasInit<SeededPropagateOp>);

// velocity[target] = 2 * position[target] -- doesn't read any parent row,
// so it's agnostic to target == parent (what LinearTraversal now feeds
// it, see below).
struct DoubleOp {
  using FieldEnum = ToyField;
  static constexpr std::array<ArgData<FieldEnum>, 1> kInputs = {
      ArgData<FieldEnum>{FieldEnum::kPosition, true},
  };
  static constexpr std::array<ArgData<FieldEnum>, 1> kOutputs = {
      ArgData<FieldEnum>{FieldEnum::kVelocity, true},
  };
  void operator()(const Vector3<float>& position, Vector3<float>* velocity_out)
      const {
    *velocity_out = position * 2.0F;
  }
};

using PropagatePass = Pass<PropagateOp, ForwardTreeTraversal>;
using SeededPropagatePass = Pass<SeededPropagateOp, ForwardTreeTraversal>;
using DoublePass = Pass<DoubleOp, ForwardLinearTraversal>;

static_assert(PassLike<PropagatePass>);
static_assert(PassLike<SeededPropagatePass>);
static_assert(PassLike<DoublePass>);

// Something missing OpType/TraversalType entirely must fail PassLike --
// it's the shape check the whole rest of algorithm_step.hpp is built on.
struct NotAPass {};
static_assert(!PassLike<NotAPass>);

// Writes kVelocity at the parent index via `+=` -- the shape
// PropagateInertiaOp (algorithms/aba/aba_ops.hpp) uses for real to fold
// each child's contribution into a shared parent row.
struct AccumulateIntoParentOp {
  using FieldEnum = ToyField;
  static constexpr std::array<ArgData<FieldEnum>, 1> kInputs = {
      ArgData<FieldEnum>{FieldEnum::kPosition, true},
  };
  static constexpr std::array<ArgData<FieldEnum>, 1> kOutputs = {
      ArgData<FieldEnum>{FieldEnum::kVelocity, false},
  };
  void operator()(
      const Vector3<float>& contribution, Vector3<float>* velocity_parent_out
  ) const {
    *velocity_parent_out += contribution;
  }
};
using AccumulatePass = Pass<AccumulateIntoParentOp, ForwardTreeTraversal>;
static_assert(PassLike<AccumulatePass>);

}  // namespace

TEST(RunPassTest, PropagatesAlongATreeInForwardOrder) {
  // 4 rows: 0,1,2 are real joints, 3 is the reserved base row joint 0's
  // parent points at. Joint 1 and 2 both parent off joint 0. PropagateOp
  // has no Initialize, so the base row is seeded manually here rather
  // than through RunPass.
  Fixture fixture(4);
  ToyView view = fixture.MakeView();
  TopologyArchetype topology{{3, 0, 0}};

  view.Store<ToyField::kPosition, float>(0, Vector3<float>(1.0F, 0.0F, 0.0F));
  view.Store<ToyField::kPosition, float>(1, Vector3<float>(2.0F, 0.0F, 0.0F));
  view.Store<ToyField::kPosition, float>(2, Vector3<float>(3.0F, 0.0F, 0.0F));
  view.Store<ToyField::kVelocity, float>(3, Vector3<float>(5.0F, 0.0F, 0.0F));

  PropagateOp op;
  RunPass<PropagatePass>(view, op, topology);

  EXPECT_TRUE(
      (view.Load<ToyField::kVelocity, float>(0).IsApprox(Vector3<float>(6.0F, 0.0F, 0.0F)))
  );
  EXPECT_TRUE(
      (view.Load<ToyField::kVelocity, float>(1).IsApprox(Vector3<float>(8.0F, 0.0F, 0.0F)))
  );
  EXPECT_TRUE(
      (view.Load<ToyField::kVelocity, float>(2).IsApprox(Vector3<float>(9.0F, 0.0F, 0.0F)))
  );
}

// Regression test for the bug where OpInvoker handed a fresh, zeroed
// local to an accumulating op instead of the view's real current value:
// joints 1 and 2 both parent off joint 0 (same topology as the test
// above), and each contributes to joint 0's kVelocity via `+=`. Joint 0's
// own pre-existing value must survive, with both children's contributions
// summed on top of it -- not the last-processed child's contribution
// alone.
TEST(RunPassTest, AccumulatesContributionsFromMultipleChildrenIntoSharedParent) {
  Fixture fixture(4);
  ToyView view = fixture.MakeView();
  TopologyArchetype topology{{3, 0, 0}};

  view.Store<ToyField::kVelocity, float>(0, Vector3<float>(100.0F, 0.0F, 0.0F));
  view.Store<ToyField::kPosition, float>(1, Vector3<float>(1.0F, 0.0F, 0.0F));
  view.Store<ToyField::kPosition, float>(2, Vector3<float>(10.0F, 0.0F, 0.0F));

  AccumulateIntoParentOp op;
  RunPass<AccumulatePass>(view, op, topology);

  EXPECT_TRUE(
      (view.Load<ToyField::kVelocity, float>(0).IsApprox(
          Vector3<float>(111.0F, 0.0F, 0.0F)
      ))
  );
}

// RunPass itself calls Initialize at the base row before Apply -- no
// separate seed call needed -- for an Op that declares one.
TEST(RunPassTest, InitializeSeedsBaseRowBeforeApply) {
  Fixture fixture(4);
  ToyView view = fixture.MakeView();
  TopologyArchetype topology{{3, 0, 0}};

  view.Store<ToyField::kPosition, float>(0, Vector3<float>(1.0F, 0.0F, 0.0F));
  view.Store<ToyField::kPosition, float>(1, Vector3<float>(2.0F, 0.0F, 0.0F));
  view.Store<ToyField::kPosition, float>(2, Vector3<float>(3.0F, 0.0F, 0.0F));

  SeededPropagateOp op(Vector3<float>(9.0F, 9.0F, 9.0F));
  RunPass<SeededPropagatePass>(view, op, topology);

  EXPECT_TRUE(
      (view.Load<ToyField::kVelocity, float>(3).IsApprox(Vector3<float>(9.0F, 9.0F, 9.0F)))
  );
  EXPECT_TRUE(
      (view.Load<ToyField::kVelocity, float>(0).IsApprox(Vector3<float>(10.0F, 9.0F, 9.0F)))
  );
}

// TreeTraversal::InitOp seeds Initialize at topology[0] -- whatever joint
// 0 claims as its own parent -- on the assumption that joint 0 is always
// a root pointing at a genuinely separate, reserved row outside [0,
// Size()) (see aba_ops.hpp: "Wired into Step for the single-root case
// only"). RunPass runs InitOp and Apply back to back with nothing in
// between, and nothing anywhere checks that assumption -- if a topology
// ever gave joint 0 an *in-range* parent (a real joint, not a genuinely
// reserved row -- exactly what an unsupported multi-root forest would
// need), Initialize would seed that real joint's own row, and Apply
// would immediately overwrite it while computing that joint's ordinary
// output, silently discarding the seed with no error of any kind. This
// documents that failure mode concretely so it doesn't have to be
// rediscovered by hand later: joint 0's parent (1) is in range here,
// Initialize seeds row 1 with (9,9,9), and Apply's very next pass
// (processing joint 1, whose parent is joint 0) overwrites it with
// position[1] + velocity[0] before RunPass returns.
TEST(RunPassTest, InSequenceInitSeedIsSilentlyClobberedWhenBaseRowIsInRange) {
  Fixture fixture(2);
  ToyView view = fixture.MakeView();
  TopologyArchetype topology{{1, 0}};  // joint 0's parent is joint 1 (in range!)

  view.Store<ToyField::kPosition, float>(0, Vector3<float>(1.0F, 0.0F, 0.0F));
  view.Store<ToyField::kPosition, float>(1, Vector3<float>(2.0F, 0.0F, 0.0F));

  SeededPropagateOp op(Vector3<float>(9.0F, 9.0F, 9.0F));
  RunPass<SeededPropagatePass>(view, op, topology);

  // Row 1 held the (9,9,9) seed only until Apply reached joint 1's own
  // turn: position[1] (2,0,0) + velocity[0] (the seed plus joint 0's own
  // contribution, 10,9,9) = (12,9,9), not the seed.
  EXPECT_TRUE(
      (view.Load<ToyField::kVelocity, float>(1).IsApprox(
          Vector3<float>(12.0F, 9.0F, 9.0F)
      ))
  );
}

// Regression test: TreeTraversal::InitOp used to unconditionally index
// topology[0] before RunPass got to Apply, out of range for a topology
// with no real joints in it (TopologyLike doesn't require non-empty, and
// a real JointTopology accepts empty input). RunPass over an empty
// topology must now do nothing at all -- neither Initialize nor Apply
// touches any row -- rather than indexing into an empty topology.
TEST(RunPassTest, EmptyTopologySkipsInitializeAndApply) {
  Fixture fixture(4);
  ToyView view = fixture.MakeView();
  TopologyArchetype topology{{}};

  SeededPropagateOp op(Vector3<float>(9.0F, 9.0F, 9.0F));
  RunPass<SeededPropagatePass>(view, op, topology);

  EXPECT_TRUE((view.Load<ToyField::kVelocity, float>(0).IsZero()));
  EXPECT_TRUE((view.Load<ToyField::kVelocity, float>(3).IsZero()));
}

// Same regression, through LinearTraversal::InitOp instead: it used to
// ignore the traversal's size entirely and always call Initialize(0), so
// a zero-length linear pass wrote to index 0 even though Apply itself
// would run zero iterations.
TEST(RunPassTest, EmptyLinearPassSkipsInitializeAndApply) {
  Fixture fixture(4);
  ToyView view = fixture.MakeView();

  SeededPropagateOp op(Vector3<float>(9.0F, 9.0F, 9.0F));
  RunPass<Pass<SeededPropagateOp, ForwardLinearTraversal>>(
      view, op, std::size_t{0}
  );

  EXPECT_TRUE((view.Load<ToyField::kVelocity, float>(0).IsZero()));
}

// LinearTraversal paired with a real OpInvoker via RunPass -- until
// engine/traversals.hpp's LinearTraversal::Apply was fixed to call its
// callable with (j, j) instead of just (j), this failed to compile
// outright (OpInvoker::operator() requires exactly two arguments; see the
// fix and its accompanying note). DoubleOp only ever reads with
// use_target=true, so it never notices target==parent, but the pairing
// itself compiling and running at all is the thing being proven here.
TEST(RunPassTest, LinearTraversalPairsWithRealOpInvoker) {
  Fixture fixture(3);
  ToyView view = fixture.MakeView();

  view.Store<ToyField::kPosition, float>(0, Vector3<float>(1.0F, 0.0F, 0.0F));
  view.Store<ToyField::kPosition, float>(1, Vector3<float>(2.0F, 0.0F, 0.0F));
  view.Store<ToyField::kPosition, float>(2, Vector3<float>(3.0F, 0.0F, 0.0F));

  DoubleOp op;
  RunPass<DoublePass>(view, op, std::size_t{3});

  EXPECT_TRUE(
      (view.Load<ToyField::kVelocity, float>(0).IsApprox(Vector3<float>(2.0F, 0.0F, 0.0F)))
  );
  EXPECT_TRUE(
      (view.Load<ToyField::kVelocity, float>(1).IsApprox(Vector3<float>(4.0F, 0.0F, 0.0F)))
  );
  EXPECT_TRUE(
      (view.Load<ToyField::kVelocity, float>(2).IsApprox(Vector3<float>(6.0F, 0.0F, 0.0F)))
  );
}

// Full Step, one pass whose Op seeds its own base row via Initialize --
// the same shape aba::Step uses for real (see
// algorithms/aba/aba_step.hpp), just with this file's toy Op.
TEST(StepTest, RunsInitializeThenApply) {
  Fixture fixture(4);
  ToyView view = fixture.MakeView();
  TopologyArchetype topology{{3, 0, 0}};

  view.Store<ToyField::kPosition, float>(0, Vector3<float>(1.0F, 0.0F, 0.0F));
  view.Store<ToyField::kPosition, float>(1, Vector3<float>(2.0F, 0.0F, 0.0F));
  view.Store<ToyField::kPosition, float>(2, Vector3<float>(3.0F, 0.0F, 0.0F));

  Step(
      Ops<SeededPropagatePass>(SeededPropagateOp(Vector3<float>(5.0F, 0.0F, 0.0F))),
      view,
      topology
  );

  EXPECT_TRUE(
      (view.Load<ToyField::kVelocity, float>(3).IsApprox(Vector3<float>(5.0F, 0.0F, 0.0F)))
  );
  EXPECT_TRUE(
      (view.Load<ToyField::kVelocity, float>(0).IsApprox(Vector3<float>(6.0F, 0.0F, 0.0F)))
  );
  EXPECT_TRUE(
      (view.Load<ToyField::kVelocity, float>(1).IsApprox(Vector3<float>(8.0F, 0.0F, 0.0F)))
  );
  EXPECT_TRUE(
      (view.Load<ToyField::kVelocity, float>(2).IsApprox(Vector3<float>(9.0F, 0.0F, 0.0F)))
  );
}

// A Step whose only pass's Op has no Initialize at all must still run
// cleanly -- RunPass's `if constexpr (OpHasInit<...>)` skips straight to
// Apply, never trying to call something PropagateOp doesn't declare.
TEST(StepTest, OpWithoutInitStillWorks) {
  Fixture fixture(3);
  ToyView view = fixture.MakeView();
  TopologyArchetype topology{{2, 0}};

  view.Store<ToyField::kPosition, float>(0, Vector3<float>(1.0F, 0.0F, 0.0F));
  view.Store<ToyField::kPosition, float>(1, Vector3<float>(2.0F, 0.0F, 0.0F));

  Step(Ops<PropagatePass>(PropagateOp{}), view, topology);

  // Base row (index 2) was never seeded -- zero-initialized by the
  // fixture -- so joint 0's propagated velocity is just its own position.
  EXPECT_TRUE(
      (view.Load<ToyField::kVelocity, float>(0).IsApprox(Vector3<float>(1.0F, 0.0F, 0.0F)))
  );
}
