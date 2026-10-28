#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <numbers>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "algorithms/aba/aba_data.hpp"
#include "algorithms/aba/aba_step.hpp"
#include "algorithms/conventions.hpp"
#include "algorithms/sim_config.hpp"
#include "domain/archetype.hpp"
#include "engine/memory/sim_allocator.hpp"
#include "engine/topology/ordering_policy.hpp"
#include "interface/actuator.hpp"

// Covers interface::JointActuator: Attach's projection of a spatial axis
// onto a joint's generalized coordinates (effort * S^T e), the errors it
// raises, and Apply's accumulation into the write half of ABA's
// double-buffered kJointTorque. Runs against a real ABA-only SimAllocator
// built from hand-made archetypes -- only joint_subspace matters to an
// actuator, and nothing here is stepped, so every other field is left
// zero. Torque is read back the way
// ABA reads it: through the View, after a SwapTorqueStep. End-to-end behavior
// through Simulation (and a real ABA step) is in interface_simulation.cpp.

using achilles::algorithms::ScalarOperationT;
using achilles::algorithms::SimConfig;
using achilles::algorithms::aba::ABAAlgorithm;
using achilles::algorithms::aba::ABAField;
using achilles::algorithms::aba::SwapTorqueStep;
using achilles::domain::Archetype;
using achilles::domain::ArchetypeField;
using achilles::domain::ArchetypeJointHandle;
using achilles::domain::ArchetypeTreeStructure;
using achilles::engine::memory::SimAllocator;
using achilles::engine::topology::TopologicalOrdering;
using achilles::interface::ActuatorAxis;
using achilles::interface::ActuatorKind;
using achilles::interface::ActuatorLike;
using achilles::interface::AngularActuator;
using achilles::interface::LinearActuator;

namespace {

using Sim = SimAllocator<ABAAlgorithm>;
using SpatialAxis = std::array<float, 6>;  // roll, pitch, yaw, x, y, z
using Torque = std::array<float, 6>;       // one per generalized coordinate

constexpr SpatialAxis kRoll = {1, 0, 0, 0, 0, 0};
constexpr SpatialAxis kPitch = {0, 1, 0, 0, 0, 0};
constexpr SpatialAxis kYaw = {0, 0, 1, 0, 0, 0};
constexpr SpatialAxis kSlideX = {0, 0, 0, 1, 0, 0};

// joint_subspace with column k = columns[k], flattened row-major the way
// Matrix6x6Assembler reads it (entry (i, k) at i * 6 + k).
std::vector<double> Subspace(const std::vector<SpatialAxis>& columns) {
  std::vector<double> flat(36, 0.0);
  for (std::size_t k = 0; k < columns.size(); ++k) {
    for (std::size_t i = 0; i < 6; ++i) {
      flat[(i * 6) + k] = columns[k][i];
    }
  }
  return flat;
}

// One single-joint root archetype whose joint moves along `columns`.
Archetype MakeJoint(const std::vector<SpatialAxis>& columns) {
  std::vector<ArchetypeField> fields;
  fields.push_back(ArchetypeField{"joint_subspace", 36, Subspace(columns)});
  return {
      "body",
      {ArchetypeTreeStructure::kNoParent},
      {ArchetypeJointHandle{0, 0}},
      true,
      std::move(fields)
  };
}

Sim MakeSim(Archetype archetype) {
  std::array<Archetype, 1> archetypes = {std::move(archetype)};
  return Sim(archetypes);
}

// The one joint MakeJoint builds: global instance 0, local joint 0.
constexpr ArchetypeJointHandle kJoint{0, 0};

template <ActuatorKind Kind>
achilles::interface::JointActuator<Kind> Attach(
    const Sim& sim,
    const ActuatorAxis& axis,
    ArchetypeJointHandle joint = kJoint
) {
  return achilles::interface::JointActuator<Kind>::Attach(
      sim.SimContext(), joint, axis
  );
}

// kJointTorque at `joint`'s row, as ABA's passes would read it right now.
Torque ReadTorque(const Sim& sim, ArchetypeJointHandle joint = kJoint) {
  std::size_t row = sim.LayoutFor<TopologicalOrdering>().ToSorted(joint);
  auto force = sim.ViewFor<ABAAlgorithm>()
                   .Load<ABAField::kJointTorque, ScalarOperationT>(row);
  auto [t0, t1, t2, t3, t4, t5] = std::get<0>(force.ToTuple()).ToTuple();
  return {t0, t1, t2, t3, t4, t5};
}

// Starts the next tick the way every integrator does (SwapTorqueStep).
void SwapTorque(const Sim& sim) {
  SwapTorqueStep::Step(sim.SimContext(), SimConfig{}, 0.0F);
}

// What the next tick reads: swap, then read.
Torque NextTickTorque(const Sim& sim) {
  SwapTorque(sim);
  return ReadTorque(sim);
}

void ExpectTorqueEq(const Torque& actual, const Torque& expected) {
  for (std::size_t k = 0; k < actual.size(); ++k) {
    EXPECT_FLOAT_EQ(actual[k], expected[k]) << "coordinate " << k;
  }
}

// A user's actuator model: derives from the base handle, built from it.
class GearedMotor : public AngularActuator {
 public:
  GearedMotor(AngularActuator base, float ratio)
      : AngularActuator(std::move(base)), ratio_(ratio) {}
  void Command(float motor_torque) const { Apply(ratio_ * motor_torque); }

 private:
  float ratio_;
};

// Has a kKind, but isn't an actuator.
struct LooksLikeAnActuator {
  static constexpr ActuatorKind kKind = ActuatorKind::kAngular;
};

static_assert(ActuatorLike<AngularActuator>);
static_assert(ActuatorLike<LinearActuator>);
static_assert(ActuatorLike<GearedMotor>);
static_assert(GearedMotor::kKind == ActuatorKind::kAngular);
static_assert(!ActuatorLike<LooksLikeAnActuator>);
static_assert(!ActuatorLike<int>);

}  // namespace

// The basic contract: a revolute joint's actuator along its own axis puts
// the effort on that joint's coordinate -- and only for the next tick, not
// the one whose torque is already being read.
TEST(JointActuator, EffortLandsOnTheJointsCoordinateNextTick) {
  Sim sim = MakeSim(MakeJoint({kYaw}));
  AngularActuator actuator = Attach<ActuatorKind::kAngular>(sim, {0, 0, 1});
  actuator.Apply(3.0F);
  ExpectTorqueEq(ReadTorque(sim), {});
  ExpectTorqueEq(NextTickTorque(sim), {3.0F, 0, 0, 0, 0, 0});
}

// Effort is per unit axis: a longer axis doesn't scale it.
TEST(JointActuator, NormalizesTheAxis) {
  Sim sim = MakeSim(MakeJoint({kYaw}));
  Attach<ActuatorKind::kAngular>(sim, {0, 0, 5}).Apply(3.0F);
  ExpectTorqueEq(NextTickTorque(sim), {3.0F, 0, 0, 0, 0, 0});
}

// A joint axis that isn't a frame axis: an actuator along that same axis
// drives it fully, and one along a frame axis drives it by the projection.
TEST(JointActuator, ProjectsOntoATiltedJointAxis) {
  Sim sim = MakeSim(MakeJoint({{0, 0.6F, 0.8F, 0, 0, 0}}));
  Attach<ActuatorKind::kAngular>(sim, {0, 3, 4}).Apply(1.0F);
  Attach<ActuatorKind::kAngular>(sim, {0, 0, 1}).Apply(10.0F);
  ExpectTorqueEq(NextTickTorque(sim), {1.0F + 8.0F, 0, 0, 0, 0, 0});
}

// An axis only partly along the joint's freedom is allowed; only that
// component does anything, as with any force on a constrained body.
TEST(JointActuator, AppliesOnlyTheComponentAlongTheFreedom) {
  Sim sim = MakeSim(MakeJoint({kYaw}));
  Attach<ActuatorKind::kAngular>(sim, {1, 0, 1}).Apply(2.0F);
  ExpectTorqueEq(
      NextTickTorque(sim), {std::numbers::sqrt2_v<float>, 0, 0, 0, 0, 0}
  );
}

// On a multi-DOF joint one actuator couples into every coordinate its axis
// has a component along (here, a ball joint driven about a diagonal).
TEST(JointActuator, SplitsAcrossEveryCoordinateTheAxisCouplesTo) {
  Sim sim = MakeSim(MakeJoint({kRoll, kPitch, kYaw}));
  Attach<ActuatorKind::kAngular>(sim, {1, 1, 0}).Apply(2.0F);
  const float share = std::numbers::sqrt2_v<float>;
  ExpectTorqueEq(NextTickTorque(sim), {share, share, 0, 0, 0, 0});
}

// The linear kind reads the linear half of the subspace (rows x, y, z) --
// a force along a prismatic joint's slide.
TEST(JointActuator, LinearActuatorDrivesAPrismaticJoint) {
  Sim sim = MakeSim(MakeJoint({kSlideX}));
  Attach<ActuatorKind::kLinear>(sim, {1, 0, 0}).Apply(4.0F);
  ExpectTorqueEq(NextTickTorque(sim), {4.0F, 0, 0, 0, 0, 0});
}

// The two kinds read different halves of S, so each is rejected on a joint
// that only has freedom in the other half.
TEST(JointActuator, ThrowsWhenTheKindDoesNotMatchTheJoint) {
  Sim revolute = MakeSim(MakeJoint({kYaw}));
  EXPECT_THROW(
      Attach<ActuatorKind::kLinear>(revolute, {0, 0, 1}), std::invalid_argument
  );
  Sim prismatic = MakeSim(MakeJoint({kSlideX}));
  EXPECT_THROW(
      Attach<ActuatorKind::kAngular>(prismatic, {1, 0, 0}),
      std::invalid_argument
  );
}

// An axis the joint can't move along at all would be an actuator that
// silently does nothing.
TEST(JointActuator, ThrowsOnAnAxisOrthogonalToEveryFreedom) {
  Sim sim = MakeSim(MakeJoint({kYaw}));
  EXPECT_THROW(
      Attach<ActuatorKind::kAngular>(sim, {1, 0, 0}), std::invalid_argument
  );
}

// A zero axis has no direction to normalize to.
TEST(JointActuator, ThrowsOnAZeroAxis) {
  Sim sim = MakeSim(MakeJoint({kYaw}));
  EXPECT_THROW(
      Attach<ActuatorKind::kAngular>(sim, {0, 0, 0}), std::invalid_argument
  );
}

// Applies within one tick accumulate -- from one actuator, from a copy of
// it (handles share their target), and from a second actuator on the same
// joint.
TEST(JointActuator, EffortsAccumulateWithinATick) {
  Sim sim = MakeSim(MakeJoint({kYaw}));
  AngularActuator first = Attach<ActuatorKind::kAngular>(sim, {0, 0, 1});
  // The copy is what's under test: copied handles share their target.
  // NOLINTNEXTLINE(performance-unnecessary-copy-initialization)
  AngularActuator copy = first;
  AngularActuator second = Attach<ActuatorKind::kAngular>(sim, {0, 0, 1});
  first.Apply(1.0F);
  first.Apply(2.0F);
  copy.Apply(4.0F);
  second.Apply(8.0F);
  ExpectTorqueEq(NextTickTorque(sim), {15.0F, 0, 0, 0, 0, 0});
}

// A user's model derived from the base handle drives it through Apply.
TEST(JointActuator, DerivedModelAppliesThroughTheBase) {
  Sim sim = MakeSim(MakeJoint({kYaw}));
  GearedMotor motor(Attach<ActuatorKind::kAngular>(sim, {0, 0, 1}), 10.0F);
  motor.Command(0.25F);
  ExpectTorqueEq(NextTickTorque(sim), {2.5F, 0, 0, 0, 0, 0});
}

// Attach resolves the handle to the row ABA's TopologicalOrdering placed
// that joint at -- with several instances, only the targeted one moves.
TEST(JointActuator, WritesOnlyTheTargetedInstancesRow) {
  // Field values are per instance (the loader broadcasts them per copy),
  // so the child's subspace is given once per copy.
  std::vector<double> child_subspace;
  for (int copy = 0; copy < 3; ++copy) {
    std::vector<double> one = Subspace({kYaw});
    child_subspace.insert(child_subspace.end(), one.begin(), one.end());
  }
  std::vector<ArchetypeField> fields;
  fields.push_back(
      ArchetypeField{"joint_subspace", 36, std::move(child_subspace)}
  );
  std::array<Archetype, 2> archetypes = {
      MakeJoint({kYaw}),
      Archetype(
          "child",
          {ArchetypeTreeStructure::kNoParent},
          std::vector<ArchetypeJointHandle>(3, ArchetypeJointHandle{0, 0}),
          false,
          std::move(fields)
      )
  };
  Sim sim(archetypes);
  // Global instances: 0 is the root, 1..3 the child's copies.
  constexpr ArchetypeJointHandle kTarget{2, 0};
  Attach<ActuatorKind::kAngular>(sim, {0, 0, 1}, kTarget).Apply(1.0F);
  SwapTorque(sim);
  for (std::size_t instance = 0; instance < 4; ++instance) {
    float expected = instance == kTarget.instance_index ? 1.0F : 0.0F;
    EXPECT_FLOAT_EQ(ReadTorque(sim, {instance, 0})[0], expected)
        << "instance " << instance;
  }
}
