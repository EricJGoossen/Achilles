#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "algorithms/aba/aba_data.hpp"
#include "algorithms/aba/aba_step.hpp"
#include "algorithms/attachment/attachment_step.hpp"
#include "algorithms/conventions.hpp"
#include "algorithms/registry.hpp"
#include "algorithms/sim_config.hpp"
#include "domain/archetype/archetype.hpp"
#include "domain/archetype/archetype_set.hpp"
#include "domain/joint_data.hpp"
#include "domain/math/vector3.hpp"
#include "engine/memory/sim_allocator.hpp"
#include "engine/topology/orderings.hpp"
#include "interface/attachment_handle.hpp"
#include "interface/attachments.hpp"

// Covers interface::ActuatorHandle: a truth actuator's effort projected onto a
// joint's generalized coordinates (effort * S^T e), the errors Configure raises
// for its axis, and Apply's accumulation. Runs against the hosted Algorithms
// built from hand-made archetypes -- only joint_subspace matters to an
// actuator, and no physics is stepped, so every other field is left zero.
// Torque is read back the way ABA reads it: through the View, after DriveStep
// and SwapTorqueStep. End-to-end behavior through Simulation (and a real ABA
// step) is in interface_simulation.cpp.

using achilles::algorithms::RegisteredAlgorithms;
using achilles::algorithms::ScalarOperationT;
using achilles::algorithms::SimConfig;
using achilles::algorithms::aba::ABAAlgorithm;
using achilles::algorithms::aba::ABAField;
using achilles::algorithms::aba::SwapTorqueStep;
using achilles::algorithms::attachment::DriveStep;
using achilles::domain::JointAttachment;
using achilles::domain::JointField;
using achilles::domain::JointRef;
using achilles::domain::archetype::Archetype;
using achilles::domain::archetype::ArchetypeSet;
using achilles::domain::math::Vector3;
using achilles::engine::memory::SimAllocatorForT;
using achilles::engine::topology::TopologicalOrdering;
using achilles::interface::AngularActuator;
using achilles::interface::ConfigureAttachments;
using achilles::interface::LinearActuator;

namespace {

using Allocator = SimAllocatorForT<RegisteredAlgorithms>;
using SimPtr = std::unique_ptr<Allocator>;
using SpatialAxis = std::array<float, 6>;  // roll, pitch, yaw, x, y, z
using Torque = std::array<float, 6>;       // one per generalized coordinate

constexpr SpatialAxis kRoll = {1, 0, 0, 0, 0, 0};
constexpr SpatialAxis kPitch = {0, 1, 0, 0, 0, 0};
constexpr SpatialAxis kYaw = {0, 0, 1, 0, 0, 0};
constexpr SpatialAxis kSlideX = {0, 0, 0, 1, 0, 0};

// One actuator: the kind's list, and the axis its file would name.
struct Mount {
  std::string kind;
  Vector3<float> axis;
};

Mount Angular(const Vector3<float>& axis) { return {"angular", axis}; }
Mount Linear(const Vector3<float>& axis) { return {"linear", axis}; }

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

std::vector<JointAttachment> Attachments(const std::vector<Mount>& mounts) {
  std::vector<JointAttachment> attachments;
  for (std::size_t i = 0; i < mounts.size(); ++i) {
    const Vector3<float>& axis = mounts[i].axis;
    attachments.push_back(JointAttachment{
        mounts[i].kind,
        0,
        "actuator_" + std::to_string(i),
        {{"axis", {axis.X(), axis.Y(), axis.Z()}}},
        "actuator"
    });
  }
  return attachments;
}

// One single-joint root archetype whose joint moves along `columns`, with
// `mounts` on it.
Archetype MakeJoint(
    const std::vector<SpatialAxis>& columns,
    const std::vector<Mount>& mounts = {}
) {
  std::vector<JointField> fields;
  fields.push_back(JointField{"joint_subspace", 36, Subspace(columns)});
  return {
      "body",
      {achilles::domain::kNoParent},
      {JointRef{0, 0}},
      true,
      std::move(fields),
      {},
      Attachments(mounts)
  };
}

// The sim of `archetypes`, its attachments configured the way Init does.
template <std::size_t N>
SimPtr MakeSim(std::array<Archetype, N> archetypes) {
  const ArchetypeSet set(archetypes);
  auto sim = std::make_unique<Allocator>(set);
  ConfigureAttachments(sim->SimContext(), set, 0);
  return sim;
}

SimPtr MakeSim(Archetype archetype) {
  return MakeSim(std::array<Archetype, 1>{std::move(archetype)});
}

// Entry `entry` of the kind's list.
AngularActuator Torqued(const SimPtr& sim, std::size_t entry = 0) {
  return AngularActuator::Find(sim->SimContext(), entry);
}
LinearActuator Forced(const SimPtr& sim, std::size_t entry = 0) {
  return LinearActuator::Find(sim->SimContext(), entry);
}

// kJointTorque at `joint`'s row, as ABA's passes would read it right now.
Torque ReadTorque(const SimPtr& sim, JointRef joint = {0, 0}) {
  std::size_t row = sim->LayoutFor<TopologicalOrdering>().ToSorted(
      joint.instance_index, joint.joint_index
  );
  auto force = sim->ViewFor<ABAAlgorithm>()
                   .Load<ABAField::kJointTorque, ScalarOperationT>(row);
  auto [t0, t1, t2, t3, t4, t5] = std::get<0>(force.ToTuple()).ToTuple();
  return {t0, t1, t2, t3, t4, t5};
}

// Starts the next tick the way every integrator does: the actuators run, then
// SwapTorqueStep hands ABA their torque.
void StartTick(const SimPtr& sim) {
  DriveStep::Step(sim->SimContext(), SimConfig{}, 0.0F);
  SwapTorqueStep::Step(sim->SimContext(), SimConfig{}, 0.0F);
}

// What the next tick reads.
Torque NextTickTorque(const SimPtr& sim) {
  StartTick(sim);
  return ReadTorque(sim);
}

void ExpectTorqueEq(const Torque& actual, const Torque& expected) {
  for (std::size_t k = 0; k < actual.size(); ++k) {
    EXPECT_FLOAT_EQ(actual[k], expected[k]) << "coordinate " << k;
  }
}

}  // namespace

// Apply writes nothing the current tick reads; the next tick gets it.
TEST(ActuatorHandle, EffortLandsOnTheJointsCoordinateNextTick) {
  SimPtr sim = MakeSim(MakeJoint({kYaw}, {Angular({0, 0, 1})}));
  Torqued(sim).Apply(3.0F);
  ExpectTorqueEq(ReadTorque(sim), {});
  ExpectTorqueEq(NextTickTorque(sim), {3.0F, 0, 0, 0, 0, 0});
}

// Effort is per unit axis: a longer axis doesn't scale it.
TEST(ActuatorHandle, NormalizesTheAxis) {
  SimPtr sim = MakeSim(MakeJoint({kYaw}, {Angular({0, 0, 5})}));
  Torqued(sim).Apply(3.0F);
  ExpectTorqueEq(NextTickTorque(sim), {3.0F, 0, 0, 0, 0, 0});
}

// A joint axis that isn't a frame axis: an actuator along that same axis
// drives it fully, and one along a frame axis drives it by the projection.
TEST(ActuatorHandle, ProjectsOntoATiltedJointAxis) {
  SimPtr sim = MakeSim(MakeJoint(
      {{0, 0.6F, 0.8F, 0, 0, 0}}, {Angular({0, 3, 4}), Angular({0, 0, 1})}
  ));
  Torqued(sim, 0).Apply(1.0F);
  Torqued(sim, 1).Apply(10.0F);
  ExpectTorqueEq(NextTickTorque(sim), {1.0F + 8.0F, 0, 0, 0, 0, 0});
}

// An axis only partly along the joint's freedom is allowed; only that
// component does anything, as with any force on a constrained body.
TEST(ActuatorHandle, AppliesOnlyTheComponentAlongTheFreedom) {
  SimPtr sim = MakeSim(MakeJoint({kYaw}, {Angular({1, 0, 1})}));
  Torqued(sim).Apply(2.0F);
  ExpectTorqueEq(
      NextTickTorque(sim), {std::numbers::sqrt2_v<float>, 0, 0, 0, 0, 0}
  );
}

// On a multi-DOF joint one actuator couples into every coordinate its axis
// has a component along (here, a ball joint driven about a diagonal).
TEST(ActuatorHandle, SplitsAcrossEveryCoordinateTheAxisCouplesTo) {
  SimPtr sim = MakeSim(MakeJoint({kRoll, kPitch, kYaw}, {Angular({1, 1, 0})}));
  Torqued(sim).Apply(2.0F);
  const float share = std::numbers::sqrt2_v<float>;
  ExpectTorqueEq(NextTickTorque(sim), {share, share, 0, 0, 0, 0});
}

// The linear kind reads the linear half of the subspace (rows x, y, z) --
// a force along a prismatic joint's slide.
TEST(ActuatorHandle, LinearActuatorDrivesAPrismaticJoint) {
  SimPtr sim = MakeSim(MakeJoint({kSlideX}, {Linear({1, 0, 0})}));
  Forced(sim).Apply(4.0F);
  ExpectTorqueEq(NextTickTorque(sim), {4.0F, 0, 0, 0, 0, 0});
}

// The two kinds read different halves of S, so each is rejected on a joint
// that only has freedom in the other half.
TEST(ActuatorHandle, ThrowsWhenTheKindDoesNotMatchTheJoint) {
  EXPECT_THROW(
      MakeSim(MakeJoint({kYaw}, {Linear({0, 0, 1})})), std::invalid_argument
  );
  EXPECT_THROW(
      MakeSim(MakeJoint({kSlideX}, {Angular({1, 0, 0})})), std::invalid_argument
  );
}

// An axis the joint can't move along at all would be an actuator that
// silently does nothing.
TEST(ActuatorHandle, ThrowsOnAnAxisOrthogonalToEveryFreedom) {
  EXPECT_THROW(
      MakeSim(MakeJoint({kYaw}, {Angular({1, 0, 0})})), std::invalid_argument
  );
}

// A zero axis has no direction to normalize to.
TEST(ActuatorHandle, ThrowsOnAZeroAxis) {
  EXPECT_THROW(
      MakeSim(MakeJoint({kYaw}, {Angular({0, 0, 0})})), std::invalid_argument
  );
}

// What's thrown for any bad attachment names it, whatever the kind rejected.
TEST(ActuatorHandle, ErrorsNameTheActuator) {
  const auto message = [](Archetype archetype) {
    try {
      (void)MakeSim(std::move(archetype));
    } catch (const std::invalid_argument& e) {
      return std::string(e.what());
    }
    return std::string();
  };
  EXPECT_NE(
      message(MakeJoint({kYaw}, {Angular({0, 0, 0})})).find("'actuator_0'"),
      std::string::npos
  );

  Archetype misspelled = MakeJoint({kYaw}, {Angular({0, 0, 1})});
  misspelled.attachments[0].config.emplace_back(
      "nope", std::vector<double>{1.0}
  );
  EXPECT_NE(
      message(std::move(misspelled)).find("actuator 'actuator_0': unknown"),
      std::string::npos
  );

  Archetype bad_motor = MakeJoint({kYaw}, {Mount{"dc_motor", {0, 0, 1}}});
  EXPECT_NE(
      message(std::move(bad_motor)).find("actuator 'actuator_0': DcMotor"),
      std::string::npos
  );
}

// A truth actuator is exact, so it takes no error settings.
TEST(ActuatorHandle, ATruthActuatorRejectsErrorSettings) {
  Archetype archetype = MakeJoint({kYaw}, {Angular({0, 0, 1})});
  archetype.attachments[0].config.emplace_back(
      "bias", std::vector<double>{1.0}
  );
  EXPECT_THROW(MakeSim(std::move(archetype)), std::invalid_argument);
}

// A truth effort lasts its tick: the next one starts from nothing.
TEST(ActuatorHandle, AnEffortIsUsedUpByItsTick) {
  SimPtr sim = MakeSim(MakeJoint({kYaw}, {Angular({0, 0, 1})}));
  Torqued(sim).Apply(3.0F);
  ExpectTorqueEq(NextTickTorque(sim), {3.0F, 0, 0, 0, 0, 0});
  ExpectTorqueEq(NextTickTorque(sim), {});
}

// Applies within one tick accumulate -- from one actuator, from a copy of
// it (handles share their target), and from a second actuator on the same
// joint.
TEST(ActuatorHandle, EffortsAccumulateWithinATick) {
  SimPtr sim =
      MakeSim(MakeJoint({kYaw}, {Angular({0, 0, 1}), Angular({0, 0, 1})}));
  AngularActuator first = Torqued(sim, 0);
  // The copy is what's under test: copied handles share their target.
  // NOLINTNEXTLINE(performance-unnecessary-copy-initialization)
  AngularActuator copy = first;
  AngularActuator second = Torqued(sim, 1);
  first.Apply(1.0F);
  first.Apply(2.0F);
  copy.Apply(4.0F);
  second.Apply(8.0F);
  ExpectTorqueEq(NextTickTorque(sim), {15.0F, 0, 0, 0, 0, 0});
}

// A handle resolves to the row its instance's joint is on -- with several
// instances, only the targeted one moves.
TEST(ActuatorHandle, WritesOnlyTheTargetedInstancesRow) {
  // Field values are per instance (the loader broadcasts them per copy),
  // so the child's subspace is given once per copy.
  std::vector<double> child_subspace;
  for (int copy = 0; copy < 3; ++copy) {
    std::vector<double> one = Subspace({kYaw});
    child_subspace.insert(child_subspace.end(), one.begin(), one.end());
  }
  std::vector<JointField> fields;
  fields.push_back(JointField{"joint_subspace", 36, std::move(child_subspace)});
  SimPtr sim = MakeSim(std::array<Archetype, 2>{
      MakeJoint({kYaw}),
      Archetype(
          "child",
          {achilles::domain::kNoParent},
          std::vector<JointRef>(3, JointRef{0, 0}),
          false,
          std::move(fields),
          {},
          Attachments({Angular({0, 0, 1})})
      )
  });
  // Global instances: 0 is the root, 1..3 the child's copies, and entry i of
  // the kind's list is on copy i.
  constexpr std::size_t kTarget = 2;
  Torqued(sim, kTarget - 1).Apply(1.0F);
  StartTick(sim);
  for (std::size_t instance = 0; instance < 4; ++instance) {
    float expected = instance == kTarget ? 1.0F : 0.0F;
    EXPECT_FLOAT_EQ(ReadTorque(sim, {instance, 0})[0], expected)
        << "instance " << instance;
  }
}
