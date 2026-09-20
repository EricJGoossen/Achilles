#pragma once

#include "algorithms/aba/aba_data.hpp"
#include "domain/joint_topology.hpp"
#include "domain/math/vector3.hpp"

namespace achilles::algorithms::aba {

// Total mechanical energy summed over every joint `view` currently holds:
// kinetic (1/2 v^T I v per body, the spatial-algebra form -- see
// domain::spatial::Inertia::Apply) plus gravitational potential (-m g.r_com,
// standard for a uniform field). Generic across whatever tree ABA steps: it
// walks kWorldTransform/kRigidBodyInertia/kSpatialVelocity per row the same
// way render::SceneRenderer walks kWorldTransform per row, rather than a
// closed-form expression derived for one specific topology (contrast
// tests/examples_two_joint_arm.cpp's own hand-derived formula for its one
// two-link case).
//
// Reads kWorldTransform/kSpatialVelocity, not kJointPosition/kJointVelocity
// -- which matters, because those two pairs are *not* the same instant in
// time right after a Step() call. ABA's own forward-kinematics pass writes
// kWorldTransform/kSpatialVelocity once per Step(), from whatever
// kJointPosition/kJointVelocity held *before* that same Step()'s VI/PI
// sub-passes then advance them by dt -- so this always reports the energy
// of the state as of the start of the most recent Step() call, one dt
// behind kJointPosition/kJointVelocity's own now-current values. The two
// inputs this function does use, kWorldTransform and kSpatialVelocity, are
// always mutually consistent with each other (ABA writes both from the same
// snapshot), so this is self-consistent -- just consistently one tick
// stale. Fine for e.g. graphing energy over a real run (see
// interface::Simulation::EnableEnergyLog, which logs each row's own
// timestamp to match), but a caller diffing this against something derived
// from kJointPosition/kJointVelocity at the same instant needs to account
// for the offset (see tests/examples_two_joint_arm.cpp's own
// GenericSystemEnergyMatchesClosedFormAtEveryTick, which compares against
// the *previous* tick's closed-form state for exactly this reason).
struct SystemEnergy {
  float kinetic = 0.0F;
  float potential = 0.0F;
  float Total() const { return kinetic + potential; }
};

// `gravity` is the real gravitational acceleration vector (e.g. (0, 0,
// -9.8) for ordinary downward gravity) -- not SimConfig::base_acceleration
// directly, which ABA treats as this vector's negation (see SimConfig's own
// comment on base_acceleration); a caller reads
// -config.base_acceleration.Linear() (lane 0, since SimConfig's fields are
// batched) to get it.
SystemEnergy ComputeSystemEnergy(
    const ABAView& view,
    const domain::JointTopology& topology,
    const domain::math::Vector3<float>& gravity
);

}  // namespace achilles::algorithms::aba
