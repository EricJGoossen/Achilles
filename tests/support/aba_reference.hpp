#pragma once

#include "algorithms/aba/aba_ops.hpp"
#include "algorithms/conventions.hpp"
#include "domain/math/activation_mask.hpp"
#include "domain/math/matrix.hpp"
#include "domain/spatial/dual.hpp"
#include "domain/spatial/inertia.hpp"
#include "domain/spatial/transform.hpp"

namespace achilles::test_support {

// A single-DOF revolute joint about Z (subspace row 2 -- yaw, per Dual's
// Roll/Pitch/Yaw/X/Y/Z row order -- driven by generalized-coordinate slot
// 0), mass-2 trivially-physically-valid inertia -- the one concrete joint
// shape every ABA full-stack test builds on, whether fed through a real
// View directly (algorithms_aba_aba_step.cpp) or through a real .arow file
// (interface_simulation.cpp). Kept in one place so a test comparing a real
// multi-joint SimAllocator/Simulation result against a hand-computed
// reference (see the Compute*/AccumulateInertia helpers below) always
// means the same physical joint on both sides.
inline algorithms::Matrix6x6 RevoluteZSubspace() {
  algorithms::Matrix6x6 s = algorithms::Matrix6x6::Zero();
  s(2, 0) = algorithms::MathematicalT(1.0F);
  return s;
}

inline algorithms::Mat6Mask DOF0ActiveMask() {
  return algorithms::Mat6Mask(true, false, false, false, false, false);
}

inline algorithms::Inertia SimpleInertia() {
  using B = algorithms::MathematicalT;
  return {
      B(2.0F),
      algorithms::Vector3::Zero(),
      B(2.0F),
      B(3.0F),
      B(4.0F),
      B(0.0F),
      B(0.0F),
      B(0.0F)
  };
}

// -- Direct-op reference computations --
//
// PropagateVelocityOp/PropagateInertiaOp/PropagateAccelerationOp's own math
// is already unit-tested directly in algorithms_aba_aba_ops.cpp. A
// multi-joint full-stack test doesn't re-derive that spatial algebra by
// hand -- it calls these same real Op types directly, in the same order
// and with the same base-row convention RunPass/ABAStep::Step itself uses,
// to build an independent reference result for a real chain/tree, then
// compares a real SimAllocator/Simulation + ABAStep traversal's own output
// against it. What's under test at the call site is whether the real
// traversal wires real joints together correctly (right parent read, right
// accumulation target, right Stride) -- not whether any one Op's formula
// is correct, which the ops file already covers.

struct AbaVelocityOutputs {
  algorithms::InertiaOperator<false> i_a;
  algorithms::Transform x_up;
  algorithms::Transform x_world;
  algorithms::Velocity v;
  algorithms::Acceleration c;
  algorithms::Force p;
};

inline AbaVelocityOutputs ComputeAbaVelocity(
    const algorithms::Transform& x_world_parent,
    const algorithms::Velocity& v_parent,
    algorithms::MathematicalT q0,
    algorithms::Velocity qd = algorithms::Velocity::Zero()
) {
  using B = algorithms::MathematicalT;
  algorithms::aba::PropagateVelocityOp op(
      algorithms::Transform::Identity(), algorithms::Velocity::Zero()
  );
  AbaVelocityOutputs out;
  algorithms::Vector6 q(q0, B(0.0F), B(0.0F), B(0.0F), B(0.0F), B(0.0F));
  op(RevoluteZSubspace(),
     SimpleInertia(),
     x_world_parent,
     algorithms::Transform::Identity(),
     q,
     qd,
     v_parent,
     &out.i_a,
     &out.x_up,
     &out.x_world,
     &out.v,
     &out.c,
     &out.p);
  return out;
}

struct AbaInertiaOutputs {
  algorithms::InertiaOperator<false> u;
  algorithms::InertiaOperator<true> d_inv;
  algorithms::Force u_force;
};

// Folds `self`'s own contribution onto (i_a_parent, p_parent) via a direct
// PropagateInertiaOp::operator() call -- the same += accumulation
// PropagateInertiaOpTest.AccumulatesIntoParentOutputsRatherThanOverwriting
// (algorithms_aba_aba_ops.cpp) proves in isolation, reused here to build a
// real multi-joint reference result instead of re-deriving it.
inline AbaInertiaOutputs AccumulateAbaInertia(
    const AbaVelocityOutputs& self,
    const algorithms::Force& tau,
    algorithms::InertiaOperator<false>& i_a_parent,
    algorithms::Force& p_parent
) {
  algorithms::aba::PropagateInertiaOp op;
  AbaInertiaOutputs out;
  op(RevoluteZSubspace(),
     DOF0ActiveMask(),
     self.x_up,
     self.i_a,
     self.c,
     tau,
     self.p,
     &i_a_parent,
     &out.u,
     &out.d_inv,
     &p_parent,
     &out.u_force);
  return out;
}

inline algorithms::Acceleration ComputeAbaAcceleration(
    const AbaInertiaOutputs& self_inertia,
    const algorithms::Transform& x_up,
    const algorithms::Acceleration& c,
    const algorithms::Acceleration& a_parent
) {
  algorithms::aba::PropagateAccelerationOp op(algorithms::Acceleration::Zero());
  algorithms::Acceleration qdd;
  algorithms::Acceleration a;
  op(RevoluteZSubspace(),
     self_inertia.d_inv,
     self_inertia.u,
     x_up,
     c,
     a_parent,
     self_inertia.u_force,
     &qdd,
     &a);
  return a;
}

}  // namespace achilles::test_support
