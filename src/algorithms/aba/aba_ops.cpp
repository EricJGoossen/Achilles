#include "algorithms/aba/aba_ops.hpp"

namespace achilles::algorithms::aba {

void PropagateVelocityOp::Initialize(Transform* x_world_out, Velocity* v_out)
    const {
  *x_world_out = x_world_base;
  *v_out = v_base;
}

void PropagateVelocityOp::operator()(
    const MotionSubspace& S,
    const Inertia& I,
    const Transform& x_world_parent,
    const Transform& x_tree,
    const Vector6& q,
    const Velocity& qd,
    const Velocity& v_parent,
    InertiaOperator<false>* I_A_out,
    Transform* x_up_out,
    Transform* x_world_out,
    Velocity* v_out,
    Acceleration* c_out,
    Force* p_out
) const {
  Velocity joint_twist = S * q;
  Transform x_joint = Transform::Exp(joint_twist);

  Transform x_up = x_tree * x_joint;

  Velocity qd_spatial = S * qd;
  Velocity v = x_up.Inverse().Apply(v_parent) + qd_spatial;

  *I_A_out = I.AsArticulated();
  *x_up_out = x_up;
  *x_world_out = x_world_parent * x_up;
  *v_out = v;

  *c_out = v.Cross(qd_spatial);
  *p_out = v.CrossForce(I.Apply(v));
}

void PropagateInertiaOp::Initialize(
    InertiaOperator<false>* I_A_base_out, Force* p_base_out
) const {
  *I_A_base_out = kIABase;
  *p_base_out = kPBase;
}
void PropagateInertiaOp::operator()(
    const MotionSubspace& S,
    const Mat6Mask& mask,
    const Transform& x_up,
    const InertiaOperator<false>& I_A,
    const Acceleration& c,
    const Force& tau,
    const Force& p,
    InertiaOperator<false>* I_A_parent_out,
    InertiaOperator<false>* U_out,
    InertiaOperator<true>* D_inv_out,
    Force* p_parent_out,
    Force* u_out
) const {
  // U = I_A S
  InertiaOperator<false> U = I_A.AsMatrix() * S;

  // D = S^T U
  InertiaOperator<false> D = S.Transpose() * U;

  // u = tau - S^T p
  Force u = tau - S.Transpose() * p;

  // D^-1
  InertiaOperator<true> D_inv = D.MaskedInverse(mask);

  // U D^-1
  Matrix6x6 U_D_inv = U.AsMatrix() * D_inv.AsMatrix();

  // I_A' = I_A - U D^-1 U^T (this joint's contribution, before
  // transforming into the parent's frame)
  InertiaOperator<false> I_A_contribution = I_A - U_D_inv * U.Transpose();

  // p' = p + I_A' c + U D^-1 u (ditto)
  Force p_contribution = p + I_A_contribution.Apply(c) + U_D_inv * u;

  *U_out = U;
  *D_inv_out = D_inv;
  *u_out = u;

  *I_A_parent_out += x_up.Apply(I_A_contribution);
  *p_parent_out += x_up.Apply(p_contribution);
}

void PropagateAccelerationOp::Initialize(Acceleration* a_base_out) const {
  *a_base_out = a_base;
}

void PropagateAccelerationOp::operator()(
    const Matrix6x6& S,
    const InertiaOperator<true>& D_inv,
    const InertiaOperator<false>& U,
    const Transform& x_up,
    const Acceleration& c,
    const Acceleration& a_parent,
    const Force& u,
    Acceleration* qdd_out,
    Acceleration* a_out
) const {
  Acceleration a_pre = x_up.Inverse().Apply(a_parent) + c;

  Acceleration qdd = D_inv.Apply(u - U.Transpose().Apply(a_pre));

  Acceleration a = a_pre + S * qdd;

  *qdd_out = qdd;
  *a_out = a;
}

}  // namespace achilles::algorithms::aba
