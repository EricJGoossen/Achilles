#include "algorithms/contact_solver/contact_solver_ops.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <xsimd/xsimd.hpp>

#include "algorithms/contact_solver/contact_solver_data.hpp"
#include "algorithms/conventions.hpp"
#include "algorithms/narrow_phase/narrow_phase_data.hpp"
#include "domain/math/matrix.hpp"
#include "util/simd_ops.hpp"

namespace achilles::algorithms::contact_solver {

namespace {

using narrow_phase::kSweptBoxContactSlots;
using Mask = decltype(MathematicalT() < MathematicalT());
using ContactPoints = math::Matrix<MathematicalT, 3, kSweptBoxContactSlots>;

// MuJoCo's mjMINVAL, and the [mjMINIMP, mjMAXIMP] range solimp's
// impedances are clamped to (getsolparam).
constexpr float kMinVal = 1e-15F;
constexpr float kMinImp = 1e-4F;
constexpr float kMaxImp = 0.9999F;

// The line search's upper bracket before any slope has bounded it.
constexpr float kNoUpperBound = 1e30F;

// MuJoCo's defaults for a body that gives no contact parameters.
constexpr float kDefaultFriction = 1.0F;
constexpr float kDefaultTimeconst = 0.02F;
constexpr float kDefaultDampratio = 1.0F;
constexpr std::array<float, 5> kDefaultSolimp = {
    0.9F, 0.95F, 0.001F, 0.5F, 2.0F
};

// Hager-Zhang's lower-bound parameter (mj_solPrimal's eta).
constexpr float kHagerZhangEta = 0.01F;

// Lanes [k kLanes, (k + 1) kLanes) of a contact's packed (slot, lane) word
// -- slot k's active bit for each target lane -- as a lane mask.
Mask SlotActive(const narrow_phase::ContactActiveMask& active, std::size_t k) {
  auto bits = static_cast<std::uint32_t>(active.AsStorage()) >> (k * kLanes);
  constexpr std::uint32_t kLaneBits = (std::uint32_t{1} << kLanes) - 1;
  return Mask::from_mask(bits & kLaneBits);
}

// The linear velocity (or acceleration) of world point p on a body, from its
// body-frame spatial motion and its world pose: v_O + omega x p, in world.
template <typename Motion>
Vector3 PointMotion(
    const Transform& x_world, const Motion& body, const Vector3& p
) {
  Motion world = x_world.Apply(body);
  return world.Linear() + world.Angular().Cross(p);
}

// x_up's force transform, child -> parent: [R, t^ R; 0, R] (angular first)
// -- the same matrix Transform::Apply(Matrix6x6) conjugates by.
Matrix6x6 ForceTransform(const Transform& x) {
  Matrix3x3 r = x.Rotation().ToRotationMatrix();
  Matrix6x6 result;
  result.SetSubmatrix(0, 0, r);
  result.SetSubmatrix(0, 3, x.Translation().Skew() * r);
  result.SetSubmatrix(3, 0, Matrix3x3::Zero());
  result.SetSubmatrix(3, 3, r);
  return result;
}

// A value of zero reads as `fallback`, per lane.
MathematicalT DefaultIfZero(const MathematicalT& value, float fallback) {
  return util::Select(
      value == MathematicalT(0), MathematicalT(fallback), value
  );
}

Solref DefaultSolref(const Solref& solref) {
  Mask unset =
      (solref(0, 0) == MathematicalT(0)) & (solref(1, 0) == MathematicalT(0));
  Solref fallback{
      MathematicalT(kDefaultTimeconst), MathematicalT(kDefaultDampratio)
  };
  return Solref::Select(unset, fallback, solref);
}

Solimp DefaultSolimp(const Solimp& solimp) {
  Mask unset(true);
  for (std::size_t i = 0; i < 5; ++i) {
    unset = unset & (solimp(i, 0) == MathematicalT(0));
  }
  Solimp fallback{
      MathematicalT(kDefaultSolimp[0]),
      MathematicalT(kDefaultSolimp[1]),
      MathematicalT(kDefaultSolimp[2]),
      MathematicalT(kDefaultSolimp[3]),
      MathematicalT(kDefaultSolimp[4])
  };
  return Solimp::Select(unset, fallback, solimp);
}

MathematicalT ClampImpedance(const MathematicalT& value) {
  return xsimd::min(
      xsimd::max(value, MathematicalT(kMinImp)), MathematicalT(kMaxImp)
  );
}

// getimpedance: the solimp sigmoid at x = |r| / width, from d0 at x = 0 to
// dwidth at x >= 1 -- a^-1 x^p below the midpoint, 1 - b^-1 (1 - x)^p above
// it -- or the mean of the two when it's flat (d0 = dwidth, or no width).
MathematicalT Impedance(
    const MathematicalT& r,
    const MathematicalT& d0,
    const MathematicalT& dwidth,
    const MathematicalT& width,
    const MathematicalT& midpoint,
    const MathematicalT& power
) {
  MathematicalT one(1);
  MathematicalT x = xsimd::min(
      xsimd::abs(r) / xsimd::max(width, MathematicalT(kMinVal)), one
  );
  MathematicalT a = one / xsimd::pow(midpoint, power - one);
  MathematicalT b = one / xsimd::pow(one - midpoint, power - one);
  MathematicalT y = util::Select(
      x <= midpoint,
      a * xsimd::pow(x, power),
      one - b * xsimd::pow(one - x, power)
  );
  Mask flat = (d0 == dwidth) | (width <= MathematicalT(kMinVal));
  return util::Select(
      flat, MathematicalT(0.5F) * (d0 + dwidth), d0 + y * (dwidth - d0)
  );
}

// A constraint's solref/solimp, as mj_referenceConstraint and
// mj_makeImpedance use them: getsolparam's clamps, refsafe's
// timeconst >= 2 dt, and K = 1 / (dw^2 tc^2 dr^2), B = 2 / (dw tc).
struct SoftConstraint {
  MathematicalT d0;
  MathematicalT dwidth;
  MathematicalT width;
  MathematicalT midpoint;
  MathematicalT power;
  MathematicalT stiffness;
  MathematicalT damping;
};

SoftConstraint MakeSoftConstraint(
    const Solref& solref, const Solimp& solimp, float dt
) {
  MathematicalT dwidth = ClampImpedance(solimp(1, 0));
  MathematicalT timeconst = xsimd::max(solref(0, 0), MathematicalT(2.0F * dt));
  MathematicalT dampratio = solref(1, 0);
  MathematicalT dw_tc = dwidth * timeconst;
  return {
      ClampImpedance(solimp(0, 0)),
      dwidth,
      xsimd::max(solimp(2, 0), MathematicalT(0)),
      ClampImpedance(solimp(3, 0)),
      xsimd::max(solimp(4, 0), MathematicalT(1)),
      MathematicalT(1) /
          xsimd::max(
              MathematicalT(kMinVal), dw_tc * dw_tc * dampratio * dampratio
          ),
      MathematicalT(2) / xsimd::max(MathematicalT(kMinVal), dw_tc)
  };
}

MathematicalT ImpedanceAt(const SoftConstraint& soft, const MathematicalT& r) {
  return Impedance(
      r, soft.d0, soft.dwidth, soft.width, soft.midpoint, soft.power
  );
}

// The rows' cost 1/2 D x^2 over x < 0, at the residual (qacc_smooth) and at
// residual + search (the warmstart), for any rows matrix.
template <typename Rows>
void AddWarmstartCost(
    const Rows& residual,
    const Rows& search,
    const Rows& weight,
    Scalar* smooth_out,
    Scalar* warm_out
) {
  MathematicalT zero(0);
  MathematicalT half(0.5F);
  MathematicalT smooth(0);
  MathematicalT warm(0);
  for (std::size_t i = 0; i < residual.Rows() * residual.Cols(); ++i) {
    MathematicalT at_smooth = residual[i];
    MathematicalT at_warm = residual[i] + search[i];
    smooth += util::Select(
        at_smooth < zero, half * weight[i] * at_smooth * at_smooth, zero
    );
    warm += util::Select(
        at_warm < zero, half * weight[i] * at_warm * at_warm, zero
    );
  }
  *smooth_out += Scalar(smooth);
  *warm_out += Scalar(warm);
}

// The rows' slope and curvature along search at alpha, PrimalEval's.
template <typename Rows>
void AddLineSearchTerms(
    const Scalar& alpha,
    const Rows& residual,
    const Rows& search,
    const Rows& weight,
    Scalar* slope_out,
    Scalar* curvature_out
) {
  MathematicalT zero(0);
  MathematicalT slope(0);
  MathematicalT curvature(0);
  for (std::size_t i = 0; i < residual.Rows() * residual.Cols(); ++i) {
    MathematicalT x = residual[i] + alpha.Value() * search[i];
    Mask on = x < zero;
    MathematicalT d_search = weight[i] * search[i];
    slope += util::Select(on, d_search * x, zero);
    curvature += util::Select(on, d_search * search[i], zero);
  }
  *slope_out += Scalar(slope);
  *curvature_out += Scalar(curvature);
}

}  // namespace

void ComputeInverseInertiaOp::Initialize(Matrix6x6* omega_base_out) const {
  *omega_base_out = Matrix6x6::Zero();
}

void ComputeInverseInertiaOp::operator()(
    const MotionSubspace& S,
    const InertiaOperator<false>& U,
    const InertiaOperator<true>& D_inv,
    const Transform& x_up,
    const Inertia& I,
    const Matrix6x6& omega_parent,
    Matrix6x6* omega_out,
    InvWeight* invweight_out,
    JointCoordinates* dof_invweight_out
) const {
  // S D^-1 S^T, and K = 1 - U D^-1 S^T
  Matrix6x6 d_inv_s = D_inv.AsMatrix() * S.Transpose();
  Matrix6x6 k = Matrix6x6::Identity() - U.AsMatrix() * d_inv_s;

  // Omega = S D^-1 S^T + K^T X^T Omega_parent X K
  Matrix6x6 xk = ForceTransform(x_up) * k;
  Matrix6x6 omega = S * d_inv_s + xk.Transpose() * omega_parent * xk;
  *omega_out = omega;

  // At the center of mass c = h / m: the linear acceleration of c per unit
  // force at c is [-c^ I] Omega [c^; I] = -c^ A c^ - c^ B + C c^ + D, for
  // Omega = [A B; C D] (angular first).
  MathematicalT m = I.Mass();
  Mask massive = m > MathematicalT(0);
  Vector3 c = Vector3::Select(
      massive, I.H() / util::Select(massive, m, MathematicalT(1)), Vector3()
  );
  Matrix3x3 c_hat = c.Skew();
  Matrix3x3 a = omega.Submatrix<3, 3>(0, 0);
  Matrix3x3 b = omega.Submatrix<3, 3>(0, 3);
  Matrix3x3 cc = omega.Submatrix<3, 3>(3, 0);
  Matrix3x3 d = omega.Submatrix<3, 3>(3, 3);
  Matrix3x3 lambda = d + cc * c_hat - c_hat * b - c_hat * a * c_hat;

  MathematicalT third(1.0F / 3.0F);
  *invweight_out = InvWeight(lambda.Trace() * third, a.Trace() * third);

  // M^-1's diagonal block D^-1 + G^T Omega_parent G, G = X U D^-1
  Matrix6x6 g = ForceTransform(x_up) * (U.AsMatrix() * D_inv.AsMatrix());
  Matrix6x6 block = D_inv.AsMatrix() + g.Transpose() * omega_parent * g;
  for (std::size_t k = 0; k < 6; ++k) {
    (*dof_invweight_out)[k] = block(k, k);
  }
}

void SetupRowsOp::operator()(
    const Vector3& normal,
    const math::Matrix<MathematicalT, kSweptBoxContactSlots, 1>& distance,
    const ContactPoints& point,
    const narrow_phase::ContactActiveMask& active,
    const Scalar& margin_target,
    const Scalar& margin_source,
    const Scalar& friction_target,
    const Scalar& friction_source,
    const Solref& solref_target,
    const Solref& solref_source,
    const Solimp& solimp_target,
    const Solimp& solimp_source,
    const InvWeight& invweight_target,
    const InvWeight& invweight_source,
    const Transform& x_target,
    const Transform& x_source,
    const Velocity& v_target,
    const Velocity& v_source,
    const Acceleration& a_target,
    const Acceleration& a_source,
    EdgeDirections* edges_out,
    ContactPoints* point_out,
    ContactRows* reference_out,
    ContactRows* weight_out,
    ContactRows* residual_out
) const {
  std::array<Mask, kSweptBoxContactSlots> slot_active{};
  Mask any_active(false);
  for (std::size_t k = 0; k < kSweptBoxContactSlots; ++k) {
    slot_active[k] = SlotActive(active, k);
    any_active = any_active | slot_active[k];
  }

  // Pair parameters: mu = max, solref/solimp the mean (mj_contactParam,
  // solmix 1:1), margin the sum (mj_assignMargin)
  MathematicalT mu = xsimd::max(
      DefaultIfZero(friction_target.Value(), kDefaultFriction),
      DefaultIfZero(friction_source.Value(), kDefaultFriction)
  );
  Solref solref =
      (DefaultSolref(solref_target) + DefaultSolref(solref_source)) *
      MathematicalT(0.5F);
  Solimp solimp =
      (DefaultSolimp(solimp_target) + DefaultSolimp(solimp_source)) *
      MathematicalT(0.5F);
  MathematicalT margin = margin_target.Value() + margin_source.Value();

  SoftConstraint soft = MakeSoftConstraint(solref, solimp, dt_);

  // mju_makeFrame: t1 from +y (or +z when n is near y), made orthogonal to
  // n; t2 = n x t1. An inactive lane's normal is replaced first, so its
  // (unused) frame is still finite.
  Vector3 n = Vector3::Select(any_active, normal, Vector3::UnitX());
  Vector3 seed = Vector3::Select(
      xsimd::abs(n.Y()) < MathematicalT(0.5F),
      Vector3::UnitY(),
      Vector3::UnitZ()
  );
  Vector3 t1 = (seed - n * n.Dot(seed)).Normalize();
  Vector3 t2 = n.Cross(t1);
  std::array<Vector3, kPyramidEdges> edges = {
      n + t1 * mu, n - t1 * mu, n + t2 * mu, n - t2 * mu
  };
  for (std::size_t e = 0; e < kPyramidEdges; ++e) {
    for (std::size_t i = 0; i < 3; ++i) {
      (*edges_out)(i, e) = edges[e][i];
    }
  }

  // mj_diagApprox, pyramidal: (1 + mu^2) (invweight_t + invweight_s)
  MathematicalT diag_a = (MathematicalT(1) + mu * mu) *
                         (invweight_target(0, 0) + invweight_source(0, 0));

  for (std::size_t k = 0; k < kSweptBoxContactSlots; ++k) {
    Vector3 p =
        Vector3::Select(slot_active[k], Vector3(point.Column(k)), Vector3());
    for (std::size_t i = 0; i < 3; ++i) {
      (*point_out)(i, k) = p[i];
    }

    MathematicalT r = distance(k, 0) - margin;
    Mask row_active = slot_active[k] & (r < MathematicalT(0));
    MathematicalT imp = ImpedanceAt(soft, r);

    // R = (1 - d) / d A_ii, then every pyramidal row's R = 2 mu^2 R
    // (mj_makeImpedance, impratio 1), and D = 1 / R
    MathematicalT weight =
        MathematicalT(1) /
        xsimd::max(
            MathematicalT(kMinVal),
            MathematicalT(2) * mu * mu * (MathematicalT(1) - imp) / imp * diag_a
        );

    Vector3 dv =
        PointMotion(x_target, v_target, p) - PointMotion(x_source, v_source, p);
    Vector3 da =
        PointMotion(x_target, a_target, p) - PointMotion(x_source, a_source, p);
    for (std::size_t e = 0; e < kPyramidEdges; ++e) {
      // aref = -B (J v) - K d r
      MathematicalT reference =
          -soft.damping * edges[e].Dot(dv) - soft.stiffness * imp * r;
      MathematicalT zero(0);
      (*reference_out)(e, k) = util::Select(row_active, reference, zero);
      (*weight_out)(e, k) = util::Select(row_active, weight, zero);
      (*residual_out)(e, k) =
          util::Select(row_active, edges[e].Dot(da) - reference, zero);
    }
  }
}

void ProjectSearchOp::operator()(
    const EdgeDirections& edges,
    const ContactPoints& point,
    const Transform& x_target,
    const Transform& x_source,
    const Acceleration& a_target,
    const Acceleration& a_source,
    ContactRows* search_out
) const {
  for (std::size_t k = 0; k < kSweptBoxContactSlots; ++k) {
    Vector3 p(point.Column(k));
    Vector3 da =
        PointMotion(x_target, a_target, p) - PointMotion(x_source, a_source, p);
    for (std::size_t e = 0; e < kPyramidEdges; ++e) {
      (*search_out)(e, k) = Vector3(edges.Column(e)).Dot(da);
    }
  }
}

void ContactForceOp::operator()(
    const Scalar& alpha,
    const ContactRows& search,
    const ContactRows& weight,
    const EdgeDirections& edges,
    const ContactPoints& point,
    ContactRows* residual_out,
    Force* wrench_target_out,
    Force* wrench_source_out
) const {
  Vector3 force_sum;
  Vector3 torque_sum;
  for (std::size_t k = 0; k < kSweptBoxContactSlots; ++k) {
    Vector3 force;
    for (std::size_t e = 0; e < kPyramidEdges; ++e) {
      MathematicalT residual =
          (*residual_out)(e, k) + alpha.Value() * search(e, k);
      (*residual_out)(e, k) = residual;
      // f = -D (J qacc - aref) while negative
      MathematicalT f = util::Select(
          residual < MathematicalT(0),
          -weight(e, k) * residual,
          MathematicalT(0)
      );
      force += Vector3(edges.Column(e)) * f;
    }
    force_sum += force;
    torque_sum += Vector3(point.Column(k)).Cross(force);
  }
  Force wrench(torque_sum, force_sum);
  *wrench_target_out += wrench;
  *wrench_source_out -= wrench;
}

void WarmstartCostOp::operator()(
    const ContactRows& residual,
    const ContactRows& search,
    const ContactRows& weight,
    Scalar* smooth_out,
    Scalar* warm_out
) const {
  AddWarmstartCost(residual, search, weight, smooth_out, warm_out);
}

void LineSearchRowsOp::operator()(
    const Scalar& alpha,
    const ContactRows& residual,
    const ContactRows& search,
    const ContactRows& weight,
    Scalar* slope_out,
    Scalar* curvature_out
) const {
  AddLineSearchTerms(alpha, residual, search, weight, slope_out, curvature_out);
}

void SetupLimitRowsOp::operator()(
    const JointCoordinates& distance,
    const JointCoordinates& side,
    const Mat6Mask& active,
    const Scalar& margin,
    const Solref& solref,
    const Solimp& solimp,
    const JointCoordinates& dof_invweight,
    const Velocity& qd,
    const Acceleration& qacc_smooth,
    JointCoordinates* weight_out,
    JointCoordinates* residual_out
) const {
  SoftConstraint soft =
      MakeSoftConstraint(DefaultSolref(solref), DefaultSolimp(solimp), dt_);
  MathematicalT zero(0);
  for (std::size_t k = 0; k < 6; ++k) {
    Mask row_active =
        util::Select(active[k], MathematicalT(1), MathematicalT(0)) != zero;
    MathematicalT r = distance[k] - margin.Value();
    MathematicalT imp = ImpedanceAt(soft, r);

    // R = (1 - d) / d A_ii, A_ii = dof_invweight0, and D = 1 / R
    MathematicalT weight =
        MathematicalT(1) / xsimd::max(
                               MathematicalT(kMinVal),
                               (MathematicalT(1) - imp) / imp * dof_invweight[k]
                           );

    // aref = -B (J v) - K d r, J = side e_k
    MathematicalT reference =
        -soft.damping * side[k] * qd[k] - soft.stiffness * imp * r;
    (*weight_out)[k] = util::Select(row_active, weight, zero);
    (*residual_out)[k] =
        util::Select(row_active, side[k] * qacc_smooth[k] - reference, zero);
  }
}

void LimitForceOp::operator()(
    const Scalar& alpha,
    const JointCoordinates& search,
    const JointCoordinates& weight,
    const JointCoordinates& side,
    JointCoordinates* residual_out,
    Force* constraint_force_out
) const {
  std::array<MathematicalT, 6> joint_force{};
  for (std::size_t k = 0; k < 6; ++k) {
    MathematicalT residual = (*residual_out)[k] + alpha.Value() * search[k];
    (*residual_out)[k] = residual;
    // f = -D (J qacc - aref) while negative, J^T f = side f e_k
    MathematicalT f = util::Select(
        residual < MathematicalT(0), -weight[k] * residual, MathematicalT(0)
    );
    joint_force[k] = side[k] * f;
  }
  *constraint_force_out += Force(
      joint_force[0],
      joint_force[1],
      joint_force[2],
      joint_force[3],
      joint_force[4],
      joint_force[5]
  );
}

void WarmstartLimitCostOp::operator()(
    const JointCoordinates& residual,
    const JointCoordinates& search,
    const JointCoordinates& weight,
    Scalar* smooth_out,
    Scalar* warm_out
) const {
  AddWarmstartCost(residual, search, weight, smooth_out, warm_out);
}

void LineSearchLimitsOp::operator()(
    const Scalar& alpha,
    const JointCoordinates& residual,
    const JointCoordinates& search,
    const JointCoordinates& weight,
    Scalar* slope_out,
    Scalar* curvature_out
) const {
  AddLineSearchTerms(alpha, residual, search, weight, slope_out, curvature_out);
}

void ConjugacyTermsOp::operator()(
    const Acceleration& search,
    const Force& grad,
    const Force& grad_prev,
    const Acceleration& m_grad,
    const Acceleration& m_grad_prev,
    Scalar* d_y_out,
    Scalar* y_my_out,
    Scalar* y_mgrad_out,
    Scalar* d_grad_out,
    Scalar* d_d_out,
    Scalar* grad_grad_out
) const {
  const Vector6& d = search.AsVector6();
  Vector6 y = (grad - grad_prev).AsVector6();
  Vector6 my = (m_grad - m_grad_prev).AsVector6();
  *d_y_out += Scalar(d.Dot(y));
  *y_my_out += Scalar(y.Dot(my));
  *y_mgrad_out += Scalar(y.Dot(m_grad.AsVector6()));
  *d_grad_out += Scalar(d.Dot(grad.AsVector6()));
  *d_d_out += Scalar(d.Dot(d));
  *grad_grad_out += Scalar(grad.AsVector6().Dot(grad.AsVector6()));
}

void ChooseWarmstartOp::operator()(
    const GlobalScalar& cost_smooth,
    const GlobalScalar& cost_warm,
    const GlobalScalar& s_m_s,
    const GlobalScalar& s_ma,
    GlobalScalar* alpha_out,
    GlobalScalar* restart_out
) const {
  // Gauss's cost at the warmstart: 1/2 s M s + s Ma, Ma being 0 at
  // qacc_smooth
  float warm = cost_warm.Value() + 0.5F * s_m_s.Value() + s_ma.Value();
  *alpha_out =
      GlobalScalar(util::Select(warm < cost_smooth.Value(), 1.0F, 0.0F));
  *restart_out = GlobalScalar(1.0F);
}

void ConjugateOp::operator()(
    const GlobalScalar& d_y,
    const GlobalScalar& y_my,
    const GlobalScalar& y_mgrad,
    const GlobalScalar& d_grad,
    const GlobalScalar& d_d,
    const GlobalScalar& grad_grad,
    GlobalScalar* beta_out,
    GlobalScalar* restart_out
) const {
  // beta_hz = (y.Mgrad - 2 (y.My / d.y) d.grad) / d.y
  float safe_d_y = util::Select(d_y.Value() < kMinVal, 1.0F, d_y.Value());
  float beta_hz =
      (y_mgrad.Value() - 2.0F * (y_my.Value() / safe_d_y) * d_grad.Value()) /
      safe_d_y;
  // eta_k = -1 / max(MINVAL, |d| min(eta, |grad|))
  float eta_k =
      -1.0F / xsimd::max(
                  kMinVal,
                  xsimd::sqrt(d_d.Value()) *
                      xsimd::min(kHagerZhangEta, xsimd::sqrt(grad_grad.Value()))
              );
  float beta = xsimd::max(eta_k, beta_hz);
  bool steepest = (d_y.Value() < kMinVal) || (restart_out->Value() > 0.5F);
  *beta_out = GlobalScalar(util::Select(steepest, 0.0F, beta));
  *restart_out = GlobalScalar(0.0F);
}

void BeginLineSearchOp::operator()(
    GlobalScalar* alpha_out, GlobalScalar* low_out, GlobalScalar* high_out
) const {
  *alpha_out = GlobalScalar(0.0F);
  *low_out = GlobalScalar(0.0F);
  *high_out = GlobalScalar(kNoUpperBound);
}

void NewtonStepOp::operator()(
    const GlobalScalar& row_slope,
    const GlobalScalar& row_curvature,
    const GlobalScalar& s_m_s,
    const GlobalScalar& s_ma,
    GlobalScalar* alpha_out,
    GlobalScalar* low_out,
    GlobalScalar* high_out
) const {
  float alpha = alpha_out->Value();
  float slope = row_slope.Value() + alpha * s_m_s.Value() + s_ma.Value();
  float curvature = row_curvature.Value() + s_m_s.Value();

  // The slope's sign at alpha bounds the minimum from that side
  float low = util::Select(slope < 0.0F, alpha, low_out->Value());
  float high = util::Select(slope > 0.0F, alpha, high_out->Value());

  float newton = alpha - slope / xsimd::max(curvature, kMinVal);
  bool inside = (newton >= low) && (newton <= high);
  *alpha_out = GlobalScalar(util::Select(inside, newton, 0.5F * (low + high)));
  *low_out = GlobalScalar(low);
  *high_out = GlobalScalar(high);
}

}  // namespace achilles::algorithms::contact_solver
