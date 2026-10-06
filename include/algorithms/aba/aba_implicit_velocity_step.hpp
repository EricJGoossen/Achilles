#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

#include "algorithms/aba/aba_data.hpp"
#include "algorithms/aba/aba_step.hpp"
#include "algorithms/conventions.hpp"
#include "algorithms/pi/pi_step.hpp"
#include "algorithms/sim_config.hpp"
#include "domain/math/activation_mask.hpp"
#include "domain/math/vector6.hpp"
#include "domain/spatial/dual.hpp"
#include "engine/pass/sim_context.hpp"
#include "engine/topology/ordering_policy.hpp"
#include "util/simd_ops.hpp"

namespace achilles::algorithms::aba {

// MuJoCo's implicit-in-velocity integrator, adapted to this engine's ABA
// -- specifically their *implicit* variant, not the cheaper
// *implicitfast* one (see below). Reference: MuJoCo's own "Computation"
// docs, https://mujoco.readthedocs.io/en/stable/computation/index.html.
//
// MuJoCo's update is
//     (M + h D) v_{t+h} = (M + h D) v_t + h M a(v_t),      D = d(c)/dv
// with c the velocity-dependent (Coriolis/centripetal/gyroscopic) bias
// force and M the joint-space mass matrix. Since ABA already gives us
// a = M^-1 (tau - c), that whole system collapses to something ABA can
// drive directly, with no need to ever form M:
//     M^-1 D = -d(a)/dv  =:  -J
//     v_{t+h} = v_t + h (M + hD)^-1 M a = v_t + h (I - h J)^-1 a
// so all this needs is J = d(qdd)/d(qd), the full joint-space Jacobian of
// joint acceleration with respect to joint velocity, plus one small dense
// solve. That is exactly what the loop below builds.
//
// J is n-by-n over the sim's own generalized coordinates (n = total active
// DOFs), NOT per-joint: MuJoCo restricts D to the *mass matrix's* sparsity
// pattern, which follows the kinematic tree -- and for a serial chain that
// pattern is dense, because every ancestor/descendant joint pair couples.
// An earlier version of this file got that wrong, restricting D to a
// per-joint block diagonal, and measured *identically* to plain
// semi-implicit Euler on every scene here. The reason is worth keeping:
// for a 1-DOF joint the diagonal entry d(qdd_i)/d(qd_i) picks up only
// that joint's own gyroscopic self-term, and a gyroscopic force does zero
// work along the direction that generates it, so the entry vanishes
// identically for a leaf joint. All of the actual effect lives in the
// off-diagonal (cross-joint) entries and in the subtree contributions a
// non-leaf joint's own diagonal picks up through them -- i.e. exactly
// what a block-diagonal restriction throws away. For the two-link arm,
// hand-deriving D from the Lagrangian (see tests/examples_two_joint_arm
// .cpp's own reference model) gives D11 = 2R sin(psi) psi_dot,
// D12 = 2R sin(psi)(phi_dot + psi_dot), D21 = -2R sin(psi) phi_dot,
// D22 = 0 -- only the leaf entry D22 is zero, and D is asymmetric, which
// matches MuJoCo's own note that the RNE derivatives are the main source
// of D's asymmetry (and why they need LU rather than Cholesky).
//
// Cost: n + 1 ABA evaluations per tick, because J's columns are built by
// finite differences -- perturb one generalized velocity, re-run ABA, read
// the column. That is the honest, obviously-correct way to get J, but it
// is *not* what MuJoCo does: they compute d(c)/dv analytically in one
// O(n) pass (mj_rneDerivative, following the RNEA-derivative formulation
// of Carpentier & Mansard), which makes their cost ~2 evaluations flat,
// independent of n. The n+1 scaling here is fine for a handful of joints
// and bad for a humanoid.
//
// WHAT THIS IS AND IS NOT FOR. This is a *stability* mechanism, not an
// energy-conservation one, and measurement here bears that out:
//   - two-joint arm, dt=1/120: mean |E| drift 2.69 vs Euler's 3.20, but
//     it *gains* energy (ends at +13.3 where Euler ends at -4.3).
//   - three-joint chain, dt=1/120: mean |E| 8.70, clearly worse than
//     Euler's 2.59 -- again by gaining energy (+52.9).
//   - but at dt=1/30, where Euler diverges to ~1e23 and aborts 10s in,
//     this survives a full 31s run and stays finite (~1e3).
// That is exactly the tradeoff implicit-Euler-in-velocity makes. It is
// unconditionally stable, which is the point, but it is not symplectic:
// applied to a gyroscopic J -- which is asymmetric and indefinite, not
// damping-like -- (I - hJ)^-1 has eigenvalues 1/(1 - h*lambda) > 1 in
// some directions, so it *amplifies* rather than damps there. For a
// passive system whose whole problem is energy drift, that is the wrong
// tool, and MuJoCo's own docs agree: they recommend RK4, not this, for
// "energy-conserving systems", noting RK4 wins "even when the timestep is
// decreased by a factor of 4 (so the computational effort is identical)".
// Use ImplicitMidpointStep (time-symmetric, and measured ~1000x better
// here) when energy behavior is what matters; reach for this when the
// problem is a large timestep causing blow-up.
//
// Note also which MuJoCo variant this is. Their recommended default,
// implicitfast, *drops* these RNE derivatives entirely (keeping only
// damping/actuator/fluid terms) precisely because they are the most
// expensive part -- for a passive, undamped chain like the scenes here
// that would leave D = 0 and degenerate to semi-implicit Euler outright.
struct ImplicitVelocityStep {
  using ScalarVelocity = domain::spatial::SpatialVelocity<ScalarOperationT>;
  using ScalarAcceleration =
      domain::spatial::SpatialAcceleration<ScalarOperationT>;
  using Vector6 = domain::math::Vector6<ScalarOperationT>;
  using MaskStorage = util::MaskStorageFor<ScalarOperationT>;
  using ActivationMask = domain::math::ActivationMask<MaskStorage, 6>;

  // One generalized coordinate: which joint row it belongs to, and which
  // slot (column of that joint's own subspace S, i.e. the index
  // kJointVelocity/kJointAcceleration actually store it at -- not a
  // spatial row) it occupies within that row's own 6-wide storage.
  struct Dof {
    std::size_t row;
    std::size_t slot;
  };

  static ScalarVelocity WithSlot(
      const ScalarVelocity& v, std::size_t slot, ScalarOperationT value
  ) {
    std::array<ScalarOperationT, 6> c{v[0], v[1], v[2], v[3], v[4], v[5]};
    c[slot] = value;
    return ScalarVelocity(Vector6(c[0], c[1], c[2], c[3], c[4], c[5]));
  }

  // Solves `system` (n-by-n, row-major) * x = `rhs` in place by Gaussian
  // elimination with partial pivoting, returning false (leaving `rhs`
  // untouched) if the system is singular to working precision. Written
  // here rather than reusing Matrix6x6::Inverse because n is the sim's own
  // DOF count, not 6, and because that routine documents itself as valid
  // only for spatial-inertia-shaped matrices -- which (I - h J) is not.
  static bool SolveInPlace(
      std::vector<ScalarOperationT>& system,
      std::vector<ScalarOperationT>& rhs,
      std::size_t n
  ) {
    for (std::size_t col = 0; col < n; ++col) {
      std::size_t pivot = col;
      for (std::size_t r = col + 1; r < n; ++r) {
        if (std::abs(system[r * n + col]) > std::abs(system[pivot * n + col])) {
          pivot = r;
        }
      }
      if (std::abs(system[pivot * n + col]) < 1e-9) {
        return false;
      }
      if (pivot != col) {
        for (std::size_t c = 0; c < n; ++c) {
          std::swap(system[pivot * n + c], system[col * n + c]);
        }
        std::swap(rhs[pivot], rhs[col]);
      }
      ScalarOperationT diagonal = system[col * n + col];
      for (std::size_t r = col + 1; r < n; ++r) {
        ScalarOperationT factor = system[r * n + col] / diagonal;
        if (factor == 0.0) {
          continue;
        }
        for (std::size_t c = col; c < n; ++c) {
          system[r * n + c] -= factor * system[col * n + c];
        }
        rhs[r] -= factor * rhs[col];
      }
    }
    for (std::size_t i = n; i-- > 0;) {
      ScalarOperationT sum = rhs[i];
      for (std::size_t c = i + 1; c < n; ++c) {
        sum -= system[i * n + c] * rhs[c];
      }
      rhs[i] = sum / system[i * n + i];
    }
    return true;
  }

  template <engine::pass::SimContextLike SimStateT>
  static void Step(
      const SimStateT& sim_state, const SimConfig& sim_config, float dt
  ) {
    const auto view = sim_state.template ViewFor<ABAView>();
    // Real joints only -- a padding row's fields aren't meaningful under
    // scalar Load (see ComputeSystemEnergy/SceneRenderer, same restriction).
    const auto& topology =
        sim_state.template TopologyFor<engine::topology::TopologicalOrdering>();
    const std::size_t rows = topology.Size();

    std::vector<Dof> dofs;
    std::vector<ScalarVelocity> original(rows);
    for (std::size_t row = 0; row < rows; ++row) {
      original[row] =
          view.template Load<ABAField::kJointVelocity, ScalarOperationT>(row);
      ActivationMask mask =
          view.template Load<ABAField::kJointActivationMask, MaskStorage>(row);
      for (std::size_t slot = 0; slot < 6; ++slot) {
        if (mask[slot]) {
          dofs.push_back({row, slot});
        }
      }
    }
    const std::size_t n = dofs.size();
    if (n == 0) {
      ABAStep::Step(sim_state, sim_config, dt);
      pi::PIStep::Step(sim_state, sim_config, dt);
      return;
    }

    // Column j of J: perturb generalized velocity j, re-run ABA, read every
    // DOF's own resulting qdd. The unperturbed run comes *last* (below) so
    // that every field ABA writes -- kWorldTransform/kSpatialVelocity, which
    // the renderer and ComputeSystemEnergy both read -- is left describing
    // the real state rather than the last probe.
    std::vector<ScalarOperationT> perturbed(n * n);
    std::vector<ScalarOperationT> epsilons(n);
    for (std::size_t j = 0; j < n; ++j) {
      const Dof& dof = dofs[j];
      ScalarOperationT base = original[dof.row][dof.slot];
      ScalarOperationT epsilon = 1e-3 * std::max(1.0, std::abs(base));
      epsilons[j] = epsilon;
      view.template Store<ABAField::kJointVelocity, ScalarOperationT>(
          dof.row, WithSlot(original[dof.row], dof.slot, base + epsilon)
      );
      ABAStep::Step(sim_state, sim_config, dt);
      for (std::size_t i = 0; i < n; ++i) {
        ScalarAcceleration qdd =
            view.template Load<ABAField::kJointAcceleration, ScalarOperationT>(
                dofs[i].row
            );
        perturbed[i * n + j] = qdd[dofs[i].slot];
      }
      view.template Store<ABAField::kJointVelocity, ScalarOperationT>(
          dof.row, original[dof.row]
      );
    }

    ABAStep::Step(sim_state, sim_config, dt);
    std::vector<ScalarOperationT> acceleration(n);
    for (std::size_t i = 0; i < n; ++i) {
      ScalarAcceleration qdd =
          view.template Load<ABAField::kJointAcceleration, ScalarOperationT>(
              dofs[i].row
          );
      acceleration[i] = qdd[dofs[i].slot];
    }

    // system = I - h J, with J(i,j) = (qdd_i(v + eps e_j) - qdd_i(v)) / eps.
    std::vector<ScalarOperationT> system(n * n, 0.0);
    for (std::size_t j = 0; j < n; ++j) {
      for (std::size_t i = 0; i < n; ++i) {
        ScalarOperationT jacobian =
            (perturbed[i * n + j] - acceleration[i]) / epsilons[j];
        system[i * n + j] = (i == j ? 1.0 : 0.0) - dt * jacobian;
      }
    }

    std::vector<ScalarOperationT> delta = acceleration;
    if (!SolveInPlace(system, delta, n)) {
      // Singular to working precision (a large enough dt can do this):
      // fall back to the ordinary explicit velocity update rather than
      // propagating a garbage solve.
      delta = acceleration;
    }

    for (std::size_t k = 0; k < n; ++k) {
      const Dof& dof = dofs[k];
      ScalarVelocity current =
          view.template Load<ABAField::kJointVelocity, ScalarOperationT>(dof.row
          );
      view.template Store<ABAField::kJointVelocity, ScalarOperationT>(
          dof.row,
          WithSlot(current, dof.slot, current[dof.slot] + dt * delta[k])
      );
    }

    pi::PIStep::Step(sim_state, sim_config, dt);
  }
};

}  // namespace achilles::algorithms::aba
