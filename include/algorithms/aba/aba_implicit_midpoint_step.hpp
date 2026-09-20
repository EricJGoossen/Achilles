#pragma once

#include <cstddef>
#include <vector>

#include "algorithms/aba/aba_data.hpp"
#include "algorithms/aba/aba_step.hpp"
#include "algorithms/sim_config.hpp"
#include "domain/math/matrix.hpp"
#include "domain/spatial/dual.hpp"
#include "domain/spatial/transform.hpp"
#include "engine/pass/sim_context.hpp"

namespace achilles::algorithms::aba {

// Implicit-midpoint integration of one full dt: evaluates ABA's own
// forward dynamics at the time-symmetric AVERAGE of the old and new state,
// rather than at the old state alone (kSemiImplicitEuler) or an
// extrapolated half-kicked velocity (kVelocityVerlet) -- see
// interface::Simulation::Integrator's own comment on why the plain
// versions lose their nice energy-error guarantee for a system with
// ABA's own velocity-dependent Coriolis/gyroscopic bias terms. The
// midpoint rule doesn't have that problem: v_mid = (v_n + v_{n+1})/2 and
// q_mid = q_n composed with Exp(S v_mid dt/2) are exactly the state ABA's
// bias terms should be evaluated at for a time-centered (and, for a
// constant mass matrix, exactly energy-conserving) step.
//
// The cost is that v_{n+1} appears on both sides of that definition (v_mid
// depends on it, and a_mid = a(q_mid, v_mid) determines it), so this is a
// genuine implicit method: `iterations` fixed-point passes, each costing
// one full ABA evaluation, converge v_{n+1} rather than solving for it in
// one shot. This never touches ABAField's own storage format or ABAStep's
// own Ops -- it just Loads/Stores ABA's existing kJointPosition/
// kJointVelocity/kJointAcceleration fields directly (the same public
// per-row access render::SceneRenderer and ComputeSystemEnergy already
// use) around repeated calls to the *existing* ABAStep::Step, the same way
// interface::Simulation's own kVelocityVerlet case calls ABAStep/VIStep/
// PIStep directly rather than through SimContext::Step's single pass.
struct ImplicitMidpointStep {
  template <engine::pass::SimContextLike SimStateT>
  static void Step(
      const SimStateT& sim_state,
      const SimConfig& sim_config,
      float dt,
      int iterations = 4
  ) {
    using ScalarTransform = domain::spatial::Transform<float>;
    using ScalarVelocity = domain::spatial::SpatialVelocity<float>;
    using ScalarAcceleration = domain::spatial::SpatialAcceleration<float>;
    using ScalarSubspace = domain::math::Matrix6x6<float>;

    const auto view = sim_state.template ViewFor<ABAView>();
    const std::size_t n = view.Size();

    // q_n/v_n/subspace are this step's fixed starting point -- untouched
    // for the rest of Step(), even though kJointPosition/kJointVelocity
    // themselves get overwritten every iteration below with the current
    // midpoint trial (ABAStep::Step has to read them from there; it has
    // no other way to receive a joint's state).
    std::vector<ScalarTransform> q_n(n);
    std::vector<ScalarVelocity> v_n(n);
    std::vector<ScalarSubspace> subspace(n);
    std::vector<ScalarVelocity> v_next(n);
    for (std::size_t row = 0; row < n; ++row) {
      q_n[row] = view.template Load<ABAField::kJointPosition, float>(row);
      v_n[row] = view.template Load<ABAField::kJointVelocity, float>(row);
      subspace[row] = view.template Load<ABAField::kJointSubspace, float>(row);
      v_next[row] = v_n[row];  // Initial guess for v_{n+1}.
    }

    for (int iter = 0; iter < iterations; ++iter) {
      for (std::size_t row = 0; row < n; ++row) {
        ScalarVelocity v_mid = (v_n[row] + v_next[row]) * 0.5F;
        ScalarVelocity qd_spatial = subspace[row] * v_mid;
        ScalarTransform q_mid =
            q_n[row] * ScalarTransform::Exp(qd_spatial * (dt * 0.5F));
        view.template Store<ABAField::kJointPosition, float>(row, q_mid);
        view.template Store<ABAField::kJointVelocity, float>(row, v_mid);
      }
      // One full tree evaluation at every joint's own midpoint trial
      // simultaneously -- ABA's recursion needs the whole tree's state at
      // once, not one joint at a time.
      ABAStep::Step(sim_state, sim_config, dt);
      for (std::size_t row = 0; row < n; ++row) {
        ScalarAcceleration a_mid =
            view.template Load<ABAField::kJointAcceleration, float>(row);
        v_next[row] = v_n[row] + a_mid.Integrate(dt);
      }
    }

    // Commit the converged result: q_{n+1} = q_n composed with the FULL-dt
    // exponential of the same converged v_mid (not the half-dt one used to
    // build each trial's q_mid above), v_{n+1} = v_next's final value.
    for (std::size_t row = 0; row < n; ++row) {
      ScalarVelocity v_mid = (v_n[row] + v_next[row]) * 0.5F;
      ScalarVelocity qd_spatial = subspace[row] * v_mid;
      ScalarTransform q_next = q_n[row] * ScalarTransform::Exp(qd_spatial * dt);
      view.template Store<ABAField::kJointPosition, float>(row, q_next);
      view.template Store<ABAField::kJointVelocity, float>(row, v_next[row]);
    }
  }
};

}  // namespace achilles::algorithms::aba
