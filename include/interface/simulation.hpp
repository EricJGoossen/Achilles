#pragma once

#include <algorithm>
#include <array>
#include <optional>
#include <string>

#include "algorithms/aba/aba_energy.hpp"
#include "algorithms/aba/aba_implicit_midpoint_step.hpp"
#include "algorithms/aba/aba_step.hpp"
#include "algorithms/pi/pi_step.hpp"
#include "algorithms/registry.hpp"
#include "algorithms/sim_config.hpp"
#include "algorithms/vi/vi_step.hpp"
#include "algorithms/viz/viz_step.hpp"
#include "domain/archetype.hpp"
#include "domain/joint_topology.hpp"
#include "domain/math/vector3.hpp"
#include "engine/memory/sim_allocator.hpp"
#include "engine/topology/layout_policy.hpp"
#include "engine/topology/ordering_policy.hpp"
#include "interface/archetype_loader.hpp"
#include "interface/sim_config_loader.hpp"
#include "render/scene_renderer.hpp"
#include "util/csv_logger.hpp"
#include "util/io.hpp"

namespace achilles::interface {

class Simulation {
  using Allocator =
      engine::memory::SimAllocatorForT<algorithms::RegisteredAlgorithms>;

 public:
  // kSemiImplicitEuler is VI/PI's own scheme as registered (one ABA solve,
  // one VI velocity kick, one PI position update -- see SimContext::Step).
  // kVelocityVerlet instead calls ABAStep/VIStep/PIStep directly, twice
  // each per tick, in the classic velocity-Verlet order: ABA at (q, v) ->
  // half a VI kick -> PI's full position update from that half-kicked
  // velocity -> ABA again at the new position -> the other half VI kick.
  // Nothing about ABAStep/VIStep/PIStep changes for this -- they're
  // ordinary static Step(sim_state, config, dt) functions any caller
  // holding a SimContext can invoke directly, as many times as it likes,
  // in whatever order (see sim_context.hpp's own Step, which is just a
  // fold-expression calling each registered Algorithm's Step once); this
  // just calls them in a different sequence than SimContext::Step's own
  // single pass-through-the-list does. Verlet's own local truncation
  // error is one order better than kSemiImplicitEuler's (O(dt^3) per step
  // vs O(dt^2)) for the *position-dependent* part of the dynamics -- ABA's
  // own Coriolis/gyroscopic terms are velocity-dependent (see SetSubsteps'
  // own comment), and the second ABA call here evaluates them at the
  // half-kicked velocity rather than the not-yet-known true v_{n+1}, so
  // that piece doesn't get the full 2nd-order benefit. Still meaningfully
  // better than kSemiImplicitEuler in practice; not the fully rigorous
  // fix (that needs a time-symmetric scheme like implicit midpoint, or a
  // true discrete-variational integrator).
  //
  // kImplicitMidpoint (algorithms::aba::ImplicitMidpointStep) is that
  // time-symmetric scheme: it evaluates ABA's bias terms at the average of
  // the old and new state (v_mid = (v_n + v_{n+1})/2), which -- unlike
  // Verlet's extrapolated half-kick -- doesn't approximate away the
  // velocity-dependent Coriolis/gyroscopic piece. The cost is that
  // v_{n+1} appears on both sides of that average, so it's a genuine
  // implicit method: several fixed-point ABA evaluations per tick (see
  // ImplicitMidpointStep's own `iterations`, not exposed here yet) rather
  // than kVelocityVerlet's fixed two.
  enum class Integrator {
    kSemiImplicitEuler,
    kVelocityVerlet,
    kImplicitMidpoint
  };

  // kSemiImplicitEuler by default -- existing callers that never call this
  // see no change at all.
  void SetIntegrator(Integrator integrator) { integrator_ = integrator; }

  // Headless by default -- constructing a Simulation never opens a window
  // unless a caller explicitly opts in, so existing/programmatic/test
  // usage (which never calls this) is completely unaffected. When set to
  // false, Step() renders one frame per call (lazily opening a window on
  // its first non-headless Step()) via render::SceneRenderer, walking
  // this sim's own viz::VizAlgorithm state -- see that module's own
  // comment (algorithms/viz/viz_data.hpp) for what gets drawn.
  void SetHeadless(bool headless) { headless_ = headless; }

  // Splits each Step(dt) call into `n` internal ABA+VI+PI evaluations of
  // dt/n each, rather than one evaluation of the full dt (n=1, the
  // default -- so existing callers that never call this see no change at
  // all). This is purely an accuracy knob: VI/PI's own semi-implicit-Euler
  // scheme (see their own comments) only gets its usual bounded-energy-
  // error guarantee for a constant mass matrix and purely position-
  // dependent forces, and ABA's own Coriolis/gyroscopic bias terms
  // (velocity-dependent) break that assumption -- in practice this shows
  // up as real, non-oscillating energy dissipation over tens of seconds,
  // not just the small per-tick error every explicit integrator has (see
  // ComputeSystemEnergy's own comment, and the demo scene's own
  // sim_config.yaml for how this gets used). Substepping doesn't fix the
  // scheme itself, just shrinks its own per-step error the ordinary way:
  // smaller dt.
  void SetSubsteps(int n) { substeps_ = std::max(1, n); }

  bool LoadConfigFile(const std::string& path) {
    try {
      config_ = LoadSimConfig(path);
    } catch (const std::exception& e) {
      return util::Warning("Failed to read config from file", e.what());
    }
    return true;
  }

  bool LoadArowFile(const std::string& path) {
    try {
      archetypes_ = LoadArchetypes(path);
    } catch (const std::exception& e) {
      return util::Warning("Failed to load world from file", e.what());
    }
    return true;
  }

  // Opens `path` as a CSV of (time, kinetic, potential, total) system
  // energy, appended to once per Step() from here on -- see
  // algorithms::aba::ComputeSystemEnergy for what "system energy" means
  // generically across any topology this sim loads. Graph the result with
  // e.g. scripts/plot.py. Returns false (the same shape as Load*) if `path`
  // couldn't be opened for writing.
  bool EnableEnergyLog(const std::string& path) {
    auto logger =
        util::CsvLogger::Open(path, {"time", "kinetic", "potential", "total"});
    if (!logger.has_value()) {
      return util::Warning("Failed to open energy log file", path.c_str());
    }
    energy_log_.emplace(std::move(*logger));
    return true;
  }

  bool Init() {
    if (archetypes_.empty()) {
      return util::Warning(
          "Simulation::Init() called before Simulation::LoadArowFile() -- no "
          "archetypes loaded."
      );
    }
    try {
      state_ = Allocator::Build(archetypes_);
    } catch (const std::exception& e) {
      return util::Warning("Simulation allocation failed", e.what());
    }
    return true;
  }

  // Steps the physics, then -- unless SetHeadless(true) (the default) --
  // renders exactly one frame of the resulting state. Returns false (the
  // same "stop" signal a caller already checks for on any other failure)
  // once the render window has been closed, so a simple `while (sim.
  // Step(dt)) {}` loop is both the headless and the visual shape.
  bool Step(float dt) {
    if (!state_.has_value()) {
      return util::Warning(
          "Simulation::Step() called before Simulation::Init() -- "
          "the sim is not yet initialized."
      );
    }
    try {
      float sub_dt = dt / static_cast<float>(substeps_);
      for (int i = 0; i < substeps_; ++i) {
        StepOnce(sub_dt);
      }
    } catch (const std::exception& e) {
      return util::Warning("Simulation step failed", e.what());
    }

    if (energy_log_.has_value()) {
      // kWorldTransform/kSpatialVelocity (what ComputeSystemEnergy reads)
      // are only ever written by ABA's own forward-kinematics pass, from
      // whichever kJointPosition/kJointVelocity were current *before* this
      // Step() call's VI/PI sub-passes advanced them -- so right after
      // context.Step() returns, they describe the state as of sim_time_
      // *before* it's incremented below, not the just-integrated one dt
      // later (see algorithms::aba::ComputeSystemEnergy's own comment).
      // Logging sim_time_ pre-increment here, rather than post-, keeps
      // this row's own (time, energy) pair internally consistent.
      algorithms::aba::SystemEnergy energy =
          algorithms::aba::ComputeSystemEnergy(
              ViewFor<algorithms::aba::ABAAlgorithm>(),
              TopologyFor<engine::topology::TopologicalOrdering>(),
              GravityVector()
          );
      std::array<float, 4> row{
          sim_time_, energy.kinetic, energy.potential, energy.Total()
      };
      energy_log_->LogRow(row);
      sim_time_ += dt;
    }

    if (!headless_) {
      if (!renderer_.has_value()) {
        renderer_.emplace("Achilles Viewer");
      }
      renderer_->PollInput();
      if (renderer_->ShouldClose()) {
        return false;
      }
      renderer_->RenderFrame(
          ViewFor<algorithms::viz::VizAlgorithm>(),
          TopologyFor<engine::topology::TopologicalOrdering>()
      );
      renderer_->SwapBuffers();
    }
    return true;
  }

  // Read-only access to one hosted Algorithm's own View, once Init() has
  // run -- the same ViewFor<AlgorithmT>() forwarding SimAllocator/SimContext
  // already expose, for a caller (or a test) that needs to inspect state
  // Step() produced. Only ever called after a successful Init(), the same
  // documented precondition Step() itself relies on -- not re-checked
  // here (state_->context, not state_.value()->context) since a runtime
  // guard would just have to invent a return value for the "precondition
  // violated" case this class's whole contract already says never happens.
  template <engine::AlgorithmLike AlgorithmT>
  typename AlgorithmT::View ViewFor() const {
    if (!state_.has_value()) {
      throw std::runtime_error(
          "Simulation::ViewFor() called before Simulation::Init() -- "
          "the sim is not yet initialized."
      );
    }
    return state_->context.template ViewFor<AlgorithmT>();
  }

  // Read-only access to the one JointTopology built for a given ordering
  // policy, once Init() has run -- same precondition and same forwarding
  // shape as ViewFor above. A renderer walks this to find each row's
  // parent (e.g. to draw a bone from a joint to its parent's world
  // position) the same way ABAStep itself does.
  template <engine::topology::LayoutPolicyLike PolicyT>
  domain::JointTopology TopologyFor() const {
    if (!state_.has_value()) {
      throw std::runtime_error(
          "Simulation::TopologyFor() called before Simulation::Init() -- "
          "the sim is not yet initialized."
      );
    }
    return state_->context.template TopologyFor<PolicyT>();
  }

 private:
  // ABA treats a fixed base's own configured acceleration as *minus* true
  // gravity (see SimConfig's own comment on base_acceleration) -- this
  // undoes that negation to recover the real gravitational acceleration
  // vector ComputeSystemEnergy wants. Lane 0 only: base_acceleration is a
  // whole-sim constant, not per-instance data, so every SIMD lane already
  // holds the same value.
  domain::math::Vector3<float> GravityVector() const {
    auto linear = config_.base_acceleration.Linear();
    return {-linear.X().get(0), -linear.Y().get(0), -linear.Z().get(0)};
  }

  // One evaluation of `integrator_` at the given dt -- see Integrator's own
  // comment for what each case does. Called once per substep by Step(),
  // never directly by a caller (substeps_ == 1 makes it identical to
  // calling this once).
  void StepOnce(float dt) {
    switch (integrator_) {
      case Integrator::kSemiImplicitEuler:
        state_->context.Step(dt, config_);
        return;
      case Integrator::kVelocityVerlet:
        algorithms::aba::ABAStep::Step(state_->context, config_, dt);
        algorithms::vi::VIStep::Step(state_->context, config_, dt * 0.5F);
        algorithms::pi::PIStep::Step(state_->context, config_, dt);
        algorithms::aba::ABAStep::Step(state_->context, config_, dt);
        algorithms::vi::VIStep::Step(state_->context, config_, dt * 0.5F);
        return;
      case Integrator::kImplicitMidpoint:
        algorithms::aba::ImplicitMidpointStep::Step(
            state_->context, config_, dt
        );
        return;
    }
  }

  algorithms::SimConfig config_;
  std::optional<Allocator::State> state_;
  std::vector<domain::Archetype> archetypes_;
  bool headless_ = true;
  std::optional<render::SceneRenderer> renderer_;
  std::optional<util::CsvLogger> energy_log_;
  float sim_time_ = 0.0F;
  int substeps_ = 1;
  Integrator integrator_ = Integrator::kSemiImplicitEuler;
};

}  // namespace achilles::interface