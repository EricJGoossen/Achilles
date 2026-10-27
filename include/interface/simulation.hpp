#pragma once

#include <optional>
#include <string>

#include "algorithms/integrators.hpp"
#include "algorithms/registry.hpp"
#include "algorithms/sim_config.hpp"
#include "algorithms/viz/viz_step.hpp"
#include "domain/archetype.hpp"
#include "domain/joint_topology.hpp"
#include "engine/memory/sim_allocator.hpp"
#include "engine/topology/layout_policy.hpp"
#include "engine/topology/ordering_policy.hpp"
#include "interface/archetype_loader.hpp"
#include "interface/sim_config_loader.hpp"
#include "render/scene_renderer.hpp"
#include "util/io.hpp"

namespace achilles::interface {

class Simulation {
  using Allocator =
      engine::memory::SimAllocatorForT<algorithms::RegisteredAlgorithms>;

 public:
  // See algorithms/integrators.hpp for what each one is.
  using Integrator = algorithms::Integrator;

  // kSemiImplicitEuler by default. Resolves `integrator` to its tick once,
  // here, so Step() itself never branches on which integrator is selected.
  // Valid before or after Init().
  void SetIntegrator(Integrator integrator) {
    integrator_step_ = algorithms::StepFnFor(integrator);
  }

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
    // Model constants derived from the initial state (algorithms::Setup).
    return RunStepList(&algorithms::StepIntegrator<algorithms::Setup>, 0.0F);
  }

  // Advances the physics by one tick of the selected integrator (see
  // SetIntegrator) over `dt`. Never renders -- see Render(). Returns false
  // (after warning) if the sim isn't initialized or the step throws.
  bool Step(algorithms::ScalarOperationT dt) {
    return RunStepList(integrator_step_, dt);
  }

  // Runs any step list (a util::TypeList of Steps, see engine::StepLike)
  // once, over the same state Step() advances -- e.g.
  // StepWith<util::TypeList<algorithms::aba::ABAStep>>(0) re-evaluates the
  // dynamics at the current (q, qd) without integrating anything. Same
  // return contract as Step().
  template <typename StepList>
  bool StepWith(algorithms::ScalarOperationT dt) {
    return RunStepList(&algorithms::StepIntegrator<StepList>, dt);
  }

  // Draws one frame of the current state through render::SceneRenderer,
  // opening the window on the first call -- walking this sim's own
  // viz::VizAlgorithm state (see algorithms/viz/viz_data.hpp for what gets
  // drawn). Separate from Step() so a caller decides how many ticks run
  // per frame; a caller that never calls this never opens a window.
  // Returns false once the window has been closed (or if the sim isn't
  // initialized).
  bool Render() {
    if (!state_.has_value()) {
      return util::Warning(
          "Simulation::Render() called before Simulation::Init() -- "
          "the sim is not yet initialized."
      );
    }
    if (!renderer_.has_value()) {
      renderer_.emplace("Achilles Viewer");
    }
    renderer_->PollInput();
    if (renderer_->ShouldClose()) {
      return false;
    }
    renderer_->RenderFrame(
        state_->context.template ViewFor<algorithms::viz::VizAlgorithm>(),
        state_->context
            .template TopologyFor<engine::topology::TopologicalOrdering>()
    );
    renderer_->SwapBuffers();
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
  // Runs one pass of a step list (`step`, an IntegratorStepFn) over this
  // sim's state: a tick (Step/StepWith) or the one-time Setup list (Init).
  bool RunStepList(
      algorithms::IntegratorStepFn step, algorithms::ScalarOperationT dt
  ) {
    if (!state_.has_value()) {
      return util::Warning(
          "Simulation step list run before Simulation::Init() -- "
          "the sim is not yet initialized."
      );
    }
    try {
      step(state_->context, config_, dt);
    } catch (const std::exception& e) {
      return util::Warning("Simulation step failed", e.what());
    }
    return true;
  }

  algorithms::SimConfig config_;
  std::optional<Allocator::State> state_;
  std::vector<domain::Archetype> archetypes_;
  std::optional<render::SceneRenderer> renderer_;
  algorithms::IntegratorStepFn integrator_step_ =
      algorithms::StepFnFor(Integrator::kSemiImplicitEuler);
};

}  // namespace achilles::interface