#pragma once

#include <optional>

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
  // Headless by default -- constructing a Simulation never opens a window
  // unless a caller explicitly opts in, so existing/programmatic/test
  // usage (which never calls this) is completely unaffected. When set to
  // false, Step() renders one frame per call (lazily opening a window on
  // its first non-headless Step()) via render::SceneRenderer, walking
  // this sim's own viz::VizAlgorithm state -- see that module's own
  // comment (algorithms/viz/viz_data.hpp) for what gets drawn.
  void SetHeadless(bool headless) { headless_ = headless; }
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
      state_->context.Step(dt, config_);
    } catch (const std::exception& e) {
      return util::Warning("Simulation step failed", e.what());
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
  bool ViewFor(typename AlgorithmT::View* output) const {
    if (!state_.has_value()) {
      return util::Warning(
          "Simulation::ViewFor() called before Simulation::Init() -- "
          "the sim is not yet initialized."
      );
    }
    *output = state_->context.template ViewFor<AlgorithmT>();
    return true;
  }

  // Read-only access to the one JointTopology built for a given ordering
  // policy, once Init() has run -- same precondition and same forwarding
  // shape as ViewFor above. A renderer walks this to find each row's
  // parent (e.g. to draw a bone from a joint to its parent's world
  // position) the same way ABAStep itself does.
  template <engine::topology::LayoutPolicyLike PolicyT>
  bool TopologyFor(domain::JointTopology* output) const {
    if (!state_.has_value()) {
      return util::Warning(
          "Simulation::TopologyFor() called before Simulation::Init() -- "
          "the sim is not yet initialized."
      );
    }
    *output = state_->context.template TopologyFor<PolicyT>();
    return true;
  }

 private:
  algorithms::SimConfig config_;
  std::optional<Allocator::State> state_;
  std::vector<domain::Archetype> archetypes_;
  bool headless_ = true;
  std::optional<render::SceneRenderer> renderer_;
};

}  // namespace achilles::interface