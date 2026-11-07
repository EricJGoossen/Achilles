#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "algorithms/integrators.hpp"
#include "algorithms/registry.hpp"
#include "algorithms/sim_config.hpp"
#include "algorithms/viz/viz_step.hpp"
#include "domain/archetype/archetype_set.hpp"
#include "domain/joint_layout.hpp"
#include "domain/joint_topology.hpp"
#include "domain/random/seed.hpp"
#include "domain/sim_clock.hpp"
#include "engine/memory/sim_allocator.hpp"
#include "engine/topology/layout_policy.hpp"
#include "engine/topology/orderings.hpp"
#include "interface/archetype_loader.hpp"
#include "interface/attachment_handle.hpp"
#include "interface/attachments.hpp"
#include "interface/sim_config_loader.hpp"
#include "render/scene_renderer.hpp"
#include "util/io.hpp"

namespace achilles::interface {

using domain::JointAddress;

// The truth sensors, by what they read, and truth actuators, by what they push
// with: sim.Sensor<truth::WorldTransform>(...),
// sim.Actuator<truth::Torque>(...).
namespace truth {
using namespace domain::sensing::truth;
using namespace domain::actuation::truth;
}  // namespace truth

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
      archetypes_ = domain::archetype::ArchetypeSet(LoadArchetypes(path));
    } catch (const std::exception& e) {
      return util::Warning("Failed to load world from file", e.what());
    }
    return true;
  }

  // Builds the sim from the loaded .arow and runs its Setup list. `seed`
  // keys every sensor's noise -- the same seed gives the same readings, run
  // after run; without one, a fresh seed is drawn (GenerateSeed) and can be
  // read back with Seed() to reproduce the run.
  bool Init(std::optional<std::uint64_t> seed = std::nullopt) {
    if (archetypes_.All().empty()) {
      return util::Warning(
          "Simulation::Init() called before Simulation::LoadArowFile() -- no "
          "archetypes loaded."
      );
    }
    seed_ = seed.value_or(GenerateSeed());
    try {
      state_ = Allocator::Build(archetypes_);
      ConfigureAttachments(state_->context, archetypes_, seed_);
    } catch (const std::exception& e) {
      state_.reset();
      return util::Warning("Simulation allocation failed", e.what());
    }
    // Model constants derived from the initial state, and every sensor's
    // and actuator's starting state (algorithms::Setup).
    return RunStepList(&algorithms::StepIntegrator<algorithms::Setup>, 0.0F);
  }

  // A seed drawn from the system's random source -- what Init() uses when
  // it's given none.
  static std::uint64_t GenerateSeed() { return domain::random::GenerateSeed(); }

  // The seed the sim was last Init()ed with.
  std::uint64_t Seed() const { return seed_; }

  // Sim time, in seconds since Init(): the sum of every dt a successful
  // Step() advanced over (domain::SimClock). 0 before Init().
  double Time() const { return Clock().time; }

  // How many Step()s have succeeded since Init().
  std::uint64_t StepCount() const { return Clock().tick; }

  // Advances the physics by one tick of the selected integrator (see
  // SetIntegrator) over `dt`. Never renders -- see Render(). Returns false
  // (after warning) if the sim isn't initialized or the step throws.
  //
  // Every integrator starts by swapping actuator torque
  // (aba::SwapTorqueStep), so the tick reads exactly what actuators
  // Apply()ed since the previous Step() -- however many times its step list
  // evaluates ABA.
  bool Step(algorithms::ScalarOperationT dt) {
    return RunStepList(integrator_step_, dt);
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

  // The built sim itself, and the archetypes it was built from -- for a
  // caller that really means to go around the handles (and a test checking
  // what they're built on). Nothing here keeps a write consistent with the
  // rest of the sim, and rows and fields can change: reads go through
  // Sensor() and effort through Actuator() everywhere else. Context()
  // throws std::runtime_error before Init().
  const algorithms::RegisteredSimContext& Context() const {
    return Context("Context");
  }
  const domain::archetype::ArchetypeSet& Archetypes() const {
    return archetypes_;
  }

  // Instance `instance` of the sensor named `name`, as a handle of its kind
  // K -- e.g. Sensor<truth::WorldTransform>("body_pose"). Instance i is the
  // one mounted on instance i of the archetype it's attached to. Throws
  // std::runtime_error before Init(), std::invalid_argument for an unknown
  // sensor or instance, or one that isn't a K.
  template <domain::attachment::SensorLike K>
  SensorHandle<K> Sensor(std::string_view name, std::size_t instance = 0)
      const {
    const auto& context = Context("Sensor");
    return SensorHandle<K>::Find(
        context, archetypes_.AttachmentEntry(name, instance, K::kListName)
    );
  }

  // Instance `instance` of the actuator named `name`, as a handle of its kind
  // K -- e.g. Actuator<truth::Torque>("elbow_motor"). Instance i is the one on
  // instance i of the archetype it's attached to. Throws std::runtime_error
  // before Init(), std::invalid_argument for an unknown actuator or instance,
  // or one that isn't a K.
  template <domain::attachment::ActuatorLike K>
  ActuatorHandle<K> Actuator(std::string_view name, std::size_t instance = 0)
      const {
    const auto& context = Context("Actuator");
    return ActuatorHandle<K>::Find(
        context, archetypes_.AttachmentEntry(name, instance, K::kListName)
    );
  }

 private:
  // The sim's clock; a stopped one before Init().
  domain::SimClock Clock() const {
    using algorithms::clock::ClockField;
    using algorithms::clock::ClockView;
    if (!state_.has_value()) {
      return {};
    }
    return state_->context.template ViewFor<ClockView>()
        .template Load<ClockField::kClock, domain::SimClock>(0);
  }

  // The built sim's context, for a call that needs one; throws before
  // Init().
  const algorithms::RegisteredSimContext& Context(const char* caller) const {
    if (!state_.has_value()) {
      throw std::runtime_error(
          std::string("Simulation::") + caller +
          "() called before Simulation::Init() -- the sim is not yet "
          "initialized."
      );
    }
    return state_->context;
  }

  // Runs one pass of a step list (`step`, an IntegratorStepFn) over this
  // sim's state: a tick (Step) or the one-time Setup list (Init).
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
  domain::archetype::ArchetypeSet archetypes_;
  std::uint64_t seed_ = 0;
  std::optional<render::SceneRenderer> renderer_;
  algorithms::IntegratorStepFn integrator_step_ =
      algorithms::StepFnFor(Integrator::kSemiImplicitEuler);
};

}  // namespace achilles::interface