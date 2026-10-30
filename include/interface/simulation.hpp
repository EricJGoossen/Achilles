#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>

#include "algorithms/integrators.hpp"
#include "algorithms/registry.hpp"
#include "algorithms/sim_config.hpp"
#include "algorithms/viz/viz_step.hpp"
#include "domain/archetype.hpp"
#include "domain/joint_layout.hpp"
#include "domain/joint_topology.hpp"
#include "engine/memory/sim_allocator.hpp"
#include "engine/topology/layout_policy.hpp"
#include "engine/topology/orderings.hpp"
#include "interface/actuator.hpp"
#include "interface/archetype_loader.hpp"
#include "interface/sensor_handle.hpp"
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

  // Builds the sim from the loaded .arow and runs its Setup list. `seed`
  // keys every sensor's noise -- the same seed gives the same readings, run
  // after run; without one, a fresh seed is drawn (GenerateSeed) and can be
  // read back with Seed() to reproduce the run.
  bool Init(std::optional<std::uint64_t> seed = std::nullopt) {
    if (archetypes_.empty()) {
      return util::Warning(
          "Simulation::Init() called before Simulation::LoadArowFile() -- no "
          "archetypes loaded."
      );
    }
    seed_ = seed.value_or(GenerateSeed());
    try {
      state_ = Allocator::Build(archetypes_);
      WriteSensorConfigs(state_->context, archetypes_, seed_);
    } catch (const std::exception& e) {
      state_.reset();
      return util::Warning("Simulation allocation failed", e.what());
    }
    // Model constants derived from the initial state, and every sensor's
    // starting state (algorithms::Setup).
    return RunStepList(&algorithms::StepIntegrator<algorithms::Setup>, 0.0F);
  }

  // A seed drawn from the system's random source -- what Init() uses when
  // it's given none.
  static std::uint64_t GenerateSeed() {
    std::random_device device;
    return (static_cast<std::uint64_t>(device()) << 32U) | device();
  }

  // The seed the sim was last Init()ed with.
  std::uint64_t Seed() const { return seed_; }

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

  // Runs any step list (a util::TypeList of Steps, see engine::StepLike)
  // once, over the same state Step() advances -- e.g.
  // StepWith<util::TypeList<algorithms::aba::ABAStep>>(0) re-evaluates the
  // dynamics at the current (q, qd) without integrating anything. Same
  // return contract as Step(). Swaps actuator torque only if StepList
  // starts with aba::SwapTorqueStep (an integrator does); otherwise it
  // reads the same torque the last Step() did, and leaves pending Apply()s
  // for the next Step().
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
      renderer_->SetDrawBones(draw_bones_);
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

  // Whether Render() draws the viewer's bone lines, joint to parent (on by
  // default; see render::SceneRenderer::SetDrawBones).
  void SetDrawBones(bool draw) {
    draw_bones_ = draw;
    if (renderer_.has_value()) {
      renderer_->SetDrawBones(draw);
    }
  }

  // Attaches an actuator to `joint`, driving it along `axis` -- any
  // direction in the joint's own frame (see JointActuator::Attach) -- and
  // returns it as T: LinearActuator/AngularActuator itself,
  // or a class deriving from one, built as T(base, args...) where `base` is
  // the attached LinearActuator/AngularActuator -- so a user's actuator
  // model comes back as its own type, no cast. e.g.
  //
  //   class DcMotor : public AngularActuator {
  //    public:
  //     DcMotor(AngularActuator base, float kt)
  //         : AngularActuator(std::move(base)), kt_(kt) {}
  //     void Command(float amps) const { Apply(kt_ * amps); }
  //    private:
  //     float kt_;
  //   };
  //   DcMotor elbow = sim.AttachActuator<DcMotor>({"arm", "elbow"},
  //                                               {1, 0, 0}, 0.05F);
  //
  // Only after a successful Init(), against the archetypes it was built
  // from; the actuator is valid until this Simulation is re-Init()ed or
  // destroyed. Throws std::invalid_argument for an unknown archetype or
  // joint, an out-of-range instance, or an axis the joint can't move along.
  template <ActuatorLike T, typename... Args>
  T AttachActuator(
      const JointAddress& joint, const ActuatorAxis& axis, Args&&... args
  ) const {
    if (!state_.has_value()) {
      throw std::runtime_error(
          "Simulation::AttachActuator() called before Simulation::Init() -- "
          "the sim is not yet initialized."
      );
    }
    return T(
        JointActuator<T::kKind>::Attach(
            state_->context, domain::ResolveJoint(archetypes_, joint), axis
        ),
        std::forward<Args>(args)...
    );
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
  engine::topology::TopologyOfT<PolicyT> TopologyFor() const {
    if (!state_.has_value()) {
      throw std::runtime_error(
          "Simulation::TopologyFor() called before Simulation::Init() -- "
          "the sim is not yet initialized."
      );
    }
    return state_->context.template TopologyFor<PolicyT>();
  }

  // The sensor `address` names (see SensorAddress), as a handle of its kind
  // K -- e.g. FindSensor<algorithms::sensing::TruthOf<aba::ABAAlgorithm,
  // aba::ABAField::kWorldTransform>>({"body_pose", 0}). Throws
  // std::runtime_error before Init(), std::invalid_argument for an unknown
  // sensor or instance, or one that isn't a K.
  template <algorithms::sensing::SensorLike K>
  SensorHandle<K> FindSensor(const SensorAddress& address) const {
    return interface::FindSensor<K>(
        Context("FindSensor"), archetypes_, address
    );
  }

  // The sensor's latest reading, as of the last Step().
  template <algorithms::sensing::SensorLike K>
  algorithms::sensing::Reading<typename K::Output> Read(SensorHandle<K> handle
  ) const {
    return ReadSensor(Context("Read"), handle);
  }

  // The sensor's configuration, and replacing it: the sensor starts over
  // from the new one (see interface::SetSensorConfig).
  template <algorithms::sensing::SensorLike K>
  K SensorConfig(SensorHandle<K> handle) const {
    return interface::SensorConfig(Context("SensorConfig"), handle);
  }
  template <algorithms::sensing::SensorLike K>
  void SetSensorConfig(SensorHandle<K> handle, const K& config) const {
    interface::SetSensorConfig(Context("SetSensorConfig"), handle, config);
  }

  // The row `joint` lives at in every View whose fields PolicyT orders --
  // e.g. RowFor<TopologicalOrdering>() for ABA's -- once Init() has run.
  // Resolved against the archetypes Init() was built from and the
  // JointLayout the context already keeps, so a caller never re-loads the
  // scene or rebuilds an ordering plan to find a joint. Throws
  // std::invalid_argument for an unknown archetype or joint, or an
  // out-of-range instance (see domain::ResolveJoint).
  template <engine::topology::LayoutPolicyLike PolicyT>
  std::size_t RowFor(const JointAddress& joint) const {
    if (!state_.has_value()) {
      throw std::runtime_error(
          "Simulation::RowFor() called before Simulation::Init() -- "
          "the sim is not yet initialized."
      );
    }
    return state_->context.template LayoutFor<PolicyT>().ToSorted(
        domain::ResolveJoint(archetypes_, joint)
    );
  }

 private:
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
  std::uint64_t seed_ = 0;
  std::optional<render::SceneRenderer> renderer_;
  bool draw_bones_ = true;
  algorithms::IntegratorStepFn integrator_step_ =
      algorithms::StepFnFor(Integrator::kSemiImplicitEuler);
};

}  // namespace achilles::interface