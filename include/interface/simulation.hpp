#pragma once

#include <algorithm>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "algorithms/aba/aba_implicit_midpoint_step.hpp"
#include "algorithms/aba/aba_implicit_velocity_step.hpp"
#include "algorithms/aba/aba_runge_kutta_step.hpp"
#include "algorithms/aba/aba_step.hpp"
#include "algorithms/pi/pi_step.hpp"
#include "algorithms/registry.hpp"
#include "algorithms/sim_config.hpp"
#include "algorithms/vi/vi_step.hpp"
#include "algorithms/viz/viz_step.hpp"
#include "domain/archetype.hpp"
#include "domain/joint_topology.hpp"
#include "engine/field_blocks.hpp"
#include "engine/memory/sim_allocator.hpp"
#include "engine/topology/layout_policy.hpp"
#include "engine/topology/ordering_policy.hpp"
#include "interface/archetype_loader.hpp"
#include "interface/sim_config_loader.hpp"
#include "interface/sim_log.hpp"
#include "render/scene_renderer.hpp"
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
  //
  // kImplicitVelocity (algorithms::aba::ImplicitVelocityStep) is a cheaper
  // alternative to kImplicitMidpoint's fixed-point iteration, modeled on
  // MuJoCo's own implicit-in-velocity integrator (see
  // ImplicitVelocityStep's own header comment for the derivation and,
  // importantly, the one real simplification it makes relative to
  // MuJoCo's actual scheme): instead of iterating ABA to convergence, it
  // runs ABA *once*, analytically linearizes its own velocity-dependent
  // bias terms around the resulting state, and solves one small implicit
  // correction per joint from that linearization. Cost is one ABA
  // evaluation plus a closed-form correction (no iteration) -- cheaper
  // than kImplicitMidpoint's several ABA evaluations per tick -- at the
  // cost of only capturing each joint's *own* velocity feedback, not the
  // cross-joint coupling a true (globally-coupled) implicit solve would.
  // kRungeKutta4 and kImplicitFast complete the set of integrators MuJoCo
  // itself ships (Euler / RK4 / implicit / implicitfast -- see their XML
  // reference's own option/integrator attribute), so this engine can be
  // benchmarked against the same menu a production simulator offers.
  //
  // kRungeKutta4 (algorithms::aba::RungeKutta4Step) is classical RK4: four
  // ABA evaluations per tick, same per-tick cost as kImplicitMidpoint, but
  // fourth-order accurate rather than second. MuJoCo points at RK4 for
  // energy-conserving systems specifically.
  //
  // kImplicitFast is MuJoCo's own recommended default, and for *this*
  // engine it is deliberately identical to kSemiImplicitEuler. That is not
  // a stub: implicitfast's whole content is integrating the *non*-RNE
  // velocity-dependent forces implicitly -- joint damping, actuator and
  // fluid-drag terms -- while explicitly dropping the Coriolis/centripetal
  // derivatives that kImplicitVelocity keeps. This engine models none of
  // those forces (no damping coefficient, no actuators, no fluid), so
  // implicitfast's D matrix is identically zero here and the scheme
  // collapses to semi-implicit Euler exactly. It is named separately
  // anyway so a benchmark across "every MuJoCo integrator" can report it
  // honestly rather than quietly omitting it -- and so that the day joint
  // damping is added, there is an obvious place for it to become real.
  enum class Integrator {
    kSemiImplicitEuler,
    kVelocityVerlet,
    kImplicitMidpoint,
    kImplicitVelocity,
    kRungeKutta4,
    kImplicitFast
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
  // the demo scene's own sim_config.yaml for how this gets used). Substepping
  // doesn't fix the scheme itself, just shrinks its own per-step error the
  // ordinary way: smaller dt.
  void SetSubsteps(int n) { substeps_ = std::max(1, n); }

  bool LoadConfigFile(const std::string& path) {
    try {
      config_ = LoadSimConfig(path);
    } catch (const std::exception& e) {
      return util::Warning("Failed to read config from file", e.what());
    }
    return true;
  }

  // Parses `path` (see interface/archetype_loader.hpp), then accepts the
  // result only if every field it supplies is one an archetype defines
  // (domain::kArchetypeFields) with the right number of values per joint
  // -- see CheckArchetypeFields.
  bool LoadArowFile(const std::string& path) {
    std::vector<domain::Archetype> archetypes;
    try {
      archetypes = LoadArchetypes(path);
    } catch (const std::exception& e) {
      return util::Warning("Failed to load world from file", e.what());
    }
    if (std::optional<std::string> problem = CheckArchetypeFields(archetypes)) {
      return util::Warning("Failed to load world from file", problem->c_str());
    }
    archetypes_ = std::move(archetypes);
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

  // Starts logging `fields` to a CSV at `path`: a "time" column, then one
  // column per selected field x real joint x scalar, named
  // "<archetype>[<copy>].<joint>.<field>.<k>" (the "[<copy>]" is omitted
  // for single-copy archetypes). Fields are global log names (e.g.
  // "joint_position", "world_transform") or "*" for all -- see engine::pass::
  // FieldLog. Must be called after Init(). Writes the current state as the
  // first row (time 0), then one row after every Step().
  //
  // Returns false (with a warning naming the problem) for an unknown field
  // name, an empty selection, an unwritable path, or a build with
  // ACHILLES_ENABLE_LOGGING=OFF, where logging is compiled out entirely.
  bool EnableLog(
      const std::string& path, const std::vector<std::string>& fields
  ) {
    if (!state_.has_value()) {
      return util::Warning(
          "Simulation::EnableLog() called before Simulation::Init()"
      );
    }
    return log_.Enable(state_->context, archetypes_, path, fields, [this] {
      RefreshAccelerationForCurrentState();
    });
  }

  // Steps the physics, then -- unless SetHeadless(true) (the default) --
  // renders exactly one frame of the resulting state. Returns false (the
  // same "stop" signal a caller already checks for on any other failure)
  // once the render window has been closed, so a simple `while (sim.
  // Step(dt)) {}` loop is both the headless and the visual shape.
  bool Step(algorithms::ScalarOperationT dt) {
    if (!state_.has_value()) {
      return util::Warning(
          "Simulation::Step() called before Simulation::Init() -- "
          "the sim is not yet initialized."
      );
    }
    try {
      algorithms::ScalarOperationT sub_dt =
          dt / static_cast<algorithms::ScalarOperationT>(substeps_);
      for (int i = 0; i < substeps_; ++i) {
        StepOnce(sub_dt);
      }
    } catch (const std::exception& e) {
      return util::Warning("Simulation step failed", e.what());
    }

    log_.AfterStep(state_->context, dt, [this] {
      RefreshAccelerationForCurrentState();
    });

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

  // Re-runs ABA's own forward-dynamics pass at whatever (q, qd) this
  // Simulation currently holds, without disturbing anything else -- dt
  // doesn't matter to ABA itself (it solves for instantaneous
  // acceleration, never integrates -- see ABAStep::Step's own comment on
  // why it ignores its own dt argument).
  //
  // Exists because kJointAcceleration/kWorldTransform/kSpatialVelocity
  // after a Step() call do *not* uniformly mean "the acceleration/pose/
  // velocity at whatever state kJointPosition/kJointVelocity now report".
  // That equivalence only holds for kSemiImplicitEuler, where ABA runs
  // exactly once, before VI/PI advance state -- for every other
  // Integrator, ABA runs more than once per tick at different intermediate
  // states (see StepOnce below), and whichever call happened last leaves
  // those fields describing *that* intermediate state, not the final
  // committed one. RungeKutta4Step's own last stage is the clearest
  // example: it evaluates ABA at q0 advanced by stage 3's own estimate,
  // which is a real but different state than the blended q_{n+1} the tick
  // actually commits to.
  //
  // A caller that only ever wants "qdd/x_world/v for the current
  // committed state, whichever Integrator produced it" -- diagnostics, or
  // a test cross-checking ABA's own formula against an independent model
  // -- should call this once after Step() and before reading those three
  // fields, rather than assume they're already fresh.
  void RefreshAccelerationForCurrentState() {
    if (!state_.has_value()) {
      throw std::runtime_error(
          "Simulation::RefreshAccelerationForCurrentState() called before "
          "Simulation::Init() -- the sim is not yet initialized."
      );
    }
    algorithms::aba::ABAStep::Step(
        state_->context, config_, algorithms::ScalarOperationT{0}
    );
  }

 private:
  // Every distinct field (storage block) across the registered
  // Algorithms, with its name and scalars per row -- what an archetype
  // field's name and length are checked against.
  static constexpr auto kFields = engine::BlockInfos<
      engine::FieldBlocksIn<algorithms::RegisteredAlgorithms>::Type>::kValue;
  static_assert(
      std::ranges::all_of(
          domain::kArchetypeFields,
          [](std::string_view name) {
            return engine::FindBlock(kFields, name).has_value();
          }
      ),
      "domain::kArchetypeFields names a field no registered Algorithm "
      "declares (as Traits<F>::kName)."
  );

  // Why `archetypes` can't be handed to the sim, or nullopt if it can:
  // a field the archetype doesn't define (domain::kArchetypeFields), or
  // one whose per-joint value count doesn't match the field's Assembler.
  static std::optional<std::string> CheckArchetypeFields(
      const std::vector<domain::Archetype>& archetypes
  ) {
    for (const domain::Archetype& archetype : archetypes) {
      for (const domain::ArchetypeField& field : archetype.Fields()) {
        std::string where =
            "archetype '" + archetype.Name() + "', field '" + field.name + "'";
        if (!domain::IsArchetypeField(field.name)) {
          std::string expected;
          for (std::string_view name : domain::kArchetypeFields) {
            if (!expected.empty()) {
              expected += ", ";
            }
            expected += name;
          }
          where += ": not an archetype field (expected one of: ";
          where += expected;
          where += ")";
          return where;
        }
        std::optional<engine::BlockInfo> described =
            engine::FindBlock(kFields, field.name);
        if (described.has_value() &&
            described->scalars != field.scalars_per_leaf) {
          where += ": needs ";
          where += std::to_string(described->scalars);
          where += " values per joint, got ";
          where += std::to_string(field.scalars_per_leaf);
          return where;
        }
      }
    }
    return std::nullopt;
  }

  // One evaluation of `integrator_` at the given dt -- see Integrator's own
  // comment for what each case does. Called once per substep by Step(),
  // never directly by a caller (substeps_ == 1 makes it identical to
  // calling this once).
  void StepOnce(algorithms::ScalarOperationT dt) {
    switch (integrator_) {
      case Integrator::kSemiImplicitEuler:
        state_->context.Step(dt, config_);
        return;
      case Integrator::kVelocityVerlet:
        algorithms::aba::ABAStep::Step(state_->context, config_, dt);
        algorithms::vi::VIStep::Step(
            state_->context, config_, dt * algorithms::ScalarOperationT{0.5}
        );
        algorithms::pi::PIStep::Step(state_->context, config_, dt);
        algorithms::aba::ABAStep::Step(state_->context, config_, dt);
        algorithms::vi::VIStep::Step(
            state_->context, config_, dt * algorithms::ScalarOperationT{0.5}
        );
        return;
      case Integrator::kImplicitMidpoint:
        algorithms::aba::ImplicitMidpointStep::Step(
            state_->context, config_, dt
        );
        return;
      case Integrator::kImplicitVelocity:
        algorithms::aba::ImplicitVelocityStep::Step(
            state_->context, config_, dt
        );
        return;
      case Integrator::kRungeKutta4:
        algorithms::aba::RungeKutta4Step::Step(state_->context, config_, dt);
        return;
      case Integrator::kImplicitFast:
        // Identical to kSemiImplicitEuler for this engine's force model --
        // see Integrator's own comment for why that is the correct
        // behavior rather than a missing implementation.
        state_->context.Step(dt, config_);
        return;
    }
  }

  algorithms::SimConfig config_;
  std::optional<Allocator::State> state_;
  std::vector<domain::Archetype> archetypes_;
  bool headless_ = true;
  std::optional<render::SceneRenderer> renderer_;
  int substeps_ = 1;
  Integrator integrator_ = Integrator::kSemiImplicitEuler;
  // An empty no-op when ACHILLES_ENABLE_LOGGING=OFF (see SimLog).
  SimLog<decltype(Allocator::State::context)> log_;
};

}  // namespace achilles::interface