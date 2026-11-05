#pragma once

#include <cmath>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "algorithms/aba/aba_data.hpp"
#include "algorithms/conventions.hpp"
#include "algorithms/sensing/sensor.hpp"
#include "domain/joint_data.hpp"
#include "domain/math/vector3.hpp"
#include "domain/spatial/dual.hpp"
#include "engine/memory/buffering_policy.hpp"
#include "engine/pass/sim_context.hpp"
#include "engine/topology/orderings.hpp"
#include "util/tmp.hpp"

namespace achilles::interface {

// A direction in the joint's own frame -- the frame its joint_subspace is
// written in. Any nonzero length; Find normalizes it.
using ActuatorAxis = domain::math::Vector3<algorithms::ScalarOperationT>;

// What an actuator pushes along its axis: a force (linear) or a torque
// (angular).
enum class ActuatorKind : std::uint8_t { kLinear, kAngular };

// An active actuator, declared in the .arow like a sensor and found the same
// way (Simulation::Actuator): drives one joint along the axis its file names,
// with one scalar effort per tick. Two actuators on the same joint (e.g. one
// per axis) add, which is how a vector effort is applied.
//
// A handle, not an object the sim owns: it holds only pointers into the
// write half of ABA's double-buffered kJointTorque (see
// engine/memory/buffering_policy.hpp), so it is cheap to copy and the sim
// never iterates actuators. It stays valid until the Simulation that made it
// is destroyed or re-Init()ed.
//
// Apply() accumulates into the *next* tick's torque. Every integrator swaps
// the halves at its start (aba::SwapTorqueStep), so whatever is Apply()ed
// between two Step() calls is exactly what the second one integrates -- and
// then it's gone: the half is zeroed, so an effort meant to persist must be
// Apply()ed again before every Step().
//
// A model of a real actuator (motor constant, gearing, saturation, ...) is
// a class deriving from LinearActuator or AngularActuator, constructed from
// the base handle Simulation::Actuator hands it (see there) and
// calling Apply() with whatever effort its own model produces.
template <ActuatorKind Kind>
class ActuatorHandle {
 public:
  static constexpr ActuatorKind kKind = Kind;
  // The `kind:` an .arow declares one of these under.
  static constexpr std::string_view kListName =
      Kind == ActuatorKind::kAngular ? "angular" : "linear";

  // The direction `actuator` acts along: its `axis` setting, the only one it
  // takes. Throws std::invalid_argument without one, or with any other.
  static ActuatorAxis AxisOf(const domain::JointAttachment& actuator) {
    algorithms::sensing::AttachmentConfig config(
        actuator.name, actuator.config
    );
    const ActuatorAxis axis = config.RequiredVector("axis");
    config.Finish();
    return axis;
  }

  // Checks `attachment` is a well-formed actuator of this kind on instance
  // `instance` of its archetype: declared as an actuator, with an axis its
  // joint can move along. An actuator keeps nothing in the sim, so that's all
  // there is to configure. Throws std::invalid_argument.
  template <engine::pass::SimContextLike SimStateT>
  static void Configure(
      const SimStateT& sim_state,
      const domain::JointAttachment& attachment,
      std::size_t instance,
      std::size_t /*entry*/,
      std::uint64_t /*stream*/
  ) {
    domain::RequireKey(attachment, "actuator");
    const ActuatorAxis axis = AxisOf(attachment);
    const std::size_t row =
        sim_state.template LayoutFor<engine::topology::TopologicalOrdering>()
            .ToSorted(instance, attachment.joint);
    try {
      (void)Find(sim_state, row, axis);
    } catch (const std::invalid_argument& e) {
      throw std::invalid_argument(
          "actuator '" + attachment.name + "': " + e.what()
      );
    }
  }

  // An actuator on the joint at `row` of ABA's Views, along `axis`, any
  // direction in the joint's frame. `sim_state` is the SimContext hosting
  // ABA, looked up the same way a Step looks up its own View. Throws
  // std::invalid_argument for a zero axis, or one the joint has no freedom
  // along at all.
  //
  // kJointTorque holds generalized forces -- one per column of the joint's
  // motion subspace S -- so a spatial effort along unit axis e (e in the
  // linear or angular half, per Kind) enters as S^T (effort * e):
  // effort * sum_i S(i, k) e_i on each coordinate k. An axis only partly
  // within the joint's free directions is allowed: like any force on a
  // constrained body, only its component along them does work. Only
  // coordinates with a non-negligible weight get a term.
  template <engine::pass::SimContextLike SimStateT>
  static ActuatorHandle Find(
      const SimStateT& sim_state, std::size_t row, const ActuatorAxis& axis
  ) {
    using algorithms::ScalarOperationT;
    using algorithms::aba::ABAField;
    const auto view = sim_state.template ViewFor<algorithms::aba::ABAView>();
    if (axis.Norm() <= kNegligible) {
      throw std::invalid_argument("ActuatorHandle: the axis must be nonzero.");
    }
    const ActuatorAxis unit = axis.Normalize();
    const auto subspace =
        view.template Load<ABAField::kJointSubspace, ScalarOperationT>(row);

    const auto weights =
        domain::spatial::AxisWeights(subspace, unit, kFirstSpatialRow);
    std::vector<Term> terms;
    for (std::size_t k = 0; k < weights.size(); ++k) {
      const ScalarOperationT weight = weights[k];
      if (std::abs(weight) > kNegligible) {
        terms.push_back(Term{
            view.template WriteSlot<ABAField::kJointTorque>(row, k), weight
        });
      }
    }
    if (terms.empty()) {
      throw std::invalid_argument(
          "ActuatorHandle: the joint has no degree of freedom along the "
          "requested axis."
      );
    }
    return ActuatorHandle(std::move(terms));
  }

  // Adds `effort` (N for a linear actuator, N*m for an angular one) along
  // this actuator's axis to the next tick's joint torque.
  void Apply(algorithms::ScalarOperationT effort) const {
    for (const Term& term : terms_) {
      term.slot.Add(effort * term.weight);
    }
  }

 private:
  // Which half of S the axis is in (domain/spatial/dual.hpp).
  static constexpr std::size_t kFirstSpatialRow =
      Kind == ActuatorKind::kAngular ? domain::spatial::kAngularRows
                                     : domain::spatial::kLinearRows;
  // Below this an axis counts as zero, and a coordinate's weight as no
  // coupling -- so float noise from an axis meant to be orthogonal to a
  // joint's freedom doesn't leave a near-zero term behind.
  static constexpr algorithms::ScalarOperationT kNegligible = 1e-6F;

  struct Term {
    engine::memory::BufferedScalar<algorithms::ScalarOperationT> slot;
    algorithms::ScalarOperationT weight;
  };

  explicit ActuatorHandle(std::vector<Term> terms) : terms_(std::move(terms)) {}

  std::vector<Term> terms_;
};

using LinearActuator = ActuatorHandle<ActuatorKind::kLinear>;
using AngularActuator = ActuatorHandle<ActuatorKind::kAngular>;

// LinearActuator, AngularActuator, or a class deriving from one of them.
template <typename T>
concept ActuatorLike = requires {
  { T::kKind } -> std::convertible_to<ActuatorKind>;
} && std::derived_from<T, ActuatorHandle<T::kKind>>;

// Every actuator kind, as sensor_kinds.hpp's SensorKinds are for sensors.
using ActuatorKinds = util::TypeList<LinearActuator, AngularActuator>;

}  // namespace achilles::interface
