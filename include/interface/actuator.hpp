#pragma once

#include <cmath>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

#include "algorithms/aba/aba_data.hpp"
#include "algorithms/conventions.hpp"
#include "domain/archetype.hpp"
#include "domain/math/vector3.hpp"
#include "domain/spatial/dual.hpp"
#include "engine/memory/buffering_policy.hpp"
#include "engine/pass/sim_context.hpp"
#include "engine/topology/ordering_policy.hpp"

namespace achilles::interface {

using domain::JointAddress;

// A direction in the joint's own frame -- the frame its joint_subspace is
// written in. Any nonzero length; Attach normalizes it.
using ActuatorAxis = domain::math::Vector3<algorithms::ScalarOperationT>;

// What an actuator pushes along its axis: a force (linear) or a torque
// (angular).
enum class ActuatorKind : std::uint8_t { kLinear, kAngular };

// An active actuator: drives one joint along one fixed axis with one scalar
// effort per tick. Two actuators on the same joint (e.g. one per axis) add,
// which is how a vector effort is applied.
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
// the base handle Simulation::AttachActuator hands it (see there) and
// calling Apply() with whatever effort its own model produces.
template <ActuatorKind Kind>
class JointActuator {
 public:
  static constexpr ActuatorKind kKind = Kind;

  // Adds `effort` (N for a linear actuator, N*m for an angular one) along
  // this actuator's axis to the next tick's joint torque.
  void Apply(algorithms::ScalarOperationT effort) const {
    for (const Term& term : terms_) {
      term.slot.Add(effort * term.weight);
    }
  }

  // An actuator on `joint` along `axis`, any direction in the joint's
  // frame. `sim_state` is the SimContext hosting ABA, looked up the same way
  // a Step looks up its own View and ordering. Throws std::invalid_argument
  // for a zero axis, or one the joint has no freedom along at all.
  //
  // kJointTorque holds generalized forces -- one per column of the joint's
  // motion subspace S -- so a spatial effort along unit axis e (e in the
  // linear or angular half, per Kind) enters as S^T (effort * e):
  // effort * sum_i S(i, k) e_i on each coordinate k. An axis only partly
  // within the joint's free directions is allowed: like any force on a
  // constrained body, only its component along them does work. Only
  // coordinates with a non-negligible weight get a term.
  template <engine::pass::SimContextLike SimStateT>
  static JointActuator Attach(
      const SimStateT& sim_state,
      domain::ArchetypeJointHandle joint,
      const ActuatorAxis& axis
  ) {
    using algorithms::ScalarOperationT;
    using algorithms::aba::ABAField;
    const auto view = sim_state.template ViewFor<algorithms::aba::ABAView>();
    const std::size_t row =
        sim_state.template LayoutFor<engine::topology::TopologicalOrdering>()
            .ToSorted(joint);
    if (axis.Norm() <= kNegligible) {
      throw std::invalid_argument("JointActuator: the axis must be nonzero.");
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
          "JointActuator: the joint has no degree of freedom along the "
          "requested axis."
      );
    }
    return JointActuator(std::move(terms));
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

  explicit JointActuator(std::vector<Term> terms) : terms_(std::move(terms)) {}

  std::vector<Term> terms_;
};

using LinearActuator = JointActuator<ActuatorKind::kLinear>;
using AngularActuator = JointActuator<ActuatorKind::kAngular>;

// LinearActuator, AngularActuator, or a class deriving from one of them.
template <typename T>
concept ActuatorLike = requires {
  { T::kKind } -> std::convertible_to<ActuatorKind>;
} && std::derived_from<T, JointActuator<T::kKind>>;

}  // namespace achilles::interface
