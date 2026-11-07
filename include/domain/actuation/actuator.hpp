#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string_view>

#include "domain/attachment/attachment.hpp"
#include "domain/attachment/attachment_config.hpp"
#include "domain/attachment/sim_quantities.hpp"
#include "domain/math/vector3.hpp"
#include "domain/sim_clock.hpp"
#include "domain/spatial/dual.hpp"
#include "util/tmp.hpp"

namespace achilles::domain::actuation {

// What a kind is written in.
using attachment::AttachmentConfig;
using attachment::AttachmentOptions;
using attachment::AttachmentTick;
using attachment::NoState;
using attachment::SimInputs;
namespace input = attachment::input;

// What an actuator pushes along its axis: a force (linear) or a torque
// (angular).
enum class EffortKind : std::uint8_t { kLinear, kAngular };

// Which half of a motion subspace an effort kind acts in (spatial/dual.hpp).
constexpr std::size_t FirstSpatialRow(EffortKind kind) {
  return kind == EffortKind::kAngular ? spatial::kAngularRows
                                      : spatial::kLinearRows;
}

// Below this an axis counts as zero, and a coordinate's weight as no coupling
// -- so float noise from an axis meant to be orthogonal to a joint's freedom
// doesn't leave a near-zero term behind.
inline constexpr float kNegligibleWeight = 1e-6F;

// The base every actuator kind derives from, the mirror of Sensor, declaring
// what it needs:
//   - Inputs: SimInputs<...> -- quantities of the sim (input::JointVelocity
//     and so on), read in place at its joint;
//   - StateT: persistent per-instance state, trivially copyable; NoState if
//     none;
//   - CommandT: what a caller sends it;
//   - OutputT: what one update is. Has a float `effort`, the effort the joint
//     gets, and whatever else the kind reports (a motor's current).
// The derived class itself is the instance's configuration: it's stored,
// whole, in the sim's own memory, so it must be trivially copyable and
// default-constructible. It provides
//
//   static constexpr std::string_view kListName = "...";  // its .arow name
//   static constexpr EffortKind kEffort = ...;            // what it pushes
//   static Self FromConfig(AttachmentConfig& config);     // its settings
//   void Drive(const AttachmentTick&, const Input&..., CommandT*, StateT*,
//              OutputT*) const;
//   void Initialize(const AttachmentTick&, StateT*) const;  // optional
//
// Drive runs before anything integrates, only when an update is due (see
// Attachment::Due): it works out the effort, applies its kind's own error from
// the shared effects (domain/error), and writes it to its output, as a sensor's
// Read does a reading. The command is read/write: a kind whose commands are
// used up (a one-tick effort) clears it. Putting the effort on the joint along
// the axis is every actuator's (AddEffort). No virtuals: an actuator is only
// ever called through its own concrete type.
template <
    typename Derived,
    typename Inputs,
    typename StateT,
    typename CommandT,
    typename OutputT>
class Actuator;

template <
    typename Derived,
    typename... Inputs,
    typename StateT,
    typename CommandT,
    typename OutputT>
class Actuator<
    Derived,
    attachment::SimInputs<Inputs...>,
    StateT,
    CommandT,
    OutputT> : public attachment::Attachment {
 public:
  using DerivedType = Derived;
  using ActuatorBase = Actuator;
  using InputList = util::TypeList<Inputs...>;
  using State = StateT;
  using Command = CommandT;
  using Output = OutputT;
  // The key an .arow declares one under.
  static constexpr std::string_view kKey = "actuator";

  Actuator() = default;
  Actuator(
      attachment::AttachmentOptions options, const math::Vector3<float>& axis
  )
      : Attachment(options), axis_(axis) {}

  // The direction it pushes along, in the joint's own frame. Any nonzero
  // length in the .arow; unit once configured (NormalizeAxis).
  const math::Vector3<float>& Axis() const { return axis_; }
  void NormalizeAxis() { axis_ = axis_.Normalize(); }

  // Throws std::invalid_argument for a zero axis, or one a joint with motion
  // subspace `subspace` has no freedom along: an actuator that would silently
  // do nothing. An axis only partly within the joint's free directions is
  // allowed: like any force on a constrained body, only its component along
  // them does work.
  template <typename Subspace>
  void CheckAxis(const Subspace& subspace) const {
    if (axis_.Norm() <= kNegligibleWeight) {
      throw std::invalid_argument("the axis must be nonzero.");
    }
    const auto weights = spatial::AxisWeights(
        subspace, axis_.Normalize(), FirstSpatialRow(Derived::kEffort)
    );
    for (const float weight : weights) {
      if (std::abs(weight) > kNegligibleWeight) {
        return;
      }
    }
    throw std::invalid_argument(
        "the joint has no degree of freedom along the requested axis."
    );
  }

  // One tick: drives when an update is due, else carries `last` forward.
  void Update(
      const SimClock& clock,
      float dt,
      const typename Inputs::Value&... inputs,
      const attachment::Reading<OutputT>& last,
      CommandT* command,
      StateT* state,
      attachment::Reading<OutputT>* out
  ) const {
    UpdateIfDue(clock, dt, last, out, [&](const auto& tick, OutputT* value) {
      static_cast<const Derived&>(*this).Drive(
          tick, inputs..., command, state, value
      );
    });
  }

  // Adds `effort` along the axis to a joint's generalized forces, one per
  // column of its motion subspace S: an effort along unit axis e enters as
  // S^T (effort * e). Only coordinates with a non-negligible weight get a term.
  template <typename Subspace, typename Force>
  void AddEffort(const Subspace& subspace, float effort, Force* torque) const {
    auto weights = spatial::AxisWeights(
        subspace, axis_, FirstSpatialRow(Derived::kEffort)
    );
    for (float& weight : weights) {
      weight = std::abs(weight) > kNegligibleWeight ? weight * effort : 0.0F;
    }
    *torque += Force(
        weights[0], weights[1], weights[2], weights[3], weights[4], weights[5]
    );
  }

  // The joint's rate along the axis, from its generalized velocity mapped
  // through S: rad/s for a torque, m/s for a force.
  template <typename Subspace, typename Velocity>
  float Rate(const Subspace& subspace, const Velocity& qd) const {
    const auto rates = subspace * qd;
    if constexpr (Derived::kEffort == EffortKind::kAngular) {
      return rates.Angular().Dot(axis_);
    } else {
      return rates.Linear().Dot(axis_);
    }
  }

 private:
  math::Vector3<float> axis_ = math::Vector3<float>::UnitX();
};

}  // namespace achilles::domain::actuation
