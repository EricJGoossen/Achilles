#pragma once

#include <type_traits>

#include "algorithms/aba/aba_step.hpp"
#include "algorithms/attachment_kinds/sim_input.hpp"
#include "algorithms/clock/clock_step.hpp"
#include "domain/actuation/dc_motor.hpp"
#include "domain/actuation/truth_actuator.hpp"
#include "domain/attachment/attachment.hpp"
#include "domain/attachment/attachment_kind.hpp"
#include "domain/sensing/accelerometer.hpp"
#include "domain/sensing/camera.hpp"
#include "domain/sensing/encoder.hpp"
#include "domain/sensing/gyroscope.hpp"
#include "domain/sensing/lidar.hpp"
#include "domain/sensing/magnetometer.hpp"
#include "domain/sensing/truth_sensor.hpp"
#include "engine/topology/orderings.hpp"
#include "util/tmp.hpp"

// The kinds the sim hosts, and how each is stored and bound to the sim: what
// the attachment algorithm is built from.
namespace achilles::algorithms::attachment {

using domain::attachment::ActuatorLike;
using domain::attachment::AttachmentLike;
using domain::attachment::AttachmentTick;
using domain::attachment::Reading;
using domain::attachment::SensorHasScene;
using domain::attachment::SensorLike;

// One of kind S's records, as stored in the attachment algorithm's segmented
// fields (attachment_data.hpp): R itself, tagged with its kind, so two kinds
// whose records share a type (two that keep NoState, say) still name different
// segments. Same size and bytes as R.
template <typename S, typename R>
struct KindRecord {
  R value{};
};

// A kind's records as the attachment algorithm stores them, and the fields it
// reads -- collected below across every registered kind.
template <typename S>
using KindState = KindRecord<S, typename S::State>;
template <typename S>
using KindCommand = KindRecord<S, typename S::Command>;
template <typename S>
using KindOutput = KindRecord<S, Reading<typename S::Output>>;
template <typename S>
using KindInputs = util::TransformT<BoundInputT, typename S::InputList>;

// A kind's scene inputs: none unless it scans.
template <typename S>
struct KindSceneOf {
  using Type = util::TypeList<>;
};
template <SensorHasScene S>
struct KindSceneOf<S> {
  using Type = util::TransformT<BoundInputT, typename S::Scene::List>;
};
template <typename S>
using KindScene = typename KindSceneOf<S>::Type;

// Whether an input can be read in place at a joint: shared, and ordered by
// joint -- as an own input always is (its field, attachment_data.hpp).
template <typename Input>
inline constexpr bool kInputOnJoints =
    InputSource<Input>::kShared && std::is_same_v<
                                       typename InputSource<Input>::Ordering,
                                       engine::topology::TopologicalOrdering>;
template <typename Quantity>
inline constexpr bool kInputOnJoints<OwnInput<Quantity>> = true;

// Whether every input of a list can.
template <typename List>
inline constexpr bool kOnJoints = false;
template <typename... Inputs>
inline constexpr bool kOnJoints<util::TypeList<Inputs...>> =
    (kInputOnJoints<Inputs> && ...);

// A list of inputs less the own ones: those read another algorithm's field.
template <typename List>
struct ForeignInputs;
template <typename... Inputs>
struct ForeignInputs<util::TypeList<Inputs...>> {
  using Type = util::ConcatT<
      util::TypeList<>,
      std::conditional_t<
          kIsOwnInput<Inputs>,
          util::TypeList<>,
          util::TypeList<Inputs>>...>;
};
template <typename List>
using ForeignInputsT = typename ForeignInputs<List>::Type;

// The concepts as type traits, for engine::pass::SegmentOps' filter.
template <typename S>
struct ScansScene : std::bool_constant<SensorHasScene<S>> {};
template <typename S>
struct IsSensor : std::bool_constant<SensorLike<S>> {};
template <typename S>
struct IsActuator : std::bool_constant<ActuatorLike<S>> {};

// Every sensor kind the sim hosts, registered once here. A kind is a type; what
// varies between instances of one kind (where it's mounted, its error model,
// its rate) is data, declared in the .arow as attachments under the kind's
// kListName. A kind with no instances costs one empty loop per pass, nothing
// more.
//
// The modelled sensors -- a joint encoder, an IMU's three parts, a scanning
// lidar and a camera -- then the truth sensors, each attached under its
// quantity's name.
using SensorKinds = util::TypeList<
    domain::sensing::Encoder,
    domain::sensing::Gyroscope,
    domain::sensing::Accelerometer,
    domain::sensing::Magnetometer,
    domain::sensing::Lidar,
    domain::sensing::Camera,
    domain::sensing::truth::JointPosition,
    domain::sensing::truth::JointVelocity,
    domain::sensing::truth::JointAcceleration,
    domain::sensing::truth::WorldTransform,
    domain::sensing::truth::SpatialVelocity,
    domain::sensing::truth::SpatialAcceleration>;

// Every actuator kind the sim hosts, registered the same way. Only what a real
// robot has a part for gets a kind: a modelled part, then the truth actuators.
// A controller is never one: it belongs to whoever is driving the sim, on the
// other side of a command.
using ActuatorKinds = util::TypeList<
    domain::actuation::DcMotor,
    domain::actuation::truth::Force,
    domain::actuation::truth::Torque>;

// Every kind the attachment algorithm hosts: each kind's instances are one
// segment of its rows, packed at that kind's own record sizes.
using AttachmentKinds = util::ConcatT<SensorKinds, ActuatorKinds>;

// Every attachment's row: one per attachment the scene declares under any
// registered kind's kListName, kind by kind -- each kind's rows one segment --
// each linked to its mount joint's row.
using AttachmentOrdering = engine::topology::
    ListOrdering<engine::topology::TopologicalOrdering, AttachmentKinds>;

// What every attachment reads whatever its kind: the sim's clock.
using ClockInput = SimInput<clock::ClockAlgorithm, clock::ClockField::kClock>;

// What every actuator touches whatever its kind, at its joint: S, to project
// its effort onto the joint's coordinates, and the torque half actuators
// accumulate into.
using SubspaceInput =
    SimInput<aba::ABAAlgorithm, aba::ABAField::kJointSubspace>;
using TorqueTarget =
    SimInput<aba::ABAAlgorithm, aba::ABAField::kJointTorqueNext>;

// Every other algorithm's field any registered kind touches, once each: at its
// mount joint or, for a scanning kind, at every body -- which of the two is
// the running Op's business, not the field's. Own inputs are the attachment
// algorithm's own fields already, so they aren't among these.
using AttachmentInputs = util::UniqueT<ForeignInputsT<util::ConcatT<
    util::TypeList<ClockInput, SubspaceInput, TorqueTarget>,
    util::ApplyT<util::ConcatT, util::TransformT<KindInputs, AttachmentKinds>>,
    util::
        ApplyT<util::ConcatT, util::TransformT<KindScene, AttachmentKinds>>>>>;

namespace detail {

template <typename Kinds>
inline constexpr bool kKindsOnJoints = false;
template <typename... Kinds>
inline constexpr bool kKindsOnJoints<util::TypeList<Kinds...>> =
    std::conjunction_v<
        std::bool_constant<kOnJoints<KindInputs<Kinds>>>...,
        std::bool_constant<kOnJoints<KindScene<Kinds>>>...>;

}  // namespace detail

// A kind reads its quantities in place at its joint, so each must be bound to a
// shared, joint-ordered field.
static_assert(detail::kKindsOnJoints<AttachmentKinds>);

// Every registered kind's records (one record type per kind's segment).
using AttachmentStates = util::TransformT<KindState, AttachmentKinds>;
using AttachmentCommands = util::TransformT<KindCommand, AttachmentKinds>;
using AttachmentOutputs = util::TransformT<KindOutput, AttachmentKinds>;

}  // namespace achilles::algorithms::attachment
