#pragma once

#include <array>
#include <cstddef>
#include <memory>

#include "algorithms/attachment/attachment_data.hpp"
#include "algorithms/attachment_kinds/attachment_kinds.hpp"
#include "algorithms/conventions.hpp"
#include "domain/sim_clock.hpp"
#include "engine/op_contract.hpp"
#include "util/tmp.hpp"

namespace achilles::algorithms::attachment {

namespace detail {

template <typename S, typename Inputs>
struct ReadOpImpl;

template <typename S, typename... Inputs>
struct ReadOpImpl<S, util::TypeList<Inputs...>> {
  using FieldEnum = AttachmentField;
  using ArgData = engine::ArgData<FieldEnum>;
  using ArgIndex = engine::ArgIndex;

  explicit ReadOpImpl(ScalarOperationT dt) : dt_(dt) {}

  static constexpr std::array<ArgData, sizeof...(Inputs) + 3> kInputs = {
      ArgData{AttachmentField::kConfig, ArgIndex::kTarget},
      ArgData{kInputFieldOf<ClockInput>, ArgIndex::kLink},
      ArgData{kInputFieldOf<Inputs>, ArgIndex::kLink}...,
      ArgData{AttachmentField::kPublished, ArgIndex::kTarget},
  };
  static constexpr std::array<ArgData, 2> kOutputs = {
      ArgData{AttachmentField::kState, ArgIndex::kTarget},
      ArgData{AttachmentField::kOutput, ArgIndex::kTarget},
  };

  void operator()(
      const S& sensor,
      const domain::SimClock& clock,
      const InputValueT<Inputs>&... inputs,
      const KindOutput<S>& last,
      KindState<S>* state,
      KindOutput<S>* out
  ) const {
    sensor.Update(
        clock, dt_, inputs..., last.value, &state->value, &out->value
    );
  }

 private:
  ScalarOperationT dt_;
};

template <typename S, typename Inputs, typename Scene>
struct ObserveOpImpl;

template <typename S, typename... Inputs, typename... SceneIns>
struct ObserveOpImpl<
    S,
    util::TypeList<Inputs...>,
    util::TypeList<SceneIns...>> {
  using FieldEnum = AttachmentField;
  using ArgData = engine::ArgData<FieldEnum>;
  using ArgIndex = engine::ArgIndex;

  explicit ObserveOpImpl(ScalarOperationT dt) : dt_(dt) {}

  static constexpr std::
      array<ArgData, sizeof...(Inputs) + sizeof...(SceneIns) + 3>
          kInputs = {
              ArgData{AttachmentField::kConfig, ArgIndex::kTarget},
              ArgData{kInputFieldOf<ClockInput>, ArgIndex::kLink},
              ArgData{kInputFieldOf<Inputs>, ArgIndex::kLink}...,
              ArgData{kInputFieldOf<SceneIns>, ArgIndex::kScene}...,
              ArgData{AttachmentField::kPublished, ArgIndex::kTarget},
          };
  static constexpr std::array<ArgData, 1> kOutputs = {
      ArgData{AttachmentField::kState, ArgIndex::kTarget},
  };

  void operator()(
      const S& sensor,
      const domain::SimClock& clock,
      const InputValueT<Inputs>&... inputs,
      const InputValueT<SceneIns>&... scene,
      const KindOutput<S>& last,
      KindState<S>* state
  ) const {
    sensor.Scan(clock, dt_, last.value, &state->value, inputs..., scene...);
  }

 private:
  ScalarOperationT dt_;
};

template <typename S, typename Inputs>
struct DriveOpImpl;

template <typename S, typename... Inputs>
struct DriveOpImpl<S, util::TypeList<Inputs...>> {
  using FieldEnum = AttachmentField;
  using ArgData = engine::ArgData<FieldEnum>;
  using ArgIndex = engine::ArgIndex;
  using Subspace = InputValueT<SubspaceInput>;
  using Torque = InputValueT<TorqueTarget>;

  explicit DriveOpImpl(ScalarOperationT dt) : dt_(dt) {}

  static constexpr std::array<ArgData, sizeof...(Inputs) + 4> kInputs = {
      ArgData{AttachmentField::kConfig, ArgIndex::kTarget},
      ArgData{kInputFieldOf<ClockInput>, ArgIndex::kLink},
      ArgData{kInputFieldOf<SubspaceInput>, ArgIndex::kLink},
      ArgData{kInputFieldOf<Inputs>, ArgIndex::kLink}...,
      ArgData{AttachmentField::kPublished, ArgIndex::kTarget},
  };
  static constexpr std::array<ArgData, 4> kOutputs = {
      ArgData{AttachmentField::kCommand, ArgIndex::kTarget},
      ArgData{AttachmentField::kState, ArgIndex::kTarget},
      ArgData{AttachmentField::kOutput, ArgIndex::kTarget},
      ArgData{kInputFieldOf<TorqueTarget>, ArgIndex::kLink},
  };

  void operator()(
      const S& actuator,
      const domain::SimClock& clock,
      const Subspace& subspace,
      const InputValueT<Inputs>&... inputs,
      const KindOutput<S>& last,
      KindCommand<S>* command,
      KindState<S>* state,
      KindOutput<S>* out,
      Torque* torque
  ) const {
    actuator.Update(
        clock,
        dt_,
        inputs...,
        last.value,
        &command->value,
        &state->value,
        &out->value
    );
    actuator.AddEffort(subspace, out->value.value.effort, torque);
  }

 private:
  ScalarOperationT dt_;
};

template <typename S, typename Inputs>
struct InitOpImpl;

template <typename S, typename... Inputs>
struct InitOpImpl<S, util::TypeList<Inputs...>> {
  using FieldEnum = AttachmentField;
  using ArgData = engine::ArgData<FieldEnum>;
  using ArgIndex = engine::ArgIndex;

  static constexpr std::array<ArgData, sizeof...(Inputs) + 1> kInputs = {
      ArgData{AttachmentField::kConfig, ArgIndex::kTarget},
      ArgData{kInputFieldOf<Inputs>, ArgIndex::kLink}...,
  };
  static constexpr std::array<ArgData, 3> kOutputs = {
      ArgData{AttachmentField::kCommand, ArgIndex::kTarget},
      ArgData{AttachmentField::kState, ArgIndex::kTarget},
      ArgData{AttachmentField::kOutput, ArgIndex::kTarget},
  };

  void operator()(
      const S& attachment,
      const InputValueT<Inputs>&... inputs,
      KindCommand<S>* command,
      KindState<S>* state,
      KindOutput<S>* out
  ) const {
    *command = KindCommand<S>{};
    // Value-initialized in place: a kind's state can be megabytes (a
    // camera's images), too big for the temporary an assignment makes.
    std::construct_at(state);
    *out = KindOutput<S>{};
    domain::attachment::InitializeState(attachment, &state->value);
    domain::attachment::InitializeOutput(
        attachment, &out->value.value, inputs...
    );
  }
};

}  // namespace detail

// One (instance, body) pair of a scanning kind: on a tick it samples, hands
// the instance its own inputs (at its mount joint) and that body's scene
// values to accumulate into its state. Run over the kind's segment against
// every body (engine::pass::LinkedSegmentScanTraversal).
template <SensorHasScene S>
using ObserveOp = detail::ObserveOpImpl<S, KindInputs<S>, KindScene<S>>;

// Runs one kind's sensor on its inputs, read in place at its mount row,
// when a sample is due, into kOutput -- or carries the last published
// reading forward when it isn't.
// Run over that kind's segment of the sensing rows only.
template <SensorLike S>
using ReadOp = detail::ReadOpImpl<S, KindInputs<S>>;

// Runs one actuator kind's instances, the mirror of ReadOp: each works out its
// effort from its command and its inputs, read in place at its joint, when an
// update is due -- or carries its last one forward when it isn't -- and adds it
// to the torque the coming tick reads. Actuators on one joint add: a list's
// rows run one at a time.
template <ActuatorLike S>
using DriveOp = detail::DriveOpImpl<S, KindInputs<S>>;

// Starts one kind's instances, sensor or actuator: clears their commands, seeds
// their state from their configuration, and fills their first output, if their
// kind has one. An output's sequence stays 0, so the first tick updates. Reads
// the kind's inputs at its mount row, like ReadOp.
template <AttachmentLike S>
using InitOp = detail::InitOpImpl<S, KindInputs<S>>;

}  // namespace achilles::algorithms::attachment
