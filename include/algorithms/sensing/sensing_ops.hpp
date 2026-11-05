#pragma once

#include <array>
#include <cstddef>

#include "algorithms/conventions.hpp"
#include "algorithms/sensing/sensing_data.hpp"
#include "algorithms/sensing/sensor.hpp"
#include "algorithms/sensing/sensor_kinds.hpp"
#include "engine/op_contract.hpp"
#include "util/tmp.hpp"

namespace achilles::algorithms::sensing {

namespace detail {

template <typename S, typename Inputs>
struct ReadOpImpl;

template <typename S, typename... Inputs>
struct ReadOpImpl<S, util::TypeList<Inputs...>> {
  using FieldEnum = SensingField;
  using ArgData = engine::ArgData<FieldEnum>;
  using ArgIndex = engine::ArgIndex;

  explicit ReadOpImpl(ScalarOperationT dt) : dt_(dt) {}

  static constexpr std::array<ArgData, sizeof...(Inputs) + 2> kInputs = {
      ArgData{SensingField::kConfig, ArgIndex::kTarget},
      ArgData{kInputFieldOf<Inputs>, ArgIndex::kLink}...,
      ArgData{SensingField::kPublished, ArgIndex::kTarget},
  };
  static constexpr std::array<ArgData, 3> kOutputs = {
      ArgData{SensingField::kClock, ArgIndex::kTarget},
      ArgData{SensingField::kState, ArgIndex::kTarget},
      ArgData{SensingField::kOutput, ArgIndex::kTarget},
  };

  void operator()(
      const S& sensor,
      const InputValueT<Inputs>&... inputs,
      const KindOutput<S>& last,
      SensorClock* clock,
      KindState<S>* state,
      KindOutput<S>* out
  ) const {
    if (IsDue(sensor, *clock, dt_, last.value)) {
      Reading<typename S::Output> reading;
      sensor.Read(
          SensorTick{dt_, clock->time, clock->tick, sensor.Stream()},
          inputs...,
          &state->value,
          &reading.value
      );
      reading.sample_time = clock->time;
      reading.sequence = last.value.sequence + 1;
      out->value = reading;
    } else {
      *out = last;
    }
    clock->time += dt_;
    ++clock->tick;
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
  using FieldEnum = SensingField;
  using ArgData = engine::ArgData<FieldEnum>;
  using ArgIndex = engine::ArgIndex;

  explicit ObserveOpImpl(ScalarOperationT dt) : dt_(dt) {}

  static constexpr std::
      array<ArgData, sizeof...(Inputs) + sizeof...(SceneIns) + 2>
          kInputs = {
              ArgData{SensingField::kConfig, ArgIndex::kTarget},
              ArgData{SensingField::kClock, ArgIndex::kTarget},
              ArgData{kInputFieldOf<Inputs>, ArgIndex::kLink}...,
              ArgData{kInputFieldOf<SceneIns>, ArgIndex::kScene}...,
          };
  static constexpr std::array<ArgData, 1> kOutputs = {
      ArgData{SensingField::kState, ArgIndex::kTarget},
  };

  void operator()(
      const S& sensor,
      const SensorClock& clock,
      const InputValueT<Inputs>&... inputs,
      const InputValueT<SceneIns>&... scene,
      KindState<S>* state
  ) const {
    if (clock.due) {
      sensor.Observe(
          SensorTick{dt_, clock.time, clock.tick, sensor.Stream()},
          inputs...,
          scene...,
          &state->value
      );
    }
  }

 private:
  ScalarOperationT dt_;
};

template <typename S, typename Inputs>
struct InitOpImpl;

template <typename S, typename... Inputs>
struct InitOpImpl<S, util::TypeList<Inputs...>> {
  using FieldEnum = SensingField;
  using ArgData = engine::ArgData<FieldEnum>;
  using ArgIndex = engine::ArgIndex;

  static constexpr std::array<ArgData, sizeof...(Inputs) + 1> kInputs = {
      ArgData{SensingField::kConfig, ArgIndex::kTarget},
      ArgData{kInputFieldOf<Inputs>, ArgIndex::kLink}...,
  };
  static constexpr std::array<ArgData, 3> kOutputs = {
      ArgData{SensingField::kClock, ArgIndex::kTarget},
      ArgData{SensingField::kState, ArgIndex::kTarget},
      ArgData{SensingField::kOutput, ArgIndex::kTarget},
  };

  void operator()(
      const S& sensor,
      const InputValueT<Inputs>&... inputs,
      SensorClock* clock,
      KindState<S>* state,
      KindOutput<S>* out
  ) const {
    *clock = SensorClock{};
    *state = KindState<S>{};
    InitializeState(sensor, &state->value);
    *out = KindOutput<S>{};
    if constexpr (SensorHasOutputInit<S>) {
      sensor.InitializeOutput(
          SensorTick{0.0F, 0.0F, 0, sensor.Stream()},
          inputs...,
          &out->value.value
      );
    }
  }
};

}  // namespace detail

// Decides, once per tick and before anything else of a scanning kind runs,
// whether each of its instances samples (Sensor::Due) -- what its Observe
// and Read then both go by (SensorClock::due, see IsDue).
template <SensorHasScene S>
struct GateOp {
  using FieldEnum = SensingField;
  using ArgData = engine::ArgData<FieldEnum>;
  using ArgIndex = engine::ArgIndex;

  explicit GateOp(ScalarOperationT dt) : dt_(dt) {}

  static constexpr std::array<ArgData, 2> kInputs = {
      ArgData{SensingField::kConfig, ArgIndex::kTarget},
      ArgData{SensingField::kPublished, ArgIndex::kTarget},
  };
  static constexpr std::array<ArgData, 1> kOutputs = {
      ArgData{SensingField::kClock, ArgIndex::kTarget},
  };

  void operator()(
      const S& sensor, const KindOutput<S>& last, SensorClock* clock
  ) const {
    clock->due = sensor.Due(*clock, dt_, last.value);
  }

 private:
  ScalarOperationT dt_;
};

// One (instance, body) pair of a scanning kind: on a tick it samples, hands
// the instance its own inputs (at its mount joint) and that body's scene
// values to accumulate into its state. Run over the kind's segment against
// every body (engine::pass::LinkedSegmentScanTraversal).
template <SensorHasScene S>
using ObserveOp =
    detail::ObserveOpImpl<S, typename S::InputList, typename S::Scene::List>;

// Runs one kind's sensor on its inputs, read in place at its mount row,
// when a sample is due, into kOutput -- or carries the last published
// reading forward when it isn't. Advances the instance's clock either way.
// Run over that kind's segment of the sensing rows only.
template <SensorLike S>
using ReadOp = detail::ReadOpImpl<S, typename S::InputList>;

// Starts one kind's instances: resets their clocks, seeds their state, and
// fills their first reading, so one is valid before any sample is taken. Its
// sequence stays 0, so the first tick still samples. Reads the kind's inputs
// at its mount row, like ReadOp.
template <SensorLike S>
using InitOp = detail::InitOpImpl<S, typename S::InputList>;

}  // namespace achilles::algorithms::sensing
