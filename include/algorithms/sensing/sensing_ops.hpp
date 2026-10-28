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

  // Input I's own field: SensorInputs' entries follow kFirstInput.
  template <typename I>
  static constexpr SensingField kInputField = static_cast<
      SensingField>(static_cast<std::size_t>(SensingField::kFirstInput) + util::kIndexOfV<I, SensorInputs>);

  static constexpr std::array<ArgData, sizeof...(Inputs) + 2> kInputs = {
      ArgData{SensingField::kConfig, ArgIndex::kTarget},
      ArgData{kInputField<Inputs>, ArgIndex::kLink}...,
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
    if (sensor.Due(*clock, dt_, last.value)) {
      Reading<typename S::Output> reading;
      sensor.Read(
          SensorTick{dt_, clock->time, clock->tick, 0},
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

}  // namespace detail

// Runs one kind's sensor on its inputs, read in place at its mount row,
// when a sample is due, into kOutput -- or carries the last published
// reading forward when it isn't. Advances the instance's clock either way.
// Run over that kind's segment of the sensing rows only.
template <SensorLike S>
using ReadOp = detail::ReadOpImpl<S, typename S::InputList>;

// Resets one kind's instances' clocks and seeds their state.
template <SensorLike S>
struct InitOp {
  using FieldEnum = SensingField;
  using ArgData = engine::ArgData<FieldEnum>;
  using ArgIndex = engine::ArgIndex;

  static constexpr std::array<ArgData, 1> kInputs = {
      ArgData{SensingField::kConfig, ArgIndex::kTarget},
  };
  static constexpr std::array<ArgData, 2> kOutputs = {
      ArgData{SensingField::kClock, ArgIndex::kTarget},
      ArgData{SensingField::kState, ArgIndex::kTarget},
  };

  void operator()(const S& sensor, SensorClock* clock, KindState<S>* state)
      const {
    *clock = SensorClock{};
    *state = KindState<S>{};
    InitializeState(sensor, &state->value);
  }
};

}  // namespace achilles::algorithms::sensing
