#pragma once

#include <array>
#include <cstddef>
#include <utility>

#include "algorithms/conventions.hpp"
#include "algorithms/sensing/sensing_data.hpp"
#include "algorithms/sensing/sensor.hpp"
#include "engine/op_contract.hpp"

namespace achilles::algorithms::sensing {

namespace detail {

template <typename S, typename Inputs>
struct ReadOpImpl;

template <typename S, std::size_t... Is>
struct ReadOpImpl<S, std::index_sequence<Is...>> {
  using FieldEnum = SensingField<S>;
  using ArgData = engine::ArgData<FieldEnum>;
  using ArgIndex = engine::ArgIndex;
  using OutputReading = Reading<typename S::Output>;

  explicit ReadOpImpl(ScalarOperationT dt) : dt_(dt) {}

  static constexpr std::array<ArgData, sizeof...(Is) + 2> kInputs = {
      ArgData{FieldEnum::kConfig, ArgIndex::kTarget},
      ArgData{SensingFields<S>::Input(Is), ArgIndex::kLink}...,
      ArgData{FieldEnum::kPublished, ArgIndex::kTarget},
  };
  static constexpr std::array<ArgData, 3> kOutputs = {
      ArgData{FieldEnum::kClock, ArgIndex::kTarget},
      ArgData{FieldEnum::kState, ArgIndex::kTarget},
      ArgData{FieldEnum::kOutput, ArgIndex::kTarget},
  };

  void operator()(
      const S& sensor,
      const InputValueT<InputAtT<S, Is>>&... inputs,
      const OutputReading& last,
      SensorClock* clock,
      typename S::State* state,
      OutputReading* out
  ) const {
    if (sensor.Due(*clock, dt_, last)) {
      OutputReading reading;
      sensor.Read(
          SensorTick{dt_, clock->time, clock->tick, 0},
          inputs...,
          state,
          &reading.value
      );
      reading.sample_time = clock->time;
      reading.sequence = last.sequence + 1;
      *out = reading;
    } else {
      *out = last;
    }
    clock->time += dt_;
    ++clock->tick;
  }

 private:
  ScalarOperationT dt_;
};

// A kind's own Initialize if it declares one; otherwise its state stays
// value-initialized.
template <SensorLike S>
void InitializeState(const S& sensor, typename S::State* state)
  requires SensorHasInit<S>
{
  sensor.Initialize(SensorTick{0.0F, 0.0F, 0, 0}, state);
}

template <SensorLike S>
void InitializeState(const S& /*sensor*/, typename S::State* /*state*/)
  requires(!SensorHasInit<S>)
{}

}  // namespace detail

// Runs the sensor on its inputs, read in place at its mount row, when a
// sample is due, into kOutput -- or carries the last published reading
// forward when it isn't. Advances the instance's clock either way.
template <SensorLike S>
using ReadOp = detail::ReadOpImpl<S, std::make_index_sequence<S::kInputCount>>;

// Resets an instance's clock and seeds its state.
template <SensorLike S>
struct InitOp {
  using FieldEnum = SensingField<S>;
  using ArgData = engine::ArgData<FieldEnum>;
  using ArgIndex = engine::ArgIndex;

  static constexpr std::array<ArgData, 1> kInputs = {
      ArgData{FieldEnum::kConfig, ArgIndex::kTarget},
  };
  static constexpr std::array<ArgData, 2> kOutputs = {
      ArgData{FieldEnum::kClock, ArgIndex::kTarget},
      ArgData{FieldEnum::kState, ArgIndex::kTarget},
  };

  void operator()(const S& sensor, SensorClock* clock, typename S::State* state)
      const {
    *clock = SensorClock{};
    *state = typename S::State{};
    detail::InitializeState(sensor, state);
  }
};

}  // namespace achilles::algorithms::sensing
