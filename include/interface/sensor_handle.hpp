#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>

#include "algorithms/conventions.hpp"
#include "algorithms/sensing/sensing_data.hpp"
#include "algorithms/sensing/sensor.hpp"
#include "algorithms/sensing/sensor_kinds.hpp"
#include "domain/joint_data.hpp"
#include "engine/pass/sim_context.hpp"
#include "util/tmp.hpp"

namespace achilles::interface {

namespace detail {

template <algorithms::sensing::SensorLike K, typename SimStateT>
std::size_t RowOf(const SimStateT& sim_state, std::size_t entry) {
  return sim_state.template LayoutFor<algorithms::sensing::SensingOrdering>()
      .ToSorted(util::kIndexOfV<K, algorithms::sensing::SensorKinds>, entry);
}

}  // namespace detail

// One sensor instance of kind K, found once (Simulation::Sensor) and kept:
// the way anything outside the sim looks into it. A handle like
// ActuatorHandle (actuator_handle.hpp), cheap enough to copy, and valid until
// the Simulation that made it is destroyed or re-Init()ed.
template <algorithms::sensing::SensorLike K>
class SensorHandle {
 public:
  // The `kind:` an .arow declares one under.
  static constexpr std::string_view kListName = K::kListName;

  // Checks `attachment` is a well-formed K -- declared as a sensor, with
  // settings K's FromConfig accepts, a setting it doesn't read rejected so a
  // typo doesn't silently do nothing -- and writes it into its row: `entry` of
  // K's list, with `stream` as its random stream. Throws
  // std::invalid_argument.
  template <engine::pass::SimContextLike SimStateT>
  static void Configure(
      const SimStateT& sim_state,
      const domain::JointAttachment& attachment,
      std::size_t /*instance*/,
      std::size_t entry,
      std::uint64_t stream
  ) {
    domain::RequireKey(attachment, "sensor");
    algorithms::sensing::AttachmentConfig config(
        attachment.name, attachment.config
    );
    K sensor = K::FromConfig(config);
    config.Finish();
    sensor.SetStream(stream);
    sim_state.template ViewFor<algorithms::sensing::SensingView>()
        .template Store<algorithms::sensing::SensingField::kConfig, K>(
            detail::RowOf<K>(sim_state, entry), sensor
        );
  }

  // The handle for `entry` of kind K's list.
  template <engine::pass::SimContextLike SimStateT>
  static SensorHandle Find(const SimStateT& sim_state, std::size_t entry) {
    return SensorHandle(
        sim_state.template ViewFor<algorithms::sensing::SensingView>(),
        detail::RowOf<K>(sim_state, entry)
    );
  }

  // The sensor's latest published reading: its value, when it was sampled,
  // and how many samples it has taken. Valid from Init: until its first
  // sample the sequence is 0 and the value is what its kind starts from
  // (see algorithms::sensing::InitOp).
  auto Read() const {
    using algorithms::sensing::SensingField;
    return view_
        .template Load<
            SensingField::kPublished,
            algorithms::sensing::KindOutput<K>>(row_)
        .value;
  }

  // The sensor's current configuration.
  K Config() const {
    return view_.template Load<algorithms::sensing::SensingField::kConfig, K>(
        row_
    );
  }

  // Replaces the sensor's configuration and starts it over: its state is
  // re-seeded by its Initialize, and its readings cleared, so it samples
  // again on the next tick. Its clock and random stream carry on.
  void SetConfig(K config) const {
    using algorithms::sensing::KindOutput;
    using algorithms::sensing::KindState;
    using algorithms::sensing::SensingField;
    config.SetStream(Config().Stream());
    view_.template Store<SensingField::kConfig, K>(row_, config);

    KindState<K> state{};
    algorithms::sensing::InitializeState(config, &state.value);
    view_.template Store<SensingField::kState, KindState<K>>(row_, state);
    view_.template Store<SensingField::kOutput, KindOutput<K>>(
        row_, KindOutput<K>{}
    );
    view_.template Store<SensingField::kPublished, KindOutput<K>>(
        row_, KindOutput<K>{}
    );
  }

 private:
  SensorHandle(algorithms::sensing::SensingView view, std::size_t row)
      : view_(view), row_(row) {}

  algorithms::sensing::SensingView view_;
  std::size_t row_;  // in the sensing rows
};

}  // namespace achilles::interface
