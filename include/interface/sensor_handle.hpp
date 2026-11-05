#pragma once

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
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

// Calls fn.template operator()<K>() for the registered kind named `kind`;
// false if none is.
template <typename Fn, typename... Kinds>
bool WithKindNamed(
    util::TypeList<Kinds...> /*kinds*/, std::string_view kind, const Fn& fn
) {
  return (
      (Kinds::kListName == kind ? (fn.template operator()<Kinds>(), true)
                                : false) ||
      ...
  );
}

// An attachment's settings as K, read by K's own FromConfig; a setting it
// doesn't read is rejected, so a typo doesn't silently do nothing.
template <algorithms::sensing::SensorLike K>
K MakeSensor(const domain::JointAttachment& attachment) {
  algorithms::sensing::SensorConfig config(attachment.name, attachment.config);
  K sensor = K::FromConfig(config);
  config.Finish();
  return sensor;
}

template <algorithms::sensing::SensorLike K, typename SimStateT>
std::size_t RowOf(const SimStateT& sim_state, std::size_t entry) {
  return sim_state.template LayoutFor<algorithms::sensing::SensingOrdering>()
      .ToSorted(util::kIndexOfV<K, algorithms::sensing::SensorKinds>, entry);
}

}  // namespace detail

// Writes one sensor's configuration into its row: `entry` of its kind's list,
// built from `attachment`'s settings, with its own random stream. Run once
// the sim is allocated and before its Setup list (whose InitStep seeds each
// sensor's state from it). Throws std::invalid_argument for a sensor of no
// registered kind or with a setting its kind doesn't take.
template <engine::pass::SimContextLike SimStateT>
void WriteSensorConfig(
    const SimStateT& sim_state,
    const domain::JointAttachment& attachment,
    std::size_t entry,
    std::uint64_t stream
) {
  using algorithms::sensing::SensingField;
  const auto view =
      sim_state.template ViewFor<algorithms::sensing::SensingView>();
  const bool known = detail::WithKindNamed(
      algorithms::sensing::SensorKinds{},
      attachment.list,
      [&]<typename K>() {
        K sensor = detail::MakeSensor<K>(attachment);
        sensor.SetStream(stream);
        view.template Store<SensingField::kConfig, K>(
            detail::RowOf<K>(sim_state, entry), sensor
        );
      }
  );
  if (!known) {
    throw std::invalid_argument(
        "sensor '" + attachment.name + "': no sensor kind named '" +
        attachment.list + "'."
    );
  }
}

// Writes every sensor's configuration into its row: `attachments` walks them
// (ArchetypeSet::ForEachAttachment), each with its own random stream drawn
// from `seed`. Throws as WriteSensorConfig does.
template <engine::pass::SimContextLike SimStateT, typename Attachments>
void WriteSensorConfigs(
    const SimStateT& sim_state,
    const Attachments& attachments,
    std::uint64_t seed
) {
  attachments.ForEachAttachment(
      seed,
      [&](const domain::JointAttachment& attachment,
          std::size_t entry,
          std::uint64_t stream) {
        WriteSensorConfig(sim_state, attachment, entry, stream);
      }
  );
}

// One sensor instance of kind K, found once (Simulation::Sensor) and kept:
// the way anything outside the sim looks into it. A handle like
// ActuatorHandle (actuator_handle.hpp), cheap enough to copy, and valid until
// the Simulation that made it is destroyed or re-Init()ed.
template <algorithms::sensing::SensorLike K>
class SensorHandle {
 public:
  // The handle for `entry` of kind K's list.
  template <engine::pass::SimContextLike SimStateT>
  static SensorHandle Find(const SimStateT& sim_state, std::size_t entry) {
    const std::size_t row = detail::RowOf<K>(sim_state, entry);
    return SensorHandle(
        sim_state.template ViewFor<algorithms::sensing::SensingView>(),
        row,
        sim_state.template TopologyFor<algorithms::sensing::SensingOrdering>(
        )[row]
    );
  }

  // The sensor's latest published reading: its value, when it was sampled,
  // and how many samples it has taken (0 until its first). A live kind has
  // no reading: this is the value alone, loaded from its input field at its
  // mount joint as it stands now (see algorithms::sensing::SensorIsLive).
  auto Read() const {
    using algorithms::sensing::SensingField;
    if constexpr (algorithms::sensing::SensorIsLive<K>) {
      return view_.template Load<
          algorithms::sensing::kInputFieldOf<typename K::Input>,
          algorithms::ScalarOperationT>(mount_);
    } else {
      return view_
          .template Load<
              SensingField::kPublished,
              algorithms::sensing::KindOutput<K>>(row_)
          .value;
    }
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
  SensorHandle(
      algorithms::sensing::SensingView view, std::size_t row, std::size_t mount
  )
      : view_(view), row_(row), mount_(mount) {}

  algorithms::sensing::SensingView view_;
  std::size_t row_;    // in the sensing rows
  std::size_t mount_;  // the joint it's mounted on, in the joints' rows
};

}  // namespace achilles::interface
