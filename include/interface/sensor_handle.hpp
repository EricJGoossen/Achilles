#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>

#include "algorithms/sensing/sensing_data.hpp"
#include "algorithms/sensing/sensor.hpp"
#include "algorithms/sensing/sensor_kinds.hpp"
#include "domain/archetype.hpp"
#include "domain/random/counter_rng.hpp"
#include "engine/pass/sim_context.hpp"
#include "util/tmp.hpp"

namespace achilles::interface {

// One instance of a sensor, by the name its .arow file gives it: instance i
// is the one mounted on instance i of the archetype it's attached to.
using SensorAddress = domain::AttachmentAddress;

// A sensor instance of kind K, found once (Simulation::FindSensor) and kept:
// a plain row index into the sensing algorithm, cheap to copy. Valid until
// the Simulation that made it is re-Init()ed from a different .arow.
template <algorithms::sensing::SensorLike K>
struct SensorHandle {
  std::size_t row = 0;
};

namespace detail {

// Calls fn.template operator()<K>() for the registered kind named `kind`;
// false if none is.
template <typename Fn, typename... Kinds>
bool WithKindNamed(
    util::TypeList<Kinds...> /*kinds*/, std::string_view kind, Fn&& fn
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
K MakeSensor(const domain::ArchetypeAttachment& attachment) {
  algorithms::sensing::SensorConfig config(attachment.name, attachment.config);
  K sensor = K::FromConfig(config);
  config.Finish();
  return sensor;
}

template <algorithms::sensing::SensorLike K, typename SimStateT>
std::size_t RowOf(const SimStateT& sim_state, std::size_t entry) {
  return sim_state.template LayoutFor<algorithms::sensing::SensingOrdering>()
      .ToSorted({util::kIndexOfV<K, algorithms::sensing::SensorKinds>, entry});
}

}  // namespace detail

// Writes every sensor's configuration into its row, built from its
// attachment's settings, each with its own random stream drawn from `seed`
// and its index. Run once the sim is allocated and before its Setup list
// (whose InitStep seeds each sensor's state from it). Throws
// std::invalid_argument for a sensor of no registered kind or with a
// setting its kind doesn't take.
template <engine::pass::SimContextLike SimStateT>
void WriteSensorConfigs(
    const SimStateT& sim_state,
    std::span<const domain::Archetype> archetypes,
    std::uint64_t seed
) {
  using algorithms::sensing::SensingField;
  const auto view =
      sim_state.template ViewFor<algorithms::sensing::SensingView>();
  // Entries count per kind, in the order the sensing ordering numbers them
  // (engine::topology::CollectAttachments).
  std::unordered_map<std::string_view, std::size_t> entries;
  std::uint64_t index = 0;
  for (const domain::Archetype& archetype : archetypes) {
    for (std::size_t i = 0; i < archetype.InstanceCount(); ++i) {
      for (const domain::ArchetypeAttachment& attachment :
           archetype.Attachments()) {
        const std::size_t entry = entries[attachment.list]++;
        const bool known = detail::WithKindNamed(
            algorithms::sensing::SensorKinds{},
            attachment.list,
            [&]<typename K>() {
              K sensor = detail::MakeSensor<K>(attachment);
              sensor.SetStream(domain::random::StreamKey(seed, index));
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
        ++index;
      }
    }
  }
}

// The handle for the sensor instance `address` names. Throws
// std::invalid_argument for an unknown name, an out-of-range instance, or
// a sensor of another kind than K.
template <
    algorithms::sensing::SensorLike K,
    engine::pass::SimContextLike SimStateT>
SensorHandle<K> FindSensor(
    const SimStateT& sim_state,
    std::span<const domain::Archetype> archetypes,
    const SensorAddress& address
) {
  const domain::ResolvedAttachment found =
      domain::ResolveAttachment(archetypes, address);
  if (found.list != K::kListName) {
    throw std::invalid_argument(
        "sensor '" + address.name + "' is a '" + std::string(found.list) +
        "', not a '" + std::string(K::kListName) + "'."
    );
  }
  return {detail::RowOf<K>(sim_state, found.entry)};
}

// The sensor's latest published reading: its value, when it was sampled,
// and how many samples it has taken (0 until its first).
template <
    algorithms::sensing::SensorLike K,
    engine::pass::SimContextLike SimStateT>
algorithms::sensing::Reading<typename K::Output> ReadSensor(
    const SimStateT& sim_state, SensorHandle<K> handle
) {
  return sim_state.template ViewFor<algorithms::sensing::SensingView>()
      .template Load<
          algorithms::sensing::SensingField::kPublished,
          algorithms::sensing::KindOutput<K>>(handle.row)
      .value;
}

// The sensor's current configuration.
template <
    algorithms::sensing::SensorLike K,
    engine::pass::SimContextLike SimStateT>
K SensorConfig(const SimStateT& sim_state, SensorHandle<K> handle) {
  return sim_state.template ViewFor<algorithms::sensing::SensingView>()
      .template Load<algorithms::sensing::SensingField::kConfig, K>(handle.row);
}

// Replaces the sensor's configuration and starts it over: its state is
// re-seeded by its Initialize, and its readings cleared, so it samples
// again on the next tick. Its clock and random stream carry on.
template <
    algorithms::sensing::SensorLike K,
    engine::pass::SimContextLike SimStateT>
void SetSensorConfig(
    const SimStateT& sim_state, SensorHandle<K> handle, K config
) {
  using algorithms::sensing::KindOutput;
  using algorithms::sensing::KindState;
  using algorithms::sensing::SensingField;
  const auto view =
      sim_state.template ViewFor<algorithms::sensing::SensingView>();
  config.SetStream(SensorConfig(sim_state, handle).Stream());
  view.template Store<SensingField::kConfig, K>(handle.row, config);

  KindState<K> state{};
  algorithms::sensing::InitializeState(config, &state.value);
  view.template Store<SensingField::kState, KindState<K>>(handle.row, state);
  view.template Store<SensingField::kOutput, KindOutput<K>>(
      handle.row, KindOutput<K>{}
  );
  view.template Store<SensingField::kPublished, KindOutput<K>>(
      handle.row, KindOutput<K>{}
  );
}

}  // namespace achilles::interface
