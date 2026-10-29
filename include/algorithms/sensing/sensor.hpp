#pragma once

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include "domain/math/quaternion.hpp"
#include "domain/math/vector3.hpp"
#include "domain/sensing/effects.hpp"
#include "domain/spatial/transform.hpp"
#include "engine/algorithm_contract.hpp"
#include "engine/assembler.hpp"
#include "engine/memory/binding.hpp"
#include "engine/topology/orderings.hpp"
#include "util/tmp.hpp"

namespace achilles::algorithms::sensing {

// One piece of sim state a sensor reads: field F of the hosted Algorithm
// AlgorithmT, at the joint the sensor is mounted on. F must be shared
// (Traits<F>::SharedAs) and ordered by joint (TopologicalOrdering): the
// sensor's own Algorithm names that same slot to read the exact block
// AlgorithmT keeps, never a copy (see SensingFieldTraits, sensing_data.hpp).
template <engine::AlgorithmLike AlgorithmT, typename AlgorithmT::Enum F>
struct SimInput {
  using Source = AlgorithmT;
  using Enum = typename AlgorithmT::Enum;
  static constexpr Enum kField = F;
};

namespace detail {

template <typename AlgorithmT, auto F>
struct SourceTraitsOf;

template <engine::FieldEnumLike EnumT, template <EnumT> class Traits, EnumT F>
struct SourceTraitsOf<engine::Algorithm<EnumT, Traits>, F> {
  using Type = Traits<F>;
  static constexpr bool kShared =
      engine::memory::FieldIsShared<EnumT, Traits, F>;
  using Ordering = engine::memory::FieldOrderingT<EnumT, Traits, F>;
  using Layout = engine::memory::FieldLayoutT<EnumT, Traits, F>;
  using Indexing = engine::memory::FieldIndexingT<EnumT, Traits, F>;
  using Buffering = engine::memory::FieldBufferingT<EnumT, Traits, F>;
};

}  // namespace detail

// The Traits<F> an input's source field was declared with, and the policies
// it resolves to.
template <typename Input>
using InputSource =
    detail::SourceTraitsOf<typename Input::Source, Input::kField>;

// What a sensor receives for one input: the source field's value read at its
// own scalar -- e.g. Transform<float> for ABA's kWorldTransform.
template <typename Input>
using InputValueT =
    engine::AssembledT<typename InputSource<Input>::Type::Assembler>;

// The input list a sensor hands its base (see Sensor below).
template <typename... Inputs>
struct SimInputs {};

// The fields a scanning sensor reads at every row of the bodies' ordering,
// not just its own mount joint -- e.g. every body's collision shape, for a
// lidar. A kind declares `using Scene = SceneInputs<...>` and an Observe
// (see Sensor) to scan them.
template <typename... Inputs>
struct SceneInputs {
  using List = util::TypeList<Inputs...>;
};

// The State of a sensor that keeps none.
struct NoState {};

// Per-instance settings every sensor has, whatever it models.
struct SensorOptions {
  // Seconds between samples; 0 samples every tick. A sensor whose period
  // hasn't elapsed reports its last reading again, unchanged.
  float period = 0.0F;
};

// What a sensor reports: its output, when it was sampled (sim time, seconds
// since Init), and how many samples it has taken -- 0 until its first.
template <typename OutputT>
struct Reading {
  OutputT value{};
  float sample_time = 0.0F;
  std::uint32_t sequence = 0;
};

// A sensor instance's own clock, kept for every sensor by its base: sim time
// of the state being sensed, and how many ticks it has seen.
struct SensorClock {
  float time = 0.0F;
  // Whether this tick samples, decided once before a scanning kind's
  // Observe and Read both go by it (see IsDue). Beside `time`, in the
  // padding before `tick`, so the clock stays 16 bytes.
  bool due = false;
  std::uint64_t tick = 0;
};
static_assert(sizeof(SensorClock) == 16);

// Everything about "now" a sensor's Read receives. stream and tick key a
// counter-based random generator (domain::random::CounterRng): a draw
// depends only on (stream, tick, draw index), never on what ran before it,
// so readings are reproducible whichever thread runs which instance.
// stream identifies the instance: its index mixed with the sim's seed (see
// Sensor::Stream).
struct SensorTick {
  float dt = 0.0F;
  float time = 0.0F;
  std::uint64_t tick = 0;
  std::uint64_t stream = 0;
};

// A sensor's settings, as its .arow file gives them (`config:`), read by
// its kind's FromConfig: each by name, with a default for when it's absent.
// Every setting given must be read -- Finish throws std::invalid_argument
// for any left over -- so a misspelled setting is an error rather than
// silently doing nothing.
class SensorConfig {
 public:
  using Entries = std::span<const std::pair<std::string, std::vector<double>>>;

  SensorConfig(std::string sensor, Entries entries)
      : sensor_(std::move(sensor)),
        entries_(entries),
        read_(entries.size(), false) {}

  float Scalar(std::string_view key, float fallback) {
    const std::vector<double>* values = Take(key, 1);
    return values == nullptr ? fallback : static_cast<float>((*values)[0]);
  }

  // A true/false setting (the .arow gives true/false or 1/0).
  bool Flag(std::string_view key, bool fallback) {
    const std::vector<double>* values = Take(key, 1);
    return values == nullptr ? fallback : (*values)[0] != 0.0;
  }

  std::uint32_t Count(std::string_view key, std::uint32_t fallback) {
    const std::vector<double>* values = Take(key, 1);
    if (values == nullptr) {
      return fallback;
    }
    if ((*values)[0] < 0.0) {
      Fail(key, "must not be negative");
    }
    return static_cast<std::uint32_t>((*values)[0]);
  }

  domain::math::Vector3<float> Vector(
      std::string_view key, const domain::math::Vector3<float>& fallback
  ) {
    const std::vector<double>* values = Take(key, 3);
    return values == nullptr ? fallback
                             : domain::math::Vector3<float>(
                                   static_cast<float>((*values)[0]),
                                   static_cast<float>((*values)[1]),
                                   static_cast<float>((*values)[2])
                               );
  }

  // A pose, given like a joint's fixed_joint_transform: {translation:
  // [x, y, z], rotation: [w, x, y, z]}, seven numbers in that order; the
  // rotation is normalized.
  domain::spatial::Transform<float> Pose(
      std::string_view key, const domain::spatial::Transform<float>& fallback
  ) {
    const std::vector<double>* values = Take(key, 7);
    if (values == nullptr) {
      return fallback;
    }
    const auto at = [&](std::size_t i) {
      return static_cast<float>((*values)[i]);
    };
    const domain::math::Quaternion<float> rotation(at(3), at(4), at(5), at(6));
    if (rotation.Norm() <= 0.0F) {
      Fail(key, "must not have a zero rotation");
    }
    return {
        domain::math::Vector3<float>(at(0), at(1), at(2)), rotation.Normalize()
    };
  }

  // The options every sensor has (SensorOptions): `period`.
  SensorOptions Options() { return {Scalar("period", 0.0F)}; }

  // One error model (domain::sensing::ErrorChain), from the settings named
  // `prefix` + scale_error, bias, bias_instability (the random walk's
  // density), noise_density, range, resolution and delay_samples -- each
  // left off when absent. A three-axis model's scale_error and bias take
  // three numbers.
  template <domain::sensing::SensorValueLike V>
  typename domain::sensing::ErrorChain<V>::Params ErrorModel(
      std::string_view prefix = {}
  ) {
    using Chain = domain::sensing::ErrorChain<V>;
    typename Chain::Params params;
    params.scale.error = Value(Key(prefix, "scale_error"), V{});
    params.bias.bias = Value(Key(prefix, "bias"), V{});
    params.walk.density = Scalar(Key(prefix, "bias_instability"), 0.0F);
    params.noise.density = Scalar(Key(prefix, "noise_density"), 0.0F);
    params.range.limit = Scalar(Key(prefix, "range"), 0.0F);
    params.resolution.resolution = Scalar(Key(prefix, "resolution"), 0.0F);
    const std::string delay = Key(prefix, "delay_samples");
    params.latency.samples = Count(delay, 0);
    constexpr std::size_t kCapacity =
        std::tuple_size_v<decltype(typename Chain::Latency::State{}.ring)>;
    if (params.latency.samples >= kCapacity) {
      Fail(delay, "must be below " + std::to_string(kCapacity));
    }
    return params;
  }

  // Throws for any setting no read asked for.
  void Finish() const {
    for (std::size_t i = 0; i < entries_.size(); ++i) {
      if (!read_[i]) {
        throw std::invalid_argument(
            "sensor '" + sensor_ + "': unknown setting '" + entries_[i].first +
            "'."
        );
      }
    }
  }

 private:
  float Value(std::string_view key, float fallback) {
    return Scalar(key, fallback);
  }
  domain::math::Vector3<float> Value(
      std::string_view key, const domain::math::Vector3<float>& fallback
  ) {
    return Vector(key, fallback);
  }

  static std::string Key(std::string_view prefix, std::string_view name) {
    return std::string(prefix) + std::string(name);
  }

  // The setting named `key`, marked read, checked to hold `count` numbers;
  // null when it isn't given.
  const std::vector<double>* Take(std::string_view key, std::size_t count) {
    for (std::size_t i = 0; i < entries_.size(); ++i) {
      if (entries_[i].first == key) {
        read_[i] = true;
        if (entries_[i].second.size() != count) {
          Fail(key, "takes " + std::to_string(count) + " number(s)");
        }
        return &entries_[i].second;
      }
    }
    return nullptr;
  }

  [[noreturn]] void Fail(std::string_view key, const std::string& why) const {
    throw std::invalid_argument(
        "sensor '" + sensor_ + "': setting '" + std::string(key) + "' " + why +
        "."
    );
  }

  std::string sensor_;
  Entries entries_;
  std::vector<bool> read_;
};

// The base every sensor kind derives from, declaring what it needs:
//   - Inputs: SimInputs<SimInput<...>...> -- sim state, read in place at the
//     mount joint;
//   - StateT: persistent per-instance state (a delay line, a drifting bias,
//     last tick's velocity, ...), trivially copyable; NoState if none;
//   - OutputT: what one sample is.
// The derived class itself is the instance's configuration: it's stored,
// whole, in the sim's own memory, so it must be trivially copyable and
// default-constructible. It provides
//
//   static constexpr std::string_view kListName = "...";  // its .arow name
//   static Self FromConfig(SensorConfig& config);          // its settings
//   void Read(const SensorTick&, const InputValueT<Inputs>&...,
//             StateT*, OutputT*) const;
//   void Initialize(const SensorTick&, StateT*) const;   // optional
//
// A scanning kind -- one that looks at every body, not just its own -- also
// declares
//
//   using Scene = SceneInputs<SimInput<...>...>;
//   void Observe(const SensorTick&, const InputValueT<Inputs>&...,
//                const InputValueT<SceneInputs>&..., StateT*) const;
//
// Observe runs, on a tick it samples, once per body (each body's scene
// values), accumulating into its state; Read then turns that into the
// reading.
//
// kListName is the list an archetype's attachments name to mount one (see
// domain::ArchetypeAttachment): every attachment under it is one instance.
//
// Read and Initialize are shaped like an Op (engine/op_contract.hpp):
// inputs by const reference, then everything written by pointer. Read runs
// only when a sample is due (see Due); Initialize once, at Init, to seed
// the state. Everything else -- storage, the passes that run it, the output
// buffer -- is generated from these declarations (sensing_data.hpp,
// sensing_ops.hpp, sensing_step.hpp). No virtuals: a sensor is only ever
// called through its own concrete type.
template <typename Derived, typename Inputs, typename StateT, typename OutputT>
class Sensor;

template <
    typename Derived,
    typename... Inputs,
    typename StateT,
    typename OutputT>
class Sensor<Derived, SimInputs<Inputs...>, StateT, OutputT> {
 public:
  using DerivedType = Derived;
  using SensorBase = Sensor;
  using InputList = util::TypeList<Inputs...>;
  static constexpr std::size_t kInputCount = sizeof...(Inputs);
  using State = StateT;
  using Output = OutputT;

  Sensor() = default;
  explicit Sensor(SensorOptions options) : options_(options) {}

  const SensorOptions& Options() const { return options_; }

  // This instance's random stream (SensorTick::stream), set by whatever
  // places it in the sim (interface::Simulation, from the sim's seed and
  // the instance's index), so no two instances draw the same noise.
  std::uint64_t Stream() const { return stream_; }
  void SetStream(std::uint64_t stream) { stream_ = stream; }

  // Whether a sample is due at `clock`, given the last one published:
  // always for the first, then once `period` has elapsed -- within half a
  // tick, so a period that's a whole number of ticks isn't missed to float
  // rounding.
  bool Due(const SensorClock& clock, float dt, const Reading<OutputT>& last)
      const {
    return last.sequence == 0 ||
           clock.time - last.sample_time >= options_.period - (0.5F * dt);
  }

 private:
  SensorOptions options_;
  std::uint64_t stream_ = 0;
};

namespace detail {

template <typename S, typename InputList>
struct SensorSignatures;

template <typename S, typename... Inputs>
struct SensorSignatures<S, util::TypeList<Inputs...>> {
  using ReadFn =
      void (S::*)(const SensorTick&, const InputValueT<Inputs>&..., typename S::State*, typename S::Output*)
          const;
  using InitializeFn = void (S::*)(const SensorTick&, typename S::State*) const;

  static constexpr bool kOnJoints =
      ((InputSource<Inputs>::kShared &&
        std::is_same_v<
            typename InputSource<Inputs>::Ordering,
            engine::topology::TopologicalOrdering>) &&
       ...);
};

template <typename S>
using SignaturesOf = SensorSignatures<S, typename S::InputList>;

template <typename S, typename InputList, typename SceneList>
struct SceneSignatures;

template <typename S, typename... Inputs, typename... SceneIns>
struct SceneSignatures<
    S,
    util::TypeList<Inputs...>,
    util::TypeList<SceneIns...>> {
  using ObserveFn =
      void (S::*)(const SensorTick&, const InputValueT<Inputs>&..., const InputValueT<SceneIns>&..., typename S::State*)
          const;

  static constexpr bool kOnJoints =
      ((InputSource<SceneIns>::kShared &&
        std::is_same_v<
            typename InputSource<SceneIns>::Ordering,
            engine::topology::TopologicalOrdering>) &&
       ...);
};

template <typename T>
concept Storable =
    std::is_trivially_copyable_v<T> && std::is_default_constructible_v<T>;

}  // namespace detail

// A sensor kind: derives from its own Sensor<S, ...>, names its .arow list
// (kListName), is storable as a record (trivially copyable,
// default-constructible -- as are its state and output), reads at least
// one sim field and only shared, joint-ordered ones (its mount joint is a
// row of TopologicalOrdering), and declares Read with exactly the parameter
// list its declarations imply (reordering or taking by value fails here,
// not deep inside the generated Op).
template <typename S>
concept SensorLike =
    requires {
      typename S::SensorBase;
      typename S::DerivedType;
    } && engine::topology::ListTagLike<S> &&
    std::same_as<typename S::DerivedType, S> &&
    std::derived_from<S, typename S::SensorBase> && detail::Storable<S> &&
    detail::Storable<typename S::State> &&
    detail::Storable<typename S::Output> && (S::kInputCount > 0) &&
    detail::SignaturesOf<S>::kOnJoints && requires {
      static_cast<typename detail::SignaturesOf<S>::ReadFn>(&S::Read);
    } && requires(SensorConfig& config) {
      { S::FromConfig(config) } -> std::same_as<S>;
    };

// One of kind S's records, as stored in the sensing algorithm's segmented
// fields (sensing_data.hpp): R itself, tagged with its kind, so two kinds
// whose records share a type (two that keep NoState, say) still name
// different segments. Same size and bytes as R.
template <typename S, typename R>
struct KindRecord {
  R value{};
};

// The optional Initialize, matched the same exact way.
template <typename S>
concept SensorHasInit = SensorLike<S> && requires {
  static_cast<typename detail::SignaturesOf<S>::InitializeFn>(&S::Initialize);
};

// A scanning sensor kind: a SensorLike that declares a Scene of shared,
// joint-ordered fields and an Observe matching its declarations exactly.
template <typename S>
concept SensorHasScene =
    SensorLike<S> && requires { typename S::Scene::List; } &&
    detail::SceneSignatures<S, typename S::InputList, typename S::Scene::List>::
        kOnJoints &&
    requires {
      static_cast<typename detail::SceneSignatures<
          S,
          typename S::InputList,
          typename S::Scene::List>::ObserveFn>(&S::Observe);
    };

// Whether an instance samples this tick: a scanning kind's was decided
// before its Observe ran (SensorClock::due, set by the gate); any other
// kind decides here.
template <SensorLike S>
bool IsDue(
    const S& /*sensor*/,
    const SensorClock& clock,
    float /*dt*/,
    const Reading<typename S::Output>& /*last*/
)
  requires SensorHasScene<S>
{
  return clock.due;
}

template <SensorLike S>
bool IsDue(
    const S& sensor,
    const SensorClock& clock,
    float dt,
    const Reading<typename S::Output>& last
)
  requires(!SensorHasScene<S>)
{
  return sensor.Due(clock, dt, last);
}

// Seeds an instance's state at Init: its kind's own Initialize if it
// declares one; otherwise the state stays value-initialized.
template <SensorLike S>
void InitializeState(const S& sensor, typename S::State* state)
  requires SensorHasInit<S>
{
  sensor.Initialize(SensorTick{0.0F, 0.0F, 0, sensor.Stream()}, state);
}

template <SensorLike S>
void InitializeState(const S& /*sensor*/, typename S::State* /*state*/)
  requires(!SensorHasInit<S>)
{}

// A kind's records as the sensing algorithm stores them, and its inputs --
// what sensor_kinds.hpp collects across every registered kind.
template <typename S>
using KindState = KindRecord<S, typename S::State>;
template <typename S>
using KindOutput = KindRecord<S, Reading<typename S::Output>>;
template <typename S>
using KindInputs = typename S::InputList;

// A kind's scene inputs: none unless it scans.
template <typename S>
struct KindSceneOf {
  using Type = util::TypeList<>;
};
template <SensorHasScene S>
struct KindSceneOf<S> {
  using Type = typename S::Scene::List;
};
template <typename S>
using KindScene = typename KindSceneOf<S>::Type;

// SensorHasScene as a type trait, for engine::pass::SegmentOps' filter.
template <typename S>
struct ScansScene : std::bool_constant<SensorHasScene<S>> {};

}  // namespace achilles::algorithms::sensing
