#pragma once

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "engine/algorithm_contract.hpp"
#include "engine/assembler.hpp"
#include "engine/memory/binding.hpp"
#include "engine/topology/ordering_policy.hpp"
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
  std::uint64_t tick = 0;
};

// Everything about "now" a sensor's Read receives. stream and tick key a
// counter-based random generator (domain::random::CounterRng): a draw
// depends only on (stream, tick, draw index), never on what ran before it,
// so readings are reproducible whichever thread runs which instance.
// stream identifies the instance (and, once Init takes one, the sim's
// seed); it's 0 until instances carry their own index.
struct SensorTick {
  float dt = 0.0F;
  float time = 0.0F;
  std::uint64_t tick = 0;
  std::uint64_t stream = 0;
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
//   void Read(const SensorTick&, const InputValueT<Inputs>&...,
//             StateT*, OutputT*) const;
//   void Initialize(const SensorTick&, StateT*) const;   // optional
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

// Seeds an instance's state at Init: its kind's own Initialize if it
// declares one; otherwise the state stays value-initialized.
template <SensorLike S>
void InitializeState(const S& sensor, typename S::State* state)
  requires SensorHasInit<S>
{
  sensor.Initialize(SensorTick{}, state);
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

}  // namespace achilles::algorithms::sensing
