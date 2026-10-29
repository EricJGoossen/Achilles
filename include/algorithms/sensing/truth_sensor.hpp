#pragma once

#include <cstddef>
#include <string_view>
#include <type_traits>
#include <utility>

#include "algorithms/sensing/sensor.hpp"
#include "engine/algorithm_contract.hpp"
#include "engine/memory/binding.hpp"
#include "engine/memory/buffering_policy.hpp"
#include "engine/topology/indexing_policy.hpp"
#include "engine/topology/ordering_policy.hpp"
#include "util/tmp.hpp"

namespace achilles::algorithms::sensing {

// An ideal sensor: reports one sim input exactly, with no error -- ground
// truth, for logging, for checking a real sensor model against, or for a
// controller that's allowed to cheat. Output is the input's own value type
// (e.g. Transform<float> for ABA's world_transform), and it's attached in
// an .arow under the input field's own name (its kName).
template <typename Input>
class TruthSensor : public Sensor<
                        TruthSensor<Input>,
                        SimInputs<Input>,
                        NoState,
                        InputValueT<Input>> {
  using Base =
      Sensor<TruthSensor<Input>, SimInputs<Input>, NoState, InputValueT<Input>>;

 public:
  using Value = InputValueT<Input>;
  static constexpr std::string_view kListName = InputSource<Input>::Type::kName;

  TruthSensor() = default;
  explicit TruthSensor(SensorOptions options) : Base(options) {}

  // Takes only the options every sensor has.
  static TruthSensor FromConfig(SensorConfig& config) {
    return TruthSensor(config.Options());
  }

  void Read(
      const SensorTick& /*tick*/,
      const Value& truth,
      NoState* /*state*/,
      Value* out
  ) const {
    *out = truth;
  }
};

// The truth sensor of AlgorithmT's field F -- e.g.
// TruthOf<aba::ABAAlgorithm, aba::ABAField::kWorldTransform>, attached in
// an .arow as kind `world_transform`.
template <engine::AlgorithmLike AlgorithmT, typename AlgorithmT::Enum F>
using TruthOf = TruthSensor<SimInput<AlgorithmT, F>>;

namespace detail {

// A field a truth sensor can read: named (kName -- what it's attached
// under), shared (so the sensing algorithm reads its block in place),
// one value per joint, single-buffered, and a value that fits in a
// Reading.
template <typename Input>
concept TruthSensableLike =
    requires {
      {
        InputSource<Input>::Type::kName
      } -> std::convertible_to<std::string_view>;
    } && InputSource<Input>::kShared &&
    std::is_same_v<
        typename InputSource<Input>::Ordering,
        engine::topology::TopologicalOrdering> &&
    std::is_same_v<
        typename InputSource<Input>::Indexing,
        engine::topology::RowIndexing> &&
    std::is_same_v<
        typename InputSource<Input>::Buffering,
        engine::memory::SingleBuffered> &&
    std::is_trivially_copyable_v<InputValueT<Input>> &&
    std::is_default_constructible_v<InputValueT<Input>>;

template <typename Input>
struct TruthInputOf {
  using Type = util::TypeList<>;
};
template <TruthSensableLike Input>
struct TruthInputOf<Input> {
  using Type = util::TypeList<Input>;
};

template <typename AlgorithmT, typename Fields>
struct TruthInputsIn;

template <
    engine::FieldEnumLike EnumT,
    template <EnumT>
    class Traits,
    std::size_t... Is>
struct TruthInputsIn<
    engine::Algorithm<EnumT, Traits>,
    std::index_sequence<Is...>> {
  using Type = util::ConcatT<typename TruthInputOf<
      SimInput<engine::Algorithm<EnumT, Traits>, static_cast<EnumT>(Is)>>::
                                 Type...>;
};

template <typename AlgorithmT>
using TruthInputsOf = typename TruthInputsIn<
    AlgorithmT,
    std::make_index_sequence<static_cast<std::size_t>(AlgorithmT::Enum::kCount
    )>>::Type;

// Keeps the first input of each name: fields sharing a block share its
// name (ABA's and VI's joint_velocity are one block), so one truth sensor
// covers them.
template <typename Kept, typename Rest>
struct FirstOfEachName {
  using Type = Kept;
};

template <typename... Kept, typename Next, typename... Rest>
struct FirstOfEachName<util::TypeList<Kept...>, util::TypeList<Next, Rest...>> {
  using Type = typename FirstOfEachName<
      std::conditional_t<
          ((InputSource<Kept>::Type::kName == InputSource<Next>::Type::kName) ||
           ...),
          util::TypeList<Kept...>,
          util::TypeList<Kept..., Next>>,
      util::TypeList<Rest...>>::Type;
};

}  // namespace detail

// One truth sensor per named, shared, joint-ordered field across
// Algorithms (a util::TypeList), one per name -- so any such field can be
// sensed by attaching a sensor under its name, with nothing listed by hand.
template <typename Algorithms>
using TruthSensorsOf = util::TransformT<
    TruthSensor,
    typename detail::FirstOfEachName<
        util::TypeList<>,
        util::ApplyT<
            util::ConcatT,
            util::TransformT<detail::TruthInputsOf, Algorithms>>>::Type>;

}  // namespace achilles::algorithms::sensing
