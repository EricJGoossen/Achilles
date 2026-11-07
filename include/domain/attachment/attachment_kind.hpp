#pragma once

#include <concepts>
#include <cstddef>
#include <string_view>
#include <type_traits>

#include "domain/actuation/actuator.hpp"
#include "domain/attachment/attachment.hpp"
#include "domain/attachment/attachment_config.hpp"
#include "domain/sensing/sensor.hpp"
#include "util/tmp.hpp"

// What a sensor or actuator kind must declare to be one.
namespace achilles::domain::attachment {

namespace detail {

template <typename S, typename InputList>
struct Signatures;

template <typename S, typename... Inputs>
struct Signatures<S, util::TypeList<Inputs...>> {
  using ReadFn =
      void (S::*)(const AttachmentTick&, const typename Inputs::Value&..., typename S::State*, typename S::Output*)
          const;
  using DriveFn =
      void (S::*)(const AttachmentTick&, const typename Inputs::Value&..., typename S::Command*, typename S::State*, typename S::Output*)
          const;
  using InitializeFn =
      void (S::*)(const AttachmentTick&, typename S::State*) const;
  using InitializeOutputFn =
      void (S::*)(const AttachmentTick&, const typename Inputs::Value&..., typename S::Output*)
          const;
};

template <typename S>
using SignaturesOf = Signatures<S, typename S::InputList>;

template <typename S, typename InputList, typename SceneList>
struct SceneSignatures;

template <typename S, typename... Inputs, typename... SceneIns>
struct SceneSignatures<
    S,
    util::TypeList<Inputs...>,
    util::TypeList<SceneIns...>> {
  using ObserveFn =
      void (S::*)(const AttachmentTick&, const typename Inputs::Value&..., const typename SceneIns::Value&..., typename S::State*)
          const;
};

template <typename T>
concept Storable =
    std::is_trivially_copyable_v<T> && std::is_default_constructible_v<T>;

// What every kind has: a name for its .arow list, records it can be stored as
// (trivially copyable, default-constructible), and settings.
template <typename S>
concept KindBasics =
    requires {
      { S::kListName } -> std::convertible_to<std::string_view>;
    } && Storable<S> && Storable<typename S::State> &&
    Storable<typename S::Command> && Storable<typename S::Output> &&
    requires(AttachmentConfig& config) {
      { S::FromConfig(config) } -> std::same_as<S>;
    };

}  // namespace detail

// A sensor kind: derives from its own Sensor<S, ...>, reads at least one
// quantity, and declares Read with exactly the parameter list its declarations
// imply (reordering or taking by value fails here, not deep inside the
// generated Op).
template <typename S>
concept SensorLike =
    requires {
      typename S::SensorBase;
      typename S::DerivedType;
    } && std::same_as<typename S::DerivedType, S> &&
    std::derived_from<S, typename S::SensorBase> && detail::KindBasics<S> &&
    (S::kInputCount > 0) && requires {
      static_cast<typename detail::SignaturesOf<S>::ReadFn>(&S::Read);
    };

// An actuator kind: derives from its own Actuator<S, ...>, names its effort,
// reports it, and declares Drive the same exact way.
template <typename S>
concept ActuatorLike =
    requires {
      typename S::ActuatorBase;
      typename S::DerivedType;
      { S::kEffort } -> std::convertible_to<actuation::EffortKind>;
    } && std::same_as<typename S::DerivedType, S> &&
    std::derived_from<S, typename S::ActuatorBase> && detail::KindBasics<S> &&
    requires(typename S::Output& out) {
      { out.effort } -> std::convertible_to<float>;
    } && requires {
      static_cast<typename detail::SignaturesOf<S>::DriveFn>(&S::Drive);
    };

template <typename S>
concept AttachmentLike = SensorLike<S> || ActuatorLike<S>;

// The optional Initialize, matched the same exact way.
template <typename S>
concept AttachmentHasInit = AttachmentLike<S> && requires {
  static_cast<typename detail::SignaturesOf<S>::InitializeFn>(&S::Initialize);
};

// The optional InitializeOutput, matched the same exact way.
template <typename S>
concept AttachmentHasOutputInit = AttachmentLike<S> && requires {
  static_cast<typename detail::SignaturesOf<S>::InitializeOutputFn>(
      &S::InitializeOutput
  );
};

// A scanning sensor kind: a SensorLike that declares a Scene and an Observe
// matching its declarations exactly.
template <typename S>
concept SensorHasScene =
    SensorLike<S> && requires { typename S::Scene::List; } && requires {
      static_cast<typename detail::SceneSignatures<
          S,
          typename S::InputList,
          typename S::Scene::List>::ObserveFn>(&S::Observe);
    };

// Seeds an instance's state from its configuration: its kind's own Initialize
// if it declares one; otherwise the state stays value-initialized.
template <AttachmentLike S>
void InitializeState(const S& kind, typename S::State* state) {
  if constexpr (AttachmentHasInit<S>) {
    kind.Initialize(AttachmentTick{0.0F, 0.0, 0, kind.Stream()}, state);
  }
}

// Fills an instance's first output from its inputs, if its kind has one.
template <AttachmentLike S, typename... Values>
void InitializeOutput(
    const S& kind, typename S::Output* out, const Values&... inputs
) {
  if constexpr (AttachmentHasOutputInit<S>) {
    kind.InitializeOutput(
        AttachmentTick{0.0F, 0.0, 0, kind.Stream()}, inputs..., out
    );
  }
}

}  // namespace achilles::domain::attachment
