#pragma once

#include <type_traits>

#include "algorithms/aba/aba_step.hpp"
#include "algorithms/broad_phase/broad_phase_step.hpp"
#include "domain/attachment/sim_quantities.hpp"
#include "engine/algorithm_contract.hpp"
#include "engine/assembler.hpp"
#include "engine/memory/binding.hpp"

// What names sim data for an attachment: a field of a hosted Algorithm, and
// which field holds each quantity a kind can ask for
// (domain/attachment/sim_quantities.hpp) -- the one place an attachment's data
// meets the algorithm that computes it.
namespace achilles::algorithms::attachment {

namespace input = domain::attachment::input;

// One piece of sim state an attachment reads: field F of the hosted Algorithm
// AlgorithmT, at the joint it's mounted on. F must be shared
// (Traits<F>::SharedAs) and ordered by joint (TopologicalOrdering): the
// attachment algorithm names that same slot to read the exact block AlgorithmT
// keeps, never a copy (see AttachmentFieldTraits, attachment_data.hpp).
template <engine::AlgorithmLike AlgorithmT, typename AlgorithmT::Enum F>
struct SimInput {
  using Source = AlgorithmT;
  using Enum = typename AlgorithmT::Enum;
  static constexpr Enum kField = F;
};

// The SimInput a kind's declared input resolves to: a domain quantity's is
// given below, and a SimInput is its own.
template <typename Input>
struct InputBinding;

template <engine::AlgorithmLike AlgorithmT, typename AlgorithmT::Enum F>
struct InputBinding<SimInput<AlgorithmT, F>> {
  using Type = SimInput<AlgorithmT, F>;
};

template <typename Input>
using BoundInputT = typename InputBinding<Input>::Type;

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
using InputSource = detail::SourceTraitsOf<
    typename BoundInputT<Input>::Source,
    BoundInputT<Input>::kField>;

// What a kind receives for one input: the source field's value read at its
// own scalar -- e.g. Transform<float> for ABA's kWorldTransform.
template <typename Input>
using InputValueT =
    engine::AssembledT<typename InputSource<Input>::Type::Assembler>;

template <>
struct InputBinding<input::JointPosition> {
  using Type = SimInput<aba::ABAAlgorithm, aba::ABAField::kJointPosition>;
};
template <>
struct InputBinding<input::JointVelocity> {
  using Type = SimInput<aba::ABAAlgorithm, aba::ABAField::kJointVelocity>;
};
template <>
struct InputBinding<input::JointAcceleration> {
  using Type = SimInput<aba::ABAAlgorithm, aba::ABAField::kJointAcceleration>;
};
template <>
struct InputBinding<input::JointSubspace> {
  using Type = SimInput<aba::ABAAlgorithm, aba::ABAField::kJointSubspace>;
};
template <>
struct InputBinding<input::WorldTransform> {
  using Type = SimInput<aba::ABAAlgorithm, aba::ABAField::kWorldTransform>;
};
template <>
struct InputBinding<input::SpatialVelocity> {
  using Type = SimInput<aba::ABAAlgorithm, aba::ABAField::kSpatialVelocity>;
};
template <>
struct InputBinding<input::SpatialAcceleration> {
  using Type = SimInput<aba::ABAAlgorithm, aba::ABAField::kSpatialAcceleration>;
};
template <>
struct InputBinding<input::CollisionPose> {
  using Type = SimInput<
      broad_phase::BroadPhaseAlgorithm,
      broad_phase::BroadPhaseField::kCollisionPose>;
};
template <>
struct InputBinding<input::CollisionExtents> {
  using Type = SimInput<
      broad_phase::BroadPhaseAlgorithm,
      broad_phase::BroadPhaseField::kCollisionExtents>;
};
template <>
struct InputBinding<input::CollisionRadius> {
  using Type = SimInput<
      broad_phase::BroadPhaseAlgorithm,
      broad_phase::BroadPhaseField::kCollisionRadius>;
};

// A quantity's Value is what reading its field gives.
template <typename Quantity>
inline constexpr bool kValueOfItsField =
    std::is_same_v<typename Quantity::Value, InputValueT<Quantity>>;
static_assert(kValueOfItsField<input::JointPosition>);
static_assert(kValueOfItsField<input::JointVelocity>);
static_assert(kValueOfItsField<input::JointAcceleration>);
static_assert(kValueOfItsField<input::JointSubspace>);
static_assert(kValueOfItsField<input::WorldTransform>);
static_assert(kValueOfItsField<input::SpatialVelocity>);
static_assert(kValueOfItsField<input::SpatialAcceleration>);
static_assert(kValueOfItsField<input::CollisionPose>);
static_assert(kValueOfItsField<input::CollisionExtents>);
static_assert(kValueOfItsField<input::CollisionRadius>);

// A quantity named for a truth sensor goes by its field's own name.
template <typename Quantity>
inline constexpr bool kNamedAsItsField =
    InputSource<Quantity>::Type::kName == Quantity::kName;
static_assert(kNamedAsItsField<input::JointPosition>);
static_assert(kNamedAsItsField<input::JointVelocity>);
static_assert(kNamedAsItsField<input::JointAcceleration>);
static_assert(kNamedAsItsField<input::WorldTransform>);
static_assert(kNamedAsItsField<input::SpatialVelocity>);
static_assert(kNamedAsItsField<input::SpatialAcceleration>);

}  // namespace achilles::algorithms::attachment
