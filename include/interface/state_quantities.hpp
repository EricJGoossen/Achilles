#pragma once

#include <type_traits>

#include "domain/attachment/sim_quantities.hpp"
#include "util/tmp.hpp"

// Which sim quantities a state handle (interface/state_handle.hpp) reaches,
// and which of them it can write: the whole of what a caller can do with
// one, listed by what each quantity is (domain/attachment/sim_quantities.hpp),
// never by what in the sim keeps it.
//
// Every quantity listed here can be read; only those marked ReadWrite can be
// written. To reach another value, give it a quantity and list it here.
namespace achilles::interface {

namespace input = domain::attachment::input;

// A quantity that can be read and written: state the sim keeps from tick to
// tick, so a value written stays until the sim itself moves it on.
template <typename Quantity>
struct ReadWrite {
  using Type = Quantity;
  static constexpr bool kWritable = true;
};

// A quantity that can only be read: one the sim works out again every tick
// from its state (a write would be overwritten, and contradict what it's
// worked out from), or one it caches as it starts.
template <typename Quantity>
struct ReadOnly {
  using Type = Quantity;
  static constexpr bool kWritable = false;
};

// Every quantity a state handle reaches, and whether it writes it.
using StateQuantities = util::TypeList<
    ReadWrite<input::JointPosition>,
    ReadWrite<input::JointVelocity>,
    ReadOnly<input::JointAcceleration>,
    ReadOnly<input::JointSubspace>,
    ReadOnly<input::WorldTransform>,
    ReadOnly<input::SpatialVelocity>,
    ReadOnly<input::SpatialAcceleration>,
    ReadOnly<input::CollisionPose>,
    ReadOnly<input::CollisionExtents>,
    ReadOnly<input::CollisionRadius>,
    ReadWrite<input::VisualPose>,
    ReadWrite<input::VisualExtents>,
    ReadWrite<input::VisualColor>,
    ReadWrite<input::VisualShape>>;

namespace detail {

template <typename Q, typename List>
struct Listing {
  static constexpr bool kReadable = false;
  static constexpr bool kWritable = false;
};
template <typename Q, typename Entry, typename... Rest>
struct Listing<Q, util::TypeList<Entry, Rest...>> {
  using Next = Listing<Q, util::TypeList<Rest...>>;
  static constexpr bool kHere = std::is_same_v<typename Entry::Type, Q>;
  static constexpr bool kReadable = kHere || Next::kReadable;
  static constexpr bool kWritable = kHere ? Entry::kWritable : Next::kWritable;
};

}  // namespace detail

// Whether a state handle can read quantity Q: whether it's listed at all.
template <typename Q>
inline constexpr bool kReadableV =
    detail::Listing<Q, StateQuantities>::kReadable;
// Whether a state handle can write quantity Q: listed ReadWrite.
template <typename Q>
inline constexpr bool kWritableV =
    detail::Listing<Q, StateQuantities>::kWritable;

}  // namespace achilles::interface
