#pragma once

#include <concepts>
#include <cstddef>
#include <type_traits>
#include <utility>

#include "algorithms/attachment/attachment_data.hpp"
#include "algorithms/attachment/attachment_step.hpp"
#include "algorithms/attachment_kinds/sim_input.hpp"
#include "domain/attachment/sim_quantities.hpp"
#include "engine/assembler.hpp"
#include "engine/pass/sim_context.hpp"
#include "interface/state_quantities.hpp"

namespace achilles::interface {

class Simulation;

namespace detail {

// The field of the running sim holding quantity Q
// (algorithms/attachment_kinds/sim_input.hpp): another algorithm's field as
// it is, an attachment's own input as the attachment algorithm's field
// holding it. Only the handle itself names it.
template <typename Bound>
struct FieldOfBound {
  using Source = typename Bound::Source;
  static constexpr auto kField = Bound::kField;
};
template <typename Quantity>
struct FieldOfBound<algorithms::attachment::OwnInput<Quantity>> {
  using Source = algorithms::attachment::AttachmentAlgorithm;
  static constexpr auto kField =
      algorithms::attachment::kOwnFieldOf<Quantity>;
};
template <typename Q>
using FieldOfT = FieldOfBound<algorithms::attachment::BoundInputT<Q>>;

}  // namespace detail

// One joint's value of one sim quantity, read and written in place from
// outside the sim: S names the quantity (domain/attachment/sim_quantities.hpp
// -- input::JointPosition), one of those a state handle reaches
// (interface/state_quantities.hpp). Found once (Simulation::State)
// and kept: cheap to copy -- copies share the value -- and valid until the
// Simulation that made it is destroyed or re-Init()ed.
//
// A raw view, for debugging and logging: a read loads the sim's own field,
// with none of the copying into a double buffer a truth sensor does, so it is
// neither synchronized nor thread safe. Use one only between Step()s, on the
// thread that steps the sim; a truth sensor is the synchronized way to read
// the same quantities.
//
// StateHandle<const S> only reads: a handle meant to look, never to touch.
// Any listed quantity can be read; only those listed ReadWrite can be had
// writable. A writable handle converts to a const one, as a pointer does to a
// pointer to const.
//
// A read gives the field as the last Step() left it -- for a value the sim
// works out each tick from its state (a body's world pose), as it worked it
// out at that step's start, from the state before the step moved it on: one
// step behind the joint state read beside it. A write lands in the field at
// once, between steps, and the sim moves on from it at the next Step():
// whatever the sim works out from it -- world poses, contacts, every
// sensor's reading -- catches up then. Nothing checks a write is one the sim
// can stand: a pose off a joint's freedoms, a body moved into another. A
// write is an input to the run, like a command, so a run is only reproduced
// if its writes are too.
template <typename S>
  requires kReadableV<std::remove_const_t<S>>
class StateHandle {
  using Spec = std::remove_const_t<S>;
  using FieldT = detail::FieldOfT<Spec>;
  using Source = typename FieldT::Source;
  using View = typename Source::View;
  static constexpr auto kField = FieldT::kField;
  using Traits = algorithms::attachment::detail::SourceTraitsOf<Source, kField>;
  // What a row of the field reads and writes as: the sim's scalar.
  using Shape = typename Traits::Type::Assembler::ScalarType;

 public:
  // The quantity it's a handle on.
  using Quantity = Spec;
  // The field's value, as the sim keeps it (e.g. Transform<float> for a
  // joint's position).
  using Value = engine::AssembledT<typename Traits::Type::Assembler>;
  // Whether this handle may write.
  static constexpr bool kWritable = !std::is_const_v<S>;

  // The read-only handle on the same value.
  template <typename Other>
    requires(!kWritable && std::same_as<const Other, S> &&
             !std::is_const_v<Other>)
  // NOLINTNEXTLINE(google-explicit-constructor) -- as T* converts to const T*.
  StateHandle(const StateHandle<Other>& writable)
      : view_(writable.view_), row_(writable.row_) {}

  // The value, as the last Step() left it.
  Value Read() const { return view_.template Load<kField, Shape>(row_); }

  // Replaces the value, from the next Step() on.
  void Write(const Value& value) const
    requires kWritable
  {
    view_.template Store<kField, Shape>(row_, value);
  }

 private:
  // Made only by Simulation::State, from a joint's address: which row of
  // the sim's field that is stays inside the handle.
  friend class Simulation;
  template <typename T>
    requires kReadableV<std::remove_const_t<T>>
  friend class StateHandle;

  // The handle on row `row` of S's field: a joint's row, from the sim's
  // per-joint layout.
  template <engine::pass::SimContextLike SimStateT>
  static StateHandle At(const SimStateT& sim_state, std::size_t row) {
    return StateHandle(sim_state.template ViewFor<Source>(), row);
  }

  StateHandle(View view, std::size_t row) : view_(std::move(view)), row_(row) {}

  View view_;
  std::size_t row_;
};

}  // namespace achilles::interface
