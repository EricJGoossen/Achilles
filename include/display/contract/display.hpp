#pragma once

#include <array>
#include <concepts>
#include <cstddef>
#include <string_view>
#include <tuple>
#include <type_traits>

#include "display/contract/frame.hpp"
#include "display/contract/input.hpp"
#include "domain/actuation/truth_actuator.hpp"
#include "domain/attachment/attachment_kind.hpp"
#include "interface/attachment_handle.hpp"
#include "util/tmp.hpp"

namespace achilles::display {

// An actuator a display can drive: one taking a plain effort (N for a linear
// one, N*m for an angular one), added to what its next tick applies. Displays
// only ever push efforts -- no loop closes inside one.
template <typename K>
concept EffortActuator =
    domain::attachment::ActuatorLike<K> &&
    std::same_as<typename K::Command, domain::actuation::EffortCommand>;

namespace detail {

template <typename List>
struct HandlesFor;
template <typename... Ks>
struct HandlesFor<util::TypeList<Ks...>> {
  using Type = std::tuple<interface::AttachmentHandle<Ks>...>;
};

// What a display may be bound to: one that shows, any attachments --
// sensors, and actuators read without being touched; one that drives, only
// actuators it can give an effort to. Nothing but an actuator's own driving
// display ever pushes on it.
template <typename List>
inline constexpr bool kShowableV = false;
template <typename... Ks>
inline constexpr bool kShowableV<util::TypeList<Ks...>> =
    sizeof...(Ks) > 0 && (domain::attachment::AttachmentLike<Ks> && ...);
template <typename List>
inline constexpr bool kDrivableV = false;
template <typename... Ks>
inline constexpr bool kDrivableV<util::TypeList<Ks...>> = sizeof...(Ks) > 0 &&
                                                          (EffortActuator<Ks> &&
                                                           ...);

}  // namespace detail

// The handles a display of attachment kinds `List` is bound to, one per
// kind, in order.
template <typename List>
using HandlesForT = typename detail::HandlesFor<List>::Type;

// The efforts a driving display asks of its actuators this frame: one per
// attachment, in order -- a display that drives is bound only to actuators.
template <typename List>
using EffortsForT = std::array<float, util::kSizeV<List>>;

// Whether a display drives its attachments: it turns its controls into
// efforts. One that doesn't only shows them.
template <typename D>
concept Drives =
    requires(const D& display, const Input& input) { display.Efforts(input); };

// A display: knows how to draw one or more specific attachments -- given
// as kinds, so a display can never be bound to the wrong ones (a camera's to
// an encoder) -- and nothing about windows or how a frame gets on screen.
// It either shows its attachments, whatever they are -- a sensor's readings,
// or what an actuator last applied, read without touching it -- or drives
// them: actuators only, each given an effort from its controls. A display
// that shows has no say over anything.
// Plain data: its members are its settings, changeable between frames
// through DisplaySet::Find. It provides
//
//   using Attachments = util::TypeList<K...>;  // what it's bound to
//   static constexpr std::string_view kName;   // what it is
//   static constexpr float kWidth, kHeight;    // its window, to start
//   void Draw(const HandlesForT<Attachments>&, Frame*) const;
//
// and, for a display that drives (K... all EffortActuators),
//
//   EffortsForT<Attachments> Efforts(const Input&) const;
//
// Draw fills the frame -- emptied first -- from its attachments' latest
// readings, read through their handles, and lays out its controls; Efforts
// turns what those controls were left at into an effort per actuator, which
// is applied before every Step() until the controls next change.
template <typename D>
concept DisplayLike =
    std::is_default_constructible_v<D> && std::is_copy_constructible_v<D> &&
    requires {
      typename D::Attachments;
      { D::kName } -> std::convertible_to<std::string_view>;
      { D::kWidth } -> std::convertible_to<float>;
      { D::kHeight } -> std::convertible_to<float>;
    } &&
    requires(
        const D& display,
        const HandlesForT<typename D::Attachments>& handles,
        Frame* frame
    ) { display.Draw(handles, frame); } &&
    ((!Drives<D> && detail::kShowableV<typename D::Attachments>) ||
     (detail::kDrivableV<typename D::Attachments> &&
      requires(const D& display, const Input& input) {
        {
          display.Efforts(input)
        } -> std::same_as<EffortsForT<typename D::Attachments>>;
      }));

// A display's handles and efforts, by the display.
template <DisplayLike D>
using HandlesOfT = HandlesForT<typename D::Attachments>;
// None for a display that only shows.
template <DisplayLike D>
using EffortsOfT = std::conditional_t<
    Drives<D>,
    EffortsForT<typename D::Attachments>,
    std::array<float, 0>>;

// Display D's efforts for `input`: none for a display that only shows.
template <DisplayLike D>
EffortsOfT<D> EffortsFrom(const D& display, const Input& input) {
  if constexpr (Drives<D>) {
    return display.Efforts(input);
  } else {
    return {};
  }
}

}  // namespace achilles::display
