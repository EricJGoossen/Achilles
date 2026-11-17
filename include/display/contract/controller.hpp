#pragma once

#include <concepts>
#include <cstddef>
#include <optional>
#include <tuple>
#include <utility>

#include "display/contract/input.hpp"
#include "interface/writable_handle.hpp"

namespace achilles::display {

// A controller: what steers the sim from a window. It draws nothing -- it
// rides on a display's window (WindowSet::Control), taking what that window
// hands back each frame (Feed) -- and writes through the handles it was made
// with before every Step() (ApplyWrites): a writable state handle
// (interface/state_handle.hpp) to set a joint's value outright, or an
// actuator's handle to command it. Whatever a display shows, a controller
// can ride on its window: a camera's, its depth's, a plot's.
class IController {
 public:
  virtual ~IController() = default;

  // Takes this frame's input: what to write until the next frame.
  virtual void Feed(const Input& input) = 0;
  // Writes it, before every Step().
  virtual void ApplyWrites() const = 0;

 protected:
  // Only as a part of a controller: copied or moved with it, never sliced
  // off.
  IController() = default;
  IController(const IController&) = default;
  IController& operator=(const IController&) = default;
  IController(IController&&) = default;
  IController& operator=(IController&&) = default;
};

// What every controller over handles H... shares: it holds them, and what
// Control() last said to write through each -- written again before every
// Step() until the next frame, so a truth actuator's one-tick effort lasts
// the frame, and a written joint stays put. A controller derives from it and
// writes only Control(); its public members are its settings, changeable
// between frames (WindowSet::FindController).
template <interface::WritableHandle... H>
class Controller : public IController {
 public:
  // What to write through each handle, in order: an empty one leaves its
  // handle alone, so a controller with nothing to say -- its window
  // unfocused, say -- neither holds a joint nor zeroes a motor.
  using Writes = std::tuple<std::optional<typename H::Written>...>;

  void Feed(const Input& input) final { writes_ = Control(input); }

  void ApplyWrites() const final { WriteEach(std::index_sequence_for<H...>{}); }

 protected:
  explicit Controller(H... handles) : handles_(std::move(handles)...) {}

  // What to write this frame, from `input`, reading whatever it needs
  // through its handles -- never writing them itself.
  virtual Writes Control(const Input& input) const = 0;

  // Its I-th handle, to read through.
  template <std::size_t I>
  const auto& Handle() const {
    return std::get<I>(handles_);
  }

 private:
  template <std::size_t... Is>
  void WriteEach(std::index_sequence<Is...> /*indices*/) const {
    (
        [&] {
          if (std::get<Is>(writes_).has_value()) {
            std::get<Is>(handles_).Write(*std::get<Is>(writes_));
          }
        }(),
        ...
    );
  }

  std::tuple<H...> handles_;
  Writes writes_{};
};

// A controller a window can be steered by.
template <typename C>
concept ControllerLike =
    std::derived_from<C, IController> && std::move_constructible<C>;

}  // namespace achilles::display
