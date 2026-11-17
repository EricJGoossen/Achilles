#pragma once

#include <cstddef>
#include <tuple>
#include <utility>

#include "display/contract/display.hpp"
#include "display/contract/frame.hpp"
#include "display/contract/input.hpp"

namespace achilles::display {

// An attached display, whatever its kind: what lets displays of every kind
// be kept together (DisplaySet), behind one virtual call each a frame.
class IAttachedDisplay {
 public:
  IAttachedDisplay() = default;
  IAttachedDisplay(const IAttachedDisplay&) = delete;
  IAttachedDisplay& operator=(const IAttachedDisplay&) = delete;
  IAttachedDisplay(IAttachedDisplay&&) = delete;
  IAttachedDisplay& operator=(IAttachedDisplay&&) = delete;
  virtual ~IAttachedDisplay() = default;

  // Fills `frame`, emptied first by the caller, from its handles.
  virtual void Draw(Frame* frame) const = 0;
  // Takes what its window handed back this frame as the efforts to apply
  // from the next ApplyEfforts() on: none for a display that only shows.
  virtual void Feed(const Input& input) = 0;
  // Adds its efforts to what its actuators apply on the next tick.
  virtual void ApplyEfforts() const = 0;
};

// A display attached to the handles it shows or drives: the display itself
// (its settings), its handles, and the efforts it last asked for. What
// attaching a display means, apart from where it's kept or which window it
// draws in (DisplaySet does those): it draws itself from its handles, turns
// what its window hands back into efforts, and pushes those onto its
// actuators.
template <DisplayLike D>
class AttachedDisplay final : public IAttachedDisplay {
 public:
  AttachedDisplay(D display, HandlesOfT<D> handles)
      : display_(std::move(display)), handles_(std::move(handles)) {}

  // Its settings, to change from the next frame on.
  D& Display() { return display_; }
  const D& Display() const { return display_; }

  void Draw(Frame* frame) const override { display_.Draw(handles_, frame); }

  void Feed(const Input& input) override {
    efforts_ = EffortsFrom(display_, input);
  }

  // Only a display that drives pushes on anything: one that shows an
  // actuator only reads it.
  void ApplyEfforts() const override {
    if constexpr (Drives<D>) {
      ApplyEach(std::make_index_sequence<std::tuple_size_v<HandlesOfT<D>>>{});
    }
  }

 private:
  // A driving display is bound only to actuators: effort i goes to handle i.
  template <std::size_t... Is>
  void ApplyEach(std::index_sequence<Is...> /*indices*/) const {
    (std::get<Is>(handles_).Apply(efforts_[Is]), ...);
  }

  D display_;
  HandlesOfT<D> handles_;
  EffortsOfT<D> efforts_{};
};

}  // namespace achilles::display
