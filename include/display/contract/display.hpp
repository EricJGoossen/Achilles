#pragma once

#include <concepts>
#include <string_view>

#include "display/contract/frame.hpp"

namespace achilles::display {

// A display: what a window draws. It's made with the handles of the
// attachments it shows -- a sensor's readings, or what an actuator last
// applied, only ever read -- taken by its constructor as exactly the kinds
// it knows how to draw, so a camera's display can't be made with an
// encoder's handle. It knows nothing about windows, or how a frame gets on
// screen, and has no say over the sim: what steers the sim from a window is
// a controller (controller.hpp), riding on a display's window.
//
// Draw fills the frame -- emptied first -- from its attachments' latest
// readings. Its public `settings`, if it has any, can be changed between
// frames (WindowSet::Find).
class IDisplay {
 public:
  virtual ~IDisplay() = default;

  virtual void Draw(Frame* frame) const = 0;

 protected:
  // Only as a part of a display: copied or moved with it, never sliced off.
  IDisplay() = default;
  IDisplay(const IDisplay&) = default;
  IDisplay& operator=(const IDisplay&) = default;
  IDisplay(IDisplay&&) = default;
  IDisplay& operator=(IDisplay&&) = default;
};

// A display a window can be opened for: one that also says what it is and
// how big its window starts.
//
//   static constexpr std::string_view kName;  // what it is
//   static constexpr float kWidth, kHeight;   // its window, to start
template <typename D>
concept DisplayLike =
    std::derived_from<D, IDisplay> && std::move_constructible<D> && requires {
      { D::kName } -> std::convertible_to<std::string_view>;
      { D::kWidth } -> std::convertible_to<float>;
      { D::kHeight } -> std::convertible_to<float>;
    };

}  // namespace achilles::display
