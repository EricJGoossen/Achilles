#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

// What a window hands back each frame, for its controllers: the other half of
// the contract with it (frame.hpp is what its display hands the window).
// Plain data -- no GL, no GUI library.
namespace achilles::display {

// A key a controller can ask about: the ones a view is steered by. To ask
// about another, add it here and to window.cpp's table of them.
enum class Key : std::uint8_t {
  kW,
  kA,
  kS,
  kD,
  kQ,
  kE,
  kR,
  kF,
  kUp,
  kDown,
  kLeft,
  kRight,
  kSpace,
  kShift,
  kControl,
  kEscape,
};
inline constexpr std::size_t kKeyCount = 16;

// A mouse button.
enum class Button : std::uint8_t { kLeft, kRight, kMiddle };
inline constexpr std::size_t kButtonCount = 3;

// The mouse, as it met a window's drawing area this frame, in the frame's
// own coordinates: [0, 1] across the area from its top-left, y down, so a
// controller never sees the window's pixels.
struct Pointer {
  // Public on purpose: plain data.
  // Whether it's over the drawing area. A button pressed there keeps the
  // pointer the area's while it's held -- a drag -- wherever it goes.
  bool over = false;
  float x = 0.0F;
  float y = 0.0F;
  // How far it moved since the last frame, in the same units.
  float dx = 0.0F;
  float dy = 0.0F;
  // Wheel notches this frame: positive away from the user.
  float scroll = 0.0F;
  std::array<bool, kButtonCount> held{};
  std::array<bool, kButtonCount> pressed{};
  std::array<bool, kButtonCount> released{};

  bool Held(Button button) const { return held[Index(button)]; }
  bool Pressed(Button button) const { return pressed[Index(button)]; }
  bool Released(Button button) const { return released[Index(button)]; }

 private:
  static std::size_t Index(Button button) {
    return static_cast<std::size_t>(button);
  }
};

// What a window hands back each frame: how long the frame took; and, while
// it's the focused window, the keyboard, and the mouse over its drawing area.
// A window that isn't focused reports no keys, and the mouse only while it's
// over its area. What it's handed to is the window's controllers
// (controller.hpp): a display only draws.
struct Input {
  // Public on purpose: plain data, which its methods only read.
  // Wall-clock seconds since the window's last frame, so what's steered by
  // a key held moves at the same speed whatever the frame rate.
  float seconds = 0.0F;
  bool focused = false;
  std::array<bool, kKeyCount> keys_held{};
  std::array<bool, kKeyCount> keys_pressed{};
  Pointer pointer;

  // Whether `key` is down, or went down this frame.
  bool Held(Key key) const { return keys_held[Index(key)]; }
  bool Pressed(Key key) const { return keys_pressed[Index(key)]; }

 private:
  static std::size_t Index(Key key) { return static_cast<std::size_t>(key); }
};

}  // namespace achilles::display
