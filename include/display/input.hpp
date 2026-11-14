#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

// What a window hands back to its display each frame: the other half of the
// contract between the two (frame.hpp is what the display hands the window).
// Plain data -- no GL, no GUI library.
namespace achilles::display {

// A key a display can ask about: the ones a view is steered by. To ask
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
// display never sees the window's pixels.
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

// What a window hands back each frame: what its controls were left at, one
// value per slider of the frame it drew, in order; and, while it's the
// focused window, the keyboard, and the mouse over its drawing area. A
// window that isn't focused reports no keys, and the mouse only while it's
// over its area.
struct Input {
  // Public on purpose: plain data, which its methods only read.
  std::vector<float> values;
  bool focused = false;
  std::array<bool, kKeyCount> keys_held{};
  std::array<bool, kKeyCount> keys_pressed{};
  Pointer pointer;

  // The value of slider `index`, or 0 for one the window hasn't drawn yet.
  float Value(std::size_t index) const {
    return index < values.size() ? values[index] : 0.0F;
  }
  // Whether `key` is down, or went down this frame.
  bool Held(Key key) const { return keys_held[Index(key)]; }
  bool Pressed(Key key) const { return keys_pressed[Index(key)]; }

 private:
  static std::size_t Index(Key key) { return static_cast<std::size_t>(key); }
};

}  // namespace achilles::display
