#pragma once

#include <array>
#include <charconv>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <system_error>
#include <vector>

// What a display hands its window each frame: half the contract between the
// two (input.hpp is the other half, what the window hands back; see
// display.hpp for who fills a Frame, window.hpp for who draws it). Plain
// data -- no GL, no GUI library.
namespace achilles::display {

struct Color {
  float r = 0.0F;
  float g = 0.0F;
  float b = 0.0F;
};

// An axis-aligned area of the drawing area: its top-left corner and size.
struct Rect {
  float x = 0.0F;
  float y = 0.0F;
  float width = 1.0F;
  float height = 1.0F;
};

// A width x height image, rows top to bottom, stretched over `where`:
// either 8-bit RGB triples (`rgb`), or one value per pixel (`values`) drawn
// as gray -- white at 0, black at `scale` and beyond, and black where a value
// isn't finite (nothing there). The spans borrow the display's source -- a
// sensor's state, read in place -- until the frame is drawn.
struct Image {
  std::span<const std::uint8_t> rgb;
  std::span<const float> values;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  float scale = 1.0F;
  Rect where;
};

// Points (xy pairs) drawn `size` pixels across, in one color.
struct Points {
  Color color;
  float size = 3.0F;
  std::vector<float> xy;
};

// Line segments (x0, y0, x1, y1 each), in one color.
struct Lines {
  Color color;
  std::vector<float> xy;
};

// One channel of a reading, plotted over time: its value this frame, and
// the sim time it was sampled at. The window keeps each trace's recent
// values -- the n-th trace is the same channel every frame -- and adds one
// only when its sample time moves on, so a reading held between samples
// isn't plotted twice. A trace with no `plot` is a plot of its own, titled
// by its label; traces naming the same `plot` share one, titled by it, a
// line each, told apart by their labels.
struct Trace {
  std::string label;
  float value = 0.0F;
  double time = 0.0;
  std::string plot;
};

// One frame of a display. The drawing area's coordinates run [0, 1] x
// [0, 1] from its top-left, y down, laid out for `aspect` (width over
// height), which the window keeps by letterboxing; aspect 0 means no drawing
// area at all, only text and plots.
struct Frame {
  // Public on purpose: plain data, which Clear() only resets.
  float aspect = 0.0F;
  Color background{0.08F, 0.09F, 0.11F};
  std::vector<std::string> text;
  std::vector<Image> images;
  std::vector<Points> points;
  std::vector<Lines> lines;
  std::vector<Trace> traces;

  // Empties it for the next frame, keeping its capacity.
  void Clear() {
    aspect = 0.0F;
    background = Color{0.08F, 0.09F, 0.11F};
    text.clear();
    images.clear();
    points.clear();
    lines.clear();
    traces.clear();
  }
};

// `value` with `digits` decimals, for a display's text.
inline std::string Fixed(float value, int digits = 2) {
  std::array<char, 32> buffer{};
  const std::to_chars_result result = std::to_chars(
      buffer.data(),
      buffer.data() + buffer.size(),
      value,
      std::chars_format::fixed,
      digits
  );
  if (result.ec != std::errc{}) {
    return "?";
  }
  return {buffer.data(), result.ptr};
}

}  // namespace achilles::display
