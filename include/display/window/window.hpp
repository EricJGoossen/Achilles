#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "display/contract/frame.hpp"
#include "display/contract/input.hpp"

namespace achilles::display {

// One display's window: its own window on the desktop, moved, resized and
// closed there, drawing whatever Frame it's handed. It holds no display -- a
// window set pairs each with its display and controllers -- only what drawing
// needs between frames: its textures, its plots' histories, whether its user
// has closed it. Every window is this one type, whatever it shows. The GUI
// makes its desktop window when it first draws, and takes it down once it's
// closed or destroyed.
//
// Present() runs only between the screen's (screen.hpp) BeginFrame() and
// EndFrame(); the textures it makes belong to the screen's shared GL
// context, current there, which must also be current when a window that
// drew is destroyed (Renderer sees to both).
class Window {
 public:
  // `title` is shown on it; two windows may share one.
  Window(std::string_view title, float width, float height);
  ~Window();

  Window(const Window&) = delete;
  Window& operator=(const Window&) = delete;
  Window(Window&& other) noexcept;
  Window& operator=(Window&& other) noexcept;

  // Draws `frame` as this window's contents this frame: its text, its
  // drawing area letterboxed to its aspect, then its traces' plots. Returns
  // how long the frame took, and the keyboard and mouse as they met this
  // window.
  Input Present(const Frame& frame);

  // False once its user has closed it.
  bool IsOpen() const { return open_; }

  const std::string& Title() const { return title_; }

 private:
  void FreeTextures();
  void Upload(std::size_t index, const Image& image);
  // The frame's drawing area.
  void DrawArea(const Frame& frame);
  // Records the mouse over the area just claimed -- `width` by `height`
  // from (`left`, `top`), on the screen -- as pointer_.
  void TrackPointer(float left, float top, float width, float height);
  void DrawTraces(const Frame& frame);

  // One trace's recent values, oldest first from `next` once full: a ring
  // of kTraceSamples samples -- about ten seconds of a sensor sampled every
  // frame at 30 frames a second, or longer of one sampling slower.
  static constexpr std::size_t kTraceSamples = 300;
  struct History {
    std::vector<float> values;
    std::size_t next = 0;
    double last_time = -1.0;
  };
  // Adds `trace`'s value to `history` if it's a new sample.
  static void Record(const Trace& trace, History* history);
  // A trace plotted on its own, then traces sharing a plot (by index into
  // frame.traces), one line each over one scale, with a legend.
  static void DrawOwnPlot(
      const Trace& trace, const History& history, std::size_t index
  );
  void DrawSharedPlot(
      const Frame& frame, const std::vector<std::size_t>& members
  );

  std::string title_;
  // The title plus a tag unique to this window, so two windows of one title
  // are still two windows.
  std::string label_;
  float width_ = 0.0F;
  float height_ = 0.0F;
  // Whether it has been placed on the screen yet: where a window first
  // opens is worked out when it first draws (see window.cpp's Place).
  bool placed_ = false;
  bool open_ = true;
  // The mouse over the drawing area, as DrawArea() last found it.
  Pointer pointer_;
  std::vector<History> histories_;
  // GL texture names, one per image of the last frame drawn (GLuint).
  std::vector<std::uint32_t> textures_;
  // An image of values, turned gray before it's uploaded.
  std::vector<std::uint8_t> gray_;
};

}  // namespace achilles::display
