#pragma once

#include <string_view>
#include <utility>

struct GLFWwindow;

namespace achilles::render {

// Owns one GLFW window plus its GL 3.3 core context (created and made
// current in the constructor, which also calls gl::LoadGLFunctions --
// nothing else in this module may call a post-1.1 GL function before a
// Window exists). Move-only: a GLFWwindow* has exactly one owner, the
// same shape Shader/Mesh use for their own GL object handles.
//
// Input is polled, not callback-driven: PollEvents() refreshes this
// frame's cursor position/button state from GLFW, and CursorDelta/
// ScrollDelta/IsLeftMouseDown read back whatever that poll produced.
// scene_renderer.cpp's own per-frame loop is the only consumer, so a
// callback-based push model would just be indirection to get back to the
// same "check it once per frame" shape a simple viewer actually needs.
class Window {
 public:
  Window(int width, int height, std::string_view title);
  ~Window();

  Window(const Window&) = delete;
  Window& operator=(const Window&) = delete;
  Window(Window&& other) noexcept;
  Window& operator=(Window&& other) noexcept;

  bool ShouldClose() const;
  void PollEvents();
  void SwapBuffers();

  std::pair<int, int> FramebufferSize() const;
  float Aspect() const;

  // (dx, dy) since the last PollEvents call, in screen pixels -- zero on
  // the very first call, and zero whenever the left mouse button isn't
  // held (scene_renderer.cpp only ever orbits the camera while dragging).
  std::pair<float, float> CursorDelta() const;
  // Accumulated scroll-wheel input since the last PollEvents call.
  float ScrollDelta() const;
  bool IsLeftMouseDown() const;

 private:
  static void ScrollCallback(
      GLFWwindow* handle, double xoffset, double yoffset
  );

  GLFWwindow* handle_ = nullptr;
  double last_cursor_x_ = 0.0;
  double last_cursor_y_ = 0.0;
  double cursor_dx_ = 0.0;
  double cursor_dy_ = 0.0;
  // Scroll is event-driven in GLFW (no polled state), so ScrollCallback
  // accumulates into this and PollEvents drains it into scroll_dy_ once
  // per frame -- the same "poll once per frame" shape every other input
  // getter on this class presents.
  double pending_scroll_dy_ = 0.0;
  double scroll_dy_ = 0.0;
  bool has_last_cursor_ = false;
};

}  // namespace achilles::render
