#pragma once

#include <memory>
#include <string_view>

namespace achilles::display {

// What every display window (window.hpp) is drawn through: GLFW, the GUI
// (Dear ImGui), and the GL context they share. Each display window is its
// own window on the desktop -- moved, resized and closed there like any
// other -- made by the GUI as it's first drawn, in a context sharing this
// one's textures. This one's own window is never shown: it only hosts them.
// One per process, used from the thread that steps the sim.
//
// Its GL context goes through EGL, like the camera sensors' headless one
// (util/gl/gpu.hpp), so both live side by side on one thread: each makes its
// own context current before it draws.
class Screen {
 public:
  // Starts GLFW and the GUI; `title` names the hidden host. Throws
  // std::runtime_error if it can't (no display server, say) or another
  // Screen is up.
  explicit Screen(std::string_view title);
  ~Screen();

  Screen(const Screen&) = delete;
  Screen& operator=(const Screen&) = delete;
  Screen(Screen&&) = delete;
  Screen& operator=(Screen&&) = delete;

  // Pumps every window's events and starts a frame; display windows draw
  // between this and EndFrame().
  void BeginFrame() const;

  // Draws the frame into every display window and shows them.
  void EndFrame() const;

  // Makes the shared GL context current: for freeing what display windows
  // made in it outside a frame.
  void MakeCurrent() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace achilles::display
