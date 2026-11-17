#pragma once

#include <string_view>
#include <utility>

#include "display/contract/controller.hpp"
#include "display/contract/display.hpp"
#include "display/window/screen.hpp"
#include "display/window_set.hpp"

namespace achilles::display {

// Shows a sim's sensors and steers it, entirely from outside it, through
// nothing but handles: each display is made with the handles it shows, and
// drawn in its own window on the desktop; each controller is made with the
// handles it writes, and rides on a window, steered by its keys and mouse.
// It never looks anything up -- a caller gets its handles from the sim
// (Sensor(), Actuator(), State(), Attachments()) and passes them in.
//
//   Renderer renderer;
//   const WindowId camera = renderer.Attach(
//       CameraView(sim.Sensor<Camera>("overhead")), "Overhead");
//   renderer.Control(camera, FlyCamera(
//       sim.State<input::JointPosition>({"scene", "camera_mount"})));
//   while (running) {
//     renderer.ApplyWrites();  // before every Step()
//     sim.Step(dt);
//     running = renderer.Draw();  // once a frame
//   }
//
// Windows, and the controllers on them, can be attached, swapped and
// detached at any point between two frames, and nothing else changes when
// one is. A display shows only: only a controller writes to the sim.
//
// The handles are good until the sim's next Init(); so then are the
// displays and controllers made with them -- detach them, or build a new
// renderer.
class Renderer {
 public:
  // Starts the screen (screen.hpp). Throws std::runtime_error if it can't
  // be started.
  explicit Renderer(std::string_view title = "Achilles");
  ~Renderer();

  Renderer(const Renderer&) = delete;
  Renderer& operator=(const Renderer&) = delete;
  Renderer(Renderer&&) = delete;
  Renderer& operator=(Renderer&&) = delete;

  // Opens a window titled `title` for `display`.
  template <DisplayLike D>
  WindowId Attach(D display, std::string_view title) {
    return windows_.Add(std::move(display), title);
  }

  // The display in window `id`, to change its settings -- or null if it's
  // gone, or its display isn't a D. Valid until windows next change or
  // Draw() runs.
  template <DisplayLike D>
  D* Find(WindowId id) {
    return windows_.Find<D>(id);
  }

  // Puts `display` in window `id`, keeping its controllers. Throws
  // std::invalid_argument if `id` is gone.
  template <DisplayLike D>
  void Swap(WindowId id, D display) {
    windows_.Swap(id, std::move(display));
  }

  // Puts `controller` on window `id`, steered by its keys and mouse; false
  // if `id` is gone.
  template <ControllerLike C>
  bool Control(WindowId id, C controller) {
    return windows_.Control(id, std::move(controller));
  }

  // The first C on window `id`, to change its settings -- or null. Valid
  // until windows next change or Draw() runs.
  template <ControllerLike C>
  C* FindController(WindowId id) {
    return windows_.FindController<C>(id);
  }

  // Takes every controller off window `id`; false if it's gone.
  bool Release(WindowId id) { return windows_.Release(id); }

  // Closes window `id`, with its display and controllers; nothing if it's
  // already gone.
  void Detach(WindowId id);

  // One frame: pumps input, draws every window, and hands each window's
  // input to its controllers. Returns false once none is up -- every window
  // closed, or none ever attached.
  bool Draw();

  // Has every controller write what it last asked for: call before every
  // Step().
  void ApplyWrites() const { windows_.ApplyWrites(); }

  const WindowSet& Windows() const { return windows_; }

 private:
  // First, so it's destroyed last: every window's textures live in its
  // context.
  Screen screen_;
  WindowSet windows_;
};

}  // namespace achilles::display
