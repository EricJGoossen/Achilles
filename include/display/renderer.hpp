#pragma once

#include <concepts>
#include <string_view>
#include <utility>

#include "display/display.hpp"
#include "display/display_set.hpp"
#include "display/screen.hpp"

namespace achilles::display {

// Shows a sim's sensors and drives its actuators, entirely from outside it,
// through nothing but their handles: each display is bound to the handles it
// is given, and drawn in its own window on the desktop. It never looks
// anything up -- a caller gets its handles from the sim (Sensor(),
// Actuator(), Attachments()) and passes them in.
//
//   Renderer renderer;
//   renderer.Attach(CameraView{}, "Arm camera",
//                   sim.Sensor<Camera>("arm_camera"));
//   renderer.Attach(TorqueControl{}, "Left wheel",
//                   sim.Actuator<truth::Torque>("left_front_motor"));
//   while (running) {
//     renderer.ApplyEfforts();  // before every Step()
//     sim.Step(dt);
//     running = renderer.Draw();  // once a frame
//   }
//
// Displays can be attached, swapped and detached at any point between two
// frames, and nothing else changes when one is. A display is bound to
// handles of the kinds it names, so one bound to the wrong kind doesn't
// compile. Sensors are shown and actuators driven by separate displays: only
// an actuator's own display ever pushes on it.
//
// The handles are good until the sim's next Init(); so then are the
// displays bound to them -- detach them, or build a new renderer.
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

  // Puts up `display` bound to `handles` -- one per attachment D is bound
  // to, in order -- in a new window titled `title`.
  template <DisplayLike D, typename... Handles>
    requires std::constructible_from<HandlesOfT<D>, Handles...>
  DisplayId Attach(D display, std::string_view title, Handles&&... handles) {
    return displays_.Add(
        std::move(display),
        HandlesOfT<D>(std::forward<Handles>(handles)...),
        title
    );
  }

  // The display `id` names, to change its settings -- or null if it's gone,
  // or isn't a D. Valid until displays next change or Draw() runs.
  template <DisplayLike D>
  D* Find(DisplayId id) {
    return displays_.Find<D>(id);
  }

  // Puts `display`, bound to `handles`, in the window of the display `id`
  // names, under the same id. Throws std::invalid_argument if `id` is gone.
  template <DisplayLike D, typename... Handles>
    requires std::constructible_from<HandlesOfT<D>, Handles...>
  void Swap(DisplayId id, D display, Handles&&... handles) {
    displays_.Swap(
        id, std::move(display), HandlesOfT<D>(std::forward<Handles>(handles)...)
    );
  }

  // Takes the display `id` names down, with its window; nothing if it's
  // already gone.
  void Detach(DisplayId id);

  // One frame: pumps input and draws every display in its window. Returns
  // false once none is up -- every window closed, or none ever attached.
  bool Draw();

  // Adds every display's efforts to its actuators for the next tick: call
  // before every Step().
  void ApplyEfforts() const { displays_.ApplyEfforts(); }

  const DisplaySet& Displays() const { return displays_; }

 private:
  // First, so it's destroyed last: every window's textures live in its
  // context.
  Screen screen_;
  DisplaySet displays_;
};

}  // namespace achilles::display
