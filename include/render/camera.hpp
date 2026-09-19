#pragma once

#include "render/mat4.hpp"

namespace achilles::render {

// A simple orbit ("arcball-lite") camera: always looks at `target_`, from
// `distance_` away, at a given (yaw_, pitch_) around it -- the standard
// shape for inspecting a bounded scene (a robot/mechanism, not an open
// world), and the simplest camera model that still gives full 3D framing
// with just mouse drag (orbit) and scroll (zoom). No free-fly/WASD mode:
// keeping to one interaction model is what "as simple as possible" means
// here, and it's the one that needs no state beyond these three numbers.
class Camera {
 public:
  // Orbit by (dyaw, dpitch) radians -- called with the drag delta while
  // the left mouse button is held (see Window::CursorDelta). Pitch is
  // clamped strictly inside (-pi/2, pi/2) so the camera can never flip
  // past looking straight up/down, which would otherwise snap yaw by pi.
  void Orbit(float dyaw, float dpitch);

  // Zoom by a scroll delta -- positive moves the camera closer,
  // multiplicative (not additive) so it feels the same whether already
  // close in or far out, and clamped well above zero so `distance_` can
  // never reach the target (which would make View() degenerate).
  void Zoom(float scroll_delta);

  Vec3 Eye() const;
  Mat4 View() const;
  static Mat4 Projection(float aspect);

 private:
  float yaw_ = 0.7F;
  float pitch_ = 0.5F;
  float distance_ = 8.0F;
  Vec3 target_ = Vec3::Zero();
};

}  // namespace achilles::render
