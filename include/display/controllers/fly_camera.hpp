#pragma once

#include <numbers>
#include <optional>
#include <utility>

#include "display/contract/controller.hpp"
#include "display/contract/input.hpp"
#include "domain/attachment/sim_quantities.hpp"
#include "domain/math/quaternion.hpp"
#include "domain/math/vector3.hpp"
#include "domain/spatial/transform.hpp"
#include "interface/state_handle.hpp"

namespace achilles::display {

// Flies a camera by hand, by setting the pose of the joint it's mounted on
// outright -- no physics, no actuator: a written joint state
// (state_handle.hpp). Put on a camera's window (or its depth view's), it steers
// by that window's keys and mouse:
//
//   W / S       forward / back, along where the camera looks
//   A / D       left / right
//   E / Q       up / down, along `up`
//   Shift       `fast` times as quick
//   drag        turn: across, about `up` (the horizon stays level); up and
//               down, about the camera's own left -- any mouse button, or a
//               trackpad's click-drag
//   scroll      dolly forward / back -- a trackpad's two-finger swipe
//
// Meant for the joint the camera is attached to, carrying nothing else: best
// one with no freedoms of its own (an all-zero joint_subspace, as each
// example's camera_mount), which nothing then moves but these writes, so it
// stays wherever it's flown. On one the sim moves, it's held where it's flown
// while it's steered. Its parent is taken to stay where it is.
//
// It's made with the joint's pose to write, the joint's place in the world
// to read (its WorldTransform, which says what frame the pose is in), and
// the camera's mount (Camera::Params::body_from_sensor) -- where on the
// joint's body the camera sits, its axes x forward, y left, z up -- so that
// forward is always where it looks. `up` is the world's
// (Simulation::Up()).
class FlyCamera final
    : public Controller<
          interface::StateHandle<domain::attachment::input::JointPosition>> {
 public:
  using Joint =
      interface::StateHandle<domain::attachment::input::JointPosition>;
  using Placement =
      interface::StateHandle<const domain::attachment::input::WorldTransform>;
  using Pose = domain::spatial::Transform<float>;
  using Vector3 = domain::math::Vector3<float>;

  struct Settings {
    // Public on purpose: a controller's settings (see controller.hpp).
    float speed = 2.0F;  // m/s
    float fast = 4.0F;   // times `speed`, while Shift is held
    // Radians turned dragging across the whole drawing area.
    float look = std::numbers::pi_v<float>;
    // Meters per scroll notch.
    float dolly = 0.5F;
    bool invert_y = false;
    Vector3 up{0.0F, 0.0F, 1.0F};
  };

  FlyCamera(Joint joint, Placement placement, const Pose& joint_from_camera)
      : FlyCamera(
            std::move(joint),
            std::move(placement),
            joint_from_camera,
            Settings{}
        ) {}
  FlyCamera(
      Joint joint,
      Placement placement,
      const Pose& joint_from_camera,
      Settings settings
  )
      : Controller(std::move(joint)),
        settings(settings),
        placement_(std::move(placement)),
        joint_from_camera_(joint_from_camera),
        camera_from_joint_(joint_from_camera.Inverse()) {}

  Settings settings;

 protected:
  Writes Control(const Input& input) const override {
    const Pointer& pointer = input.pointer;
    const bool dragging = pointer.over && (pointer.Held(Button::kLeft) ||
                                           pointer.Held(Button::kRight) ||
                                           pointer.Held(Button::kMiddle));
    const float forward = Axis(input, Key::kW, Key::kS);
    const float left = Axis(input, Key::kA, Key::kD);
    const float up = Axis(input, Key::kE, Key::kQ);
    const bool moving = forward != 0.0F || left != 0.0F || up != 0.0F;
    const bool scrolling = pointer.over && pointer.scroll != 0.0F;
    if (!moving && !dragging && !scrolling) {
      return {};
    }

    const float step = settings.speed * input.seconds *
                       (input.Held(Key::kShift) ? settings.fast : 1.0F);
    const float yaw = dragging ? -pointer.dx * settings.look : 0.0F;
    const float pitch = dragging ? (settings.invert_y ? -1.0F : 1.0F) *
                                       pointer.dy * settings.look
                                 : 0.0F;
    const float ahead =
        (forward * step) + (scrolling ? pointer.scroll * settings.dolly : 0.0F);

    // The camera's own pose, moved in its own axes, then back on the joint;
    // up, the world's, in the frame the joint's pose is in.
    const Pose pose = Handle<0>().Read();
    const Vector3 rise = FrameUp(pose);
    const Pose camera = pose * joint_from_camera_;
    const auto& turn = camera.Rotation();
    const Vector3 moved = camera.Translation() +
                          turn.Rotate(Vector3(ahead, left * step, 0.0F)) +
                          (rise * (up * step));
    const auto turned = Quaternion::Exp(rise * yaw) * turn *
                        Quaternion::Exp(Vector3(0.0F, pitch, 0.0F));
    Pose joint = Pose(moved, turned) * camera_from_joint_;
    joint.NormalizeRotationInPlace();
    return Writes{joint};
  }

 private:
  using Quaternion = domain::math::Quaternion<float>;

  // +1 while `plus` is held, -1 while `minus` is, 0 for both or neither.
  static float Axis(const Input& input, Key plus, Key minus) {
    return (input.Held(plus) ? 1.0F : 0.0F) - (input.Held(minus) ? 1.0F : 0.0F);
  }

  // The world's up in the frame the joint's pose `pose` is in. That frame
  // is where the joint's world pose puts it with `pose` taken off -- read
  // once, the first time it's needed, before this has written anything (so
  // the world pose, worked out at the last step, is of `pose` itself), and
  // kept: the joint's parent stays where it is.
  Vector3 FrameUp(const Pose& pose) const {
    if (!world_from_frame_.has_value()) {
      world_from_frame_ =
          placement_.Read().Rotation() * pose.Rotation().Inverse();
    }
    return world_from_frame_->Inverse().Rotate(settings.up).Normalize();
  }

  Placement placement_;
  Pose joint_from_camera_;
  Pose camera_from_joint_;
  // Set by the first FrameUp(), from a const Control(): a cache.
  mutable std::optional<Quaternion> world_from_frame_;
};

}  // namespace achilles::display
