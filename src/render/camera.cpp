#include "render/camera.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "render/mat4.hpp"

namespace achilles::render {

namespace {
constexpr float kPitchLimit = std::numbers::pi_v<float> * 0.5F - 0.01F;
constexpr float kMinDistance = 0.5F;
constexpr float kMaxDistance = 500.0F;
constexpr float kFovYRadians = 60.0F * std::numbers::pi_v<float> / 180.0F;
constexpr float kNear = 0.05F;
constexpr float kFar = 1000.0F;
}  // namespace

void Camera::Orbit(float dyaw, float dpitch) {
  yaw_ += dyaw;
  pitch_ = std::clamp(pitch_ + dpitch, -kPitchLimit, kPitchLimit);
}

void Camera::Zoom(float scroll_delta) {
  // exp() turns an additive scroll delta into a multiplicative zoom, so
  // repeated small scroll steps compound the same way whether the camera
  // starts close in or far out (see this class's own header comment).
  distance_ = std::clamp(
      distance_ * std::exp(-scroll_delta * 0.1F), kMinDistance, kMaxDistance
  );
}

Vec3 Camera::Eye() const {
  float cos_pitch = std::cos(pitch_);
  Vec3 offset(
      distance_ * cos_pitch * std::cos(yaw_),
      distance_ * std::sin(pitch_),
      distance_ * cos_pitch * std::sin(yaw_)
  );
  return target_ + offset;
}

Mat4 Camera::View() const {
  return LookAt(Eye(), target_, Vec3(0.0F, 1.0F, 0.0F));
}

Mat4 Camera::Projection(float aspect) {
  return Perspective(kFovYRadians, aspect, kNear, kFar);
}

}  // namespace achilles::render
