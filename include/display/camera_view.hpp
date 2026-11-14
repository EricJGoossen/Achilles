#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <optional>
#include <span>
#include <string_view>
#include <tuple>

#include "display/camera_image.hpp"
#include "display/display.hpp"
#include "display/frame.hpp"
#include "domain/math/vector3.hpp"
#include "domain/sensing/camera.hpp"
#include "domain/spatial/transform.hpp"
#include "util/tmp.hpp"

namespace achilles::display {

// What a camera sees: its latest color image over the whole drawing area.
// ShowAxes also draws the world's axes, `axis_length` meters each, through
// the camera's own lens. Its depth is its own view (depth_view.hpp).
template <bool ShowAxes>
struct BasicCameraView {
  using Attachments = util::TypeList<domain::sensing::Camera>;
  static constexpr std::string_view kName =
      ShowAxes ? "camera with axes" : "camera";
  static constexpr float kWidth = 660.0F;
  static constexpr float kHeight = 540.0F;

  // A display's settings: public on purpose (see display.hpp).
  float axis_length = 2.0F;

  void Draw(const HandlesForT<Attachments>& handles, Frame* out) const {
    const std::optional<CameraImage> latest =
        CameraImage::Latest(std::get<0>(handles), out);
    if (!latest.has_value()) {
      return;
    }
    const std::size_t pixels =
        std::size_t{latest->frame.width} * latest->frame.height;
    constexpr Rect kWholeArea = {0.0F, 0.0F, 1.0F, 1.0F};
    out->images.push_back(
        {.rgb = std::span(latest->pixels->rgb).first(3 * pixels),
         .width = latest->frame.width,
         .height = latest->frame.height,
         .where = kWholeArea}
    );
    if constexpr (ShowAxes) {
      DrawWorldAxes(
          latest->frame, latest->config, axis_length, kWholeArea, out
      );
    }
  }

  // The part of the image-space segment p0-p1 inside [0, 1] x [0, 1], or
  // nothing (Liang-Barsky).
  static std::optional<std::array<float, 4>> ClipToImage(
      const std::array<float, 2>& p0, const std::array<float, 2>& p1
  ) {
    const std::array<float, 2> d = {p1[0] - p0[0], p1[1] - p0[1]};
    float enter = 0.0F;
    float leave = 1.0F;
    for (std::size_t axis = 0; axis < 2; ++axis) {
      if (d[axis] == 0.0F) {
        if (p0[axis] < 0.0F || p0[axis] > 1.0F) {
          return std::nullopt;
        }
        continue;
      }
      const float t0 = (0.0F - p0[axis]) / d[axis];
      const float t1 = (1.0F - p0[axis]) / d[axis];
      enter = std::max(enter, std::min(t0, t1));
      leave = std::min(leave, std::max(t0, t1));
    }
    if (enter > leave) {
      return std::nullopt;
    }
    return std::array<float, 4>{
        p0[0] + (enter * d[0]),
        p0[1] + (enter * d[1]),
        p0[0] + (leave * d[0]),
        p0[1] + (leave * d[1])
    };
  }

 private:
  // Draws the world's axes from its origin, x red, y green and z blue,
  // `length` meters each, through `camera`'s lens from the pose `frame` was
  // taken at, over the image at `area`. An overlay: in the frame, never in
  // the camera's image, and always in front of the scene.
  static void DrawWorldAxes(
      const domain::sensing::CameraFrame& frame,
      const domain::sensing::Camera& camera,
      float length,
      const Rect& area,
      Frame* out
  ) {
    using Vector3 = domain::math::Vector3<float>;
    const domain::spatial::Transform<float> camera_from_world =
        frame.world_from_camera.Inverse();
    const auto in_camera = [&](const Vector3& point) {
      return camera_from_world.Rotation().Rotate(point) +
             camera_from_world.Translation();
    };
    const Vector3 origin = camera_from_world.Translation();
    const float near = camera.GetParams().near;
    const auto draw = [&](const Vector3& tip, const Color& color) {
      Vector3 a = origin;
      Vector3 b = in_camera(tip);
      // In front of the lens only: a point at or behind it has no image.
      if (a.X() < near && b.X() < near) {
        return;
      }
      const float cut = (near - a.X()) / (b.X() - a.X());
      if (a.X() < near) {
        a = a + ((b - a) * cut);
      } else if (b.X() < near) {
        b = a + ((b - a) * cut);
      }
      const std::optional<std::array<float, 4>> segment =
          ClipToImage(camera.ImagePoint(a), camera.ImagePoint(b));
      if (!segment.has_value()) {
        return;
      }
      const std::array<float, 4>& s = *segment;
      out->lines.push_back(
          {.color = color,
           .xy =
               {area.x + (s[0] * area.width),
                area.y + (s[1] * area.height),
                area.x + (s[2] * area.width),
                area.y + (s[3] * area.height)}}
      );
    };
    draw(Vector3(length, 0.0F, 0.0F), {0.85F, 0.25F, 0.25F});
    draw(Vector3(0.0F, length, 0.0F), {0.25F, 0.85F, 0.25F});
    draw(Vector3(0.0F, 0.0F, length), {0.25F, 0.45F, 0.95F});
  }
};

using CameraView = BasicCameraView<false>;
using CameraAxesView = BasicCameraView<true>;

}  // namespace achilles::display
