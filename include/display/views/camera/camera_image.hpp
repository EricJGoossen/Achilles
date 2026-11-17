#pragma once

#include <optional>

#include "display/contract/frame.hpp"
#include "domain/sensing/camera.hpp"
#include "interface/attachment_handle.hpp"

namespace achilles::display {

// A camera's latest image: the frame it took, the camera's configuration,
// and the pixels the frame was read into. What both of a camera's views
// (camera_view.hpp, depth_view.hpp) start from.
struct CameraImage {
  domain::sensing::CameraFrame frame;
  domain::sensing::Camera config;
  const domain::sensing::Camera::Image* pixels = nullptr;

  // The camera's latest image, with when it was taken noted in `out`'s text
  // and `out` laid out for its aspect -- or nothing, noted in `out`'s text,
  // until its first image arrives.
  static std::optional<CameraImage> Latest(
      const interface::AttachmentHandle<domain::sensing::Camera>& camera,
      Frame* out
  ) {
    const domain::sensing::CameraFrame frame = camera.Read().value;
    if (frame.width == 0) {
      out->text.emplace_back("waiting for the camera's first image");
      return std::nullopt;
    }
    out->text.push_back(
        "captured at " + Fixed(static_cast<float>(frame.capture_time)) + " s"
    );
    out->aspect =
        static_cast<float>(frame.width) / static_cast<float>(frame.height);
    return CameraImage{
        frame, camera.Config(), &camera.State().images[frame.image]
    };
  }
};

}  // namespace achilles::display
