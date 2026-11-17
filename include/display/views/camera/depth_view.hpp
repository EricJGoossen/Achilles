#pragma once

#include <cstddef>
#include <optional>
#include <span>
#include <string_view>
#include <tuple>

#include "display/contract/display.hpp"
#include "display/contract/frame.hpp"
#include "display/views/camera/camera_image.hpp"
#include "domain/sensing/camera.hpp"
#include "util/tmp.hpp"

namespace achilles::display {

// How far a camera sees: its latest depth over the whole drawing area, as
// gray -- white up close, black at the camera's far plane and wherever
// nothing was hit. The same camera's other output (camera_view.hpp shows its
// color), for a scene where distance is the point.
struct DepthView {
  using Attachments = util::TypeList<domain::sensing::Camera>;
  static constexpr std::string_view kName = "camera depth";
  static constexpr float kWidth = 660.0F;
  static constexpr float kHeight = 540.0F;

  void Draw(const HandlesForT<Attachments>& handles, Frame* out) const {
    const std::optional<CameraImage> latest =
        CameraImage::Latest(std::get<0>(handles), out);
    if (!latest.has_value()) {
      return;
    }
    const std::size_t pixels =
        std::size_t{latest->frame.width} * latest->frame.height;
    out->images.push_back(
        {.values = std::span(latest->pixels->depth).first(pixels),
         .width = latest->frame.width,
         .height = latest->frame.height,
         .scale = latest->config.GetParams().far}
    );
  }
};

}  // namespace achilles::display
