#pragma once

#include <cstddef>
#include <optional>
#include <span>
#include <string_view>

#include "display/contract/display.hpp"
#include "display/contract/frame.hpp"
#include "display/views/camera/camera_image.hpp"
#include "domain/sensing/camera.hpp"
#include "interface/attachment_handle.hpp"

namespace achilles::display {

// How far a camera sees: its latest depth over the whole drawing area, as
// gray -- white up close, black at the camera's far plane and wherever
// nothing was hit. The same camera's other output (camera_view.hpp shows its
// color), for a scene where distance is the point.
class DepthView final : public IDisplay {
 public:
  static constexpr std::string_view kName = "camera depth";
  static constexpr float kWidth = 660.0F;
  static constexpr float kHeight = 540.0F;

  explicit DepthView(interface::SensorHandle<domain::sensing::Camera> camera)
      : camera_(camera) {}

  void Draw(Frame* out) const override {
    const std::optional<CameraImage> latest = CameraImage::Latest(camera_, out);
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

 private:
  interface::SensorHandle<domain::sensing::Camera> camera_;
};

}  // namespace achilles::display
