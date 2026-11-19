#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <string_view>

#include "display/contract/display.hpp"
#include "display/contract/frame.hpp"
#include "domain/sensing/lidar.hpp"
#include "interface/attachment_handle.hpp"

namespace achilles::display {

// A lidar's latest scan from above, in its own frame: forward (+x) up the
// window, left (+y) to the left, a point per beam that hit something --
// each channel flattened onto the horizontal plane -- inside range rings at
// every quarter of its max range.
class LidarView final : public IDisplay {
 public:
  static constexpr std::string_view kName = "lidar";
  static constexpr float kWidth = 480.0F;
  static constexpr float kHeight = 520.0F;

  explicit LidarView(interface::SensorHandle<domain::sensing::Lidar> lidar)
      : lidar_(lidar) {}

  void Draw(Frame* out) const override {
    const domain::sensing::Lidar::Params params = lidar_.Config().GetParams();
    out->aspect = 1.0F;
    // The plot spans the max range from its center to each edge.
    const float scale = 0.5F / params.max_range;
    const auto plot = [&](float x, float y) {
      return std::array<float, 2>{0.5F - (y * scale), 0.5F - (x * scale)};
    };

    constexpr int kRingSegments = 64;
    Lines& rings = out->lines.emplace_back(Lines{.color = {0.3F, 0.3F, 0.35F}});
    for (int ring = 1; ring <= 4; ++ring) {
      const float radius = params.max_range * static_cast<float>(ring) / 4.0F;
      for (int i = 0; i < kRingSegments; ++i) {
        const float a0 = 2.0F * std::numbers::pi_v<float> *
                         static_cast<float>(i) / kRingSegments;
        const float a1 = 2.0F * std::numbers::pi_v<float> *
                         static_cast<float>(i + 1) / kRingSegments;
        const auto p0 = plot(radius * std::cos(a0), radius * std::sin(a0));
        const auto p1 = plot(radius * std::cos(a1), radius * std::sin(a1));
        rings.xy.insert(rings.xy.end(), {p0[0], p0[1], p1[0], p1[1]});
      }
    }
    const auto origin = plot(0.0F, 0.0F);
    const auto ahead = plot(params.max_range / 8.0F, 0.0F);
    out->lines.push_back(
        {.color = {0.9F, 0.75F, 0.2F},
         .xy = {origin[0], origin[1], ahead[0], ahead[1]}}
    );

    // Beam i's direction, as the lidar aims it (Lidar::AimBeams).
    const domain::sensing::LidarScan scan = lidar_.Read().value;
    Points& hits = out->points.emplace_back(
        Points{.color = {0.35F, 0.85F, 0.45F}, .size = 3.0F}
    );
    const float azimuth_step =
        params.horizontal_fov / static_cast<float>(params.horizontal_samples);
    const float elevation_step =
        params.channels > 1
            ? params.vertical_fov / static_cast<float>(params.channels - 1)
            : 0.0F;
    for (std::uint32_t i = 0; i < scan.beams; ++i) {
      const float range = scan.ranges[i];
      if (!std::isfinite(range)) {
        continue;
      }
      const std::uint32_t channel = i / params.horizontal_samples;
      const std::uint32_t step = i % params.horizontal_samples;
      const float azimuth = (-0.5F * params.horizontal_fov) +
                            ((static_cast<float>(step) + 0.5F) * azimuth_step);
      const float elevation =
          params.channels > 1
              ? (-0.5F * params.vertical_fov) +
                    (static_cast<float>(channel) * elevation_step)
              : 0.0F;
      const float flat = range * std::cos(elevation);
      const auto p = plot(flat * std::cos(azimuth), flat * std::sin(azimuth));
      hits.xy.insert(hits.xy.end(), {p[0], p[1]});
    }
    out->text.push_back(
        "max range " + Fixed(params.max_range, 1) + " m, rings every " +
        Fixed(params.max_range / 4.0F, 1) + " m"
    );
  }

 private:
  interface::SensorHandle<domain::sensing::Lidar> lidar_;
};

}  // namespace achilles::display
