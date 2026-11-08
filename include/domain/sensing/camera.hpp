#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <span>
#include <stdexcept>
#include <string_view>

#include "domain/geometry/unit_meshes.hpp"
#include "domain/geometry/visual_shape.hpp"
#include "domain/math/matrix.hpp"
#include "domain/math/quaternion.hpp"
#include "domain/math/scalar.hpp"
#include "domain/math/vector3.hpp"
#include "domain/sensing/sensor.hpp"
#include "domain/spatial/transform.hpp"
#include "util/gl/gpu.hpp"

namespace achilles::domain::sensing {

// How many meshes one camera can draw a sample: the fixed capacity that
// keeps its records a fixed size. Every seen body is one, a cylinder two (its
// stripe).
inline constexpr std::size_t kCameraMaxInstances = 1024;

// One image a camera keeps: 8-bit RGB triples and a depth per pixel
// (meters along the camera's view axis, +inf where nothing was hit), rows
// top to bottom, each row left to right -- MaxW x MaxH of room, of which a
// camera uses its own width x height.
template <std::uint32_t MaxW, std::uint32_t MaxH>
struct CameraImage {
  std::array<std::uint8_t, std::size_t{3} * MaxW * MaxH> rgb{};
  std::array<float, std::size_t{MaxW} * MaxH> depth{};
};

// What a camera keeps between samples: what it sees this sample, gathered
// body by body (Observe) for Read to draw; two images, so the one a reader
// was handed (latest) stays untouched while the next sample is written into
// the other; and when and from where the image it's waiting on was rendered.
template <std::uint32_t MaxW, std::uint32_t MaxH>
struct CameraState {
  std::array<util::gl::Instance, kCameraMaxInstances> seen{};
  std::uint32_t seen_count = 0;
  std::uint32_t seen_bodies = 0;
  std::array<CameraImage<MaxW, MaxH>, 2> images{};
  std::uint32_t latest = 0;
  double pending_time = 0.0;
  spatial::Transform<float> pending_pose;
};

// A camera's reading: which of its state's images is the latest, the
// image's size, and the sim time and world pose it was rendered at (one
// sample before the reading, for a pipelined camera -- see BasicCamera).
// Small on purpose: the attachment pipeline copies a reading forward every
// tick a camera doesn't sample, so the pixels stay in the state, read in
// place (interface::AttachmentHandle::State). width == 0 until the first
// image arrives.
struct CameraFrame {
  std::uint32_t image = 0;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  double capture_time = 0.0;
  spatial::Transform<float> world_from_camera;
};

// A body's world pose -- what a camera renders from, at its own body, and
// places every body it sees at -- and how each body looks.
using CameraPose = input::WorldTransform;
using CameraVisualPose = input::VisualPose;
using CameraVisualExtents = input::VisualExtents;
using CameraVisualColor = input::VisualColor;
using CameraVisualShape = input::VisualShape;

// A pinhole camera: renders every body it can see (its visual shape) from
// its mount, on the GPU (util/gl/gpu.hpp), into an RGB image and a depth
// image. A scanning kind, like Lidar: each sample, Observe runs once per
// body, adding the body's shape to what the camera sees, and Read draws it
// all and starts over. Distortion and noise belong in Read too, once
// modelled, as shared error effects (domain/error).
//
// MaxW x MaxH is the room its records keep for one image: a kind's records
// are a fixed size, so this is part of the kind, and a camera's own
// width/height is a setting up to it.
//
// Rendering is pipelined by default: each sample starts this tick's render
// and publishes the previous one's, which the GPU finished long ago, so a
// sample never waits on it -- the image arrives one sample late, as from a
// real camera, and its CameraFrame::capture_time says when it was taken.
// `pipelined: false` waits for this tick's image instead.
template <std::uint32_t MaxW, std::uint32_t MaxH>
class BasicCamera : public Sensor<
                        BasicCamera<MaxW, MaxH>,
                        SimInputs<CameraPose>,
                        CameraState<MaxW, MaxH>,
                        CameraFrame> {
  using Base = Sensor<
      BasicCamera<MaxW, MaxH>,
      SimInputs<CameraPose>,
      CameraState<MaxW, MaxH>,
      CameraFrame>;

 public:
  using Scene = SceneInputs<
      CameraPose,
      CameraVisualPose,
      CameraVisualExtents,
      CameraVisualColor,
      CameraVisualShape>;
  using Pose = CameraPose::Value;
  using Extents = CameraVisualExtents::Value;
  using Color = CameraVisualColor::Value;
  using ShapeCode = CameraVisualShape::Value;
  using Image = CameraImage<MaxW, MaxH>;
  using State = CameraState<MaxW, MaxH>;

  static constexpr std::string_view kListName = "camera";
  static constexpr std::uint32_t kMaxWidth = MaxW;
  static constexpr std::uint32_t kMaxHeight = MaxH;

  struct Params {
    // Where the camera sits on its body: a pose in the body's frame.
    // Sensor axes: x forward (the view axis), y left, z up -- the image's
    // rows run down -z, its columns along -y.
    spatial::Transform<float> body_from_sensor;
    std::uint32_t width = MaxW;
    std::uint32_t height = MaxH;
    // Radians across the image's width; square pixels set the height's.
    float horizontal_fov = std::numbers::pi_v<float> / 2.0F;
    // What it sees, in meters along the view axis.
    float near = 0.05F;
    float far = 50.0F;
    // rgb in [0, 1] wherever nothing is hit.
    math::Vector3<float> background{0.08F, 0.09F, 0.11F};
    bool pipelined = true;
  };

  BasicCamera() = default;
  // Throws std::invalid_argument for an image outside [1, MaxW] x
  // [1, MaxH], a field of view outside (0, pi), or an empty depth range.
  BasicCamera(AttachmentOptions options, const Params& params)
      : Base(options), params_(params) {
    if (params_.width == 0 || params_.width > MaxW || params_.height == 0 ||
        params_.height > MaxH) {
      throw std::invalid_argument(
          "Camera: width and height must be in [1, kMaxWidth] x "
          "[1, kMaxHeight]."
      );
    }
    if (!(params_.horizontal_fov > 0.0F &&
          params_.horizontal_fov < std::numbers::pi_v<float>)) {
      throw std::invalid_argument("Camera: horizontal_fov must be in (0, pi).");
    }
    if (params_.near <= 0.0F || params_.far <= params_.near) {
      throw std::invalid_argument("Camera: needs 0 < near < far.");
    }
  }

  // Settings: period, mount (a pose in the body's frame, identity when
  // absent), width, height, horizontal_fov, near, far, background,
  // pipelined.
  static BasicCamera FromConfig(AttachmentConfig& config) {
    const AttachmentOptions options = config.Options();
    Params params;
    params.body_from_sensor = config.BodyFromSensor();
    params.width = config.Count("width", params.width);
    params.height = config.Count("height", params.height);
    params.horizontal_fov =
        config.Scalar("horizontal_fov", params.horizontal_fov);
    params.near = config.Scalar("near", params.near);
    params.far = config.Scalar("far", params.far);
    params.background = config.Vector("background", params.background);
    params.pipelined = config.Flag("pipelined", params.pipelined);
    return {options, params};
  }

  const Params& GetParams() const { return params_; }

  // Where `point` (in the camera's own axes, in front of it: x > 0) lands
  // in its image, as fractions of the image's width and height from its
  // top-left corner -- outside [0, 1] when out of frame. The lens's one
  // forward model: distortion belongs here too, once modelled.
  std::array<float, 2> ImagePoint(const math::Vector3<float>& point) const {
    return {
        0.5F * (1.0F - (FocalX() * point.Y() / point.X())),
        0.5F * (1.0F - (FocalY() * point.Z() / point.X()))
    };
  }

  // One body: unless its extents are zero, adds its shape -- at its world
  // pose composed with its visual pose, shifted out along that frame's +Y by
  // its own half-height (shapes are drawn from the joint outward, the way a
  // limb hangs off its joint), colored by its color or, when that's zero, a
  // fallback of its own -- to what this sample sees; a cylinder adds its
  // stripe too. Throws std::length_error past kCameraMaxInstances.
  void Observe(
      const AttachmentTick& /*tick*/,
      const Pose& /*pose*/,
      const Pose& body_pose,
      const Pose& visual_pose,
      const Extents& extents,
      const Color& color,
      const ShapeCode& shape,
      State* state
  ) const {
    if (extents.IsZero()) {
      return;
    }
    const spatial::Transform<float> world_from_shape =
        body_pose * geometry::ToVisualPose(visual_pose) *
        spatial::Transform<float>(
            math::Vector3<float>(0.0F, extents.Y(), 0.0F),
            math::Quaternion<float>::Identity()
        );
    const math::Vector3<float> rgb =
        color.IsZero() ? FallbackColor(state->seen_bodies) : color;
    ++state->seen_bodies;
    switch (geometry::ShapeOf(shape.Value())) {
      case geometry::VisualShape::kSphere:
        See(world_from_shape, extents, rgb, kSphereMesh, state);
        break;
      case geometry::VisualShape::kCylinder:
        See(world_from_shape, extents, rgb, kCylinderMesh, state);
        See(world_from_shape, extents, StripeColor(rgb), kStripeMesh, state);
        break;
      case geometry::VisualShape::kBox:
        See(world_from_shape, extents, rgb, kBoxMesh, state);
        break;
    }
  }

  // Renders what this sample saw from the camera, then publishes an image:
  // this tick's, or -- pipelined -- the previous sample's.
  void Read(
      const AttachmentTick& tick,
      const Pose& pose,
      State* state,
      CameraFrame* out
  ) const {
    const spatial::Transform<float> world_from_camera =
        pose * params_.body_from_sensor;
    util::gl::Render(
        tick.stream,
        MakeView(world_from_camera),
        Meshes(),
        std::span(state->seen).first(state->seen_count)
    );
    state->seen_count = 0;
    state->seen_bodies = 0;
    const std::uint32_t next = 1U - state->latest;
    Image& image = state->images[next];
    const std::size_t pixels = std::size_t{params_.width} * params_.height;
    const bool ready = util::gl::Readback(
        tick.stream,
        params_.pipelined,
        std::span(image.rgb).first(3 * pixels),
        std::span(image.depth).first(pixels)
    );
    const double rendered_at =
        params_.pipelined ? state->pending_time : tick.time;
    const spatial::Transform<float> rendered_from =
        params_.pipelined ? state->pending_pose : world_from_camera;
    state->pending_time = tick.time;
    state->pending_pose = world_from_camera;
    if (!ready) {
      *out = CameraFrame{};
      return;
    }
    state->latest = next;
    *out = {next, params_.width, params_.height, rendered_at, rendered_from};
  }

 private:
  // Meshes()' order.
  static constexpr std::uint32_t kBoxMesh = 0;
  static constexpr std::uint32_t kSphereMesh = 1;
  static constexpr std::uint32_t kCylinderMesh = 2;
  static constexpr std::uint32_t kStripeMesh = 3;

  // The unit shapes every camera draws with, as the GPU takes them.
  static std::span<const util::gl::MeshData> Meshes() {
    static const std::array<util::gl::MeshData, 4> kMeshes = {
        ToMeshData(geometry::UnitCube()),
        ToMeshData(geometry::UnitSphere()),
        ToMeshData(geometry::UnitCylinder()),
        ToMeshData(geometry::UnitCylinderStripe()),
    };
    return kMeshes;
  }
  static util::gl::MeshData ToMeshData(const geometry::UnitMesh& mesh) {
    return {mesh.vertices, mesh.indices};
  }

  // Adds one instance of `mesh`: `world_from_shape` scaled by `extents`
  // along its own axes (the unit mesh spans [-1, 1]^3), colored `rgb`.
  static void See(
      const spatial::Transform<float>& world_from_shape,
      const Extents& extents,
      const math::Vector3<float>& rgb,
      std::uint32_t mesh,
      State* state
  ) {
    if (state->seen_count == kCameraMaxInstances) {
      throw std::length_error(
          "Camera: sees more than kCameraMaxInstances meshes."
      );
    }
    util::gl::Instance& instance = state->seen[state->seen_count++];
    const math::Matrix<float, 3, 3> rotation =
        world_from_shape.Rotation().ToRotationMatrix();
    const math::Vector3<float>& t = world_from_shape.Translation();
    const std::array<float, 3> scale = {extents.X(), extents.Y(), extents.Z()};
    for (std::size_t col = 0; col < 3; ++col) {
      for (std::size_t row = 0; row < 3; ++row) {
        instance.model[(col * 4) + row] = rotation(row, col) * scale[col];
      }
      instance.model[(col * 4) + 3] = 0.0F;
    }
    instance.model[12] = t.X();
    instance.model[13] = t.Y();
    instance.model[14] = t.Z();
    instance.model[15] = 1.0F;
    instance.rgb = {rgb.X(), rgb.Y(), rgb.Z()};
    instance.mesh = mesh;
  }

  // A deterministic, reasonably spread-out color for the `index`th body seen
  // that gives none of its own: golden-ratio hue stepping keeps consecutive
  // bodies distinct without any random state.
  static math::Vector3<float> FallbackColor(std::uint32_t index) {
    constexpr float kGoldenRatioConjugate = 0.6180339887F;
    const float hue =
        std::fmod(static_cast<float>(index) * kGoldenRatioConjugate, 1.0F);
    const float h6 = hue * 6.0F;
    const auto sector = static_cast<int>(h6);
    const float f = h6 - static_cast<float>(sector);
    constexpr float kValue = 0.9F;
    constexpr float kSaturation = 0.55F;
    const float p = kValue * (1.0F - kSaturation);
    const float q = kValue * (1.0F - (kSaturation * f));
    const float t = kValue * (1.0F - (kSaturation * (1.0F - f)));
    switch (sector % 6) {
      case 0:
        return {kValue, t, p};
      case 1:
        return {q, kValue, p};
      case 2:
        return {p, kValue, t};
      case 3:
        return {p, q, kValue};
      case 4:
        return {t, p, kValue};
      default:
        return {kValue, p, q};
    }
  }

  // The color a cylinder's stripe is drawn in: a darker shade of the body's
  // own, or -- for a body already too dark for darker to show, like a black
  // tire -- a lighter one instead.
  static math::Vector3<float> StripeColor(const math::Vector3<float>& rgb) {
    const float luminance =
        (0.2126F * rgb.X()) + (0.7152F * rgb.Y()) + (0.0722F * rgb.Z());
    if (luminance < 0.3F) {
      constexpr float kLighten = 0.55F;
      return {
          rgb.X() + ((1.0F - rgb.X()) * kLighten),
          rgb.Y() + ((1.0F - rgb.Y()) * kLighten),
          rgb.Z() + ((1.0F - rgb.Z()) * kLighten)
      };
    }
    constexpr float kDarken = 0.4F;
    return {rgb.X() * kDarken, rgb.Y() * kDarken, rgb.Z() * kDarken};
  }

  // GL's eye axes in the sensor's: right is -y, up is +z, and the eye looks
  // down its own -z, which is the sensor's +x.
  static constexpr std::array<std::array<float, 3>, 3> kEyeFromSensor = {{
      {0.0F, -1.0F, 0.0F},
      {0.0F, 0.0F, 1.0F},
      {-1.0F, 0.0F, 0.0F},
  }};

  // The pinhole's focal lengths, in half image widths and half heights.
  float FocalX() const {
    return 1.0F / std::tan(0.5F * params_.horizontal_fov);
  }
  float FocalY() const {
    return FocalX() * static_cast<float>(params_.width) /
           static_cast<float>(params_.height);
  }

  // The view (world to GL eye) and perspective projection, column-major.
  util::gl::CameraView MakeView(
      const spatial::Transform<float>& world_from_camera
  ) const {
    const spatial::Transform<float> sensor_from_world =
        world_from_camera.Inverse();
    const math::Matrix<float, 3, 3> rotation =
        sensor_from_world.Rotation().ToRotationMatrix();
    const math::Vector3<float>& t = sensor_from_world.Translation();
    const std::array<float, 3> translation = {t.X(), t.Y(), t.Z()};

    util::gl::CameraView view;
    for (std::size_t row = 0; row < 3; ++row) {
      float offset = 0.0F;
      for (std::size_t k = 0; k < 3; ++k) {
        offset += kEyeFromSensor[row][k] * translation[k];
      }
      for (std::size_t col = 0; col < 3; ++col) {
        float sum = 0.0F;
        for (std::size_t k = 0; k < 3; ++k) {
          sum += kEyeFromSensor[row][k] * rotation(k, col);
        }
        view.view[(col * 4) + row] = sum;
      }
      view.view[12 + row] = offset;
    }
    view.view[15] = 1.0F;

    const float n = params_.near;
    const float f = params_.far;
    view.projection[0] = FocalX();
    view.projection[5] = FocalY();
    view.projection[10] = (f + n) / (n - f);
    view.projection[11] = -1.0F;
    view.projection[14] = 2.0F * f * n / (n - f);

    view.width = params_.width;
    view.height = params_.height;
    view.background = {
        params_.background.X(), params_.background.Y(), params_.background.Z()
    };
    return view;
  }

  Params params_;
};

// The camera kind the sim hosts: room for 640 x 480 images.
using Camera = BasicCamera<640, 480>;

}  // namespace achilles::domain::sensing
