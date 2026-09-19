#include "render/scene_renderer.hpp"

#include <cmath>
#include <cstddef>
#include <string_view>
#include <vector>

#include "algorithms/viz/viz_data.hpp"
#include "domain/joint_topology.hpp"
#include "domain/math/vector3.hpp"
#include "domain/spatial/transform.hpp"
#include "render/mat4.hpp"

namespace achilles::render {

namespace {

using achilles::algorithms::viz::VizField;
using achilles::algorithms::viz::VizView;
using ScalarTransform = domain::spatial::Transform<float>;
using ScalarVector3 = domain::math::Vector3<float>;

constexpr std::string_view kLitVertexSource = R"(#version 330 core
layout(location = 0) in vec3 a_pos;
layout(location = 1) in vec3 a_normal;
uniform mat4 u_model;
uniform mat4 u_view;
uniform mat4 u_proj;
out vec3 v_world_normal;
void main() {
  v_world_normal = mat3(u_model) * a_normal;
  gl_Position = u_proj * u_view * u_model * vec4(a_pos, 1.0);
}
)";

constexpr std::string_view kLitFragmentSource = R"(#version 330 core
in vec3 v_world_normal;
uniform vec3 u_color;
out vec4 frag_color;
void main() {
  vec3 n = normalize(v_world_normal);
  vec3 light_dir = normalize(vec3(0.5, 1.0, 0.35));
  float diffuse = max(dot(n, light_dir), 0.0);
  vec3 result = u_color * (0.35 + 0.65 * diffuse);
  frag_color = vec4(result, 1.0);
}
)";

constexpr std::string_view kLineVertexSource = R"(#version 330 core
layout(location = 0) in vec3 a_pos;
uniform mat4 u_view;
uniform mat4 u_proj;
void main() {
  gl_Position = u_proj * u_view * vec4(a_pos, 1.0);
}
)";

constexpr std::string_view kLineFragmentSource = R"(#version 330 core
uniform vec3 u_color;
out vec4 frag_color;
void main() {
  frag_color = vec4(u_color, 1.0);
}
)";

// A deterministic, reasonably-spread-out fallback color for a joint that
// specifies kVisualExtents (so it should be drawn at all) but not
// kVisualColor -- golden-ratio hue stepping keeps consecutive rows
// visually distinct without needing any random state.
Vec3 FallbackColor(std::size_t row) {
  constexpr float kGoldenRatioConjugate = 0.6180339887F;
  float hue = std::fmod(static_cast<float>(row) * kGoldenRatioConjugate, 1.0F);
  float h6 = hue * 6.0F;
  auto sector = static_cast<int>(h6);
  float f = h6 - static_cast<float>(sector);
  constexpr float kValue = 0.9F;
  constexpr float kSaturation = 0.55F;
  float p = kValue * (1.0F - kSaturation);
  float q = kValue * (1.0F - kSaturation * f);
  float t = kValue * (1.0F - kSaturation * (1.0F - f));
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

std::vector<float> BuildGridLines(float half_extent, float step) {
  std::vector<float> verts;
  for (float x = -half_extent; x <= half_extent + 1e-4F; x += step) {
    verts.insert(verts.end(), {x, 0.0F, -half_extent, x, 0.0F, half_extent});
  }
  for (float z = -half_extent; z <= half_extent + 1e-4F; z += step) {
    verts.insert(verts.end(), {-half_extent, 0.0F, z, half_extent, 0.0F, z});
  }
  return verts;
}

}  // namespace

SceneRenderer::SceneRenderer(std::string_view title)
    : window_(1280, 800, title),
      lit_shader_(kLitVertexSource, kLitFragmentSource),
      line_shader_(kLineVertexSource, kLineFragmentSource) {
  grid_.Update(BuildGridLines(10.0F, 1.0F));
  axis_x_.Update(std::vector<float>{0, 0, 0, 2, 0, 0});
  axis_y_.Update(std::vector<float>{0, 0, 0, 0, 2, 0});
  axis_z_.Update(std::vector<float>{0, 0, 0, 0, 0, 2});
}

bool SceneRenderer::ShouldClose() const { return window_.ShouldClose(); }

void SceneRenderer::PollInput() {
  window_.PollEvents();
  if (window_.IsLeftMouseDown()) {
    auto [dx, dy] = window_.CursorDelta();
    constexpr float kOrbitSpeed = 0.005F;
    camera_.Orbit(-dx * kOrbitSpeed, dy * kOrbitSpeed);
  }
  camera_.Zoom(window_.ScrollDelta());
}

void SceneRenderer::RenderFrame(
    const VizView& view, const domain::JointTopology& topology
) {
  auto [width, height] = window_.FramebufferSize();
  glViewport(0, 0, width, height);
  glEnable(GL_DEPTH_TEST);
  glClearColor(0.08F, 0.09F, 0.11F, 1.0F);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  Mat4 proj = Camera::Projection(window_.Aspect());
  Mat4 view_matrix = camera_.View();

  line_shader_.Use();
  line_shader_.SetMat4("u_view", view_matrix);
  line_shader_.SetMat4("u_proj", proj);
  line_shader_.SetVec3("u_color", 0.32F, 0.32F, 0.36F);
  grid_.Draw();
  line_shader_.SetVec3("u_color", 0.85F, 0.25F, 0.25F);
  axis_x_.Draw();
  line_shader_.SetVec3("u_color", 0.25F, 0.85F, 0.25F);
  axis_y_.Draw();
  line_shader_.SetVec3("u_color", 0.25F, 0.45F, 0.95F);
  axis_z_.Draw();

  lit_shader_.Use();
  lit_shader_.SetMat4("u_view", view_matrix);
  lit_shader_.SetMat4("u_proj", proj);

  std::vector<float> bone_verts;
  std::size_t row_count = topology.Size();
  for (std::size_t row = 0; row < row_count; ++row) {
    ScalarVector3 extents = view.Load<VizField::kVisualExtents, float>(row);
    if (extents.IsZero()) {
      continue;
    }

    ScalarTransform transform =
        view.Load<VizField::kWorldTransform, float>(row);
    ScalarVector3 color = view.Load<VizField::kVisualColor, float>(row);
    Vec3 rgb = color.IsZero() ? FallbackColor(row)
                              : Vec3(color.X(), color.Y(), color.Z());

    Mat4 model = FromTransformAndScale(
        transform, Vec3(extents.X(), extents.Y(), extents.Z())
    );
    lit_shader_.SetMat4("u_model", model);
    lit_shader_.SetVec3("u_color", rgb.X(), rgb.Y(), rgb.Z());
    cube_.Draw();

    std::size_t parent_row = topology[row];
    ScalarTransform parent_transform =
        view.Load<VizField::kWorldTransform, float>(parent_row);
    const ScalarVector3& p0 = transform.Translation();
    const ScalarVector3& p1 = parent_transform.Translation();
    bone_verts.insert(
        bone_verts.end(), {p0.X(), p0.Y(), p0.Z(), p1.X(), p1.Y(), p1.Z()}
    );
  }

  bones_.Update(bone_verts);
  line_shader_.Use();
  line_shader_.SetMat4("u_view", view_matrix);
  line_shader_.SetMat4("u_proj", proj);
  line_shader_.SetVec3("u_color", 0.9F, 0.75F, 0.2F);
  bones_.Draw();
}

void SceneRenderer::SwapBuffers() { window_.SwapBuffers(); }

}  // namespace achilles::render
