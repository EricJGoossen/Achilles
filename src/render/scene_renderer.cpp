#include "render/scene_renderer.hpp"

#include <cmath>
#include <cstddef>
#include <string_view>
#include <vector>

#include "algorithms/conventions.hpp"
#include "algorithms/viz/viz_data.hpp"
#include "domain/joint_topology.hpp"
#include "domain/math/quaternion.hpp"
#include "domain/math/vector3.hpp"
#include "domain/spatial/transform.hpp"
#include "render/mat4.hpp"

namespace achilles::render {

namespace {

using achilles::algorithms::ScalarOperationT;
using achilles::algorithms::viz::VizField;
using achilles::algorithms::viz::VizView;
using ScalarTransform = domain::spatial::Transform<ScalarOperationT>;
using ScalarVector3 = domain::math::Vector3<ScalarOperationT>;
using ScalarQuaternion = domain::math::Quaternion<ScalarOperationT>;

// The simulation runs in ScalarOperationT (double, see algorithms/
// conventions.hpp), but the renderer -- GPU vertex data, Vec3/Mat4 in
// render/mat4.hpp -- is deliberately float32 throughout, matching the
// conventional GL_FLOAT vertex format. These two convert at exactly that
// boundary, rather than letting float32 leak back into the simulation's
// own types or double leak into the render ones.
Vec3 ToRenderVec3(const ScalarVector3& v) {
  return {
      static_cast<float>(v.X()),
      static_cast<float>(v.Y()),
      static_cast<float>(v.Z())
  };
}
domain::spatial::Transform<float> ToRenderTransform(const ScalarTransform& t) {
  const ScalarQuaternion& q = t.Rotation();
  return {
      ToRenderVec3(t.Translation()),
      domain::math::Quaternion<float>(
          static_cast<float>(q.W()),
          static_cast<float>(q.X()),
          static_cast<float>(q.Y()),
          static_cast<float>(q.Z())
      )
  };
}

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
    ScalarVector3 extents =
        view.Load<VizField::kVisualExtents, ScalarOperationT>(row);
    if (extents.IsZero()) {
      continue;
    }

    ScalarTransform transform =
        view.Load<VizField::kWorldTransform, ScalarOperationT>(row);
    ScalarVector3 color =
        view.Load<VizField::kVisualColor, ScalarOperationT>(row);
    Vec3 rgb = color.IsZero() ? FallbackColor(row) : ToRenderVec3(color);

    // The cube mesh itself is centered on the origin ([-1, 1]^3, see
    // CubeMesh's own comment), so placing it directly at `transform` would
    // draw every joint's box straddling its own pivot -- centered on the
    // joint rather than extending away from it the way a bone/limb segment
    // actually should. Offsetting by half the box's own length along its
    // local +Y (the same axis fixed_joint_transform's own translation uses
    // to place a child joint, see two_joint_arm.arow's own comment) instead
    // draws it from the joint outward, so a child joint offset by this same
    // convention sits right at the box's far face rather than the middle.
    // `transform` itself -- the real, unshifted joint position -- is still
    // what the bone line below is drawn from/to, so this offset is purely
    // cosmetic (the cube's own placement), never the physics.
    ScalarTransform box_pose =
        transform *
        ScalarTransform(
            ScalarVector3(0.0, extents.Y(), 0.0), ScalarQuaternion::Identity()
        );
    Mat4 model = FromTransformAndScale(
        ToRenderTransform(box_pose), ToRenderVec3(extents)
    );
    lit_shader_.SetMat4("u_model", model);
    lit_shader_.SetVec3("u_color", rgb.X(), rgb.Y(), rgb.Z());
    cube_.Draw();

    std::size_t parent_row = topology[row];
    ScalarTransform parent_transform =
        view.Load<VizField::kWorldTransform, ScalarOperationT>(parent_row);
    Vec3 p0 = ToRenderVec3(transform.Translation());
    Vec3 p1 = ToRenderVec3(parent_transform.Translation());
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
