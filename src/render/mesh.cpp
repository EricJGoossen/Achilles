#include "render/mesh.hpp"

#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <span>
#include <utility>
#include <vector>

#include "render/gl.hpp"

namespace achilles::render {

namespace {

// clang-format off
// 24 vertices (4 per face x 6 faces, so each face gets its own flat
// normal rather than sharing a smoothed corner normal), interleaved
// position(3)/normal(3). No particular winding order is relied on --
// scene_renderer.cpp never enables GL_CULL_FACE, so both a front and back
// interpretation of every triangle stays visible; only the normals (used
// for lighting, not culling) need to be right.
constexpr std::array<GLfloat, 24UL * 6UL> kCubeVertices = {
    // +X
     1,-1,-1,  1, 0, 0,
     1, 1,-1,  1, 0, 0,
     1, 1, 1,  1, 0, 0,
     1,-1, 1,  1, 0, 0,
    // -X
    -1,-1, 1, -1, 0, 0,
    -1, 1, 1, -1, 0, 0,
    -1, 1,-1, -1, 0, 0,
    -1,-1,-1, -1, 0, 0,
    // +Y
    -1, 1,-1,  0, 1, 0,
    -1, 1, 1,  0, 1, 0,
     1, 1, 1,  0, 1, 0,
     1, 1,-1,  0, 1, 0,
    // -Y
    -1,-1, 1,  0,-1, 0,
    -1,-1,-1,  0,-1, 0,
     1,-1,-1,  0,-1, 0,
     1,-1, 1,  0,-1, 0,
    // +Z
    -1,-1, 1,  0, 0, 1,
     1,-1, 1,  0, 0, 1,
     1, 1, 1,  0, 0, 1,
    -1, 1, 1,  0, 0, 1,
    // -Z
     1,-1,-1,  0, 0,-1,
    -1,-1,-1,  0, 0,-1,
    -1, 1,-1,  0, 0,-1,
     1, 1,-1,  0, 0,-1,
};
// clang-format on

constexpr std::array<GLuint, 36> kCubeIndices = [] {
  std::array<GLuint, 36> indices{};
  for (GLuint face = 0; face < 6; ++face) {
    GLuint base = face * 4;
    std::size_t offset = static_cast<std::size_t>(face) * 6;
    indices[offset + 0] = base + 0;
    indices[offset + 1] = base + 1;
    indices[offset + 2] = base + 2;
    indices[offset + 3] = base + 0;
    indices[offset + 4] = base + 2;
    indices[offset + 5] = base + 3;
  }
  return indices;
}();

// The sphere's grid: kSphereStacks bands from pole to pole, each split into
// kSphereSlices around. Fine enough to read as round at the sizes a
// rolling ball or a joint is drawn at.
constexpr GLuint kSphereStacks = 16;
constexpr GLuint kSphereSlices = 32;

// A unit sphere's vertices, interleaved position/normal -- the same vector
// twice, since a unit sphere's outward normal is its position. One vertex
// per grid point, (kSphereStacks + 1) rings of (kSphereSlices + 1), the
// seam's column duplicated so every band is a plain run of quads.
std::vector<GLfloat> SphereVertices() {
  std::vector<GLfloat> vertices;
  vertices.reserve(
      static_cast<std::size_t>(kSphereStacks + 1) * (kSphereSlices + 1) * 6
  );
  for (GLuint stack = 0; stack <= kSphereStacks; ++stack) {
    float polar = std::numbers::pi_v<float> * static_cast<float>(stack) /
                  static_cast<float>(kSphereStacks);
    for (GLuint slice = 0; slice <= kSphereSlices; ++slice) {
      float azimuth = 2.0F * std::numbers::pi_v<float> *
                      static_cast<float>(slice) /
                      static_cast<float>(kSphereSlices);
      float x = std::sin(polar) * std::cos(azimuth);
      float y = std::cos(polar);
      float z = std::sin(polar) * std::sin(azimuth);
      vertices.insert(vertices.end(), {x, y, z, x, y, z});
    }
  }
  return vertices;
}

// Two triangles per grid quad, indexing SphereVertices' rings.
std::vector<GLuint> SphereIndices() {
  std::vector<GLuint> indices;
  indices.reserve(static_cast<std::size_t>(kSphereStacks) * kSphereSlices * 6);
  constexpr GLuint kRing = kSphereSlices + 1;
  for (GLuint stack = 0; stack < kSphereStacks; ++stack) {
    for (GLuint slice = 0; slice < kSphereSlices; ++slice) {
      GLuint top = (stack * kRing) + slice;
      GLuint bottom = top + kRing;
      indices.insert(
          indices.end(), {top, bottom, top + 1, top + 1, bottom, bottom + 1}
      );
    }
  }
  return indices;
}

// The cylinder's side wall is split into kCylinderSlices around -- the same
// count as the sphere's, so the two read as equally round side by side.
constexpr GLuint kCylinderSlices = kSphereSlices;

// Appends one quad (4 vertices, 2 triangles) with a single flat normal.
void AppendQuad(
    std::vector<GLfloat>& vertices,
    std::vector<GLuint>& indices,
    const std::array<std::array<GLfloat, 3>, 4>& corners,
    const std::array<GLfloat, 3>& normal
) {
  auto base = static_cast<GLuint>(vertices.size() / 6);
  for (const std::array<GLfloat, 3>& corner : corners) {
    vertices.insert(
        vertices.end(),
        {corner[0], corner[1], corner[2], normal[0], normal[1], normal[2]}
    );
  }
  indices.insert(
      indices.end(), {base, base + 1, base + 2, base, base + 2, base + 3}
  );
}

// A unit cylinder about Y: the side wall is one ring of vertices at each
// end whose normals point straight out (x, 0, z), the seam's column
// duplicated like the sphere's; each cap is a fan around its own center
// vertex, with its own flat +-Y normal rather than sharing the wall's.
std::pair<std::vector<GLfloat>, std::vector<GLuint>> CylinderGeometry() {
  std::vector<GLfloat> vertices;
  std::vector<GLuint> indices;
  for (GLuint slice = 0; slice <= kCylinderSlices; ++slice) {
    float azimuth = 2.0F * std::numbers::pi_v<float> *
                    static_cast<float>(slice) /
                    static_cast<float>(kCylinderSlices);
    float x = std::cos(azimuth);
    float z = std::sin(azimuth);
    vertices.insert(vertices.end(), {x, -1, z, x, 0, z, x, 1, z, x, 0, z});
  }
  for (GLuint slice = 0; slice < kCylinderSlices; ++slice) {
    GLuint bottom = slice * 2;
    GLuint top = bottom + 1;
    indices.insert(
        indices.end(), {bottom, top, bottom + 2, bottom + 2, top, top + 2}
    );
  }
  for (float y : {-1.0F, 1.0F}) {
    auto center = static_cast<GLuint>(vertices.size() / 6);
    vertices.insert(vertices.end(), {0, y, 0, 0, y, 0});
    for (GLuint slice = 0; slice <= kCylinderSlices; ++slice) {
      float azimuth = 2.0F * std::numbers::pi_v<float> *
                      static_cast<float>(slice) /
                      static_cast<float>(kCylinderSlices);
      vertices.insert(
          vertices.end(), {std::cos(azimuth), y, std::sin(azimuth), 0, y, 0}
      );
    }
    for (GLuint slice = 0; slice < kCylinderSlices; ++slice) {
      indices.insert(
          indices.end(), {center, center + 1 + slice, center + 2 + slice}
      );
    }
  }
  return {std::move(vertices), std::move(indices)};
}

// CylinderStripe's quads. kStripeHalfWidth is in the unit cylinder's own
// radii, and kStripeLift how far outside the surface the marking sits --
// scaled with the cylinder like everything else, so both stay proportional
// to the wheel whatever its size.
std::pair<std::vector<GLfloat>, std::vector<GLuint>> CylinderStripeGeometry() {
  constexpr float kStripeHalfWidth = 0.15F;
  constexpr float kStripeLift = 1.02F;
  constexpr float kW = kStripeHalfWidth;
  constexpr float kL = kStripeLift;
  std::vector<GLfloat> vertices;
  std::vector<GLuint> indices;
  // Side wall, along the rim at +X.
  AppendQuad(
      vertices,
      indices,
      {{{kL, -kL, -kW}, {kL, kL, -kW}, {kL, kL, kW}, {kL, -kL, kW}}},
      {1, 0, 0}
  );
  // A spoke on each cap, from the axis out to the side-wall stripe.
  for (float y : {-kL, kL}) {
    AppendQuad(
        vertices,
        indices,
        {{{0, y, -kW}, {kL, y, -kW}, {kL, y, kW}, {0, y, kW}}},
        {0, y > 0 ? 1.0F : -1.0F, 0}
    );
  }
  return {std::move(vertices), std::move(indices)};
}

// glVertexAttribPointer's last parameter is a byte offset encoded as a
// pointer -- a historical GL API wart every loader/wrapper has to work
// around somehow. std::bit_cast (same tool gl.cpp's own Load<Fn> uses for
// its function-pointer conversion, and the one
// engine/topology/orderings.hpp already uses for its own pointer casts)
// stands in for reinterpret_cast here: well-defined as long as both sides are
// the same size and trivially copyable, which holds for std::size_t/const void*
// on every platform this project targets.
const void* ByteOffset(std::size_t bytes) {
  return std::bit_cast<const void*>(bytes);
}

}  // namespace

SolidMesh SolidMesh::Cube() { return {kCubeVertices, kCubeIndices}; }

SolidMesh SolidMesh::Sphere() {
  std::vector<GLfloat> vertices = SphereVertices();
  std::vector<GLuint> indices = SphereIndices();
  return {vertices, indices};
}

SolidMesh SolidMesh::Cylinder() {
  auto [vertices, indices] = CylinderGeometry();
  return {vertices, indices};
}

SolidMesh SolidMesh::CylinderStripe() {
  auto [vertices, indices] = CylinderStripeGeometry();
  return {vertices, indices};
}

SolidMesh::SolidMesh(
    std::span<const GLfloat> vertices, std::span<const GLuint> indices
)
    : index_count_(static_cast<GLsizei>(indices.size())) {
  gl::g_functions.gen_vertex_arrays(1, &vao_);
  gl::g_functions.gen_buffers(1, &vbo_);
  gl::g_functions.gen_buffers(1, &ebo_);

  gl::g_functions.bind_vertex_array(vao_);

  gl::g_functions.bind_buffer(gl::kArrayBuffer, vbo_);
  gl::g_functions.buffer_data(
      gl::kArrayBuffer,
      static_cast<gl::GLsizeiptr>(vertices.size() * sizeof(GLfloat)),
      vertices.data(),
      gl::kStaticDraw
  );

  gl::g_functions.bind_buffer(gl::kElementArrayBuffer, ebo_);
  gl::g_functions.buffer_data(
      gl::kElementArrayBuffer,
      static_cast<gl::GLsizeiptr>(indices.size() * sizeof(GLuint)),
      indices.data(),
      gl::kStaticDraw
  );

  constexpr auto kStride = static_cast<GLsizei>(6 * sizeof(GLfloat));
  gl::g_functions.vertex_attrib_pointer(
      0, 3, GL_FLOAT, GL_FALSE, kStride, nullptr
  );
  gl::g_functions.enable_vertex_attrib_array(0);
  gl::g_functions.vertex_attrib_pointer(
      1, 3, GL_FLOAT, GL_FALSE, kStride, ByteOffset(3 * sizeof(GLfloat))
  );
  gl::g_functions.enable_vertex_attrib_array(1);

  gl::g_functions.bind_vertex_array(0);
}

SolidMesh::~SolidMesh() {
  gl::g_functions.delete_buffers(1, &ebo_);
  gl::g_functions.delete_buffers(1, &vbo_);
  gl::g_functions.delete_vertex_arrays(1, &vao_);
}

// glDelete{Buffers,VertexArrays} silently ignore a 0 name (the GL spec's
// own "no-op on an unused/zero name" contract), so the moved-from
// source's zeroed-out handles need no extra guard here or in the
// destructor above -- the same reasoning Shader's own move already
// relies on for glDeleteProgram(0).
SolidMesh::SolidMesh(SolidMesh&& other) noexcept
    : vao_(std::exchange(other.vao_, 0)),
      vbo_(std::exchange(other.vbo_, 0)),
      ebo_(std::exchange(other.ebo_, 0)),
      index_count_(std::exchange(other.index_count_, 0)) {}

SolidMesh& SolidMesh::operator=(SolidMesh&& other) noexcept {
  if (this != &other) {
    gl::g_functions.delete_buffers(1, &ebo_);
    gl::g_functions.delete_buffers(1, &vbo_);
    gl::g_functions.delete_vertex_arrays(1, &vao_);
    vao_ = std::exchange(other.vao_, 0);
    vbo_ = std::exchange(other.vbo_, 0);
    ebo_ = std::exchange(other.ebo_, 0);
    index_count_ = std::exchange(other.index_count_, 0);
  }
  return *this;
}

void SolidMesh::Draw() const {
  gl::g_functions.bind_vertex_array(vao_);
  glDrawElements(GL_TRIANGLES, index_count_, GL_UNSIGNED_INT, nullptr);
  gl::g_functions.bind_vertex_array(0);
}

LineMesh::LineMesh() {
  gl::g_functions.gen_vertex_arrays(1, &vao_);
  gl::g_functions.gen_buffers(1, &vbo_);

  gl::g_functions.bind_vertex_array(vao_);
  gl::g_functions.bind_buffer(gl::kArrayBuffer, vbo_);
  gl::g_functions.vertex_attrib_pointer(
      0,
      3,
      GL_FLOAT,
      GL_FALSE,
      static_cast<GLsizei>(3 * sizeof(GLfloat)),
      nullptr
  );
  gl::g_functions.enable_vertex_attrib_array(0);
  gl::g_functions.bind_vertex_array(0);
}

LineMesh::~LineMesh() {
  gl::g_functions.delete_buffers(1, &vbo_);
  gl::g_functions.delete_vertex_arrays(1, &vao_);
}

LineMesh::LineMesh(LineMesh&& other) noexcept
    : vao_(std::exchange(other.vao_, 0)),
      vbo_(std::exchange(other.vbo_, 0)),
      vertex_count_(std::exchange(other.vertex_count_, 0)) {}

LineMesh& LineMesh::operator=(LineMesh&& other) noexcept {
  if (this != &other) {
    gl::g_functions.delete_buffers(1, &vbo_);
    gl::g_functions.delete_vertex_arrays(1, &vao_);
    vao_ = std::exchange(other.vao_, 0);
    vbo_ = std::exchange(other.vbo_, 0);
    vertex_count_ = std::exchange(other.vertex_count_, 0);
  }
  return *this;
}

void LineMesh::Update(std::span<const float> xyz) {
  gl::g_functions.bind_buffer(gl::kArrayBuffer, vbo_);
  gl::g_functions.buffer_data(
      gl::kArrayBuffer,
      static_cast<gl::GLsizeiptr>(xyz.size() * sizeof(float)),
      xyz.data(),
      gl::kDynamicDraw
  );
  vertex_count_ = static_cast<GLsizei>(xyz.size() / 3);
}

void LineMesh::Draw() const {
  if (vertex_count_ == 0) {
    return;
  }
  gl::g_functions.bind_vertex_array(vao_);
  glDrawArrays(GL_LINES, 0, vertex_count_);
  gl::g_functions.bind_vertex_array(0);
}

}  // namespace achilles::render
