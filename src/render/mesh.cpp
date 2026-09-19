#include "render/mesh.hpp"

#include <array>
#include <bit>
#include <cstddef>
#include <span>
#include <utility>

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

// glVertexAttribPointer's last parameter is a byte offset encoded as a
// pointer -- a historical GL API wart every loader/wrapper has to work
// around somehow. std::bit_cast (same tool gl.cpp's own Load<Fn> uses for
// its function-pointer conversion, and the one engine/topology/layout.hpp
// already uses for its own pointer casts) stands in for reinterpret_cast
// here: well-defined as long as both sides are the same size and
// trivially copyable, which holds for std::size_t/const void* on every
// platform this project targets.
const void* ByteOffset(std::size_t bytes) {
  return std::bit_cast<const void*>(bytes);
}

}  // namespace

CubeMesh::CubeMesh() : index_count_(static_cast<GLsizei>(kCubeIndices.size())) {
  gl::g_functions.gen_vertex_arrays(1, &vao_);
  gl::g_functions.gen_buffers(1, &vbo_);
  gl::g_functions.gen_buffers(1, &ebo_);

  gl::g_functions.bind_vertex_array(vao_);

  gl::g_functions.bind_buffer(gl::kArrayBuffer, vbo_);
  gl::g_functions.buffer_data(
      gl::kArrayBuffer,
      static_cast<gl::GLsizeiptr>(kCubeVertices.size() * sizeof(GLfloat)),
      kCubeVertices.data(),
      gl::kStaticDraw
  );

  gl::g_functions.bind_buffer(gl::kElementArrayBuffer, ebo_);
  gl::g_functions.buffer_data(
      gl::kElementArrayBuffer,
      static_cast<gl::GLsizeiptr>(kCubeIndices.size() * sizeof(GLuint)),
      kCubeIndices.data(),
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

CubeMesh::~CubeMesh() {
  gl::g_functions.delete_buffers(1, &ebo_);
  gl::g_functions.delete_buffers(1, &vbo_);
  gl::g_functions.delete_vertex_arrays(1, &vao_);
}

// glDelete{Buffers,VertexArrays} silently ignore a 0 name (the GL spec's
// own "no-op on an unused/zero name" contract), so the moved-from
// source's zeroed-out handles need no extra guard here or in the
// destructor above -- the same reasoning Shader's own move already
// relies on for glDeleteProgram(0).
CubeMesh::CubeMesh(CubeMesh&& other) noexcept
    : vao_(std::exchange(other.vao_, 0)),
      vbo_(std::exchange(other.vbo_, 0)),
      ebo_(std::exchange(other.ebo_, 0)),
      index_count_(std::exchange(other.index_count_, 0)) {}

CubeMesh& CubeMesh::operator=(CubeMesh&& other) noexcept {
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

void CubeMesh::Draw() const {
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
