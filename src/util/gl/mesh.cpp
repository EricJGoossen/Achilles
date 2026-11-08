#include "util/gl/mesh.hpp"

#include <bit>
#include <cstddef>
#include <cstdint>
#include <span>
#include <utility>

#include "util/gl/gl.hpp"

namespace achilles::util::gl {

// GL's own types, as the plain ones a mesh's data comes in.
static_assert(sizeof(GLfloat) == sizeof(float));
static_assert(sizeof(GLuint) == sizeof(std::uint32_t));

namespace {

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

SolidMesh::SolidMesh(
    std::span<const float> vertices, std::span<const std::uint32_t> indices
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

  // Per instance: the model matrix's four columns (attributes 2-5), then
  // its color (6), each advancing once per instance rather than per vertex.
  gl::g_functions.gen_buffers(1, &instance_vbo_);
  gl::g_functions.bind_buffer(gl::kArrayBuffer, instance_vbo_);
  constexpr auto kInstanceStride =
      static_cast<GLsizei>(kInstanceFloats * sizeof(GLfloat));
  for (GLuint column = 0; column < 4; ++column) {
    gl::g_functions.vertex_attrib_pointer(
        2 + column,
        4,
        GL_FLOAT,
        GL_FALSE,
        kInstanceStride,
        ByteOffset(std::size_t{column} * 4 * sizeof(GLfloat))
    );
    gl::g_functions.enable_vertex_attrib_array(2 + column);
    gl::g_functions.vertex_attrib_divisor(2 + column, 1);
  }
  gl::g_functions.vertex_attrib_pointer(
      6,
      3,
      GL_FLOAT,
      GL_FALSE,
      kInstanceStride,
      ByteOffset(16 * sizeof(GLfloat))
  );
  gl::g_functions.enable_vertex_attrib_array(6);
  gl::g_functions.vertex_attrib_divisor(6, 1);

  gl::g_functions.bind_vertex_array(0);
}

SolidMesh::~SolidMesh() {
  gl::g_functions.delete_buffers(1, &instance_vbo_);
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
      instance_vbo_(std::exchange(other.instance_vbo_, 0)),
      index_count_(std::exchange(other.index_count_, 0)),
      instance_count_(std::exchange(other.instance_count_, 0)) {}

SolidMesh& SolidMesh::operator=(SolidMesh&& other) noexcept {
  if (this != &other) {
    gl::g_functions.delete_buffers(1, &instance_vbo_);
    gl::g_functions.delete_buffers(1, &ebo_);
    gl::g_functions.delete_buffers(1, &vbo_);
    gl::g_functions.delete_vertex_arrays(1, &vao_);
    vao_ = std::exchange(other.vao_, 0);
    vbo_ = std::exchange(other.vbo_, 0);
    ebo_ = std::exchange(other.ebo_, 0);
    instance_vbo_ = std::exchange(other.instance_vbo_, 0);
    index_count_ = std::exchange(other.index_count_, 0);
    instance_count_ = std::exchange(other.instance_count_, 0);
  }
  return *this;
}

void SolidMesh::SetInstances(std::span<const float> instances) {
  gl::g_functions.bind_buffer(gl::kArrayBuffer, instance_vbo_);
  gl::g_functions.buffer_data(
      gl::kArrayBuffer,
      static_cast<gl::GLsizeiptr>(instances.size() * sizeof(float)),
      instances.data(),
      gl::kDynamicDraw
  );
  instance_count_ = static_cast<GLsizei>(instances.size() / kInstanceFloats);
}

void SolidMesh::Draw() const {
  if (instance_count_ == 0) {
    return;
  }
  gl::g_functions.bind_vertex_array(vao_);
  gl::g_functions.draw_elements_instanced(
      GL_TRIANGLES, index_count_, GL_UNSIGNED_INT, nullptr, instance_count_
  );
  gl::g_functions.bind_vertex_array(0);
}

}  // namespace achilles::util::gl
