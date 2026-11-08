#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include "util/gl/gl.hpp"

namespace achilles::util::gl {

// A lit, indexed triangle mesh on the GPU, drawn instanced: its vertices
// (interleaved position(3)/normal(3)) and indices (3 per triangle) are
// uploaded once, and every copy of it drawn is one instance -- a model
// matrix and a color, uploaded together per draw (SetInstances) -- so a
// draw costs one call per mesh, however many copies share it.
class SolidMesh {
 public:
  // Floats per instance: a model matrix, column-major (16), then rgb (3).
  static constexpr std::size_t kInstanceFloats = 19;

  // Uploads `vertices` and `indices`; needs a current GL context.
  SolidMesh(
      std::span<const float> vertices, std::span<const std::uint32_t> indices
  );
  ~SolidMesh();

  SolidMesh(const SolidMesh&) = delete;
  SolidMesh& operator=(const SolidMesh&) = delete;
  SolidMesh(SolidMesh&& other) noexcept;
  SolidMesh& operator=(SolidMesh&& other) noexcept;

  // Replaces the instances Draw() draws: kInstanceFloats per instance.
  void SetInstances(std::span<const float> instances);
  // One instanced draw of every instance last set; nothing if none.
  void Draw() const;

 private:
  GLuint vao_ = 0;
  GLuint vbo_ = 0;
  GLuint ebo_ = 0;
  GLuint instance_vbo_ = 0;
  GLsizei index_count_ = 0;
  GLsizei instance_count_ = 0;
};

}  // namespace achilles::util::gl
