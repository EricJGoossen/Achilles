#pragma once

#include <array>
#include <string_view>

#include "util/gl/gl.hpp"

namespace achilles::util::gl {

// A compiled+linked GL program, owning its own GLuint for its whole
// lifetime (move-only) -- no shader hot-reloading or caching: the camera
// GPU (src/util/gl/gpu_egl.cpp) builds its one lit shader once. Source is
// passed in as plain strings (inline GLSL) rather than read from a file: no
// asset path to resolve.
class Shader {
 public:
  Shader(std::string_view vertex_source, std::string_view fragment_source);
  ~Shader();

  Shader(const Shader&) = delete;
  Shader& operator=(const Shader&) = delete;
  Shader(Shader&& other) noexcept;
  Shader& operator=(Shader&& other) noexcept;

  void Use() const;

  // Silently no-ops for a uniform name not present in this program (e.g.
  // optimized out for being unused in that particular shader), so a
  // caller needn't know which uniforms survived compilation.
  // `column_major` is a 4x4 matrix in GL's own column-major order.
  void SetMat4(const char* name, const std::array<float, 16>& column_major)
      const;

 private:
  GLuint program_ = 0;
};

}  // namespace achilles::util::gl
