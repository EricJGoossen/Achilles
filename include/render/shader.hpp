#pragma once

#include <string_view>

#include "render/gl.hpp"
#include "render/mat4.hpp"

namespace achilles::render {

// A compiled+linked GL program, owning its own GLuint for its whole
// lifetime (move-only, same ownership shape as Window below) -- no shader
// hot-reloading or caching, since scene_renderer.cpp only ever builds two
// of these (a lit shader for boxes, a flat-color shader for lines) once at
// startup. Source is passed in as plain strings (see scene_renderer.cpp's
// own inline GLSL) rather than read from a file: keeping the whole
// renderer to one self-contained executable, no asset path to resolve, is
// the same "as simple as possible" choice the rest of this module makes.
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
  // optimized out for being unused in that particular shader) -- a
  // renderer calling SetVec3("u_color", ...) against the flat line shader
  // and SetMat4("u_model", ...) against both shaders shouldn't have to
  // know which uniforms each one actually declares.
  void SetMat4(const char* name, const Mat4& value) const;
  void SetVec3(const char* name, float x, float y, float z) const;

 private:
  GLuint program_ = 0;
};

}  // namespace achilles::render
