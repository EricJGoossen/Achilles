#include "render/shader.hpp"

#include <array>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include "render/gl.hpp"
#include "render/mat4.hpp"

namespace achilles::render {

namespace {

GLuint CompileStage(GLenum stage, std::string_view source) {
  GLuint id = gl::g_functions.create_shader(stage);
  const gl::GLchar* src = source.data();
  auto length = static_cast<GLint>(source.size());
  gl::g_functions.shader_source(id, 1, &src, &length);
  gl::g_functions.compile_shader(id);

  GLint ok = 0;
  gl::g_functions.get_shaderiv(id, gl::kCompileStatus, &ok);
  if (ok == 0) {
    GLint log_length = 0;
    gl::g_functions.get_shaderiv(id, gl::kInfoLogLength, &log_length);
    std::string log(static_cast<std::size_t>(log_length), '\0');
    gl::g_functions.get_shader_info_log(id, log_length, nullptr, log.data());
    gl::g_functions.delete_shader(id);
    throw std::runtime_error(
        "achilles::render: shader compilation failed: " + log
    );
  }
  return id;
}

}  // namespace

Shader::Shader(std::string_view vertex_source, std::string_view fragment_source)
    : program_(gl::g_functions.create_program()) {
  GLuint vertex = CompileStage(gl::kVertexShader, vertex_source);
  GLuint fragment = CompileStage(gl::kFragmentShader, fragment_source);

  gl::g_functions.attach_shader(program_, vertex);
  gl::g_functions.attach_shader(program_, fragment);
  gl::g_functions.link_program(program_);

  gl::g_functions.delete_shader(vertex);
  gl::g_functions.delete_shader(fragment);

  GLint ok = 0;
  gl::g_functions.get_programiv(program_, gl::kLinkStatus, &ok);
  if (ok == 0) {
    GLint log_length = 0;
    gl::g_functions.get_programiv(program_, gl::kInfoLogLength, &log_length);
    std::string log(static_cast<std::size_t>(log_length), '\0');
    gl::g_functions.get_program_info_log(
        program_, log_length, nullptr, log.data()
    );
    gl::g_functions.delete_program(program_);
    program_ = 0;
    throw std::runtime_error("achilles::render: shader link failed: " + log);
  }
}

Shader::~Shader() {
  if (program_ != 0) {
    gl::g_functions.delete_program(program_);
  }
}

Shader::Shader(Shader&& other) noexcept
    : program_(std::exchange(other.program_, 0)) {}

Shader& Shader::operator=(Shader&& other) noexcept {
  if (this != &other) {
    if (program_ != 0) {
      gl::g_functions.delete_program(program_);
    }
    program_ = std::exchange(other.program_, 0);
  }
  return *this;
}

void Shader::Use() const { gl::g_functions.use_program(program_); }

void Shader::SetMat4(const char* name, const Mat4& value) const {
  GLint location = gl::g_functions.get_uniform_location(program_, name);
  if (location < 0) {
    return;
  }
  // Mat4 is stored row-major (see mat4.hpp's own comment); transpose=TRUE
  // tells GL to read it that way rather than assuming column-major, so
  // this uploads the matrix as-is with no hand transposition needed.
  std::array<GLfloat, 16> flat{};
  for (std::size_t row = 0; row < 4; ++row) {
    for (std::size_t col = 0; col < 4; ++col) {
      flat[(row * 4) + col] = value(row, col);
    }
  }
  gl::g_functions.uniform_matrix4fv(location, 1, GL_TRUE, flat.data());
}

void Shader::SetVec3(const char* name, float x, float y, float z) const {
  GLint location = gl::g_functions.get_uniform_location(program_, name);
  if (location < 0) {
    return;
  }
  gl::g_functions.uniform3f(location, x, y, z);
}

}  // namespace achilles::render
