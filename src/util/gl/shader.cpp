#include "util/gl/shader.hpp"

#include <array>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include "util/gl/gl.hpp"

namespace achilles::util::gl {

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
        "achilles::util::gl: shader compilation failed: " + log
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
    throw std::runtime_error("achilles::util::gl: shader link failed: " + log);
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

void Shader::SetMat4(
    const char* name, const std::array<float, 16>& column_major
) const {
  GLint location = gl::g_functions.get_uniform_location(program_, name);
  if (location < 0) {
    return;
  }
  gl::g_functions.uniform_matrix4fv(location, 1, GL_FALSE, column_major.data());
}

}  // namespace achilles::util::gl
