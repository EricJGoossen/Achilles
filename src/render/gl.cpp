#include "render/gl.hpp"

#include <GLFW/glfw3.h>

#include <bit>
#include <stdexcept>
#include <string>

namespace achilles::render::gl {

namespace {

template <typename Fn>
Fn Load(const char* name) {
  auto* proc = glfwGetProcAddress(name);
  if (proc == nullptr) {
    throw std::runtime_error(
        std::string("achilles::render: failed to load OpenGL function ") + name
    );
  }
  // glfwGetProcAddress returns an opaque function pointer as GLFWglproc
  // (itself a void(*)()) -- converting that to this entry point's real
  // signature is exactly what every GL loader (GLAD included) needs to
  // do, and reinterpret_cast is the only standard tool for a function-
  // pointer-to-function-pointer conversion. std::bit_cast requires equal
  // size, which holds for every function pointer on any platform this
  // project targets.
  return std::bit_cast<Fn>(proc);
}

}  // namespace

void LoadGLFunctions() {
  g_functions.gen_vertex_arrays = Load<GenVertexArraysFn>("glGenVertexArrays");
  g_functions.bind_vertex_array = Load<BindVertexArrayFn>("glBindVertexArray");
  g_functions.delete_vertex_arrays =
      Load<DeleteVertexArraysFn>("glDeleteVertexArrays");
  g_functions.gen_buffers = Load<GenBuffersFn>("glGenBuffers");
  g_functions.bind_buffer = Load<BindBufferFn>("glBindBuffer");
  g_functions.buffer_data = Load<BufferDataFn>("glBufferData");
  g_functions.delete_buffers = Load<DeleteBuffersFn>("glDeleteBuffers");
  g_functions.vertex_attrib_pointer =
      Load<VertexAttribPointerFn>("glVertexAttribPointer");
  g_functions.enable_vertex_attrib_array =
      Load<EnableVertexAttribArrayFn>("glEnableVertexAttribArray");
  g_functions.create_shader = Load<CreateShaderFn>("glCreateShader");
  g_functions.shader_source = Load<ShaderSourceFn>("glShaderSource");
  g_functions.compile_shader = Load<CompileShaderFn>("glCompileShader");
  g_functions.get_shaderiv = Load<GetShaderivFn>("glGetShaderiv");
  g_functions.get_shader_info_log =
      Load<GetShaderInfoLogFn>("glGetShaderInfoLog");
  g_functions.delete_shader = Load<DeleteShaderFn>("glDeleteShader");
  g_functions.create_program = Load<CreateProgramFn>("glCreateProgram");
  g_functions.attach_shader = Load<AttachShaderFn>("glAttachShader");
  g_functions.link_program = Load<LinkProgramFn>("glLinkProgram");
  g_functions.get_programiv = Load<GetProgramivFn>("glGetProgramiv");
  g_functions.get_program_info_log =
      Load<GetProgramInfoLogFn>("glGetProgramInfoLog");
  g_functions.use_program = Load<UseProgramFn>("glUseProgram");
  g_functions.delete_program = Load<DeleteProgramFn>("glDeleteProgram");
  g_functions.get_uniform_location =
      Load<GetUniformLocationFn>("glGetUniformLocation");
  g_functions.uniform_matrix4fv =
      Load<UniformMatrix4fvFn>("glUniformMatrix4fv");
  g_functions.uniform3f = Load<Uniform3fFn>("glUniform3f");
  g_functions.uniform1f = Load<Uniform1fFn>("glUniform1f");
}

}  // namespace achilles::render::gl
