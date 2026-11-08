#include "util/gl/gl.hpp"

#include <EGL/egl.h>

#include <bit>
#include <stdexcept>
#include <string>

namespace achilles::util::gl {

namespace {

template <typename Fn>
Fn Load(const char* name) {
  auto* proc = eglGetProcAddress(name);
  if (proc == nullptr) {
    throw std::runtime_error(
        std::string("achilles::util::gl: failed to load OpenGL function ") +
        name
    );
  }
  // eglGetProcAddress returns an opaque function pointer as
  // __eglMustCastToProperFunctionPointerType (itself a void(*)()) --
  // converting that to this entry point's real
  // signature is exactly what every GL loader (GLAD included) needs to
  // do, and reinterpret_cast is the only standard tool for a function-
  // pointer-to-function-pointer conversion. std::bit_cast requires equal
  // size, which holds for every function pointer on any platform this
  // project targets.
  return std::bit_cast<Fn>(proc);
}

}  // namespace

void LoadGLFunctions() {
  if (g_functions.gen_vertex_arrays != nullptr) {
    return;
  }
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
  g_functions.uniform4f = Load<Uniform4fFn>("glUniform4f");
  g_functions.uniform1i = Load<Uniform1iFn>("glUniform1i");
  g_functions.buffer_sub_data = Load<BufferSubDataFn>("glBufferSubData");
  g_functions.vertex_attrib_divisor =
      Load<VertexAttribDivisorFn>("glVertexAttribDivisor");
  g_functions.draw_elements_instanced =
      Load<DrawElementsInstancedFn>("glDrawElementsInstanced");
  g_functions.gen_framebuffers = Load<GenFramebuffersFn>("glGenFramebuffers");
  g_functions.bind_framebuffer = Load<BindFramebufferFn>("glBindFramebuffer");
  g_functions.delete_framebuffers =
      Load<DeleteFramebuffersFn>("glDeleteFramebuffers");
  g_functions.framebuffer_texture_2d =
      Load<FramebufferTexture2DFn>("glFramebufferTexture2D");
  g_functions.framebuffer_renderbuffer =
      Load<FramebufferRenderbufferFn>("glFramebufferRenderbuffer");
  g_functions.check_framebuffer_status =
      Load<CheckFramebufferStatusFn>("glCheckFramebufferStatus");
  g_functions.gen_renderbuffers =
      Load<GenRenderbuffersFn>("glGenRenderbuffers");
  g_functions.bind_renderbuffer =
      Load<BindRenderbufferFn>("glBindRenderbuffer");
  g_functions.renderbuffer_storage =
      Load<RenderbufferStorageFn>("glRenderbufferStorage");
  g_functions.delete_renderbuffers =
      Load<DeleteRenderbuffersFn>("glDeleteRenderbuffers");
  g_functions.draw_buffers = Load<DrawBuffersFn>("glDrawBuffers");
  g_functions.clear_bufferfv = Load<ClearBufferfvFn>("glClearBufferfv");
  g_functions.map_buffer_range = Load<MapBufferRangeFn>("glMapBufferRange");
  g_functions.unmap_buffer = Load<UnmapBufferFn>("glUnmapBuffer");
  g_functions.active_texture = Load<ActiveTextureFn>("glActiveTexture");
}

}  // namespace achilles::util::gl
