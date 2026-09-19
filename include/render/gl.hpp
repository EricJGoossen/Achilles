#pragma once

// Minimal, hand-rolled OpenGL 3.3 core loader -- exactly the subset of GL
// this renderer actually calls, loaded via GLFW's own glfwGetProcAddress
// once a context exists (see LoadGLFunctions, defined in gl.cpp, called by
// Window's own constructor right after glfwMakeContextCurrent). A full
// loader (GLAD/glbinding/gl3w) needs either offline codegen or an extra
// dependency this module intentionally avoids -- GLFW is already required
// for windowing, so reusing its own loader entry point for the rest of GL
// keeps the whole renderer's dependency footprint at exactly one library.
//
// GL_VERSION_1_1 entry points (glEnable, glViewport, glDrawArrays,
// glClear, ...) are declared by the system <GL/gl.h> and always resolve
// at ordinary link time against libGL -- called directly, unprefixed,
// wherever this module uses them. Only the post-1.1 core this renderer
// needs (buffers, shaders, VAOs, programs, uniforms) is declared here as
// a runtime-loaded function pointer.

#include <GL/gl.h>

#include <cstddef>

namespace achilles::render::gl {

// Khronos-registry values, stable across every implementation -- hand-
// declared rather than assumed present in whatever <GL/gl.h> a given
// system ships (most Linux distros' copy is still nominally a GL 1.1/1.2
// header and doesn't define these).
inline constexpr GLenum kArrayBuffer = 0x8892;
inline constexpr GLenum kElementArrayBuffer = 0x8893;
inline constexpr GLenum kStaticDraw = 0x88E4;
inline constexpr GLenum kDynamicDraw = 0x88E8;
inline constexpr GLenum kFragmentShader = 0x8B30;
inline constexpr GLenum kVertexShader = 0x8B31;
inline constexpr GLenum kCompileStatus = 0x8B81;
inline constexpr GLenum kLinkStatus = 0x8B82;
inline constexpr GLenum kInfoLogLength = 0x8B84;

// GLchar/GLsizeiptr aren't in classic GL/gl.h either (they arrived with
// the shader-object extensions) -- declared here to their well-known
// underlying types rather than pulled in from a second system header.
using GLchar = char;
using GLsizeiptr = std::ptrdiff_t;

using GenVertexArraysFn = void (*)(GLsizei, GLuint*);
using BindVertexArrayFn = void (*)(GLuint);
using DeleteVertexArraysFn = void (*)(GLsizei, const GLuint*);
using GenBuffersFn = void (*)(GLsizei, GLuint*);
using BindBufferFn = void (*)(GLenum, GLuint);
using BufferDataFn = void (*)(GLenum, GLsizeiptr, const void*, GLenum);
using DeleteBuffersFn = void (*)(GLsizei, const GLuint*);
using VertexAttribPointerFn =
    void (*)(GLuint, GLint, GLenum, GLboolean, GLsizei, const void*);
using EnableVertexAttribArrayFn = void (*)(GLuint);
using CreateShaderFn = GLuint (*)(GLenum);
using ShaderSourceFn =
    void (*)(GLuint, GLsizei, const GLchar* const*, const GLint*);
using CompileShaderFn = void (*)(GLuint);
using GetShaderivFn = void (*)(GLuint, GLenum, GLint*);
using GetShaderInfoLogFn = void (*)(GLuint, GLsizei, GLsizei*, GLchar*);
using DeleteShaderFn = void (*)(GLuint);
using CreateProgramFn = GLuint (*)();
using AttachShaderFn = void (*)(GLuint, GLuint);
using LinkProgramFn = void (*)(GLuint);
using GetProgramivFn = void (*)(GLuint, GLenum, GLint*);
using GetProgramInfoLogFn = void (*)(GLuint, GLsizei, GLsizei*, GLchar*);
using UseProgramFn = void (*)(GLuint);
using DeleteProgramFn = void (*)(GLuint);
using GetUniformLocationFn = GLint (*)(GLuint, const GLchar*);
using UniformMatrix4fvFn = void (*)(GLint, GLsizei, GLboolean, const GLfloat*);
using Uniform3fFn = void (*)(GLint, GLfloat, GLfloat, GLfloat);
using Uniform1fFn = void (*)(GLint, GLfloat);

// Every post-1.1 entry point this renderer uses, bundled into one table
// instead of ~25 separate namespace-scope globals: a GL loader
// necessarily needs *some* runtime-mutable state (these are resolved by
// LoadGLFunctions, not known until then), but there's no reason that
// needs more than one flagged exception to this codebase's own
// avoid-non-const-global-variables rule (see g_functions below) rather
// than one per entry point.
struct FunctionTable {
  GenVertexArraysFn gen_vertex_arrays = nullptr;
  BindVertexArrayFn bind_vertex_array = nullptr;
  DeleteVertexArraysFn delete_vertex_arrays = nullptr;
  GenBuffersFn gen_buffers = nullptr;
  BindBufferFn bind_buffer = nullptr;
  BufferDataFn buffer_data = nullptr;
  DeleteBuffersFn delete_buffers = nullptr;
  VertexAttribPointerFn vertex_attrib_pointer = nullptr;
  EnableVertexAttribArrayFn enable_vertex_attrib_array = nullptr;
  CreateShaderFn create_shader = nullptr;
  ShaderSourceFn shader_source = nullptr;
  CompileShaderFn compile_shader = nullptr;
  GetShaderivFn get_shaderiv = nullptr;
  GetShaderInfoLogFn get_shader_info_log = nullptr;
  DeleteShaderFn delete_shader = nullptr;
  CreateProgramFn create_program = nullptr;
  AttachShaderFn attach_shader = nullptr;
  LinkProgramFn link_program = nullptr;
  GetProgramivFn get_programiv = nullptr;
  GetProgramInfoLogFn get_program_info_log = nullptr;
  UseProgramFn use_program = nullptr;
  DeleteProgramFn delete_program = nullptr;
  GetUniformLocationFn get_uniform_location = nullptr;
  UniformMatrix4fvFn uniform_matrix4fv = nullptr;
  Uniform3fFn uniform3f = nullptr;
  Uniform1fFn uniform1f = nullptr;
};

inline FunctionTable g_functions;

// Resolves every pointer in g_functions via GLFW's own
// glfwGetProcAddress. Must be called once, after a current GL context
// exists. Throws std::runtime_error naming the first entry point that
// failed to resolve, rather than leaving a null function pointer for some
// later draw call to crash on.
void LoadGLFunctions();

}  // namespace achilles::render::gl
