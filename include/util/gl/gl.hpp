#pragma once

// Minimal, hand-rolled OpenGL 3.3 core loader -- exactly the subset of GL
// Achilles calls, loaded through EGL's eglGetProcAddress once a context
// exists (see LoadGLFunctions, defined in gl.cpp). The one context
// Achilles makes is the camera sensors' headless EGL context
// (util/gl/gpu.hpp); under libglvnd the pointers EGL hands back dispatch to
// whichever context is current, so one load serves any context.
// A full loader (GLAD/glbinding/gl3w) needs either offline codegen or an
// extra dependency this module intentionally avoids.
//
// GL_VERSION_1_1 entry points (glEnable, glViewport, glDrawArrays,
// glClear, glReadPixels, glTexImage2D, ...) are declared by the system
// <GL/gl.h> and resolve at ordinary link time against libOpenGL -- called
// directly, unprefixed, wherever this module uses them. Only the post-1.1
// core Achilles needs (buffers, shaders, VAOs, programs, uniforms,
// framebuffers, instancing) is declared here as a runtime-loaded function
// pointer.

#include <GL/gl.h>

#include <cstddef>

namespace achilles::util::gl {

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
inline constexpr GLenum kStreamRead = 0x88E1;
inline constexpr GLenum kPixelPackBuffer = 0x88EB;
inline constexpr GLenum kFramebuffer = 0x8D40;
inline constexpr GLenum kRenderbuffer = 0x8D41;
inline constexpr GLenum kFramebufferComplete = 0x8CD5;
inline constexpr GLenum kColorAttachment0 = 0x8CE0;
inline constexpr GLenum kColorAttachment1 = 0x8CE1;
inline constexpr GLenum kDepthAttachment = 0x8D00;
inline constexpr GLenum kDepthComponent24 = 0x81A6;
inline constexpr GLenum kR32F = 0x822E;
inline constexpr GLenum kRed = 0x1903;
inline constexpr GLenum kTexture0 = 0x84C0;
inline constexpr GLenum kClampToEdge = 0x812F;
inline constexpr GLenum kMapReadBit = 0x0001;

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
using Uniform4fFn = void (*)(GLint, GLfloat, GLfloat, GLfloat, GLfloat);
using Uniform1iFn = void (*)(GLint, GLint);
using BufferSubDataFn =
    void (*)(GLenum, std::ptrdiff_t, GLsizeiptr, const void*);
using VertexAttribDivisorFn = void (*)(GLuint, GLuint);
using DrawElementsInstancedFn =
    void (*)(GLenum, GLsizei, GLenum, const void*, GLsizei);
using GenFramebuffersFn = void (*)(GLsizei, GLuint*);
using BindFramebufferFn = void (*)(GLenum, GLuint);
using DeleteFramebuffersFn = void (*)(GLsizei, const GLuint*);
using FramebufferTexture2DFn = void (*)(GLenum, GLenum, GLenum, GLuint, GLint);
using FramebufferRenderbufferFn = void (*)(GLenum, GLenum, GLenum, GLuint);
using CheckFramebufferStatusFn = GLenum (*)(GLenum);
using GenRenderbuffersFn = void (*)(GLsizei, GLuint*);
using BindRenderbufferFn = void (*)(GLenum, GLuint);
using RenderbufferStorageFn = void (*)(GLenum, GLenum, GLsizei, GLsizei);
using DeleteRenderbuffersFn = void (*)(GLsizei, const GLuint*);
using DrawBuffersFn = void (*)(GLsizei, const GLenum*);
using ClearBufferfvFn = void (*)(GLenum, GLint, const GLfloat*);
using MapBufferRangeFn = void* (*)(GLenum,
                                   std::ptrdiff_t,
                                   GLsizeiptr,
                                   GLbitfield);
using UnmapBufferFn = GLboolean (*)(GLenum);
using ActiveTextureFn = void (*)(GLenum);

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
  Uniform4fFn uniform4f = nullptr;
  Uniform1iFn uniform1i = nullptr;
  BufferSubDataFn buffer_sub_data = nullptr;
  VertexAttribDivisorFn vertex_attrib_divisor = nullptr;
  DrawElementsInstancedFn draw_elements_instanced = nullptr;
  GenFramebuffersFn gen_framebuffers = nullptr;
  BindFramebufferFn bind_framebuffer = nullptr;
  DeleteFramebuffersFn delete_framebuffers = nullptr;
  FramebufferTexture2DFn framebuffer_texture_2d = nullptr;
  FramebufferRenderbufferFn framebuffer_renderbuffer = nullptr;
  CheckFramebufferStatusFn check_framebuffer_status = nullptr;
  GenRenderbuffersFn gen_renderbuffers = nullptr;
  BindRenderbufferFn bind_renderbuffer = nullptr;
  RenderbufferStorageFn renderbuffer_storage = nullptr;
  DeleteRenderbuffersFn delete_renderbuffers = nullptr;
  DrawBuffersFn draw_buffers = nullptr;
  ClearBufferfvFn clear_bufferfv = nullptr;
  MapBufferRangeFn map_buffer_range = nullptr;
  UnmapBufferFn unmap_buffer = nullptr;
  ActiveTextureFn active_texture = nullptr;
};

inline FunctionTable g_functions;

// Resolves every pointer in g_functions via eglGetProcAddress. Must be
// called after a current GL context exists; a second call is a no-op.
// Throws std::runtime_error naming the first entry point that failed to
// resolve, rather than leaving a null function pointer for some later draw
// call to crash on.
void LoadGLFunctions();

}  // namespace achilles::util::gl
