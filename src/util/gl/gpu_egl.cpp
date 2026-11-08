#include <EGL/egl.h>
#include <EGL/eglext.h>

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <map>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "util/gl/gl.hpp"
#include "util/gl/gpu.hpp"
#include "util/gl/mesh.hpp"
#include "util/gl/shader.hpp"

// The camera sensors' GPU (util/gl/gpu.hpp), on OpenGL 3.3 core
// through EGL: a context with no surface at all -- no window, no display
// server -- found on EGL's device platform (any GPU, or Mesa's software
// device) or Mesa's surfaceless platform, whichever comes first.
namespace achilles::util::gl {

namespace {

// An Instance's model matrix then rgb, as SolidMesh uploads them.
static_assert(SolidMesh::kInstanceFloats == 16 + 3);

constexpr std::string_view kVertexSource = R"(#version 330 core
layout(location = 0) in vec3 a_pos;
layout(location = 1) in vec3 a_normal;
layout(location = 2) in mat4 a_model;
layout(location = 6) in vec3 a_color;
uniform mat4 u_view;
uniform mat4 u_proj;
out vec3 v_world_normal;
out vec3 v_color;
out float v_depth;
void main() {
  vec4 world = a_model * vec4(a_pos, 1.0);
  vec4 eye = u_view * world;
  v_world_normal = mat3(a_model) * a_normal;
  v_color = a_color;
  v_depth = -eye.z;
  gl_Position = u_proj * eye;
}
)";

// Attachment 0 the lit color, attachment 1 the depth along the view axis
// (the eye's -z), in meters.
constexpr std::string_view kFragmentSource = R"(#version 330 core
in vec3 v_world_normal;
in vec3 v_color;
in float v_depth;
layout(location = 0) out vec4 frag_color;
layout(location = 1) out float frag_depth;
void main() {
  vec3 n = normalize(v_world_normal);
  vec3 light_dir = normalize(vec3(0.5, 1.0, 0.35));
  float diffuse = max(dot(n, light_dir), 0.0);
  frag_color = vec4(v_color * (0.35 + 0.65 * diffuse), 1.0);
  frag_depth = v_depth;
}
)";

[[noreturn]] void Fail(const std::string& what) {
  throw std::runtime_error("achilles camera GPU: " + what);
}

template <typename Fn>
Fn LoadEgl(const char* name) {
  auto* proc = eglGetProcAddress(name);
  return proc == nullptr ? nullptr : std::bit_cast<Fn>(proc);
}

// An OpenGL 3.3 core context on `display`, or EGL_NO_CONTEXT.
EGLContext CreateContext(EGLDisplay display) {
  EGLint major = 0;
  EGLint minor = 0;
  if (eglInitialize(display, &major, &minor) == EGL_FALSE) {
    return EGL_NO_CONTEXT;
  }
  if (eglBindAPI(EGL_OPENGL_API) == EGL_FALSE) {
    return EGL_NO_CONTEXT;
  }
  const std::array<EGLint, 7> attributes = {
      EGL_CONTEXT_MAJOR_VERSION,
      3,
      EGL_CONTEXT_MINOR_VERSION,
      3,
      EGL_CONTEXT_OPENGL_PROFILE_MASK,
      EGL_CONTEXT_OPENGL_CORE_PROFILE_BIT,
      EGL_NONE
  };
  EGLContext context = eglCreateContext(
      display, EGL_NO_CONFIG_KHR, EGL_NO_CONTEXT, attributes.data()
  );
  if (context == EGL_NO_CONTEXT) {
    eglTerminate(display);
  }
  return context;
}

// The headless context: the first device that makes one, else Mesa's
// surfaceless platform.
std::pair<EGLDisplay, EGLContext> OpenHeadless() {
  auto get_platform_display =
      LoadEgl<PFNEGLGETPLATFORMDISPLAYEXTPROC>("eglGetPlatformDisplayEXT");
  if (get_platform_display == nullptr) {
    Fail("EGL has no eglGetPlatformDisplayEXT.");
  }
  auto query_devices = LoadEgl<PFNEGLQUERYDEVICESEXTPROC>("eglQueryDevicesEXT");
  if (query_devices != nullptr) {
    std::array<EGLDeviceEXT, 8> devices{};
    EGLint count = 0;
    if (query_devices(
            static_cast<EGLint>(devices.size()), devices.data(), &count
        ) == EGL_TRUE) {
      for (EGLint i = 0; i < count; ++i) {
        EGLDisplay display = get_platform_display(
            EGL_PLATFORM_DEVICE_EXT,
            devices[static_cast<std::size_t>(i)],
            nullptr
        );
        if (display == EGL_NO_DISPLAY) {
          continue;
        }
        EGLContext context = CreateContext(display);
        if (context != EGL_NO_CONTEXT) {
          return {display, context};
        }
      }
    }
  }
  EGLDisplay display = get_platform_display(
      EGL_PLATFORM_SURFACELESS_MESA, EGL_DEFAULT_DISPLAY, nullptr
  );
  if (display != EGL_NO_DISPLAY) {
    EGLContext context = CreateContext(display);
    if (context != EGL_NO_CONTEXT) {
      return {display, context};
    }
  }
  Fail(
      "no EGL device or surfaceless platform could make an OpenGL 3.3 core "
      "context."
  );
}

// Owns the EGL display and context, and makes the context current --
// constructed before, and so destroyed after, every GL object of Gpu's.
class HeadlessContext {
 public:
  HeadlessContext() {
    std::tie(display_, context_) = OpenHeadless();
    MakeCurrent();
    gl::LoadGLFunctions();
  }

  // Never destroyed: it lives with Gpu, for the process (see Gpu::Get).
  ~HeadlessContext() = default;
  HeadlessContext(const HeadlessContext&) = delete;
  HeadlessContext& operator=(const HeadlessContext&) = delete;
  HeadlessContext(HeadlessContext&&) = delete;
  HeadlessContext& operator=(HeadlessContext&&) = delete;

  // Current already (the usual case: one sim stepping) costs one query.
  void MakeCurrent() const {
    if (eglGetCurrentContext() == context_) {
      return;
    }
    if (eglMakeCurrent(display_, EGL_NO_SURFACE, EGL_NO_SURFACE, context_) ==
        EGL_FALSE) {
      Fail("eglMakeCurrent failed.");
    }
  }

 private:
  EGLDisplay display_ = EGL_NO_DISPLAY;
  EGLContext context_ = EGL_NO_CONTEXT;
};

// One image size's framebuffer: lit color, depth along the view axis, and
// the depth buffer the draw tests against. Shared by every camera of that
// size -- each copies its image out into its own pixel buffers right
// after drawing.
struct Target {
  GLuint framebuffer = 0;
  GLuint color = 0;
  GLuint depth = 0;
  GLuint depth_buffer = 0;
};

// One camera's readback: two pairs of pixel pack buffers (rgb and depth),
// alternated render to render, so the previous render's pixels can be
// copied out while this one's are still on their way.
struct Readbacks {
  std::array<GLuint, 2> rgb{};
  std::array<GLuint, 2> depth{};
  std::array<std::uint32_t, 2> width{};
  std::array<std::uint32_t, 2> height{};
  int last = -1;
  int previous = -1;
  std::uint64_t last_used = 0;
};

class Gpu {
 public:
  // Made on first use and never destroyed: it lives for the process, and
  // tearing GL down during static destruction -- after whatever else has
  // already gone -- is a crash waiting to happen for nothing gained.
  // Use Get(): public only for it to make the one Gpu with.
  Gpu() : shader_(kVertexSource, kFragmentSource) {}

  static Gpu& Get() {
    static Gpu* const kGpu = std::make_unique<Gpu>().release();
    return *kGpu;
  }

  void Render(
      std::uint64_t key,
      const CameraView& view,
      std::span<const MeshData> meshes,
      std::span<const Instance> instances
  ) {
    context_.MakeCurrent();
    // Each mesh's instances, packed the way SolidMesh uploads them.
    batches_.resize(meshes.size());
    for (std::vector<float>& batch : batches_) {
      batch.clear();
    }
    for (const Instance& instance : instances) {
      std::vector<float>& batch = batches_.at(instance.mesh);
      batch.insert(batch.end(), instance.model.begin(), instance.model.end());
      batch.insert(batch.end(), instance.rgb.begin(), instance.rgb.end());
    }
    const Target& target = TargetFor(view.width, view.height);
    const auto width = static_cast<GLsizei>(view.width);
    const auto height = static_cast<GLsizei>(view.height);

    gl::g_functions.bind_framebuffer(gl::kFramebuffer, target.framebuffer);
    glViewport(0, 0, width, height);
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    constexpr std::array<GLenum, 2> kDrawBuffers = {
        gl::kColorAttachment0, gl::kColorAttachment1
    };
    gl::g_functions.draw_buffers(2, kDrawBuffers.data());
    const std::array<GLfloat, 4> background = {
        view.background[0], view.background[1], view.background[2], 1.0F
    };
    const std::array<GLfloat, 4> nothing = {
        std::numeric_limits<float>::infinity(), 0.0F, 0.0F, 0.0F
    };
    gl::g_functions.clear_bufferfv(GL_COLOR, 0, background.data());
    gl::g_functions.clear_bufferfv(GL_COLOR, 1, nothing.data());
    glClear(GL_DEPTH_BUFFER_BIT);

    shader_.Use();
    shader_.SetMat4("u_view", view.view);
    shader_.SetMat4("u_proj", view.projection);
    for (std::size_t i = 0; i < meshes.size(); ++i) {
      SolidMesh& mesh = MeshFor(meshes[i]);
      mesh.SetInstances(batches_[i]);
      mesh.Draw();
    }

    Readbacks& readbacks = ReadbacksFor(key);
    const int slot = readbacks.last == 0 ? 1 : 0;
    const std::size_t pixels = std::size_t{view.width} * view.height;
    SizeBuffer(readbacks.rgb[static_cast<std::size_t>(slot)], pixels * 3);
    SizeBuffer(
        readbacks.depth[static_cast<std::size_t>(slot)], pixels * sizeof(float)
    );
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    gl::g_functions.bind_buffer(
        gl::kPixelPackBuffer, readbacks.rgb[static_cast<std::size_t>(slot)]
    );
    glReadBuffer(gl::kColorAttachment0);
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, nullptr);
    gl::g_functions.bind_buffer(
        gl::kPixelPackBuffer, readbacks.depth[static_cast<std::size_t>(slot)]
    );
    glReadBuffer(gl::kColorAttachment1);
    glReadPixels(0, 0, width, height, gl::kRed, GL_FLOAT, nullptr);
    gl::g_functions.bind_buffer(gl::kPixelPackBuffer, 0);
    gl::g_functions.bind_framebuffer(gl::kFramebuffer, 0);
    glFlush();

    readbacks.width[static_cast<std::size_t>(slot)] = view.width;
    readbacks.height[static_cast<std::size_t>(slot)] = view.height;
    readbacks.previous = readbacks.last;
    readbacks.last = slot;
  }

  bool Readback(
      std::uint64_t key,
      bool pipelined,
      std::span<std::uint8_t> rgb,
      std::span<float> depth
  ) {
    context_.MakeCurrent();
    auto found = readbacks_.find(key);
    if (found == readbacks_.end()) {
      return false;
    }
    const Readbacks& readbacks = found->second;
    const int slot = pipelined ? readbacks.previous : readbacks.last;
    if (slot < 0) {
      return false;
    }
    const auto index = static_cast<std::size_t>(slot);
    const std::uint32_t width = readbacks.width[index];
    const std::uint32_t height = readbacks.height[index];
    const std::size_t pixels = std::size_t{width} * height;
    // A camera whose image size changed since (SetSensorConfig) has no
    // image of its new size yet.
    if (rgb.size() != pixels * 3 || depth.size() != pixels) {
      return false;
    }
    CopyRows(readbacks.rgb[index], std::size_t{width} * 3, height, rgb);
    CopyRows(readbacks.depth[index], width, height, depth);
    return true;
  }

 private:
  // A camera's buffers not rendered into for this many renders are freed
  // -- a sim re-Init()ed, or one gone, leaves its cameras' behind.
  static constexpr std::uint64_t kStaleAfter = 4096;

  // `data`'s mesh on the GPU, uploaded the first time it's drawn.
  SolidMesh& MeshFor(const MeshData& data) {
    auto found = meshes_.find(data.vertices.data());
    if (found == meshes_.end()) {
      found =
          meshes_
              .emplace(
                  data.vertices.data(), SolidMesh(data.vertices, data.indices)
              )
              .first;
    }
    return found->second;
  }

  const Target& TargetFor(std::uint32_t width, std::uint32_t height) {
    auto [found, made] = targets_.try_emplace({width, height});
    Target& target = found->second;
    if (!made) {
      return target;
    }
    const auto w = static_cast<GLsizei>(width);
    const auto h = static_cast<GLsizei>(height);
    gl::g_functions.gen_framebuffers(1, &target.framebuffer);
    gl::g_functions.bind_framebuffer(gl::kFramebuffer, target.framebuffer);

    glGenTextures(1, &target.color);
    glBindTexture(GL_TEXTURE_2D, target.color);
    glTexImage2D(
        GL_TEXTURE_2D, 0, GL_RGBA8, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr
    );
    gl::g_functions.framebuffer_texture_2d(
        gl::kFramebuffer, gl::kColorAttachment0, GL_TEXTURE_2D, target.color, 0
    );

    glGenTextures(1, &target.depth);
    glBindTexture(GL_TEXTURE_2D, target.depth);
    glTexImage2D(
        GL_TEXTURE_2D,
        0,
        static_cast<GLint>(gl::kR32F),
        w,
        h,
        0,
        gl::kRed,
        GL_FLOAT,
        nullptr
    );
    gl::g_functions.framebuffer_texture_2d(
        gl::kFramebuffer, gl::kColorAttachment1, GL_TEXTURE_2D, target.depth, 0
    );
    glBindTexture(GL_TEXTURE_2D, 0);

    gl::g_functions.gen_renderbuffers(1, &target.depth_buffer);
    gl::g_functions.bind_renderbuffer(gl::kRenderbuffer, target.depth_buffer);
    gl::g_functions.renderbuffer_storage(
        gl::kRenderbuffer, gl::kDepthComponent24, w, h
    );
    gl::g_functions.framebuffer_renderbuffer(
        gl::kFramebuffer,
        gl::kDepthAttachment,
        gl::kRenderbuffer,
        target.depth_buffer
    );

    if (gl::g_functions.check_framebuffer_status(gl::kFramebuffer) !=
        gl::kFramebufferComplete) {
      Fail(
          "a " + std::to_string(width) + "x" + std::to_string(height) +
          " render target is incomplete."
      );
    }
    gl::g_functions.bind_framebuffer(gl::kFramebuffer, 0);
    return target;
  }

  Readbacks& ReadbacksFor(std::uint64_t key) {
    ++renders_;
    if (renders_ % kStaleAfter == 0) {
      FreeStale();
    }
    auto [found, made] = readbacks_.try_emplace(key);
    Readbacks& readbacks = found->second;
    if (made) {
      gl::g_functions.gen_buffers(2, readbacks.rgb.data());
      gl::g_functions.gen_buffers(2, readbacks.depth.data());
    }
    readbacks.last_used = renders_;
    return readbacks;
  }

  void FreeStale() {
    for (auto it = readbacks_.begin(); it != readbacks_.end();) {
      if (renders_ - it->second.last_used > kStaleAfter) {
        gl::g_functions.delete_buffers(2, it->second.rgb.data());
        gl::g_functions.delete_buffers(2, it->second.depth.data());
        it = readbacks_.erase(it);
      } else {
        ++it;
      }
    }
  }

  // Gives a pixel pack buffer room for `bytes` (contents undefined).
  static void SizeBuffer(GLuint buffer, std::size_t bytes) {
    gl::g_functions.bind_buffer(gl::kPixelPackBuffer, buffer);
    gl::g_functions.buffer_data(
        gl::kPixelPackBuffer,
        static_cast<gl::GLsizeiptr>(bytes),
        nullptr,
        gl::kStreamRead
    );
  }

  // Copies a pixel pack buffer's rows into `out`, flipped: GL reads the
  // bottom row first, an image starts at its top.
  template <typename T>
  static void CopyRows(
      GLuint buffer, std::size_t row_values, std::size_t rows, std::span<T> out
  ) {
    const std::size_t bytes = row_values * rows * sizeof(T);
    gl::g_functions.bind_buffer(gl::kPixelPackBuffer, buffer);
    const void* mapped = gl::g_functions.map_buffer_range(
        gl::kPixelPackBuffer,
        0,
        static_cast<gl::GLsizeiptr>(bytes),
        gl::kMapReadBit
    );
    if (mapped == nullptr) {
      gl::g_functions.bind_buffer(gl::kPixelPackBuffer, 0);
      Fail("mapping a camera's pixels failed.");
    }
    const auto* source = static_cast<const std::byte*>(mapped);
    const std::size_t row_bytes = row_values * sizeof(T);
    for (std::size_t row = 0; row < rows; ++row) {
      std::memcpy(
          out.data() + (row * row_values),
          source + ((rows - 1 - row) * row_bytes),
          row_bytes
      );
    }
    gl::g_functions.unmap_buffer(gl::kPixelPackBuffer);
    gl::g_functions.bind_buffer(gl::kPixelPackBuffer, 0);
  }

  HeadlessContext context_;
  Shader shader_;
  std::unordered_map<const float*, SolidMesh> meshes_;
  // Reused draw to draw, so packing allocates only as a scene grows.
  std::vector<std::vector<float>> batches_;
  std::map<std::pair<std::uint32_t, std::uint32_t>, Target> targets_;
  std::unordered_map<std::uint64_t, Readbacks> readbacks_;
  std::uint64_t renders_ = 0;
};

}  // namespace

void Render(
    std::uint64_t key,
    const CameraView& view,
    std::span<const MeshData> meshes,
    std::span<const Instance> instances
) {
  Gpu::Get().Render(key, view, meshes, instances);
}

bool Readback(
    std::uint64_t key,
    bool pipelined,
    std::span<std::uint8_t> rgb,
    std::span<float> depth
) {
  return Gpu::Get().Readback(key, pipelined, rgb, depth);
}

}  // namespace achilles::util::gl
