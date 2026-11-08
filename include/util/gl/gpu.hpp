#pragma once

#include <array>
#include <cstdint>
#include <span>

namespace achilles::util::gl {

// The GPU the camera sensors render through: one process-wide headless
// context (EGL, no window), made on first use and confined to the thread
// that steps the sim. It knows nothing of the sim or of any one camera: a
// camera hands over the meshes to draw, the instances of them it sees, and
// its own view and projection, already worked out, and gets an image back.
// What it keeps between calls -- each mesh uploaded once, render targets
// pooled by size, each camera's readback buffers -- is a cache.
//
// GL-free on purpose: this header declares, src/util/gl/gpu_egl.cpp defines
// (ACHILLES_WITH_GPU), and gpu_none.cpp stands in without a GPU -- every
// entry point there throws, which only a scene with a camera ever reaches.

// One mesh to draw: vertices (interleaved position(3)/normal(3)) and
// indices (3 per triangle). Uploaded on first use and cached by its vertex
// data's address, so it must stay alive and unchanged for the process.
struct MeshData {
  std::span<const float> vertices;
  std::span<const std::uint32_t> indices;
};

// One copy of a mesh in the image: its model matrix (column-major, mesh to
// world), its rgb in [0, 1], and which of Render()'s meshes it is.
struct Instance {
  std::array<float, 16> model{};
  std::array<float, 3> rgb{};
  std::uint32_t mesh = 0;
};

// One camera's draw: where it looks from and how it projects, both
// column-major and in GL's own conventions (eye looking down -Z, clip
// space in [-1, 1]), the image size, and the color where nothing is hit.
struct CameraView {
  std::array<float, 16> view{};
  std::array<float, 16> projection{};
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::array<float, 3> background{};
};

// Draws `instances` of `meshes` from `view` into the render target pooled
// for its size, then starts reading it back into the pixel buffers kept for
// camera `key` (its random stream: unique per camera instance). Readback()
// collects the result. Throws std::out_of_range for an instance naming a
// mesh past `meshes`.
void Render(
    std::uint64_t key,
    const CameraView& view,
    std::span<const MeshData> meshes,
    std::span<const Instance> instances
);

// Copies camera `key`'s image into `rgb` (3 bytes per pixel) and `depth`
// (meters along the view axis, +inf where nothing was hit), rows top to
// bottom. `pipelined` copies the image of the Render() *before* the last
// one -- finished long ago, so this never waits on the GPU, at the cost of
// one sample's latency -- and returns false when there's none yet;
// otherwise it waits for the last Render() and returns true. Writes nothing
// on false.
bool Readback(
    std::uint64_t key,
    bool pipelined,
    std::span<std::uint8_t> rgb,
    std::span<float> depth
);

}  // namespace achilles::util::gl
