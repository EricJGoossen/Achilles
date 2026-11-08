#include <cstdint>
#include <span>
#include <stdexcept>

#include "util/gl/gpu.hpp"

// The camera sensors' GPU in a build without one (ACHILLES_WITH_GPU=OFF):
// every entry point throws, so a scene with a camera fails its first
// sample -- Simulation::Step() warns and returns false -- and a scene
// without one never gets here.
namespace achilles::util::gl {

namespace {

[[noreturn]] void NoGpu() {
  throw std::runtime_error(
      "a camera sensor needs the GPU, and this build has none "
      "(ACHILLES_WITH_GPU=OFF)."
  );
}

}  // namespace

void Render(
    std::uint64_t /*key*/,
    const CameraView& /*view*/,
    std::span<const MeshData> /*meshes*/,
    std::span<const Instance> /*instances*/
) {
  NoGpu();
}

bool Readback(
    std::uint64_t /*key*/,
    bool /*pipelined*/,
    std::span<std::uint8_t> /*rgb*/,
    std::span<float> /*depth*/
) {
  NoGpu();
}

}  // namespace achilles::util::gl
