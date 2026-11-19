// The pan-tilt example: the two-joint arm swinging under gravity, watched
// by a camera on a pan-tilt rig (rig.arow): the camera sensor's own demo, so
// both of what it outputs, each in its own window -- its color image
// (display::CameraView) and its depth (display::DepthView) -- both bound to
// the camera's one handle. Either window flies the camera
// (display::FlyCamera): W/A/S/D to move, E/Q up and down, Shift for faster,
// a drag to look around -- setting the pose of the rig's tilt joint, which
// carries nothing but the camera, outright. Closing the windows ends the
// run.
//
// Usage: pan_tilt <pan_tilt.arow> <sim_config.yaml>
#include <algorithm>
#include <chrono>
#include <cstddef>
#include <exception>
#include <iostream>
#include <span>
#include <string>

#include "../shared/flying_camera.hpp"
#include "display/renderer.hpp"
#include "display/views/camera/depth_view.hpp"
#include "domain/sensing/camera.hpp"
#include "interface/attachments.hpp"
#include "interface/simulation.hpp"

int main(int argc, char** argv) {
  const std::span<char*> args(argv, static_cast<std::size_t>(argc));
  if (args.size() < 3) {
    std::cerr << "usage: pan_tilt <pan_tilt.arow> <sim_config.yaml>\n";
    return 1;
  }
  achilles::interface::Simulation sim;
  if (!sim.LoadArowFile(args[1]) || !sim.LoadConfigFile(args[2]) ||
      !sim.Init()) {
    return 1;
  }
  sim.SetIntegrator(achilles::interface::Simulation::Integrator::kRungeKutta4);

  try {
    using achilles::domain::sensing::Camera;
    achilles::display::Renderer renderer("Achilles: pan-tilt camera");
    achilles::examples::AttachFlyingCamera(
        renderer, sim, "rig_camera", "Rig camera"
    );
    const auto camera = std::get<achilles::interface::NamedAttachments<Camera>>(
                            sim.Attachments()
    )
                            .front();
    renderer.Control(
        renderer.Attach(
            achilles::display::DepthView(camera.handle), "Rig camera depth"
        ),
        achilles::examples::FlyCameraFor(sim, camera)
    );

    // Real time: each frame, as many fixed steps as the wall clock covers,
    // clamped so a hitch doesn't turn into a burst.
    constexpr float kDt = 0.002F;
    constexpr float kMaxFrameTime = 0.25F;
    float behind = 0.0F;
    auto last = std::chrono::steady_clock::now();
    bool running = true;
    while (running) {
      const auto now = std::chrono::steady_clock::now();
      behind += std::min(
          std::chrono::duration<float>(now - last).count(), kMaxFrameTime
      );
      last = now;
      while (running && behind >= kDt) {
        renderer.ApplyWrites();
        running = sim.Step(kDt);
        behind -= kDt;
      }
      running = running && renderer.Draw();
    }
  } catch (const std::exception& e) {
    std::cerr << "pan_tilt: " << e.what() << '\n';
    return 1;
  }
  return 0;
}
