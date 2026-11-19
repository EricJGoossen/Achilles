#pragma once

// A camera's window the example programs share: its image, flown by hand.
#include <cstddef>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>

#include "display/controllers/fly_camera.hpp"
#include "display/renderer.hpp"
#include "display/views/camera/camera_view.hpp"
#include "domain/attachment/sim_quantities.hpp"
#include "domain/sensing/camera.hpp"
#include "interface/attachments.hpp"
#include "interface/simulation.hpp"

namespace achilles::examples {

// A FlyCamera for `camera`, as Simulation::Attachments() lists it: flying the
// joint it's attached to, along the world's up. Its joint should carry
// nothing but the camera (each example's camera_mount).
inline display::FlyCamera FlyCameraFor(
    const interface::Simulation& sim,
    const interface::NamedAttachment<domain::sensing::Camera>& camera
) {
  namespace input = domain::attachment::input;
  return {
      sim.State<input::JointPosition>(camera.joint),
      sim.State<const input::WorldTransform>(camera.joint),
      camera.handle.Config().GetParams().body_from_sensor,
      {.up = sim.Up()}
  };
}

// Opens a window titled `title` on camera `name`'s image (instance
// `instance`), flown by its keys and mouse (display::FlyCamera): W/A/S/D to
// move, E/Q up and down, Shift for faster, a drag to look around. Throws
// std::invalid_argument if the scene has no such camera.
inline display::WindowId AttachFlyingCamera(
    display::Renderer& renderer,
    const interface::Simulation& sim,
    std::string_view name,
    std::string_view title,
    std::size_t instance = 0
) {
  using Camera = domain::sensing::Camera;
  // Kept: a loop over std::get of the temporary itself would outlive it.
  const interface::AttachmentLists lists = sim.Attachments();
  for (const interface::NamedAttachment<Camera>& camera :
       std::get<interface::NamedAttachments<Camera>>(lists)) {
    if (camera.name == name && camera.instance == instance) {
      const display::WindowId window =
          renderer.Attach(display::CameraView(camera.handle), title);
      renderer.Control(window, FlyCameraFor(sim, camera));
      return window;
    }
  }
  throw std::invalid_argument(
      "no camera named '" + std::string(name) + "' to fly"
  );
}

}  // namespace achilles::examples
