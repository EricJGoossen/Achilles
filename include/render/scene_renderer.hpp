#pragma once

#include <string_view>

#include "algorithms/viz/viz_data.hpp"
#include "domain/joint_topology.hpp"
#include "render/camera.hpp"
#include "render/mesh.hpp"
#include "render/shader.hpp"
#include "render/window.hpp"

namespace achilles::render {

// Ties every other piece of this module (Window, Camera, Shader, Mesh)
// into the one thing a caller actually wants: hand it a VizView plus the
// JointTopology it was resolved against (see viz_data.hpp/interface::
// Simulation::TopologyFor) once per tick, get a rendered frame. Owns
// everything it needs -- one lit shader for boxes, one flat shader for
// lines, one cube mesh reused (re-transformed) per visible joint, and two
// dynamic line meshes (a ground grid built once, bones rebuilt every
// frame) -- so src/render_main.cpp's own loop is just
// "Simulation::Step(dt); scene_renderer.RenderFrame(sim.ViewFor<...>(),
// sim.TopologyFor<...>())".
class SceneRenderer {
 public:
  explicit SceneRenderer(std::string_view title = "Achilles Viewer");

  bool ShouldClose() const;
  // Refreshes input state and orbits/zooms the camera from it -- call once
  // per frame, before RenderFrame.
  void PollInput();

  // Draws one frame: every row in [0, topology.Size()) whose
  // kVisualExtents is nonzero is drawn as an oriented box at its
  // kWorldTransform, sized by those extents and colored by kVisualColor
  // (falling back to a deterministic per-row color when kVisualColor is
  // exactly zero, so an archetype that sets extents but not color still
  // renders as something other than black); a bone line connects each
  // visible row to its own topology parent's world position. Requires
  // `view`'s kWorldTransform to already reflect a real Simulation::Step()
  // call -- see viz_data.hpp's own header comment.
  void RenderFrame(
      const algorithms::viz::VizView& view,
      const domain::JointTopology& topology
  );

  void SwapBuffers();

 private:
  Window window_;
  Camera camera_;
  Shader lit_shader_;
  Shader line_shader_;
  CubeMesh cube_;
  LineMesh grid_;
  LineMesh axis_x_;
  LineMesh axis_y_;
  LineMesh axis_z_;
  LineMesh bones_;
};

}  // namespace achilles::render
