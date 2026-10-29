#pragma once

#include <cstddef>
#include <span>

#include "render/gl.hpp"

namespace achilles::render {

// A lit, indexed triangle mesh of one unit shape (vertices within
// [-1, 1]^3, interleaved position(3)/normal(3)) -- the shapes visible
// joints are drawn as, scaled per joint by its own kVisualExtents
// half-extents (see mat4.hpp's own FromTransformAndScale). Cube(), Sphere()
// and Cylinder() build the shapes algorithms::viz::VisualShape names. Built
// and uploaded once; Draw() just binds and issues one indexed draw call, so
// a frame with many joints costs one draw call per joint, not one buffer
// upload per joint -- plenty fast for the joint counts an articulated-body
// sim actually reaches, and far simpler than instancing.
class SolidMesh {
 public:
  // A unit cube, one position+normal per corner-face vertex so face
  // normals stay flat-shaded.
  static SolidMesh Cube();
  // A unit sphere, a latitude/longitude grid whose normals are its
  // positions, so it shades smoothly.
  static SolidMesh Sphere();
  // A unit cylinder about the Y axis (radius 1, y in [-1, 1]): a smoothly
  // shaded side wall plus two flat-shaded end caps.
  static SolidMesh Cylinder();
  // The marking drawn over Cylinder() in a contrasting color so its spin
  // stays visible: one stripe along the side wall at +X, and a spoke from
  // the axis out to that stripe on each cap. It sits just outside the
  // cylinder's own surface, so the two never z-fight.
  static SolidMesh CylinderStripe();

  ~SolidMesh();

  SolidMesh(const SolidMesh&) = delete;
  SolidMesh& operator=(const SolidMesh&) = delete;
  // Movable (transfers the GL object names, same shape as Shader/Window's
  // own move) so a class embedding a SolidMesh -- e.g. SceneRenderer,
  // itself embedded in interface::Simulation -- stays movable too, rather
  // than forcing every owner up the chain to become an optional<T> the
  // caller can never relocate.
  SolidMesh(SolidMesh&& other) noexcept;
  SolidMesh& operator=(SolidMesh&& other) noexcept;

  void Draw() const;

 private:
  // Uploads `vertices` (interleaved position/normal, 6 floats per vertex)
  // and `indices` (3 per triangle).
  SolidMesh(std::span<const GLfloat> vertices, std::span<const GLuint> indices);

  GLuint vao_ = 0;
  GLuint vbo_ = 0;
  GLuint ebo_ = 0;
  GLsizei index_count_ = 0;
};

// A plain GL_LINES vertex buffer (position-only, xyz triples) re-uploaded
// every frame via Update -- used for both the per-frame bone/skeleton
// segments (which move every tick) and the one-time ground grid/axes
// gizmo (which just never changes after its first Update). Kept dynamic
// rather than split into a separate static-vs-dynamic mesh type: the
// vertex count/content is trivial to re-upload wholesale each frame, and
// a single type covers both cases without the added complexity of ever
// diverging.
class LineMesh {
 public:
  LineMesh();
  ~LineMesh();

  LineMesh(const LineMesh&) = delete;
  LineMesh& operator=(const LineMesh&) = delete;
  LineMesh(LineMesh&& other) noexcept;
  LineMesh& operator=(LineMesh&& other) noexcept;

  // `xyz` is a flat, tightly-packed list of vertex positions, 3 floats per
  // vertex, 2 vertices per segment (i.e. xyz.size() must be a multiple of
  // 6) -- drawn as GL_LINES, so every consecutive pair is one independent
  // segment, not a connected strip.
  void Update(std::span<const float> xyz);
  void Draw() const;

 private:
  GLuint vao_ = 0;
  GLuint vbo_ = 0;
  GLsizei vertex_count_ = 0;
};

}  // namespace achilles::render
