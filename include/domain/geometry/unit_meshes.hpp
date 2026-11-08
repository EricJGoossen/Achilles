#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <span>
#include <utility>
#include <vector>

namespace achilles::domain::geometry {

// One unit shape as triangles: vertices within [-1, 1]^3, interleaved
// position(3)/normal(3), and 3 indices per triangle. The shapes a body's
// visual is drawn as (domain/geometry/visual_shape.hpp), scaled per body by
// its half-extents -- plain data, for whatever draws them to upload.
struct UnitMesh {
  std::span<const float> vertices;
  std::span<const std::uint32_t> indices;
};

namespace detail {

// clang-format off
// 24 vertices (4 per face x 6 faces, so each face gets its own flat
// normal rather than sharing a smoothed corner normal), interleaved
// position(3)/normal(3). No particular winding order is relied on --
// the camera GPU never enables back-face culling, so both a front and back
// interpretation of every triangle stays visible; only the normals (used
// for lighting, not culling) need to be right.
inline constexpr std::array<float, 24UL * 6UL> kCubeVertices = {
    // +X
     1,-1,-1,  1, 0, 0,
     1, 1,-1,  1, 0, 0,
     1, 1, 1,  1, 0, 0,
     1,-1, 1,  1, 0, 0,
    // -X
    -1,-1, 1, -1, 0, 0,
    -1, 1, 1, -1, 0, 0,
    -1, 1,-1, -1, 0, 0,
    -1,-1,-1, -1, 0, 0,
    // +Y
    -1, 1,-1,  0, 1, 0,
    -1, 1, 1,  0, 1, 0,
     1, 1, 1,  0, 1, 0,
     1, 1,-1,  0, 1, 0,
    // -Y
    -1,-1, 1,  0,-1, 0,
    -1,-1,-1,  0,-1, 0,
     1,-1,-1,  0,-1, 0,
     1,-1, 1,  0,-1, 0,
    // +Z
    -1,-1, 1,  0, 0, 1,
     1,-1, 1,  0, 0, 1,
     1, 1, 1,  0, 0, 1,
    -1, 1, 1,  0, 0, 1,
    // -Z
     1,-1,-1,  0, 0,-1,
    -1,-1,-1,  0, 0,-1,
    -1, 1,-1,  0, 0,-1,
     1, 1,-1,  0, 0,-1,
};
// clang-format on

inline constexpr std::array<std::uint32_t, 36> kCubeIndices = [] {
  std::array<std::uint32_t, 36> indices{};
  for (std::uint32_t face = 0; face < 6; ++face) {
    std::uint32_t base = face * 4;
    std::size_t offset = static_cast<std::size_t>(face) * 6;
    indices[offset + 0] = base + 0;
    indices[offset + 1] = base + 1;
    indices[offset + 2] = base + 2;
    indices[offset + 3] = base + 0;
    indices[offset + 4] = base + 2;
    indices[offset + 5] = base + 3;
  }
  return indices;
}();

// The sphere's grid: kSphereStacks bands from pole to pole, each split into
// kSphereSlices around. Fine enough to read as round at the sizes a
// rolling ball or a joint is drawn at.
inline constexpr std::uint32_t kSphereStacks = 16;
inline constexpr std::uint32_t kSphereSlices = 32;

// A unit sphere's vertices, interleaved position/normal -- the same vector
// twice, since a unit sphere's outward normal is its position. One vertex
// per grid point, (kSphereStacks + 1) rings of (kSphereSlices + 1), the
// seam's column duplicated so every band is a plain run of quads.
inline std::vector<float> SphereVertices() {
  std::vector<float> vertices;
  vertices.reserve(
      static_cast<std::size_t>(kSphereStacks + 1) * (kSphereSlices + 1) * 6
  );
  for (std::uint32_t stack = 0; stack <= kSphereStacks; ++stack) {
    float polar = std::numbers::pi_v<float> * static_cast<float>(stack) /
                  static_cast<float>(kSphereStacks);
    for (std::uint32_t slice = 0; slice <= kSphereSlices; ++slice) {
      float azimuth = 2.0F * std::numbers::pi_v<float> *
                      static_cast<float>(slice) /
                      static_cast<float>(kSphereSlices);
      float x = std::sin(polar) * std::cos(azimuth);
      float y = std::cos(polar);
      float z = std::sin(polar) * std::sin(azimuth);
      vertices.insert(vertices.end(), {x, y, z, x, y, z});
    }
  }
  return vertices;
}

// Two triangles per grid quad, indexing SphereVertices' rings.
inline std::vector<std::uint32_t> SphereIndices() {
  std::vector<std::uint32_t> indices;
  indices.reserve(static_cast<std::size_t>(kSphereStacks) * kSphereSlices * 6);
  constexpr std::uint32_t kRing = kSphereSlices + 1;
  for (std::uint32_t stack = 0; stack < kSphereStacks; ++stack) {
    for (std::uint32_t slice = 0; slice < kSphereSlices; ++slice) {
      std::uint32_t top = (stack * kRing) + slice;
      std::uint32_t bottom = top + kRing;
      indices.insert(
          indices.end(), {top, bottom, top + 1, top + 1, bottom, bottom + 1}
      );
    }
  }
  return indices;
}

// The cylinder's side wall is split into kCylinderSlices around -- the same
// count as the sphere's, so the two read as equally round side by side.
inline constexpr std::uint32_t kCylinderSlices = kSphereSlices;

// Appends one quad (4 vertices, 2 triangles) with a single flat normal.
inline void AppendQuad(
    std::vector<float>& vertices,
    std::vector<std::uint32_t>& indices,
    const std::array<std::array<float, 3>, 4>& corners,
    const std::array<float, 3>& normal
) {
  auto base = static_cast<std::uint32_t>(vertices.size() / 6);
  for (const std::array<float, 3>& corner : corners) {
    vertices.insert(
        vertices.end(),
        {corner[0], corner[1], corner[2], normal[0], normal[1], normal[2]}
    );
  }
  indices.insert(
      indices.end(), {base, base + 1, base + 2, base, base + 2, base + 3}
  );
}

// A unit cylinder about Y: the side wall is one ring of vertices at each
// end whose normals point straight out (x, 0, z), the seam's column
// duplicated like the sphere's; each cap is a fan around its own center
// vertex, with its own flat +-Y normal rather than sharing the wall's.
inline std::pair<std::vector<float>, std::vector<std::uint32_t>>
CylinderGeometry() {
  std::vector<float> vertices;
  std::vector<std::uint32_t> indices;
  for (std::uint32_t slice = 0; slice <= kCylinderSlices; ++slice) {
    float azimuth = 2.0F * std::numbers::pi_v<float> *
                    static_cast<float>(slice) /
                    static_cast<float>(kCylinderSlices);
    float x = std::cos(azimuth);
    float z = std::sin(azimuth);
    vertices.insert(vertices.end(), {x, -1, z, x, 0, z, x, 1, z, x, 0, z});
  }
  for (std::uint32_t slice = 0; slice < kCylinderSlices; ++slice) {
    std::uint32_t bottom = slice * 2;
    std::uint32_t top = bottom + 1;
    indices.insert(
        indices.end(), {bottom, top, bottom + 2, bottom + 2, top, top + 2}
    );
  }
  for (float y : {-1.0F, 1.0F}) {
    auto center = static_cast<std::uint32_t>(vertices.size() / 6);
    vertices.insert(vertices.end(), {0, y, 0, 0, y, 0});
    for (std::uint32_t slice = 0; slice <= kCylinderSlices; ++slice) {
      float azimuth = 2.0F * std::numbers::pi_v<float> *
                      static_cast<float>(slice) /
                      static_cast<float>(kCylinderSlices);
      vertices.insert(
          vertices.end(), {std::cos(azimuth), y, std::sin(azimuth), 0, y, 0}
      );
    }
    for (std::uint32_t slice = 0; slice < kCylinderSlices; ++slice) {
      indices.insert(
          indices.end(), {center, center + 1 + slice, center + 2 + slice}
      );
    }
  }
  return {std::move(vertices), std::move(indices)};
}

// CylinderStripe's quads. kStripeHalfWidth is in the unit cylinder's own
// radii, and kStripeLift how far outside the surface the marking sits --
// scaled with the cylinder like everything else, so both stay proportional
// to the wheel whatever its size.
inline std::pair<std::vector<float>, std::vector<std::uint32_t>>
CylinderStripeGeometry() {
  constexpr float kStripeHalfWidth = 0.15F;
  constexpr float kStripeLift = 1.02F;
  constexpr float kW = kStripeHalfWidth;
  constexpr float kL = kStripeLift;
  std::vector<float> vertices;
  std::vector<std::uint32_t> indices;
  // Side wall, along the rim at +X.
  AppendQuad(
      vertices,
      indices,
      {{{kL, -kL, -kW}, {kL, kL, -kW}, {kL, kL, kW}, {kL, -kL, kW}}},
      {1, 0, 0}
  );
  // A spoke on each cap, from the axis out to the side-wall stripe.
  for (float y : {-kL, kL}) {
    AppendQuad(
        vertices,
        indices,
        {{{0, y, -kW}, {kL, y, -kW}, {kL, y, kW}, {0, y, kW}}},
        {0, y > 0 ? 1.0F : -1.0F, 0}
    );
  }
  return {std::move(vertices), std::move(indices)};
}

}  // namespace detail

// A unit cube, one position+normal per corner-face vertex so face normals
// stay flat-shaded.
inline UnitMesh UnitCube() {
  return {detail::kCubeVertices, detail::kCubeIndices};
}

// A unit sphere, a latitude/longitude grid whose normals are its positions,
// so it shades smoothly.
inline UnitMesh UnitSphere() {
  static const std::vector<float> kVertices = detail::SphereVertices();
  static const std::vector<std::uint32_t> kIndices = detail::SphereIndices();
  return {kVertices, kIndices};
}

// A unit cylinder about the Y axis (radius 1, y in [-1, 1]): a smoothly
// shaded side wall plus two flat-shaded end caps.
inline UnitMesh UnitCylinder() {
  static const auto kGeometry = detail::CylinderGeometry();
  return {kGeometry.first, kGeometry.second};
}

// The marking drawn over UnitCylinder() in a contrasting color so its spin
// stays visible: one stripe along the side wall at +X, and a spoke from the
// axis out to that stripe on each cap. It sits just outside the cylinder's
// own surface, so the two never z-fight.
inline UnitMesh UnitCylinderStripe() {
  static const auto kGeometry = detail::CylinderStripeGeometry();
  return {kGeometry.first, kGeometry.second};
}

}  // namespace achilles::domain::geometry
