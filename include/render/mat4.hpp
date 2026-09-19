#pragma once

#include <cmath>

#include "domain/math/matrix.hpp"
#include "domain/math/vector3.hpp"
#include "domain/spatial/transform.hpp"

namespace achilles::render {

// The renderer's own 4x4, scalar-float math -- deliberately not built on
// algorithms::conventions' MathematicalT (an xsimd::batch<float> lane
// group meant for the sim's own per-joint SIMD kernels). A frame's worth
// of camera/model matrices is a handful of values, not a lane-parallel
// workload, and GL's own uniform upload wants one plain float[16] per
// matrix regardless -- so this stays scalar throughout, reusing
// domain::math::Matrix<float, 4, 4> for its storage/multiply rather than
// inventing a second matrix type. Header-only and GL-free on purpose: this
// is the part of the visualization module that's actually unit-testable
// without a live GL context/window (see tests/render_mat4.cpp), unlike
// shader/mesh/window/scene_renderer, which need one.
using Mat4 = domain::math::Matrix<float, 4, 4>;
using Vec3 = domain::math::Vector3<float>;

// Row-major storage (Matrix<T, M, N>::operator()(i, j) at data_[i*N+j]),
// addressed with the conventional column-vector convention every formula
// below is written in (clip = Projection * View * Model * point) -- so
// each function here is just the textbook matrix, entered row by row via
// Mat4's own row-major variadic constructor. GL's own uniform upload
// (glUniformMatrix4fv) wants column-major by default; scene_renderer.cpp
// uploads with transpose=GL_TRUE specifically so this storage order never
// has to be hand-transposed first.

inline Mat4 Identity() { return Mat4::Identity(); }

// Right-handed perspective projection, OpenGL NDC (z in [-1, 1] after the
// divide) -- the same matrix classically presented via gluPerspective,
// just written out directly rather than built up from cotangent helpers.
inline Mat4 Perspective(
    float fovy_radians, float aspect, float znear, float zfar
) {
  float f = 1.0F / std::tan(fovy_radians * 0.5F);
  float nf_inv = 1.0F / (znear - zfar);
  // clang-format off
  return Mat4{
      f / aspect, 0.0F, 0.0F,                          0.0F,
      0.0F,       f,    0.0F,                          0.0F,
      0.0F,       0.0F, (zfar + znear) * nf_inv,        2.0F * zfar * znear * nf_inv,
      0.0F,       0.0F, -1.0F,                          0.0F
  };
  // clang-format on
}

// Right-handed view matrix (the gluLookAt-equivalent): orthonormal
// (right, true_up, -forward) basis, with each row's own translation
// column folded in as -dot(axis, eye) so the whole transform maps `eye`
// to the origin in one matrix, not a separate rotate-then-translate pair.
inline Mat4 LookAt(const Vec3& eye, const Vec3& target, const Vec3& up) {
  Vec3 forward = (target - eye).Normalize();
  Vec3 right = forward.Cross(up).Normalize();
  Vec3 true_up = right.Cross(forward);

  // clang-format off
  return Mat4{
      right.X(),    right.Y(),    right.Z(),    -right.Dot(eye),
      true_up.X(),  true_up.Y(),  true_up.Z(),  -true_up.Dot(eye),
      -forward.X(), -forward.Y(), -forward.Z(), forward.Dot(eye),
      0.0F,         0.0F,         0.0F,         1.0F
  };
  // clang-format on
}

// A model matrix from a rigid-body pose plus a per-axis scale applied in
// the pose's own local frame (rotation * diag(scale), so an anisotropic
// scale still rotates with the joint rather than smearing along world
// axes) -- what scene_renderer.cpp uses to place each joint's unit cube
// mesh (vertices in [-1, 1]^3) at its own world transform, sized by its
// own kVisualExtents half-extents (see algorithms/viz/viz_data.hpp).
inline Mat4 FromTransformAndScale(
    const domain::spatial::Transform<float>& transform, const Vec3& scale
) {
  domain::math::Matrix<float, 3, 3> rotation =
      transform.Rotation().ToRotationMatrix();
  const domain::math::Vector3<float>& t = transform.Translation();

  Mat4 m = Mat4::Identity();
  for (std::size_t row = 0; row < 3; ++row) {
    m(row, 0) = rotation(row, 0) * scale.X();
    m(row, 1) = rotation(row, 1) * scale.Y();
    m(row, 2) = rotation(row, 2) * scale.Z();
  }
  m(0, 3) = t.X();
  m(1, 3) = t.Y();
  m(2, 3) = t.Z();
  return m;
}

}  // namespace achilles::render
