#pragma once

#include <cstdint>
#include <string_view>

#include "algorithms/conventions.hpp"
#include "algorithms/shared_slots.hpp"
#include "domain/joint_topology.hpp"
#include "domain/math/scalar.hpp"
#include "domain/math/vector3.hpp"
#include "domain/spatial/transform.hpp"
#include "engine/field_contract.hpp"
#include "engine/topology/layout_policy.hpp"
#include "engine/topology/orderings.hpp"
#include "engine/view/view_contract.hpp"
#include "engine/view/view_factory.hpp"

namespace achilles::algorithms::viz {

// The visual-representation half of a joint's per-tick state: where it is
// (kWorldTransform, read-only here -- ABA's own Propagate*VelocityOp is
// still the only thing that ever writes it) and how to draw it
// (kVisualPose/kVisualExtents/kVisualColor/kVisualShape, static per-archetype
// appearance data, set once at load time and never mutated at runtime). Like
// VIField/PIField, this has nothing to do with *simulating* the system -- see
// VizAlgorithm in viz_step.hpp, which has no Step at all for exactly that
// reason: there is no per-tick computation of its own to run, only state other
// algorithms (ABA) or the archetype loader populate.
//
// A joint with no "visual_extents" given in its archetype gets
// PlanarLayout's/Vector3's own zero-fill default (Vector3::Zero()), which a
// renderer is expected to treat as "draw nothing for this joint" -- the
// same "absent data reads as an inert default" convention every other
// field in this codebase already follows (see e.g. Arena::ZeroFill's own
// comment), rather than a separate present/absent flag. Likewise a joint
// with no "visual_shape" reads as 0, VisualShape::kBox, and one with no
// "visual_pose" reads as a zero transform, whose rotation part a renderer
// reads as identity (see ToVisualPose) -- the same convention broad_phase's
// own kCollisionPose follows (broad_phase_data.hpp) -- so the shape just sits
// on the joint frame.
enum class VizField : uint8_t {
  kWorldTransform,  // x_world -- ABA's own kWorldTransform, read-only here
  kVisualPose,      // x_visual (the drawn shape's frame, in the joint's own)
  kVisualExtents,   // half-extents of the shape drawn at this joint, or
                    // Zero() to draw nothing
  kVisualColor,     // rgb in [0, 1]; Zero() lets a renderer pick a fallback
  kVisualShape,     // which shape to draw, a VisualShape code
  kCount
};

// What kVisualShape's code means. Stored as a plain scalar, since archetype
// data is read as doubles (see SimAllocator's Populate pass) and no integer
// scalar field exists to read it into; a reader rounds it (ShapeOf). Every
// shape is scaled by kVisualExtents and drawn from kVisualPose's frame outward
// along that frame's +Y, so a sphere of radius r has extents (r, r, r) and
// sits with its bottom on the frame's origin. A cylinder's axis is that same
// +Y: extents (r, w, r) draw a disc of radius r and thickness 2w, with a
// contrasting stripe across its faces and rim so a spinning wheel visibly
// turns.
enum class VisualShape : uint8_t {
  kBox = 0,
  kSphere = 1,
  kCylinder = 2,
};

// kVisualShape's code as a VisualShape -- anything unrecognized reads as a
// box, the same inert default an absent field gets.
inline VisualShape ShapeOf(ScalarOperationT code) {
  if (code > 0.5F && code < 1.5F) {
    return VisualShape::kSphere;
  }
  if (code > 1.5F && code < 2.5F) {
    return VisualShape::kCylinder;
  }
  return VisualShape::kBox;
}

// kVisualPose as a usable transform: an unset (zero) rotation reads as
// identity, by adding 1 to w exactly when the quaternion is zero, so
// composing it can't zero the joint's own rotation out -- the same fix
// broad_phase's ComputeSweptBoxBoundsOp applies to kCollisionPose.
template <typename T>
domain::spatial::Transform<T> ToVisualPose(
    const domain::spatial::Transform<T>& pose
) {
  const domain::math::Quaternion<T>& q = pose.Rotation();
  bool is_zero =
      q.W() == T{0} && q.X() == T{0} && q.Y() == T{0} && q.Z() == T{0};
  return {
      pose.Translation(),
      domain::math::Quaternion<T>(is_zero ? T{1} : q.W(), q.X(), q.Y(), q.Z())
  };
}
static_assert(engine::FieldEnumLike<VizField>);

template <VizField F>
struct VizFieldTraits;

// Every VizField binds Ordering = TopologicalOrdering, matching ABA's own
// choice for kWorldTransform (aba_data.hpp) -- not just for the shared
// field, but for kVisualPose/kVisualExtents/kVisualColor/kVisualShape too. A
// View addresses every one of its fields by the same "sorted row" index, so
// unless every field shares the exact ordering policy that produced that row
// numbering, row i of kVisualExtents and row i of kWorldTransform could
// silently refer to two different joints (see engine/memory/binding.hpp's own
// comment on FieldOrderingT). VI's own fields make the identical choice for the
// identical reason (vi_data.hpp).
template <>
struct VizFieldTraits<VizField::kWorldTransform> {
  // SharedAs = WorldTransformSlot so this is ABA's own kWorldTransform
  // block (aba_data.hpp), not a private, unconnected copy -- otherwise
  // this field would always read stale (zero-seeded) data, since nothing
  // in algorithms::viz itself ever writes it.
  using Assembler = spatial::TransformAssembler<ScalarOperationT>;
  using SharedAs = WorldTransformSlot;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
};
template <>
struct VizFieldTraits<VizField::kVisualPose> {
  using Assembler = spatial::TransformAssembler<ScalarOperationT>;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
  static constexpr std::string_view kName = "visual_pose";
};
template <>
struct VizFieldTraits<VizField::kVisualExtents> {
  using Assembler = math::Vector3Assembler<ScalarOperationT>;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
  static constexpr std::string_view kName = "visual_extents";
};
template <>
struct VizFieldTraits<VizField::kVisualColor> {
  using Assembler = math::Vector3Assembler<ScalarOperationT>;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
  static constexpr std::string_view kName = "visual_color";
};
template <>
struct VizFieldTraits<VizField::kVisualShape> {
  using Assembler = math::ScalarAssembler<ScalarOperationT>;
  using Ordering = engine::topology::TopologicalOrdering;
  using Layout = engine::topology::PlanarLayout;
  static constexpr std::string_view kName = "visual_shape";
};
static_assert(engine::FieldTraitsLike<VizField, VizFieldTraits>);

using VizView = engine::view::ViewFactory<VizField, VizFieldTraits>;
static_assert(engine::view::ViewLike<VizView, VizField>);

using VizTopology = domain::JointTopology;
static_assert(domain::TopologyLike<VizTopology>);

}  // namespace achilles::algorithms::viz
