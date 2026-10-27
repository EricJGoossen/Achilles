#pragma once

namespace achilles::algorithms {

// Cross-algorithm shared-slot tags (see engine::memory::FieldIsShared and
// SimAllocator::Carve, engine/memory/binding.hpp/sim_allocator.hpp): one
// tag per physical per-joint quantity that more than one hosted
// Algorithm's own Field enum names its own copy of. Naming the same tag
// as another field's own SharedAs is what makes SimAllocator carve that
// field's block exactly once and hand every Algorithm naming it the same
// underlying memory -- without it, e.g. ABA's own kJointVelocity and VI's
// own kJointVelocity would each get their own private, unconnected block,
// and VI's integrated result would never actually reach ABA's next tick.
//
// Declared here, in their own header, rather than inside any one
// algorithm's own data header (aba_data.hpp/vi_data.hpp/pi_data.hpp) --
// every one of those needs to name the same type identity, and none of
// them is a natural owner of a tag the others also depend on.
//
// A tag two algorithms both name must also agree on that field's shape
// (Assembler's byte layout/alignment/lane count and Ordering policy) --
// SimAllocator::Carve asserts this at Build() time; it's not something
// this header can check on its own.
struct JointSubspaceSlot;
struct JointPositionSlot;
struct JointVelocitySlot;
struct JointAccelerationSlot;
// ABA's own kWorldTransform (aba_data.hpp) -- shared so a read-only
// consumer (e.g. algorithms::viz's own kWorldTransform, viz/viz_data.hpp,
// or algorithms::broad_phase's, broad_phase/broad_phase_data.hpp) can see the
// exact world pose ABA already computes every tick, instead of re-deriving it
// from kJointPosition/kFixedJointTransform itself.
struct WorldTransformSlot;
// The broad phase's collision shape (broad_phase/broad_phase_data.hpp):
// its static per-body description, the world pose it derives from it each
// tick, and the candidate-pair bitmask it ends on -- shared so the narrow
// phase (narrow_phase/narrow_phase_data.hpp) reads the same shapes, the
// same pose and only the pairs the broad phase kept, without either phase
// re-deriving the other's work.
struct CollisionExtentsSlot;
struct CollisionRadiusSlot;
struct CollisionMarginSlot;
struct WorldShapePoseSlot;
struct BoundsOverlapSlot;
// ABA's velocity-independent factors and per-body state
// (aba/aba_data.hpp) -- shared so the contact solver
// (contact_solver/contact_solver_data.hpp) can apply M and M^-1 to its own
// vectors with the same x_up, I, U and D^-1 ABA just computed, and read the
// body velocities its contact rows are evaluated at, without refactoring
// the mass matrix itself.
struct RigidBodyInertiaSlot;
struct ParentToBodyTransformSlot;
struct SpatialVelocitySlot;
struct UTermSlot;
struct DInvTermSlot;
// The narrow phase's contacts (narrow_phase/narrow_phase_data.hpp) -- shared
// so the contact solver reads them as written.
struct ContactNormalSlot;
struct ContactDistanceSlot;
struct ContactPointSlot;
struct ContactActiveSlot;
// The joint-limit detector's rows (joint_limit/joint_limit_data.hpp) and the
// margin they were detected within -- shared so a solver reads them as
// written, the same way it reads the narrow phase's contacts.
struct JointLimitMarginSlot;
struct JointLimitDistanceSlot;
struct JointLimitSideSlot;
struct JointLimitActiveSlot;

}  // namespace achilles::algorithms
