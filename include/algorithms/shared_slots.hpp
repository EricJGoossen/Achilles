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

}  // namespace achilles::algorithms
