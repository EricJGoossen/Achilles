#pragma once

#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace achilles::domain {

// The leaf nodes of an archetype, as the loader assembles them: what a scene
// says about its joints, one value at a time.

// Marks a root joint in tree_structure.
inline constexpr std::size_t kNoParent =
    std::numeric_limits<std::size_t>::max();

// One joint of one instance of one archetype, by the names the .arow files
// use: `instance` counts that archetype's copies from 0. How a caller names a
// joint; ArchetypeSet::RowOf is what turns it into a row.
struct JointAddress {
  std::string archetype;
  std::string joint;
  std::size_t instance = 0;
};

// One joint of one instance, by number rather than by name: which
// instance, numbering every archetype's instances end to end, in order --
// two instances of one archetype get two different values -- and which of
// its joints.
struct JointRef {
  std::size_t instance_index;
  std::size_t joint_index;
};

// One named field's flattened per-(instance, joint) scalar data for one
// Archetype, as read from the user (e.g. a .arow file) -- see
// interface/archetype_loader.hpp for the reader that builds these.
// `name` is resolved against a specific Algorithm<EnumT, Traits>'s
// Traits<F>::kName only once SimAllocator binds this Archetype against a
// concrete Algorithm pack (see SimAllocator's Archetype-taking
// constructor); Archetype itself, and whatever builds it, never needs to
// know about any EnumT.
//
// `values` is laid out instance-major, then joint-major, then leaf-minor --
// element (instance i, joint j, leaf l) lives at
// `(i * joint_count + j) * scalars_per_leaf + l` -- matching the
// instance-major-then-local-joint indexing tree_structure/root_parents
// already use. `scalars_per_leaf` must equal the bound field's own
// Assembler::kNumFields, and the leaves themselves must be given in that
// Assembler's own flattening order (e.g. a Transform field is [translation.x,
// .y, .z, rotation.w, .x, .y, .z] -- Quaternion's own ToTuple() is (W(), X(),
// Y(), Z()), scalar-first, not scalar-last) -- there is no generic
// domain-type reflection to derive this automatically, so whatever builds a
// JointField is responsible for matching each field's own Assembler's leaf
// order.
struct JointField {
  std::string name;
  std::size_t scalars_per_leaf = 0;
  std::vector<double> values;
};

// One value of `field`: leaf `leaf` of joint `joint` of instance `instance`.
inline double JointFieldAt(
    const JointField& field,
    std::size_t instance,
    std::size_t joint,
    std::size_t leaf,
    std::size_t joint_count
) {
  std::size_t index =
      (instance * joint_count + joint) * field.scalars_per_leaf + leaf;
  assert(index < field.values.size() && "JointFieldAt: index out of range.");
  return field.values[index];
}

// Something an archetype carries on one of its joints that isn't a joint
// itself -- a sensor, say: every instance of the archetype gets one, linked
// to that instance's copy of `joint`. `list` names which list it belongs
// to; an ordering planning that list (engine::topology::ListOrdering) gives
// each one a row. `name` is what it goes by, and
// `config` is its own settings as named lists of numbers, the same shape a
// joint's fields take -- read by whatever the list's entries are. What an entry
// *is* stays out of here: the archetype only records names and numbers, so
// new kinds never change this type.
struct JointAttachment {
  std::string list;
  std::size_t joint = 0;
  std::string name;
  std::vector<std::pair<std::string, std::vector<double>>> config;
};

}  // namespace achilles::domain
