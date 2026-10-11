#pragma once

#include <algorithm>
#include <array>
#include <cassert>
#include <cstddef>
#include <limits>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace achilles::domain {

struct ArchetypeJointHandle {
  // A global index into the flattened instance list: every archetype's
  // instances laid end to end in the order they appear in the `tree` span
  // passed to Ordering::Build. Names a specific INSTANCE, not an archetype
  // type -- two different instances of the same archetype get two
  // different values here.
  std::size_t instance_index;
  // The local joint index (0..that instance's own archetype's
  // tree_structure.size() - 1) within the target instance that is the
  // actual attachment point.
  std::size_t joint_index;
};

// One archetype's own local joint ordering, bundled with what Ordering::
// Build needs to place and wire up its block -- moved here from
// ordering_policy.hpp (formerly "ordering policy") since it's really a
// property of the archetype's own shape, not of any one ordering strategy.
// ordering_policy.hpp includes this header for it rather than defining its
// own copy.
struct ArchetypeTreeStructure {
  static constexpr std::size_t kNoParent =
      std::numeric_limits<std::size_t>::max();

  // True for exactly one archetype in the whole span passed to Build: its
  // instances need no external parent at all -- they attach directly to
  // the reserved base row -- so its root_parents entries, if any, are
  // never resolved.
  bool is_root_archetype = false;

  // The physical parent array for a single instance of this archetype
  // (kNoParent marks that instance's own root joint). Every instance of
  // this archetype shares this exact shape.
  std::span<const std::size_t> tree_structure;
  // The external attachment of the root of each instance of this
  // archetype -- root_parents.size() is this archetype's instance count.
  std::span<const ArchetypeJointHandle> root_parents;
};

// The fields an archetype supplies to the sim -- the only fields a .arow
// may contain. This list is what defines an archetype's contents: fields
// are matched to an Algorithm's storage by name (Traits<F>::kName), but no
// field declares itself loadable. How many values each takes per joint
// comes from the matching field's own Assembler, so it isn't restated
// here. interface::Simulation rejects any .arow field not on this list (or
// with the wrong length) when loading, and static_asserts that every name
// here really is some registered field's kName.
inline constexpr std::array<std::string_view, 9> kArchetypeFields = {
    "joint_subspace",
    "joint_activation_mask",
    "fixed_joint_transform",
    "rigid_body_inertia",
    "joint_position",
    "joint_velocity",
    "joint_torque",
    "visual_extents",
    "visual_color",
};

constexpr bool IsArchetypeField(std::string_view name) {
  return std::ranges::find(kArchetypeFields, name) != kArchetypeFields.end();
}

// One named field's flattened per-(instance, joint) scalar data for one
// Archetype, as read from the user (e.g. a .arow file) -- see
// engine/topology/archetype_loader.hpp for the reader that builds these.
// `name` is resolved against a specific Algorithm<EnumT, Traits>'s
// Traits<F>::kName only once SimAllocator binds this Archetype against a
// concrete Algorithm pack (see SimAllocator's Archetype-taking
// constructor); Archetype itself, and whatever builds it, never needs to
// know about any EnumT.
//
// `values` is laid out instance-major, then joint-major, then leaf-minor --
// element (instance i, joint j, leaf l) lives at
// `(i * joint_count + j) * scalars_per_leaf + l` -- matching the
// instance-major-then-local-joint indexing ArchetypeTreeStructure's own
// tree_structure/root_parents already use. `scalars_per_leaf` must equal
// the bound field's own Assembler::kNumFields, and the leaves themselves
// must be given in that Assembler's own flattening order (e.g. a Transform
// field is [translation.x, .y, .z, rotation.w, .x, .y, .z] -- Quaternion's
// own ToTuple() is (W(), X(), Y(), Z()), scalar-first, not scalar-last) --
// there is no generic domain-type reflection to derive this automatically,
// so whatever builds an ArchetypeField is responsible for matching each
// field's own Assembler's leaf order.
struct ArchetypeField {
  std::string name;
  std::size_t scalars_per_leaf = 0;
  std::vector<double> values;
};

// A free function, not a member, so ArchetypeField stays a plain data
// struct like ArchetypeJointHandle/ArchetypeTreeStructure above it
// (matching this codebase's convention of keeping a type "real class"
// methods-plus-data only when it actually owns invariants beyond its own
// shape).
inline double ArchetypeFieldAt(
    const ArchetypeField& field,
    std::size_t instance,
    std::size_t joint,
    std::size_t leaf,
    std::size_t joint_count
) {
  std::size_t index =
      (instance * joint_count + joint) * field.scalars_per_leaf + leaf;
  assert(
      index < field.values.size() && "ArchetypeFieldAt: index out of range."
  );
  return field.values[index];
}

// Everything read from the user about one archetype: its tree shape (owned
// here, unlike ArchetypeTreeStructure's borrowed spans, which TreeStructure()
// hands out on demand) plus every field's per-instance-per-joint data.
// Built by whatever reads the user's archetype description (see
// archetype_loader.hpp); consumed by SimAllocator's Archetype-taking
// constructor, which accumulates every hosted Algorithm's field orderings,
// picks a Layout from them, and populates each hosted field's carved block
// from whichever ArchetypeField matches that field's own Traits<F>::kName.
class Archetype {
 public:
  Archetype(
      std::string name,
      std::vector<std::size_t> tree_structure,
      std::vector<ArchetypeJointHandle> root_parents,
      bool is_root_archetype,
      std::vector<ArchetypeField> fields,
      std::vector<std::string> joint_names = {}
  )
      : name_(std::move(name)),
        tree_structure_(std::move(tree_structure)),
        root_parents_(std::move(root_parents)),
        is_root_archetype_(is_root_archetype),
        fields_(std::move(fields)),
        joint_names_(std::move(joint_names)) {
    assert(
        (joint_names_.empty() || joint_names_.size() == tree_structure_.size()
        ) &&
        "Archetype: joint_names must be empty or name every joint."
    );
  }

  const std::string& Name() const { return name_; }
  std::size_t JointCount() const { return tree_structure_.size(); }
  std::size_t InstanceCount() const { return root_parents_.size(); }
  bool IsRootArchetype() const { return is_root_archetype_; }

  // A borrowing view onto this Archetype's own owned storage, shaped for
  // Ordering::Build/ArchetypeDepths -- valid only as long as this Archetype
  // outlives it.
  ArchetypeTreeStructure TreeStructure() const {
    return ArchetypeTreeStructure{
        is_root_archetype_,
        std::span<const std::size_t>(tree_structure_),
        std::span<const ArchetypeJointHandle>(root_parents_)
    };
  }

  std::span<const ArchetypeField> Fields() const { return fields_; }

  // Human-readable name for local joint `joint`, e.g. for log column
  // headers. Archetypes built without names (most tests) fall back to
  // "j<index>".
  std::string JointName(std::size_t joint) const {
    if (joint < joint_names_.size()) {
      return joint_names_[joint];
    }
    return "j" + std::to_string(joint);
  }

  const ArchetypeField* FindField(std::string_view field_name) const {
    for (const ArchetypeField& field : fields_) {
      if (field.name == field_name) {
        return &field;
      }
    }
    return nullptr;
  }

 private:
  std::string name_;
  std::vector<std::size_t> tree_structure_;
  std::vector<ArchetypeJointHandle> root_parents_;
  bool is_root_archetype_;
  std::vector<ArchetypeField> fields_;
  std::vector<std::string> joint_names_;
};

}  // namespace achilles::domain
