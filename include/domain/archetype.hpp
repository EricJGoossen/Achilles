#pragma once

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
      std::vector<ArchetypeField> fields
  )
      : name_(std::move(name)),
        tree_structure_(std::move(tree_structure)),
        root_parents_(std::move(root_parents)),
        is_root_archetype_(is_root_archetype),
        fields_(std::move(fields)) {}

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
};

}  // namespace achilles::domain
