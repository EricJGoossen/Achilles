#pragma once

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <limits>
#include <optional>
#include <span>
#include <stdexcept>
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

// Something an archetype carries on one of its joints that isn't a joint
// itself -- a sensor, say: every instance of the archetype gets one, linked
// to that instance's copy of `joint`. `list` names which list it belongs
// to; an ordering planning that list (engine::topology::ListOrdering) gives
// each one a row. `name` addresses it (see AttachmentAddress), and `config`
// is its own settings as named lists of numbers, the same shape a joint's
// fields take -- read by whatever the list's entries are. What an entry
// *is* stays out of here: the archetype only records names and numbers, so
// new kinds never change this type.
struct ArchetypeAttachment {
  std::string list;
  std::size_t joint = 0;
  std::string name;
  std::vector<std::pair<std::string, std::vector<double>>> config;
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
// struct like ArchetypeJointHandle above it (matching this codebase's
// convention of keeping a type "real class" methods-plus-data only when it
// actually owns invariants beyond its own shape).
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

// One archetype's own local joint ordering, bundled with what Ordering::
// Build needs to place and wire up its block -- moved here from
// orderings.hpp (formerly "ordering policy") since it's really a
// property of the archetype's own shape, not of any one ordering strategy.
// orderings.hpp includes this header for it rather than defining its
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
  // Every list entry one instance carries, in declaration order -- the
  // same for every instance.
  std::span<const ArchetypeAttachment> attachments;
  // The archetype's field data, for an ordering that places rows by it
  // (e.g. engine::topology::PairOrdering's filter).
  std::span<const ArchetypeField> fields;
};

// Everything read from the user about one archetype: its tree shape (owned
// here, unlike ArchetypeTreeStructure's borrowed spans, which TreeStructure()
// hands out on demand), every field's per-instance-per-joint data, and the
// list entries (ArchetypeAttachment) each instance carries.
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
      std::vector<std::string> joint_names = {},
      std::vector<ArchetypeAttachment> attachments = {}
  )
      : name_(std::move(name)),
        tree_structure_(std::move(tree_structure)),
        root_parents_(std::move(root_parents)),
        is_root_archetype_(is_root_archetype),
        fields_(std::move(fields)),
        joint_names_(std::move(joint_names)),
        attachments_(std::move(attachments)) {
    assert(
        (joint_names_.empty() || joint_names_.size() == tree_structure_.size()
        ) &&
        "Archetype: joint_names, if given, must name every joint."
    );
    for (const ArchetypeAttachment& attachment : attachments_) {
      assert(
          attachment.joint < tree_structure_.size() &&
          "Archetype: an attachment names a joint this archetype doesn't have."
      );
    }
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
        std::span<const ArchetypeJointHandle>(root_parents_),
        std::span<const ArchetypeAttachment>(attachments_),
        std::span<const ArchetypeField>(fields_)
    };
  }

  std::span<const ArchetypeAttachment> Attachments() const {
    return attachments_;
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

  // The local joint index named `joint_name`, or nullopt -- always nullopt
  // for an Archetype built without joint names.
  std::optional<std::size_t> JointIndex(std::string_view joint_name) const {
    for (std::size_t j = 0; j < joint_names_.size(); ++j) {
      if (joint_names_[j] == joint_name) {
        return j;
      }
    }
    return std::nullopt;
  }

 private:
  std::string name_;
  std::vector<std::size_t> tree_structure_;
  std::vector<ArchetypeJointHandle> root_parents_;
  bool is_root_archetype_;
  std::vector<ArchetypeField> fields_;
  std::vector<std::string> joint_names_;
  std::vector<ArchetypeAttachment> attachments_;
};

// One joint of one instance of one archetype, by the names the .arow files
// use: `instance` counts that archetype's copies from 0.
struct JointAddress {
  std::string archetype;
  std::string joint;
  std::size_t instance = 0;
};

// One archetype among `archetypes`, by where it sits: its index there, and
// the global index of its first instance -- instances are numbered end to
// end, in archetype order, the same numbering every ordering policy and
// JointLayout use.
struct ArchetypeSpan {
  std::size_t index = 0;
  std::size_t first_instance = 0;
};

// The first archetype `matches` accepts; nullopt if none does.
template <typename Pred>
std::optional<ArchetypeSpan> FindArchetype(
    std::span<const Archetype> archetypes, Pred matches
) {
  std::size_t first_instance = 0;
  for (std::size_t i = 0; i < archetypes.size(); ++i) {
    if (matches(archetypes[i])) {
      return ArchetypeSpan{i, first_instance};
    }
    first_instance += archetypes[i].InstanceCount();
  }
  return std::nullopt;
}

// Throws std::invalid_argument unless `instance` is one of `archetype`'s
// copies -- `what` names the thing looked up, for the message.
inline void CheckInstance(
    const Archetype& archetype, std::size_t instance, const std::string& what
) {
  if (instance >= archetype.InstanceCount()) {
    throw std::invalid_argument(
        what + " has " + std::to_string(archetype.InstanceCount()) +
        " instance(s); instance " + std::to_string(instance) +
        " is out of range."
    );
  }
}

// The ArchetypeJointHandle `address` names among `archetypes`. Throws
// std::invalid_argument for an unknown archetype or joint, or an
// out-of-range instance: this resolves user input, so a miss is an
// expected error, not a bug.
inline ArchetypeJointHandle ResolveJoint(
    std::span<const Archetype> archetypes, const JointAddress& address
) {
  const std::optional<ArchetypeSpan> found =
      FindArchetype(archetypes, [&](const Archetype& archetype) {
        return archetype.Name() == address.archetype;
      });
  if (!found.has_value()) {
    throw std::invalid_argument(
        "no archetype named '" + address.archetype + "'."
    );
  }
  const Archetype& archetype = archetypes[found->index];
  const std::optional<std::size_t> joint = archetype.JointIndex(address.joint);
  if (!joint.has_value()) {
    throw std::invalid_argument(
        "archetype '" + address.archetype + "' has no joint named '" +
        address.joint + "'."
    );
  }
  CheckInstance(
      archetype, address.instance, "archetype '" + address.archetype + "'"
  );
  return {found->first_instance + address.instance, *joint};
}

// One instance of a named attachment (a sensor, say): `instance` counts
// the copies of the archetype it's attached to from 0.
struct AttachmentAddress {
  std::string name;
  std::size_t instance = 0;
};

// Where an attachment instance sits in its list: the list's name, its
// entry index there, and the attachment itself. A list numbers its entries
// archetype by archetype, instance by instance, in each archetype's own
// declaration order (the order engine::topology::CollectAttachments
// gives them rows in).
struct ResolvedAttachment {
  std::string_view list;
  std::size_t entry = 0;
  const ArchetypeAttachment* attachment = nullptr;
};

// The entry `address` names among `archetypes`. Throws
// std::invalid_argument for an unknown name or an out-of-range instance:
// this resolves user input, so a miss is an expected error, not a bug.
inline ResolvedAttachment ResolveAttachment(
    std::span<const Archetype> archetypes, const AttachmentAddress& address
) {
  const std::optional<ArchetypeSpan> found =
      FindArchetype(archetypes, [&](const Archetype& archetype) {
        return std::ranges::any_of(
            archetype.Attachments(),
            [&](const ArchetypeAttachment& a) { return a.name == address.name; }
        );
      });
  if (!found.has_value()) {
    throw std::invalid_argument(
        "nothing is attached under the name '" + address.name + "'."
    );
  }
  const Archetype& archetype = archetypes[found->index];
  CheckInstance(archetype, address.instance, "'" + address.name + "'");

  const std::span<const ArchetypeAttachment> own = archetype.Attachments();
  const auto target = std::ranges::find_if(
      own, [&](const ArchetypeAttachment& a) { return a.name == address.name; }
  );
  const auto in_list = [&](const ArchetypeAttachment& a) {
    return a.list == target->list;
  };
  // Entries of this list before this archetype, then before this instance
  // of it, then before this attachment within the instance.
  std::size_t entry = 0;
  for (std::size_t i = 0; i < found->index; ++i) {
    entry += archetypes[i].InstanceCount() *
             static_cast<std::size_t>(
                 std::ranges::count_if(archetypes[i].Attachments(), in_list)
             );
  }
  entry += address.instance *
           static_cast<std::size_t>(std::ranges::count_if(own, in_list));
  entry +=
      static_cast<std::size_t>(std::count_if(own.begin(), target, in_list));
  return {target->list, entry, &*target};
}

}  // namespace achilles::domain
