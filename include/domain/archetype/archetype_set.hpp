#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include "domain/archetype/archetype.hpp"
#include "domain/joint_data.hpp"
#include "domain/random/counter_rng.hpp"

namespace achilles::domain::archetype {

// A scene's archetypes, together: the Archetype data, and everything that
// has to look across all of it -- names are looked up here, and instances
// are numbered here, end to end in archetype order, the same numbering every
// ordering policy and JointLayout use.
class ArchetypeSet {
 public:
  ArchetypeSet() = default;
  explicit ArchetypeSet(std::vector<Archetype> archetypes)
      : archetypes_(std::move(archetypes)) {}
  explicit ArchetypeSet(std::span<const Archetype> archetypes)
      : archetypes_(archetypes.begin(), archetypes.end()) {}

  std::span<const Archetype> All() const { return archetypes_; }

  // `archetype`'s field named `field_name`, or null.
  static const JointField* FindField(
      const Archetype& archetype, std::string_view field_name
  ) {
    for (const JointField& field : archetype.fields) {
      if (field.name == field_name) {
        return &field;
      }
    }
    return nullptr;
  }

  // The local joint index named `joint_name`, or nullopt -- always nullopt
  // for an Archetype built without joint names.
  static std::optional<std::size_t> JointIndex(
      const Archetype& archetype, std::string_view joint_name
  ) {
    for (std::size_t j = 0; j < archetype.joint_names.size(); ++j) {
      if (archetype.joint_names[j] == joint_name) {
        return j;
      }
    }
    return std::nullopt;
  }

  // The row `address` lives at in `layout` -- anything with
  // ToSorted(instance, joint), like a JointLayout. Throws
  // std::invalid_argument for an unknown archetype or joint, or an
  // out-of-range instance: this resolves user input, so a miss is an expected
  // error, not a bug.
  template <typename Layout>
  std::size_t RowOf(const JointAddress& address, const Layout& layout) const {
    const JointRef joint = ResolveJoint(address);
    return layout.ToSorted(joint.instance_index, joint.joint_index);
  }

  // Where instance `instance` of the attachment named `name` sits in `list`:
  // a list numbers its entries archetype by archetype, instance by instance,
  // in each archetype's own declaration order (the order
  // engine::topology::CollectAttachments gives them rows in). Throws
  // std::invalid_argument for an unknown name, an out-of-range instance, or
  // an attachment of another list: this resolves user input, so a miss is an
  // expected error, not a bug.
  std::size_t AttachmentEntry(
      std::string_view name, std::size_t instance, std::string_view list
  ) const {
    const Located found = LocateAttachment(name);
    const Archetype& archetype = archetypes_[found.index];
    CheckInstance(archetype, instance, "'" + std::string(name) + "'");

    const std::span<const JointAttachment> own = archetype.attachments;
    const JointAttachment* target = &Named(archetype, name, list);
    const auto in_list = [&](const JointAttachment& a) {
      return a.list == list;
    };
    // Entries of this list before this archetype, then before this instance
    // of it, then before this attachment within the instance.
    std::size_t entry = 0;
    for (std::size_t i = 0; i < found.index; ++i) {
      entry += archetypes_[i].root_parents.size() *
               static_cast<std::size_t>(
                   std::ranges::count_if(archetypes_[i].attachments, in_list)
               );
    }
    entry += instance *
             static_cast<std::size_t>(std::ranges::count_if(own, in_list));
    entry +=
        static_cast<std::size_t>(std::count_if(own.data(), target, in_list));
    return entry;
  }

  // The attachment named `name`, checked to be of `list`. Throws
  // std::invalid_argument as AttachmentEntry does.
  const JointAttachment& Attachment(
      std::string_view name, std::string_view list
  ) const {
    return Named(archetypes_[LocateAttachment(name).index], name, list);
  }

  // The row in `layout` of the joint instance `instance` of the attachment
  // named `name` is on: instance i is on instance i of its archetype. Throws
  // std::invalid_argument as AttachmentEntry does.
  template <typename Layout>
  std::size_t AttachmentRow(
      std::string_view name,
      std::size_t instance,
      std::string_view list,
      const Layout& layout
  ) const {
    const Located found = LocateAttachment(name);
    const Archetype& archetype = archetypes_[found.index];
    CheckInstance(archetype, instance, "'" + std::string(name) + "'");
    return layout.ToSorted(
        found.first_instance + instance, Named(archetype, name, list).joint
    );
  }

  // Calls fn(field, first_instance, instance_count, joint_count) for each
  // archetype that has a field named `field_name`, with the global index of
  // its first instance.
  template <typename Fn>
  void ForEachField(std::string_view field_name, const Fn& fn) const {
    ForEachWithFirstInstance([&](const Archetype& archetype,
                                 std::size_t first_instance) {
      if (const JointField* field = FindField(archetype, field_name)) {
        fn(*field,
           first_instance,
           archetype.root_parents.size(),
           archetype.tree_structure.size());
      }
    });
  }

  // Calls fn(attachment, instance, entry, stream) for every attachment of
  // every instance: `instance` is the global index of the archetype instance
  // carrying it, `entry` is where it sits in its list (see AttachmentEntry)
  // and `stream` is its own random stream, drawn from `seed` and its place
  // among every attachment, whatever its list.
  template <typename Fn>
  void ForEachAttachment(std::uint64_t seed, const Fn& fn) const {
    std::unordered_map<std::string_view, std::size_t> entries;
    std::uint64_t index = 0;
    ForEachWithFirstInstance([&](const Archetype& archetype,
                                 std::size_t first_instance) {
      for (std::size_t i = 0; i < archetype.root_parents.size(); ++i) {
        for (const JointAttachment& attachment : archetype.attachments) {
          fn(attachment,
             first_instance + i,
             entries[attachment.list]++,
             random::StreamKey(seed, index++));
        }
      }
    });
  }

  // The address of every named joint of every instance, archetype by
  // archetype.
  std::vector<JointAddress> Joints() const {
    std::vector<JointAddress> joints;
    for (const Archetype& archetype : archetypes_) {
      for (std::size_t i = 0; i < archetype.root_parents.size(); ++i) {
        for (const std::string& joint : archetype.joint_names) {
          joints.push_back({archetype.name, joint, i});
        }
      }
    }
    return joints;
  }

 private:
  JointRef ResolveJoint(const JointAddress& address) const {
    const std::optional<Located> found = Find([&](const Archetype& archetype) {
      return archetype.name == address.archetype;
    });
    if (!found.has_value()) {
      throw std::invalid_argument(
          "no archetype named '" + address.archetype + "'."
      );
    }
    const Archetype& archetype = archetypes_[found->index];
    const std::optional<std::size_t> joint =
        JointIndex(archetype, address.joint);
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

  // One archetype by where it sits: its index here, and the global index of
  // its first instance.
  struct Located {
    std::size_t index = 0;
    std::size_t first_instance = 0;
  };

  // Calls fn(archetype, first_instance) for each archetype, with the global
  // index of its first instance: instances are numbered end to end, in
  // archetype order, the numbering JointRef and every ordering use.
  template <typename Fn>
  void ForEachWithFirstInstance(const Fn& fn) const {
    std::size_t first_instance = 0;
    for (const Archetype& archetype : archetypes_) {
      fn(archetype, first_instance);
      first_instance += archetype.root_parents.size();
    }
  }

  // The first archetype `matches` accepts; nullopt if none does.
  template <typename Pred>
  std::optional<Located> Find(Pred matches) const {
    std::optional<Located> found;
    std::size_t index = 0;
    ForEachWithFirstInstance([&](const Archetype& archetype,
                                 std::size_t first_instance) {
      if (!found.has_value() && matches(archetype)) {
        found = Located{index, first_instance};
      }
      ++index;
    });
    return found;
  }

  // The archetype carrying the attachment named `name`; throws
  // std::invalid_argument if none does.
  Located LocateAttachment(std::string_view name) const {
    const std::optional<Located> found = Find([&](const Archetype& archetype) {
      return std::ranges::any_of(
          archetype.attachments,
          [&](const JointAttachment& a) { return a.name == name; }
      );
    });
    if (!found.has_value()) {
      throw std::invalid_argument(
          "nothing is attached under the name '" + std::string(name) + "'."
      );
    }
    return *found;
  }

  // `archetype`'s attachment named `name`, which it must carry; throws
  // std::invalid_argument if it isn't of `list`.
  static const JointAttachment& Named(
      const Archetype& archetype, std::string_view name, std::string_view list
  ) {
    const JointAttachment& attachment = *std::ranges::find_if(
        archetype.attachments,
        [&](const JointAttachment& a) { return a.name == name; }
    );
    if (attachment.list != list) {
      throw std::invalid_argument(
          "'" + std::string(name) + "' is a '" + attachment.list +
          "', not a '" + std::string(list) + "'."
      );
    }
    return attachment;
  }

  // Throws std::invalid_argument unless `instance` is one of `archetype`'s
  // copies -- `what` names the thing looked up, for the message.
  static void CheckInstance(
      const Archetype& archetype, std::size_t instance, const std::string& what
  ) {
    if (instance >= archetype.root_parents.size()) {
      throw std::invalid_argument(
          what + " has " + std::to_string(archetype.root_parents.size()) +
          " instance(s); instance " + std::to_string(instance) +
          " is out of range."
      );
    }
  }

  std::vector<Archetype> archetypes_;
};

}  // namespace achilles::domain::archetype
