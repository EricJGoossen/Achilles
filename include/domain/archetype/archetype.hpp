#pragma once

#include <cassert>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include "domain/joint_data.hpp"

namespace achilles::domain::archetype {

// Everything read from the user about one archetype: its tree shape (owned
// here), every field's
// per-instance-per-joint data, and the list entries (JointAttachment) each
// instance carries. Built by whatever reads the user's archetype description
// (see archetype_loader.hpp); consumed by SimAllocator's Archetype-taking
// constructor, which accumulates every hosted Algorithm's field orderings,
// picks a Layout from them, and populates each hosted field's carved block
// from whichever JointField matches that field's own Traits<F>::kName.
//
// Plain data: anything asked of it goes through ArchetypeSet
// (archetype_set.hpp). Its constructor is only here to check that the joint
// names and attachments fit the tree.
struct Archetype {
  Archetype(
      std::string name_in,
      std::vector<std::size_t> tree_structure_in,
      std::vector<JointRef> root_parents_in,
      bool is_root_archetype_in,
      std::vector<JointField> fields_in,
      std::vector<std::string> joint_names_in = {},
      std::vector<JointAttachment> attachments_in = {}
  )
      : name(std::move(name_in)),
        tree_structure(std::move(tree_structure_in)),
        root_parents(std::move(root_parents_in)),
        is_root_archetype(is_root_archetype_in),
        fields(std::move(fields_in)),
        joint_names(std::move(joint_names_in)),
        attachments(std::move(attachments_in)) {
    assert(
        (joint_names.empty() || joint_names.size() == tree_structure.size()) &&
        "Archetype: joint_names, if given, must name every joint."
    );
    for (const JointAttachment& attachment : attachments) {
      assert(
          attachment.joint < tree_structure.size() &&
          "Archetype: an attachment names a joint this archetype doesn't have."
      );
    }
  }

  std::string name;
  std::vector<std::size_t> tree_structure;
  std::vector<JointRef> root_parents;
  bool is_root_archetype;
  std::vector<JointField> fields;
  // By local joint index; empty for an Archetype built without joint names.
  std::vector<std::string> joint_names;
  std::vector<JointAttachment> attachments;
};

}  // namespace achilles::domain::archetype
