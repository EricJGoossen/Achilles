#include "interface/archetype_loader.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cstddef>
#include <filesystem>
#include <iterator>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "domain/archetype.hpp"
#include "util/yaml.hpp"

// -- .arow format --
//
// Each .arow file (plain YAML; the extension is only a naming convention)
// describes exactly one archetype:
//
//   archetype: wheel        # this archetype's name, unique across the
//                            # whole included set
//   copies: 4                # optional, default 1 -- how many instances
//   attach:                  # omit only on the single root archetype
//     archetype: chassis      # another archetype's name (declared here or
//                              # pulled in via includes)
//     joint: axle_mount        # a joint name within ONE instance of it
//   includes:                # optional -- other .arow files this one
//     - chassis.arow           # references; paths are relative to this
//                               # file's own directory, loaded recursively
//   joints:
//     - name: hub
//       parent: null           # omitted/null = this joint's own local root
//       fields:
//         joint_subspace: [0, 0, 1, 0, 0, 0]
//         fixed_joint_transform: {translation: [0, 0, 0], rotation: [0, 0, 0,
//         1]}
//         ...
//
// A field's value can be a flat sequence or an arbitrarily nested
// sequence/mapping of scalars (mappings flattened in key order, the same
// tolerance yaml-cpp itself gives node iteration) -- every scalar leaf
// found is appended, in encounter order, to that field's flat value list.
// Nothing here knows what shape any particular field "should" have; it's
// on whoever writes the .arow file to give a field's leaves in the same
// order its bound Assembler flattens to (see ArchetypeField's own comment,
// engine/topology/archetype.hpp) -- the loader stays generic on purpose,
// so a brand new field needs no change here.
//
// A field must be given on either every joint of an archetype or none of
// them (no partial coverage -- see BuildField below), and every instance
// of an archetype shares the exact same per-joint field values; `copies`
// only repeats the shape, it never varies the data.
//
// Static joints aren't supported yet -- every joint this loader produces
// is non-static, matching ArchetypeTreeStructure/Ordering, which have no
// notion of "static" at all today.

namespace achilles::interface {

using domain::Archetype;
using domain::ArchetypeField;
using domain::ArchetypeJointHandle;
using domain::ArchetypeTreeStructure;

namespace {

// One field's raw per-leaf values for a single instance of one joint, in
// the order they were parsed -- broadcast across every instance of the
// owning archetype when the real ArchetypeField is built (see BuildField).
struct ParsedJoint {
  std::string name;
  std::string parent_name;  // empty => this joint is its archetype's root
  std::vector<std::pair<std::string, std::vector<double>>> fields;
};

struct ParsedArchetype {
  std::string name;
  std::size_t copies = 1;
  bool has_attach = false;
  std::string attach_archetype;
  std::string attach_joint;
  std::vector<ParsedJoint> joints;
  std::string source_file;  // for error messages only
};

[[noreturn]] void Fail(const std::string& file, const std::string& message) {
  throw ArchetypeLoadError(file + ": " + message);
}

double ParseScalar(const YAML::Node& node, const std::string& file) {
  double as_double = 0.0;
  if (util::TryParseNumber(node, as_double)) {
    return as_double;
  }
  if (node.IsScalar()) {
    const std::string& raw = node.Scalar();
    if (raw == "true") {
      return 1.0;
    }
    if (raw == "false") {
      return 0.0;
    }
  }
  Fail(file, "field value '" + node.Scalar() + "' is not a number or boolean.");
}

void FlattenNode(
    const YAML::Node& node, const std::string& file, std::vector<double>& out
) {
  if (node.IsSequence()) {
    for (const auto& child : node) {
      FlattenNode(child, file, out);
    }
  } else if (node.IsMap()) {
    for (const auto& entry : node) {
      FlattenNode(entry.second, file, out);
    }
  } else if (node.IsScalar()) {
    out.push_back(ParseScalar(node, file));
  } else {
    Fail(
        file, "a field value must be a number, boolean, sequence, or mapping."
    );
  }
}

std::filesystem::path ResolveInclude(
    const std::filesystem::path& including_file, const std::string& include
) {
  std::filesystem::path base = including_file.parent_path();
  return (base / include).lexically_normal();
}

YAML::Node LoadYamlFile(const std::filesystem::path& path) {
  try {
    return YAML::LoadFile(path.string());
  } catch (const YAML::BadFile&) {
    Fail(path.string(), "could not be opened.");
  } catch (const YAML::Exception& e) {
    Fail(path.string(), std::string("YAML syntax error: ") + e.what());
  }
}

std::size_t ParseCopies(const YAML::Node& doc, const ParsedArchetype& parsed) {
  YAML::Node copies = doc["copies"];
  if (!copies) {
    return 1;
  }
  auto count = copies.as<std::size_t>();
  if (count == 0) {
    Fail(
        parsed.source_file,
        "archetype '" + parsed.name + "': copies must be >= 1."
    );
  }
  return count;
}

void ParseAttach(const YAML::Node& doc, ParsedArchetype& parsed) {
  YAML::Node attach = doc["attach"];
  if (!attach) {
    return;
  }
  YAML::Node attach_archetype = attach["archetype"];
  YAML::Node attach_joint = attach["joint"];
  if (!attach_archetype || !attach_joint) {
    Fail(
        parsed.source_file,
        "archetype '" + parsed.name +
            "': attach must have both 'archetype' and 'joint'."
    );
  }
  parsed.has_attach = true;
  parsed.attach_archetype = attach_archetype.as<std::string>();
  parsed.attach_joint = attach_joint.as<std::string>();
}

ParsedJoint ParseJoint(
    const YAML::Node& joint_node, const ParsedArchetype& parsed
) {
  ParsedJoint joint;
  YAML::Node joint_name = joint_node["name"];
  if (!joint_name) {
    Fail(
        parsed.source_file,
        "archetype '" + parsed.name + "': a joint is missing 'name'."
    );
  }
  joint.name = joint_name.as<std::string>();
  if (YAML::Node parent = joint_node["parent"]; parent && !parent.IsNull()) {
    joint.parent_name = parent.as<std::string>();
  }
  if (YAML::Node fields = joint_node["fields"]; fields && fields.IsMap()) {
    for (const auto& field_entry : fields) {
      std::vector<double> values;
      FlattenNode(field_entry.second, parsed.source_file, values);
      joint.fields.emplace_back(
          field_entry.first.as<std::string>(), std::move(values)
      );
    }
  }
  return joint;
}

std::vector<ParsedJoint> ParseJoints(
    const YAML::Node& doc, const ParsedArchetype& parsed
) {
  YAML::Node joints = doc["joints"];
  if (!joints || !joints.IsSequence() || joints.size() == 0) {
    Fail(
        parsed.source_file,
        "archetype '" + parsed.name + "': must declare at least one joint."
    );
  }
  std::vector<ParsedJoint> result;
  result.reserve(joints.size());
  for (const auto& joint_node : joints) {
    result.push_back(ParseJoint(joint_node, parsed));
  }
  return result;
}

void ParseArchetypeFile(
    const std::filesystem::path& path,
    std::unordered_map<std::string, ParsedArchetype>& archetypes,
    std::unordered_set<std::string>& in_progress_files
);

void ProcessIncludes(
    const YAML::Node& doc,
    const std::filesystem::path& path,
    std::unordered_map<std::string, ParsedArchetype>& archetypes,
    std::unordered_set<std::string>& in_progress_files
) {
  YAML::Node includes = doc["includes"];
  if (!includes || !includes.IsSequence()) {
    return;
  }
  for (const auto& include : includes) {
    ParseArchetypeFile(
        ResolveInclude(path, include.as<std::string>()),
        archetypes,
        in_progress_files
    );
  }
}

void ParseArchetypeFile(
    const std::filesystem::path& path,
    std::unordered_map<std::string, ParsedArchetype>& archetypes,
    std::unordered_set<std::string>& in_progress_files
) {
  std::string canonical = std::filesystem::weakly_canonical(path).string();
  if (!in_progress_files.insert(canonical).second) {
    Fail(
        path.string(),
        "include cycle -- this file (transitively) includes itself."
    );
  }

  YAML::Node doc = LoadYamlFile(path);
  ProcessIncludes(doc, path, archetypes, in_progress_files);

  YAML::Node name_node = doc["archetype"];
  if (!name_node || !name_node.IsScalar()) {
    Fail(path.string(), "missing required top-level 'archetype: <name>' key.");
  }
  ParsedArchetype parsed;
  parsed.name = name_node.as<std::string>();
  parsed.source_file = path.string();
  parsed.copies = ParseCopies(doc, parsed);
  ParseAttach(doc, parsed);
  parsed.joints = ParseJoints(doc, parsed);

  if (auto it = archetypes.find(parsed.name); it != archetypes.end()) {
    Fail(
        parsed.source_file,
        "archetype name '" + parsed.name + "' also declared in " +
            it->second.source_file + " -- archetype names must be unique."
    );
  }
  archetypes.emplace(parsed.name, std::move(parsed));
  in_progress_files.erase(canonical);
}

// Local joint index for `name` within `archetype`, or throws.
std::size_t LocalJointIndex(
    const ParsedArchetype& archetype, const std::string& name
) {
  for (std::size_t i = 0; i < archetype.joints.size(); ++i) {
    if (archetype.joints[i].name == name) {
      return i;
    }
  }
  Fail(
      archetype.source_file,
      "archetype '" + archetype.name + "': no joint named '" + name + "'."
  );
}

// Resolves every joint's parent_name to a local physical index (or
// kNoParent), and validates the result is a single tree: exactly one
// local root, every joint reachable from it.
std::vector<std::size_t> BuildTreeStructure(const ParsedArchetype& archetype) {
  std::size_t n = archetype.joints.size();
  std::vector<std::size_t> parents(n, ArchetypeTreeStructure::kNoParent);
  std::vector<std::vector<std::size_t>> children(n);
  std::size_t root_count = 0;

  for (std::size_t i = 0; i < n; ++i) {
    const std::string& parent_name = archetype.joints[i].parent_name;
    if (parent_name.empty()) {
      ++root_count;
      continue;
    }
    std::size_t parent_index = LocalJointIndex(archetype, parent_name);
    if (parent_index == i) {
      Fail(
          archetype.source_file,
          "archetype '" + archetype.name + "': joint '" +
              archetype.joints[i].name + "' names itself as its own parent."
      );
    }
    parents[i] = parent_index;
    children[parent_index].push_back(i);
  }

  if (root_count != 1) {
    Fail(
        archetype.source_file,
        "archetype '" + archetype.name +
            "': must have exactly one joint with "
            "no parent (found " +
            std::to_string(root_count) + ")."
    );
  }

  std::size_t local_root = std::distance(
      parents.begin(),
      std::find(
          parents.begin(), parents.end(), ArchetypeTreeStructure::kNoParent
      )
  );
  std::vector<bool> visited(n, false);
  std::vector<std::size_t> frontier = {local_root};
  std::size_t visited_count = 0;
  while (!frontier.empty()) {
    std::vector<std::size_t> next;
    for (std::size_t node : frontier) {
      if (visited[node]) {
        continue;
      }
      visited[node] = true;
      ++visited_count;
      for (std::size_t child : children[node]) {
        next.push_back(child);
      }
    }
    frontier = std::move(next);
  }
  if (visited_count != n) {
    Fail(
        archetype.source_file,
        "archetype '" + archetype.name +
            "': joints do not form a single tree (a joint's parent chain "
            "cycles back on itself, or names an unreachable structure)."
    );
  }

  return parents;
}

// A field must be declared on every joint of an archetype or none of them,
// with the same leaf count throughout -- partial coverage would leave
// some (instance, joint) slots in ArchetypeField::values with no defined
// source, which BuildTreeStructure's own caller has no reasonable default
// for. Returns every such field, broadcasting each joint's one-instance
// values across every one of the archetype's `copies`.
std::vector<ArchetypeField> BuildFields(const ParsedArchetype& archetype) {
  std::vector<std::string> field_order;
  for (const auto& [name, values] : archetype.joints.front().fields) {
    field_order.push_back(name);
  }

  std::vector<ArchetypeField> result;
  for (const std::string& field_name : field_order) {
    std::size_t scalars_per_leaf = 0;
    std::vector<double> per_instance;  // joint-major, leaf-minor
    for (const ParsedJoint& joint : archetype.joints) {
      auto it = std::find_if(
          joint.fields.begin(),
          joint.fields.end(),
          [&](const auto& entry) { return entry.first == field_name; }
      );
      if (it == joint.fields.end()) {
        Fail(
            archetype.source_file,
            "archetype '" + archetype.name + "': field '" + field_name +
                "' is declared on some joints but not joint '" + joint.name +
                "'."
        );
      }
      if (scalars_per_leaf == 0) {
        scalars_per_leaf = it->second.size();
      } else if (it->second.size() != scalars_per_leaf) {
        Fail(
            archetype.source_file,
            "archetype '" + archetype.name + "': field '" + field_name +
                "' has " + std::to_string(it->second.size()) +
                " values on joint '" + joint.name + "' but " +
                std::to_string(scalars_per_leaf) + " on an earlier joint."
        );
      }
      per_instance.insert(
          per_instance.end(), it->second.begin(), it->second.end()
      );
    }

    std::vector<double> values;
    values.reserve(per_instance.size() * archetype.copies);
    for (std::size_t instance = 0; instance < archetype.copies; ++instance) {
      values.insert(values.end(), per_instance.begin(), per_instance.end());
    }
    result.push_back(
        ArchetypeField{field_name, scalars_per_leaf, std::move(values)}
    );
  }
  return result;
}

// Depth-first placement order (root first), and validates the attach
// graph is a tree while computing it: exactly one root, every other
// archetype's attach target exists and isn't reached via a cycle.
std::vector<std::string> DepthFirstOrder(
    const std::unordered_map<std::string, ParsedArchetype>& archetypes
) {
  const ParsedArchetype* root = nullptr;
  for (const auto& [name, archetype] : archetypes) {
    if (!archetype.has_attach) {
      if (root != nullptr) {
        Fail(
            archetype.source_file,
            "archetype '" + archetype.name + "' has no 'attach', but '" +
                root->name + "' (" + root->source_file +
                ") already claims to be the root -- exactly one archetype "
                "may omit attach."
        );
      }
      root = &archetype;
    }
  }
  if (root == nullptr) {
    Fail(
        "<root file>",
        "no archetype without 'attach' was found -- exactly one root is "
        "required."
    );
  }
  if (root->copies > 1) {
    Fail(
        root->source_file,
        "root archetype '" + root->name + "' has copies = " +
            std::to_string(root->copies) + " -- the root must not be repeated."
    );
  }

  std::unordered_map<std::string, std::vector<std::string>> children_of;
  for (const auto& [name, archetype] : archetypes) {
    if (!archetype.has_attach) {
      continue;
    }
    auto target = archetypes.find(archetype.attach_archetype);
    if (target == archetypes.end()) {
      Fail(
          archetype.source_file,
          "archetype '" + archetype.name + "': attach.archetype '" +
              archetype.attach_archetype +
              "' was never declared (missing an "
              "'includes' entry?)."
      );
    }
    children_of[archetype.attach_archetype].push_back(name);
  }

  // No separate cycle check needed here: children_of is built from each
  // non-root archetype contributing exactly one entry, under its own one
  // attach.archetype -- so no name can ever appear as a child of two
  // different parents, and this DFS can never push (or visit) the same
  // name twice. A cycle among non-root archetypes (each only reachable
  // from another member of the same cycle, never from the real root) just
  // means those archetypes never get pushed at all, caught below by the
  // same "not every archetype is reachable" check that catches a plain
  // dangling attach target too.
  std::vector<std::string> order;
  std::vector<std::string> stack = {root->name};
  while (!stack.empty()) {
    std::string current = stack.back();
    stack.pop_back();
    order.push_back(current);
    if (auto it = children_of.find(current); it != children_of.end()) {
      for (const std::string& child : it->second) {
        stack.push_back(child);
      }
    }
  }
  if (order.size() != archetypes.size()) {
    Fail(
        root->source_file,
        "not every loaded archetype is reachable from the root '" + root->name +
            "' through attach -- check every non-root archetype's "
            "attach.archetype "
            "for a cycle or a typo."
    );
  }
  return order;
}

}  // namespace

namespace {

std::vector<Archetype> LoadArchetypesImpl(const std::filesystem::path& root_file
) {
  std::unordered_map<std::string, ParsedArchetype> parsed;
  std::unordered_set<std::string> in_progress_files;
  ParseArchetypeFile(root_file, parsed, in_progress_files);

  std::vector<std::string> order = DepthFirstOrder(parsed);

  std::unordered_map<std::string, std::size_t> instance_offset;
  std::unordered_map<std::string, std::size_t> placement_index;
  std::size_t running_total = 0;
  for (std::size_t i = 0; i < order.size(); ++i) {
    const ParsedArchetype& archetype = parsed.at(order[i]);
    instance_offset[archetype.name] = running_total;
    placement_index[archetype.name] = i;
    running_total += archetype.copies;
  }

  std::vector<Archetype> result;
  result.reserve(order.size());
  for (const std::string& name : order) {
    const ParsedArchetype& archetype = parsed.at(name);
    std::vector<std::size_t> tree_structure = BuildTreeStructure(archetype);
    std::vector<ArchetypeField> fields = BuildFields(archetype);
    bool is_root = !archetype.has_attach;

    std::vector<ArchetypeJointHandle> root_parents(
        archetype.copies, ArchetypeJointHandle{0, 0}
    );
    if (!is_root) {
      const ParsedArchetype& target = parsed.at(archetype.attach_archetype);
      std::size_t target_joint =
          LocalJointIndex(target, archetype.attach_joint);
      std::size_t target_offset = instance_offset.at(target.name);

      if (target.copies == archetype.copies) {
        for (std::size_t i = 0; i < archetype.copies; ++i) {
          root_parents[i] =
              ArchetypeJointHandle{target_offset + i, target_joint};
        }
      } else if (target.copies == 1) {
        for (std::size_t i = 0; i < archetype.copies; ++i) {
          root_parents[i] = ArchetypeJointHandle{target_offset, target_joint};
        }
      } else {
        Fail(
            archetype.source_file,
            "archetype '" + archetype.name + "' has " +
                std::to_string(archetype.copies) +
                " copies but its attach "
                "target '" +
                target.name + "' has " + std::to_string(target.copies) +
                " -- copies must either match 1:1 or the target must have "
                "exactly one copy shared by every instance."
        );
      }
    }

    result.emplace_back(
        archetype.name,
        std::move(tree_structure),
        std::move(root_parents),
        is_root,
        std::move(fields)
    );
  }
  return result;
}

}  // namespace

std::vector<Archetype> LoadArchetypes(const std::filesystem::path& root_file) {
  // Every explicit validation failure above already throws
  // ArchetypeLoadError with a specific file/archetype named -- this only
  // catches what slips through it: a malformed value a YAML::Node::as<T>()
  // call rejects (e.g. `copies: "four"`) that no explicit check above
  // happens to guard first. Still surfaced as ArchetypeLoadError, just
  // with less specific context than a deliberate Fail() call gives.
  try {
    return LoadArchetypesImpl(root_file);
  } catch (const ArchetypeLoadError&) {
    throw;
  } catch (const YAML::Exception& e) {
    throw ArchetypeLoadError(
        root_file.string() + ": malformed .arow content: " + e.what()
    );
  }
}

}  // namespace achilles::interface
