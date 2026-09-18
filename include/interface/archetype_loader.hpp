#pragma once

#include <filesystem>
#include <stdexcept>
#include <vector>

#include "domain/archetype.hpp"

namespace achilles::interface {

// Thrown for anything wrong with a .arow file or the archetype tree it (and
// whatever it transitively `includes`) describes: a missing/unreadable
// file, a YAML syntax error, a field value whose element count is
// inconsistent, an unresolved `attach` target, more or less than one root
// archetype, or a cycle in the attach graph. The message always names the
// offending file and archetype.
class ArchetypeLoadError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

// Loads one root .arow file -- and, recursively, every other .arow file it
// `includes` (paths resolved relative to the including file's own
// directory) -- into a flat list of Archetype, ready to hand to
// SimAllocator. Kept deliberately small and schema-light (see the format
// comment in archetype_loader.cpp): a handful of fields per archetype/
// joint, generic flat-array field values keyed by name (matched against a
// concrete Algorithm's Traits<F>::kName only later, by SimAllocator), and
// nothing yet for static joints (every joint this loader produces is
// non-static -- see ArchetypeTreeStructure/Ordering, which have no notion
// of "static" at all today).
//
// Validates, and throws ArchetypeLoadError if not satisfied:
//  - Exactly one archetype in the whole included set has no `attach` key
//    (the root) and that archetype has no more than one copy -- a root
//    is a single, un-instanced thing (the world, a vehicle base, ...),
//    never a repeated archetype.
//  - Every other archetype's `attach.archetype` names an archetype that
//    was actually loaded (declared by itself or one of its own
//    `includes`).
//  - The resulting attach graph, treating each archetype as one node, is a
//    tree: no archetype (in)directly attaches to itself.
//  - Every joint's own `parent` (by name, within the same archetype) forms
//    a single tree with exactly one local root.
std::vector<domain::Archetype> LoadArchetypes(
    const std::filesystem::path& root_file
);

}  // namespace achilles::interface
