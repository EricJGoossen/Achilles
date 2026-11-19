# Writing docs

## Audience

- Docs are written by agents for agents. Assume no human reads them.
- Humans read the comments in source files. Anything a human needs goes in
  a comment, not a doc (see [comments.md](comments.md)).
- Docs hold the longer notes that don't fit in a comment: design reasons,
  invariants that span several files, pitfalls, and how pieces connect.

## Kinds of doc

- `docs/conventions/`: the repo's rules. Indexed from `CLAUDE.md`.
- `docs/file-tree.md`: every source file, with one line on what it holds.
  Adding, moving or deleting a file updates it in the same commit.
- `docs/code/`: optional notes on one source file each. The name flattens
  the file's path, the same way test names do:
  `include/engine/allocation/sim_allocator.hpp` ->
  `docs/code/engine_allocation_sim_allocator.md`.
- A file with a doc links it from its top comment. A doc without that link
  is never found, so add both together.

## Content

- Describe the code as it is. No history, no plans, no progress notes.
- Don't repeat what the file's comments already say. Link to them instead.
- Refer to code by symbol name or file path, never by line number.
- Convention docs state rules, never list the code that breaks them.
- Short bullets and headings. Cut anything an agent wouldn't act on.

## Upkeep

- A change to a file that has a doc updates the doc in the same commit.
- A doc that disagrees with the code is a bug: fix the doc, or flag the
  code if the doc describes the intended design.
- Delete a doc when its file is deleted, and remove the link when a doc is.
