# Writing docs

## Audience

- Docs are written by agents for agents. Assume no human reads them.
- Humans read the comments in source files. Anything a human needs goes in
  a comment, not a doc (see [comments.md](comments.md)).
- Docs hold the longer notes that don't fit in a comment. For file docs,
  see [File docs](#file-docs).

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

## File docs

A doc in `docs/code/` has these sections, in this order, leaving out any
with nothing to say:

1. **Role**: what the file is for, in 2-3 sentences.
2. **Connections**: what it uses, what uses it, and how data flows
   through it.
3. **Invariants**: rules that span files and break silently if one side
   changes.
4. **Pitfalls**: easy mistakes when using or changing it.
5. **When changing it**: the other files and docs to update.

At most 400 words.

## Content

- Describe the code as it is. No history, no plans, no progress notes.
- Don't repeat what the file's comments already say. Link to them instead.
- Refer to code by symbol name or file path, never by line number.
- Convention docs state rules, never list the code that breaks them.
- Short bullets and headings. Cut anything an agent wouldn't act on.

## Upkeep

Docs are only useful while they match the code, so keeping them current is
part of every change, not a separate task.

- A change to a file that has a doc updates the doc in the same commit.
- Before every report back after changing code, check each changed file's
  top comment for a `Doc:` link, and update that doc if the change touches
  anything it says.
- Also update every convention doc the change affects: `file-tree.md` for
  added, moved or deleted files, `file-formats.md` for `.arow` or YAML
  changes, `recipes.md` for a changed way to add something, the glossary for
  new terms, and the layer tables for new folders.
- A file whose behavior spans other files, and that has no doc, should get
  one when a change makes that connection harder to see.
- `scripts/check-conventions.py --since <rev>` warns about changed files
  whose doc didn't change (`doc-update`).
- A doc that disagrees with the code is a bug: fix the doc, or flag the
  code if the doc describes the intended design.
- Delete a doc when its file is deleted, and remove the link when a doc is.
