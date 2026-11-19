# Workflow

## Every branch

- Never change `main`. All work happens on a branch.
- Before every report back to Eric after changing code, format every file
  you changed: run `scripts/check-format.sh`, and `clang-format -i` any
  changed file it doesn't cover, such as `tests/`. Eric never sees
  unformatted code. This holds on every branch, even while builds and tests
  are held.
- Before every report back after changing code, update the docs the change
  affects: each changed file's linked doc, and any convention doc
  ([writing-docs.md](writing-docs.md#upkeep)).
- Every commit is clang-format clean (the pre-commit hook checks this).
- Each top-level folder (`algorithms`, `display`, `domain`, `engine`,
  `interface`) gets its own commits, unless the change is small bug fixes
  totaling under 100 lines.
- Commit messages follow 50/72 and are at most 10 lines.
- A branch is finished only after clang-tidy, clang-format, a build and the
  tests all pass.
- Agents may push branches, but never open a PR or MR. When a branch is
  finished, give Eric a PR title and description
  ([pull-requests.md](pull-requests.md)).
- New third-party libraries need Eric's explicit approval.

## Attribution

Code Eric has reviewed is his; code he hasn't is Claude's.

- On `claude/` branches, commits and PR descriptions carry the attribution
  line.
- On `EricG/` branches, they never do.

## Proposing edits

- Whenever Claude proposes an edit, or offers a choice between options, each
  option states its expected effect on runtime: cost per tick in the sim
  loop, init time and memory. State it even when the effect is trivial or
  none, so Eric never has to ask.
- Give the expectation from reasoning about the code. Never run a long
  benchmark to measure it.

## `claude/` branches

The default for any feature Eric asks Claude to write.

- No oversight needed until the branch is finished; most of these are never
  merged.
- Build and run tests as you go, following
  [building-and-testing.md](building-and-testing.md).
- Run only the tests for files you changed. The full suite is slow.
- Never skip a test. If one fails, fix it.

## `EricG/` branches

Branches Eric builds himself. Claude makes zero design decisions here unless
explicitly asked.

1. Start by talking through the plan thoroughly. If the plan is not
   detailed enough to code without making design decisions, say so.
2. Divide the changes into blocks.
3. Do not build or write tests while making blocks.
4. If Eric is reviewing as you go: do not commit. When a block is done, give
   him a commit message; he reviews and commits.
5. If Eric is not watching: store each block as a patch in the repo root, in
   order, so he can review them later. When a later change revises an
   earlier block, fold it into that block's patch, so he never reviews code
   that was later replaced.
6. Once everything is approved: write tests, build, run tests, clang-format,
   clang-tidy. Then give him a PR title and description.

## Review

- Eric reviews by messaging directly. Never leave `TODO`, `REVIEW` or
  question comments in code for him.
- Read his whole message before changing anything. Group points that touch
  the same code into one consistent fix.
- Each point gets one of three responses:
  - Fix it.
  - Ask. On `EricG/` branches, any point that needs a design decision is a
    question back to him.
  - Push back, with the reason.
- Change only what the points ask for. Anything else you notice goes in the
  reply. On `claude/` branches, small related fixes are allowed if the
  reply says so.
- On `EricG/` branches, each fix goes into the block or patch it belongs
  to, and that block's commit message is updated if its content changed.
- Reply with one entry per point, in his order. Then format every changed
  file, and say what wasn't built or tested.
- When Eric declines a change for now but it sounds worthwhile, draft an
  issue for it ([pull-requests.md](pull-requests.md)).
- When his feedback states a general rule, propose adding it to these docs.
