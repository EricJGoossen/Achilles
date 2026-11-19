# Workflow

## Every branch

- Never change `main`. All work happens on a branch.
- Before every report back to Eric after changing code, format every file
  you changed: run `scripts/check-format.sh`, and `clang-format -i` any
  changed file it doesn't cover, such as `tests/`. Eric never sees
  unformatted code. This holds on every branch, even while builds and tests
  are held.
- Every commit is clang-format clean (the pre-commit hook checks this).
- Each top-level folder (`algorithms`, `display`, `domain`, `engine`,
  `interface`) gets its own commits, unless the change is small bug fixes
  totaling under 100 lines.
- Commit messages follow 50/72 and are at most 10 lines.
- A branch is finished only after clang-tidy, clang-format, a build and the
  tests all pass. Then it gets a PR whose description matches the work.
- New third-party libraries need Eric's explicit approval.

## `claude/` branches

The default for any feature Eric asks Claude to write.

- No oversight needed until the PR; most of these are never merged.
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
   him a commit message without the attribution trailer; he reviews and
   commits.
5. If Eric is not watching: store each block as a patch in the repo root, in
   order, so he can review them later. When a later change revises an
   earlier block, fold it into that block's patch, so he never reviews code
   that was later replaced.
6. Once everything is approved: write tests, build, run tests, clang-format,
   clang-tidy. Then give him a PR title and description without the
   attribution line.
