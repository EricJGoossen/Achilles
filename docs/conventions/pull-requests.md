# Pull requests and issues

## Opening

- Agents may push branches, but never open a PR or MR. When a branch is
  finished, give Eric a title and description in the reply; he opens the PR.
- A branch is finished only after clang-tidy, clang-format, a build and the
  tests pass.
- One topic per branch. If a branch grew several topics, suggest splitting
  it before handing it over.

## Title

- Imperative, at most 72 characters, stating the outcome.
- Never a branch name.

## Description

In this order, leaving out any section with nothing to say:

1. **Summary**: what changes for someone using the sim, and why. At most 4
   sentences.
2. **Changes**: bullets grouped by top-level folder, saying what changed and
   why. Not a list of files.
3. **Breaking changes**: API, `.arow`, YAML and behavior changes, each with
   how to migrate.
4. **Decisions**: choices made and alternatives rejected, each with its
   runtime effect. On `claude/` branches, flag every decision the agent
   made.
5. **Testing**: what was built and run, and the results. Anything not
   covered.
6. **Docs**: which docs changed.

Follow-ups and known gaps are never a section: each becomes an issue.

## Writing

- At most 500 words, not counting command output.
- A PR may describe what changed and what it replaced.
- Refer to code by symbol name or file path, never by line number.
- If the branch significantly changes the sim, name the example that shows
  it.
- The attribution line goes on `claude/` branches only (see
  [workflow.md](workflow.md)).

## Issues

Agents draft issues in their reply; they never file them.

### When

- Every follow-up or known gap a branch leaves.
- Every change Eric declines for now that sounds worthwhile.

### Title

- Imperative, at most 72 characters.

### Body

In this order:

1. **Problem**: what's wrong or missing, and why it matters.
2. **Proposal**: the change, and the alternatives considered.
3. **Runtime effect**: expected cost per tick, init time and memory.
4. **Scope**: the top-level folders and main files touched.
5. **Done when**: a checkable condition, such as a passing test or a rule
   the convention check enforces.
6. **Context**: the branch or PR it came from.

At most 250 words.
