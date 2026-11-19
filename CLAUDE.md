# Achilles

A rigid-body simulator built for SIMD first and a GPU port later.

The rules live in `docs/conventions/`. Read the ones that apply to the task
before starting it. When code and the docs disagree, follow the docs, and
never copy a pattern that breaks them. A source file may also link its own
doc in `docs/code/` from its top comment; read it before changing the file.
To find where something lives, read [docs/file-tree.md](docs/file-tree.md).

## Always

- Never change `main`.
- Features Eric asks for go on a `claude/` branch unless he says otherwise.
- On `EricG/` branches, make zero design decisions unless asked.
- Never build unbounded (see building-and-testing).
- Format every changed file before every report back (see workflow).
- Check `git branch --show-current` before changing anything.

## Conventions

| Doc | Read when |
|---|---|
| [workflow](docs/conventions/workflow.md) | Starting work, committing, making blocks or PRs |
| [building-and-testing](docs/conventions/building-and-testing.md) | Building, running checks or tests |
| [testing](docs/conventions/testing.md) | Writing or changing tests |
| [examples](docs/conventions/examples.md) | Adding or changing examples |
| [file-formats](docs/conventions/file-formats.md) | Reading or writing `.arow` or YAML files |
| [recipes](docs/conventions/recipes.md) | Adding an algorithm, kind, sim value, policy, domain object or example |
| [glossary](docs/conventions/glossary.md) | Meeting an unfamiliar term |
| [layers](docs/conventions/layers.md) | Adding or moving files, or adding an include |
| [util](docs/conventions/util.md) | Working in `util/` |
| [domain](docs/conventions/domain.md) | Working in `domain/` |
| [policy](docs/conventions/policy.md) | Working in `policy/` |
| [engine](docs/conventions/engine.md) | Working in `engine/` |
| [algorithms](docs/conventions/algorithms.md) | Working in `algorithms/` |
| [interface](docs/conventions/interface.md) | Working in `interface/` |
| [code-rules](docs/conventions/code-rules.md) | Writing any code, especially code that runs during a step |
| [errors](docs/conventions/errors.md) | Adding a check, assert or throw |
| [comments](docs/conventions/comments.md) | Writing any comment |
| [writing-docs](docs/conventions/writing-docs.md) | Writing or updating any doc |
| [sim-model](docs/conventions/sim-model.md) | Adding sensors, actuators or kinds |

## Planned

- A check script for include direction, folder depth, unique file names and
  namespaces.
- A doc for handling review comments.
- Defining the public API.
