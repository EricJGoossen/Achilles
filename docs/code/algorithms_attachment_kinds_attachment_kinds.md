# attachment_kinds.hpp

## Role

The registered sensor and actuator kinds, and everything derived from that
list: the attachment row ordering, the inputs every kind reads, and the
per-kind state, command and output records.

## Connections

- Lists domain kinds from `domain/sensing/` and `domain/actuation/`.
- The attachment algorithm (`algorithms/attachment/`) builds its fields,
  ops and steps from these lists.
- `AttachmentOrdering` gives each attachment a row, linked to its mount
  joint's row in `TopologicalOrdering`.
- Inputs resolve through `sim_input.hpp`; the clock comes from
  `clock::ClockAlgorithm`, and actuators reach ABA's subspace and torque.
- `interface/attachments.hpp` matches each `.arow` `kind:` against these
  kinds' `kListName`.

## Invariants

- Each kind's rows form one segment, in list order: sensors first, then
  actuators.
- A kind's inputs are read in place at its joint, so each must be bound to a
  shared, joint-ordered field; a `static_assert` checks this.
- Every kind's configuration, state, command and output must be trivially
  copyable: they live in the arena.

## Pitfalls

- A kind with no instances still costs one empty loop per pass.
- A large record (a camera's images) sits in the arena once per instance.

## When changing it

- [file-formats.md](../conventions/file-formats.md) for a new `kind` name.
- [recipes.md](../conventions/recipes.md) if adding a kind takes different
  steps.
- `docs/file-tree.md` if files move.
