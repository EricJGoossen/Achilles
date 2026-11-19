# traversals.hpp

## Role

The ways a pass walks rows: forward or backward over a tree, linearly, over
a list's links, over one segment, or over pairs. Each is built from a
source, range and scan policy (`row_policies.hpp`), all resolved at compile
time, so each costs the same as a hand-written loop.

## Connections

- Named in step files as `Pass<Op, Traversal>`, for example ABA's
  `TreeTraversal<kForward, kLanes>`.
- Driven by `algorithm_step.hpp`, which hands it the op invoker plus
  everything the `Step` call got after the view.
- Walks a `domain::JointTopology` (`domain/joint_topology.hpp`) or, for a
  linear pass, a bare row count.

## Invariants

- `Stride` must match the op's lane count: `kLanes` for a batched op, 1 for
  a scalar one. The traversal walks `Size() / Stride` groups and divides
  parent rows by `Stride`.
- A batched tree pass relies on every row and its parent sitting in
  different lane groups (`JointTopology::IsBatchSafe`), which the tree
  orderings guarantee.
- A forward tree pass relies on every parent coming before its child
  (`IsForwardSafe`).
- `ApplyToBase` runs an op's `Initialize` once, at the base row, before its
  pass.

## Pitfalls

- A wrong `Stride` compiles and silently reads the wrong rows.
- A tree traversal over a `LinearOrdering` field is only correct when the
  archetypes have no joint dependencies.

## When changing it

- `row_policies.hpp` for the building blocks.
- `tests/engine_pass_traversals.cpp`, at lane sizes 4, 8 and 32, in both
  directions.
