# traversals.hpp

## Role

The ways a pass walks rows: forward or backward over a tree, linearly, over
a list's links, over one segment, or over pairs -- every lane group, or only
the groups a gate opens (`GatedPairTraversal`). Each is built from a
source, range and scan policy (`row_policies.hpp`), all resolved at compile
time, so each costs the same as a hand-written loop.

## Connections

- Named in step files as `Pass<Op, Traversal>`, for example ABA's
  `TreeTraversal<kForward, kLanes>`.
- Driven by `algorithm_step.hpp`, which hands it the op invoker plus
  everything the `Step` call got after the view.
- Walks a `domain::JointTopology` (`domain/joint_topology.hpp`) or, for a
  linear pass, a bare row count.
- A gated pair pass takes a `PairGate` (`row_policies.hpp`, built with
  `MakePairGate<F>(view)`) after the pair topology. The narrow phase gates
  on the broad phase's `kBoundsOverlap`, the contact solver on the narrow
  phase's `kContactActive`.

## Invariants

- `Stride` must match the op's lane count: `kLanes` for a batched op, 1 for
  a scalar one. The traversal walks `Size() / Stride` groups and divides
  parent rows by `Stride`.
- A batched tree pass relies on every row and its parent sitting in
  different lane groups (`JointTopology::IsBatchSafe`), which the tree
  orderings guarantee.
- A group's parent is either lane-aligned (lane `i` under lane `i` of the
  parent's group: an archetype's own joints, or 1:1 copies) or one shared
  row (copies attached to a single-copy joint). A shared row is handed as a
  `BroadcastIndex` naming only the lanes under it, so padding lanes add
  nothing to it (`ParentSource::SharedParent`). Which one is fixed by the
  topology at load.
- A forward tree pass relies on every parent coming before its child
  (`IsForwardSafe`).
- `ApplyToBase` runs an op's `Initialize` once, at the base row, before its
  pass.
- A gated pair pass loads and stores nothing for a closed group. That's only
  right for an op whose reductions add exactly zero there, and whose
  per-pair outputs are read only by passes gated on the same field, so a
  skipped group's stale values are never read.
- A gate field must be one word per pair lane group
  (`topology::LaneGroupIndexing`), written before the gated passes each
  tick. A group the writer itself skips must be cleared first (the narrow
  phase's `ClearContactsOp`).

## Pitfalls

- A wrong `Stride` compiles and silently reads the wrong rows.
- Callables of a batched tree pass take either source type: a group index
  or a `BroadcastIndex`.
- A tree traversal over a `LinearOrdering` field is only correct when the
  archetypes have no joint dependencies.
- Gating is a data-dependent branch per lane group, which the code rules
  allow only with Eric's approval (given for issue #23). Don't gate a new
  pass without checking the invariant above.

## When changing it

- `row_policies.hpp` for the building blocks.
- `tests/engine_pass_traversals.cpp`, at lane sizes 4, 8 and 32, in both
  directions.
