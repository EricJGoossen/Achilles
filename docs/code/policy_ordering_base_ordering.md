# orderings.hpp

## Role

The ordering policies: how a field's rows are numbered and padded. Linear,
topological, list and pair orderings each build an `OrderingPlan` at init,
which becomes the `JointTopology` and `JointLayout` the loop walks.

## Connections

- Named per field as `Ordering` in its traits; resolved by
  `engine/binding.hpp`.
- Planned by `sim_allocator.hpp`, once per distinct policy, from the
  `ArchetypeSet`.
- Produces `ordering_plan.hpp` plans; the `.cpp` holds the shared
  consolidation logic.
- `ListOrdering<Base, Kinds>` gives the attachments their rows, linked to
  joint rows (`algorithms/attachment_kinds/attachment_kinds.hpp`).
  `PairOrdering` gives collision pairs theirs
  (`algorithms/row_sets/collision_pairs.hpp`).

## Invariants

- Instance numbering is end to end in archetype order, matching
  `ArchetypeSet` and `JointLayout`.
- Each archetype's block is its joint count times its instance count rounded
  up to the lane size, so a lane group never straddles two archetypes.
- `TopologicalOrdering` pads each dependency level to a lane multiple, so no
  parent and child share a lane group, and adds the reserved base row that
  root joints point to.
- Padding rows point at the base row.
- The list ordering's segment order is the order of its kind list.

## Pitfalls

- An archetype that indirectly attaches to itself has no depth; planning
  rejects it.
- Fields with different orderings have different row counts and padding
  even within one algorithm.

## When changing it

- `ordering_plan.hpp` and `domain/topology/joint_topology.hpp` if the plan's shape
  changes.
- `tests/engine_topology_orderings.cpp`, at lane sizes 4, 8 and 32.
