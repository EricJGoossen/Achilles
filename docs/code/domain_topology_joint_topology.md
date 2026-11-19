# joint_topology.hpp

## Role

The structure a traversal walks: each row's source index (a parent, or a
link into another ordering), plus the segmented and pair variants for list
and pair orderings.

## Connections

- Built by `engine/allocation/sim_allocator.hpp` from an ordering's plan
  (`policy/ordering/`), stored in the arena.
- Reached by steps through `SimContext::TopologyFor<Policy>()`.
- Walked by every traversal in `policy/traversal/`.
- `TopologyLike` is the concept generic engine code takes.

## Invariants

- A parent past the end of the rows means the reserved base row.
- `lane_size` is the width the ordering padded to; it comes from the hosted
  algorithms' lane count, never a hardcoded batch type.
- The tree orderings assert `IsBatchSafe` when they plan; nothing checks on
  construction, since a list topology's sources are links, not parents.
- A pair topology's lane group shares one source; a padding lane's partner
  is `kNoPartner`.

## Pitfalls

- `Size()` counts raw rows, including padding. A batched traversal walks
  `Size() / Stride` groups.

## When changing it

- `policy/traversal/` and `policy/ordering/`.
- `tests/domain_joint_topology.cpp`.
