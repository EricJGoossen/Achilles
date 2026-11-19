# sim_allocator.hpp

## Role

Builds a whole sim at init: plans every row order, measures and carves the
one `Arena`, fills every field from the archetypes, and hands back the
`SimContext` the loop runs on. The only place the sim allocates.

## Connections

- Built by `interface::Simulation::Init` through `SimAllocatorForT` over the
  hosting list (`algorithms/registry.hpp`).
- Reads each field's traits through `binding.hpp` (`FieldOrderingT`,
  `FieldLayoutT`, `FieldIndexingT`, `FieldBufferingT`, `SharedAs`).
- Plans rows with the ordering policies (`engine/topology/orderings.hpp`),
  one `OrderingPlan` per distinct policy, keyed by `SlotId<Policy>()`.
- Uses `SimContext`'s own `UsedOrderingsFor` and `TopologyTupleFor`
  (`engine/pass/sim_context.hpp`), so both sides agree on the topology tuple.
- Reads field values from `domain::archetype::ArchetypeSet` by each field's
  `kName`.

Flow, in order: plan rows, measure bytes, construct the `Arena`, copy the
plans' topologies and layouts into it, carve one parity per buffer group,
then per field: carve (or reuse a shared slot), seed padding, populate from
the archetypes, replicate into a second buffer half.

## Invariants

- The measure pass and the carve pass must agree exactly: the `Arena` is
  sized once and asserts on overflow. Any new per-field allocation goes in
  both `MeasureField` and `Carve`.
- The first algorithm in the hosting list that names a `SharedAs` tag owns
  its block. Later algorithms reuse it and must match its shape; `Carve`
  asserts this.
- Seeding and populating place every row through the field layout's
  `RowOffset`, never a fixed per-row step, so a non-planar layout
  (`BlockedLayout`) is filled the same way.
- Populate writes raw leaves in the field assembler's flattening order. The
  `.arow` must give leaves in that same order (`domain::JointField`).
- Seed runs before populate, so real data always wins over a padding seed.
- Nothing built here is resized. A new scene means a new `SimAllocator`.

## Pitfalls

- A field with no `kName` is never populated: it starts zeroed, and its ops
  must fill it.
- `PlanMap` and other standard containers here are init-time scratch. None
  of them may reach the loop.
- Reordering the hosting list changes which algorithm owns each shared slot.

## When changing it

- `engine/pass/sim_context.hpp` if the topology or parity tuples change.
- `tests/engine_memory_sim_allocator.cpp`.
- [file-formats.md](../conventions/file-formats.md) if populating changes
  what `.arow` values mean.
