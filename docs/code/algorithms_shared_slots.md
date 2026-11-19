# shared_slots.hpp

## Role

The tags that make fields of different algorithms one block of memory. Each
is a forward-declared struct named after a physical quantity. Naming the
same tag as `SharedAs` in two algorithms' traits makes them read and write
the same block.

## Connections

- Named in `*_data.hpp` traits across ABA, the integrators, the collision
  pipeline, joint limits, the contact solver, the attachments and the clock.
- `engine/allocation/sim_allocator.hpp` carves each tag once and reuses it.
- `algorithms/attachment_kinds/sim_input.hpp` reads physics values through
  these slots.
- `JointTorqueSlot` and `AttachmentOutputSlot` are also buffer groups.

## Invariants

- The first algorithm in the hosting list that names a tag owns the block.
  Everyone else reuses it.
- Every field naming a tag must agree on assembler, layout, ordering and
  buffering; `SimAllocator` asserts this at build time.
- A sensor or state handle can only read a value through a shared,
  joint-ordered field.

## Pitfalls

- Two fields that should share but name different tags get separate blocks:
  one algorithm's result never reaches the other, and nothing fails.

## When changing it

- Every `*_data.hpp` that names the tag.
- The hosting list order in `registry.hpp` and `physics_algorithms.hpp` if
  ownership must change.
- [recipes.md](../conventions/recipes.md) if the sharing steps change.
