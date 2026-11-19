# sim_input.hpp

## Role

Binds each sim quantity a kind or state handle can ask for to the field
that holds it. The one place where attachment data meets the algorithm that
computes it.

## Connections

- Quantities are declared in `domain/attachment/sim_quantities.hpp`.
- `SimInput<Algorithm, Field>` points at another algorithm's field, mostly
  ABA's and the broad phase's. `OwnInput<Quantity>` points at one of the
  attachment algorithm's own fields, loaded from the `.arow` (the visuals
  cameras read).
- The attachment algorithm (`attachment_kinds.hpp`, `algorithms/attachment/`)
  reads every kind's inputs through these bindings.
- `interface/state_handle.hpp` finds a quantity's field the same way;
  `interface/state_quantities.hpp` lists which quantities it may reach.

## Invariants

- A bound field must be shared (`SharedAs`) and ordered by joint
  (`TopologicalOrdering`), so it is read in place, never copied.
- A quantity's `Value` must be what reading its field gives;
  `kValueOfItsField` asserts this for every binding.

## Pitfalls

- Binding a quantity to a field whose slot nobody else shares gives the
  attachment algorithm its own empty block, which nothing ever writes.

## When changing it

- Follow [recipes.md](../conventions/recipes.md), "Expose a new sim value".
- `interface/state_quantities.hpp` if state handles should reach a new
  value.
