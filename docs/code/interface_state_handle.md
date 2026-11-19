# state_handle.hpp

## Role

A raw handle on one joint's value of one sim quantity, for debugging and
logging. `StateHandle<const S>` only reads; `StateHandle<S>` also writes,
for quantities listed writable.

## Connections

- Made only by `Simulation::State` from a joint address.
- Finds the quantity's field through
  `algorithms/attachment_kinds/sim_input.hpp`; which quantities it reaches,
  and which it writes, comes from `interface/state_quantities.hpp`.
- Shares the write interface with actuator handles
  (`interface/writable_handle.hpp`).
- Truth sensors read the same quantities, synchronized.

## Invariants

- Reads and writes go straight to the sim's field: no double buffer, no
  synchronization, not thread safe. Use it only between `Step()`s, on the
  thread that steps the sim.
- Users never see an algorithm or a row; the row stays inside the handle.
- A writable handle converts to a read-only one, like a pointer to const.
- Valid until its `Simulation` is destroyed or re-`Init`ed.

## Pitfalls

- A derived value (a world pose) reads one step behind the joint state next
  to it.
- Nothing checks a write is one the sim can stand: a pose off a joint's
  freedoms, or a body moved into another.
- A run is only reproduced if its writes are too.

## When changing it

- `interface/state_quantities.hpp` for which quantities it reaches.
- [recipes.md](../conventions/recipes.md) "Expose a new sim value".
- `tests/interface_state_handle.cpp`.
