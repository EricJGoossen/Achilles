# sim_context.hpp

## Role

The runtime object every step receives as `sim_state`: every hosted
algorithm's view, one topology and layout per ordering policy, and one
parity per buffer group. Also runs step lists.

## Connections

- Built by `engine/memory/sim_allocator.hpp`, which reuses this file's
  `UsedOrderingsFor`, `UsedBufferGroupsFor` and `TopologyTupleFor`.
- Steps reach their data with `ViewFor<View>()` and
  `TopologyFor<Policy>()`, and swap buffers with `FlipParity<Group>()`.
- `algorithms/integrators.hpp` names `RegisteredSimContext` and calls
  `Step<StepList>(dt, config)`.
- `interface::Simulation` holds one, and handles hold views taken from it.

## Invariants

- Everything is keyed by type at compile time. Naming a view, policy or
  group the sim doesn't host fails to compile.
- A `SimContext` and its views are cheap copies of pointers into the arena.
  They stay valid exactly as long as the arena does, not the allocator.
- Every step is called with the same `(sim_state, config, dt)` shape,
  whether it uses them or not.
- Hosting and stepping are separate: an algorithm can be hosted and never
  stepped by the selected integrator.

## Pitfalls

- Handles and copied contexts outlive nothing: re-`Init` builds a new arena
  and leaves old copies dangling.
- `Step` takes the concrete `algorithms::ScalarOperationT` for `dt`.

## When changing it

- `sim_allocator.hpp`, which must build the tuples this file declares.
- `tests/engine_pass_sim_context.cpp`.
