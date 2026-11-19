# simulation.hpp

## Role

The object a user drives: load the `.arow` and config, `Init`, `Step`, and
look up handles. Owns the archetypes, the built sim and the selected
integrator.

## Connections

- Loads through `interface/archetype_loader.hpp` and
  `interface/sim_config_loader.hpp`.
- `Init` builds the sim with `engine::memory::SimAllocatorForT` over the
  hosting list, configures every attachment (`interface/attachments.hpp`),
  then runs `algorithms::Setup`.
- `Step` calls the integrator function resolved by `SetIntegrator`
  (`algorithms/integrators.hpp`).
- `Sensor`, `Actuator` and `State` build handles
  (`attachment_handle.hpp`, `state_handle.hpp`) from names, through
  `domain::archetype::ArchetypeSet`.
- Examples and the display use only this object and its handles.
- `simulation.cpp` defines everything that runs once, outside the tick:
  loading, `Init`, `Attachments`, `Joints`, `Up` and the private lookups.
  `SetIntegrator`, `Step` and the handle templates stay in the header.

## Invariants

- Lifecycle: load, then `Init`, then `Step`. `Init` again rebuilds
  everything and invalidates every handle taken before.
- The integrator is resolved once, so `Step` never branches on it.
- The same seed gives the same run on the same build; `Seed()` reports the
  one drawn when none was given.
- `Time()` and `StepCount()` come from the sim clock and advance only on a
  successful `Step`.

## Pitfalls

- The load functions, `Init` and `Step` catch exceptions and return `false`
  after a warning, while handle lookups throw.
- `Context()` and `Archetypes()` bypass the handles; nothing keeps a raw
  write consistent.

## When changing it

- `examples/` and `include/display/`, which call it.
- [interface.md](../conventions/interface.md).
- `tests/interface_simulation.cpp`.
