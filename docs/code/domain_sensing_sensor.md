# sensor.hpp

## Role

The base every sensor kind derives from. A kind declares its inputs, state
and output, and provides `Read` (and, for a scanning kind, `Observe`); the
storage and passes that run it are generated from those declarations.

## Connections

- Mirrors `domain/actuation/actuator.hpp`; both build on
  `domain/attachment/attachment.hpp` for options, ticks, readings and
  `Due`.
- Kinds derive from it in `domain/sensing/` and are registered in
  `algorithms/attachment_kinds/attachment_kinds.hpp`.
- Inputs are bound to fields by `algorithms/attachment_kinds/sim_input.hpp`;
  `attachment::ReadStep` calls `Update` and `Observe`.
- Error models come from the effects in `domain/error/`.
- Users read results through `interface::SensorHandle`.

## Invariants

- The kind object is its configuration and lives in the arena: trivially
  copyable and default-constructible.
- `Read` runs only when a sample is due; otherwise the last reading carries
  forward.
- A sensor never names an algorithm; its inputs are quantities.
- `Read`'s parameters follow the op shape: inputs by const reference, then
  outputs by pointer.

## Pitfalls

- A scanning kind's `Observe` accumulates into state across bodies; `Read`
  must turn that into the reading and reset what it needs to.

## When changing it

- `domain/actuation/actuator.hpp`, so the two stay mirrors.
- `algorithms/attachment/` if the kind contract changes.
- [recipes.md](../conventions/recipes.md) "Add a sensor or actuator kind".
