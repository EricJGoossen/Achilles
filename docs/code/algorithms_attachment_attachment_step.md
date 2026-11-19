# attachment_step.hpp

## Role

Runs every sensor and actuator kind. `DriveStep` runs actuators,
`ReadStep` runs sensors and publishes outputs, and `InitStep` starts every
attachment once at init. Each kind runs over its own segment of the
attachment rows.

## Connections

- Kinds and their records come from
  `algorithms/attachment_kinds/attachment_kinds.hpp`; their inputs are bound
  by `sim_input.hpp`.
- Ops come from `attachment_ops.hpp`; they call each kind's `Read`,
  `Observe`, `Drive` or `Initialize` (`domain/sensing/sensor.hpp`,
  `domain/actuation/actuator.hpp`).
- `Actuation`, `Sensing` and `AttachmentSetup` are spliced into the tick and
  setup lists in `algorithms/integrators.hpp`.
- `interface::AttachmentHandle` reads the published outputs and writes
  commands.

## Invariants

- `DriveStep` runs right before `aba::SwapTorqueStep`: it accumulates
  efforts into the torque half that swap hands ABA.
- `ReadStep` flips the output parity after its passes, so what a caller
  reads after `Step()` is this tick's reading.
- `InitStep` runs after the physics setup, so kinds that fill their first
  output from inputs see the initial pose.
- Drive reads fresh joint state but last evaluation's derived values, such
  as world poses.

## Pitfalls

- A scanning kind's `Observe` runs once per body per sample; its cost grows
  with the number of bodies.

## When changing it

- `algorithms/integrators.hpp` if the steps or their order change.
- `interface/attachment_handle.hpp` if what is published changes.
