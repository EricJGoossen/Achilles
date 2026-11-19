# actuator.hpp

## Role

The base every actuator kind derives from, the mirror of `Sensor`. A kind
declares its inputs, state, command and output, and provides `Drive`; the
base puts the resulting effort on the joint along the actuator's axis.

## Connections

- Mirrors `domain/sensing/sensor.hpp`; both build on
  `domain/attachment/attachment.hpp`.
- Kinds derive from it in `domain/actuation/` and are registered in
  `algorithms/attachment_kinds/attachment_kinds.hpp`.
- `attachment::DriveStep` calls it each tick; `AddEffort` writes into ABA's
  torque write half through the actuators' `TorqueTarget`.
- Users send commands through `interface::ActuatorHandle`.

## Invariants

- The kind object is its configuration and lives in the arena: trivially
  copyable and default-constructible.
- `kEffort` says whether it pushes a force (linear) or a torque (angular).
- An effort along unit axis `e` enters the joint as `S^T (effort * e)`, one
  term per subspace column with a non-negligible weight.
- The axis is normalized at configure time and must lie at least partly in
  the joint's free directions; a zero or fully constrained axis is rejected.
- `Drive` runs only when an update is due; otherwise the last output carries
  forward.

## Pitfalls

- A command that should last one tick must be cleared by `Drive`; otherwise
  it keeps applying.
- `Drive` sees fresh joint state but last evaluation's derived values.

## When changing it

- `domain/sensing/sensor.hpp`, so the two stay mirrors.
- `algorithms/attachment/` if the kind contract changes.
- [sim-model.md](../conventions/sim-model.md) for what may be a kind.
