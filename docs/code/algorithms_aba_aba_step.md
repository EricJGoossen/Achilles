# aba_step.hpp

## Role

ABA's steps. `ABAStep` solves joint accelerations from state and torque in
three tree passes: velocity forward, inertia backward, acceleration forward.
`SwapTorqueStep` hands ABA the torque actuators applied since the last tick.

## Connections

- Ops come from `aba_ops.hpp`, fields from `aba_data.hpp`.
- Reads `world_base_transform`, `base_velocity` and `base_acceleration` from
  `algorithms/sim_config.hpp`.
- Run by every tick list and by `Setup` (`algorithms/integrators.hpp`),
  sometimes several times per tick.
- Its results are shared through slots: the integrators read `qdd`, the
  collision pipeline reads world poses, the contact solver reads `x_up`,
  `I`, `U` and `D^-1`, and sensors read state through `sim_input.hpp`.

## Invariants

- The traversals use `Stride = kLanes`, since every ABA op runs batched.
- The base row is seeded by each pass's own `Initialize`; gravity enters as
  the base acceleration.
- `SwapTorqueStep` runs once per tick, after `attachment::DriveStep`: it
  flips the torque parity, then zeroes the new write half.

## Pitfalls

- `base_acceleration` is minus gravity: `[0, 9.81, 0]` is gravity along -y.
- Only one root is supported: the base row is seeded for a single tree.
- `dt` is unused; ABA solves accelerations and integrates nothing.

## When changing it

- `aba_ops.hpp`, and the integrator lists if the step's cost or order
  changes.
- `tests/algorithms_aba_aba_step.cpp` and the ABA reference cases in
  `tests/support/aba_reference.hpp`.
