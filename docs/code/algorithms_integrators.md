# integrators.hpp

## Role

What one tick runs. Each integrator is an ordered list of steps; this file
also holds the setup list run once at init, and the table that turns an
`Integrator` value into the function that steps it.

## Connections

- Composed from every algorithm's step file, plus `attachment::Actuation`,
  `attachment::Sensing` and `attachment::AttachmentSetup`.
- `interface::Simulation` resolves the selected integrator and contact
  solver once with `StepFnFor` and calls that tick every `Step()`;
  `Init()` runs `Setup`.
- Hosting is separate: `registry.hpp` decides what memory exists.

## Invariants

- Every tick list starts with `Effort`: actuators drive, then
  `aba::SwapTorqueStep` hands ABA their torque.
- Every tick list ends with `clock::TickStep`, so the whole tick reads the
  time it started from.
- Sensing runs once per tick, at the tick's starting state, after any solve
  and before anything integrates.
- `kIntegrators` is indexed by the enum; a `static_assert` checks the order.
- Each entry holds one tick per `ContactSolver`. Ticks without a contact
  solve (`kStepsWithout`) hold the same tick for each.
- `Setup` runs ABA at the initial pose before the contact solver's
  `InvWeightStep` and before the attachments start.

## Pitfalls

- Only `SemiImplicitEuler` (and `ImplicitFast`, which equals it) runs the
  collision pipeline and contact solver. The other integrators have no
  contact.
- An integrator may run ABA several times per tick; the torque swap and
  sensing must still happen once.
- The integrator is chosen at init and stays fixed until the next init.

## When changing it

- `tests/algorithms_integrators.cpp`.
- [algorithms.md](../conventions/algorithms.md) for step-list rules.
