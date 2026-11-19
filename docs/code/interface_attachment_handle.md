# attachment_handle.hpp

## Role

The one handle for a sensor or actuator instance: read its published output,
replace its configuration, and, for an actuator, write commands.
`SensorHandle` and `ActuatorHandle` are aliases of it.

## Connections

- Made by `Simulation::Sensor` and `Simulation::Actuator`; `Configure` is
  called for every attachment at init by `interface/attachments.hpp`.
- Holds a view of the attachment algorithm
  (`algorithms/attachment/attachment_data.hpp`) and its row.
- `Read` sees what `attachment::ReadStep` published; `Write` and `Apply`
  leave commands for the next `attachment::DriveStep`.
- Shares the write interface with writable state handles
  (`interface/writable_handle.hpp`).

## Invariants

- `Configure` checks everything about one `.arow` attachment: its kind, every
  setting (an unread setting is rejected), and an actuator's axis. It throws
  `std::invalid_argument` naming the attachment.
- A handle is a view and a row: cheap to copy, and copies share the
  instance.
- Valid until its `Simulation` is destroyed or re-`Init`ed.
- `SetConfig` restarts the instance but keeps its random stream.

## Pitfalls

- A truth actuator's effort lasts one tick: it must be applied again before
  every `Step()`. A motor holds its last command.
- `SetConfig` skips the axis checks `Configure` makes.
- `State()` is valid only until the next `Step()`.

## When changing it

- `interface/simulation.hpp` and `interface/attachments.hpp`.
- `interface/writable_handle.hpp` if the write interface changes.
