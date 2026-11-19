# Examples

A branch that significantly changes the sim adds an example showing it.

## Layout

- Each example has its own folder, `examples/<name>/`, holding:
  - `<name>.arow`: the scene.
  - `sim_config.yaml`: the sim config.
  - `<name>.cpp`: the program, if the example has one.
- Robots, sensors and actuators used by more than one example live in
  `examples/shared/robots/`, `examples/shared/sensors/` and
  `examples/shared/actuators/`, one per file.
- The scene stays in the example's own folder.
- A robot includes either both its sensors and its actuators, or neither.
  When it includes neither, the scene includes them.
- Program code used by more than one example, such as the real-time loop,
  lives in `examples/shared/` as headers, and each gets its own test.
- A scene with no program of its own runs in `examples/viewer/`.
- `scripts/run-<name>.sh` builds and runs the example with its own files.
- File formats follow [file-formats.md](file-formats.md).

## The program

- Talks to the sim only through `Simulation` and the `interface` handles.
- All control logic, such as controllers and waypoint following, lives in
  the program. The sim only receives commands through actuators.
- Starts with a comment saying what the example shows, and a usage line.
- Runs in real time through the shared real-time loop.
- Supports `--headless` and `--seconds N`. A headless run prints state
  instead of opening windows, so the example runs without a display.
- Tuning values and geometry are named constants, with their quantity and
  unit in a comment.

## The test

- Each example gets `tests/examples_<name>.cpp`.
- The test loads the example's real files, never a copy.
- It checks that the scene behaves the way the program assumes.
- Each test simulates at most 2 s.
