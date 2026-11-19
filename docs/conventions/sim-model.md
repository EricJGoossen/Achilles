# Sim model

- Controllers never belong in the sim; they sit on the caller's side of a
  command.
- Sensors and actuators are exact mirrors of each other, and never name an
  algorithm: their data comes through the engine.
- Only add actuator kinds for parts that physically exist.
- Kinds only get custom code when their shape truly differs. Error logic
  lives in shared effects.
- No new object per kind: use templates, or `using` aliases.
- Shared logic never lives in a file named after one kind.
- Motors carry no CAN or message protocol logic.
