# domain

Objects that are just data and behavior: no knowledge of memory, pipelines
or the engine.

## Structure

- Everything is in its own subfolder. No files sit directly in `domain/`.
- Each subfolder is a layer. It includes only `util`, itself and lower
  layers, never higher ones or peers in the same layer.

| Layer | Folders |
|---|---|
| 0 | `math`, `random`, `clock`, `layout`, `topology` |
| 1 | `spatial`, `geometry`, `error`, `archetype` |
| 2 | `attachment` |
| 3 | `sensing`, `actuation` |

- A concept that applies to several objects gets its own
  `<topic>_contract.hpp`; one that applies to a single object lives in that
  object's file.

## Objects

- Prefer objects over free functions.
- Objects own only their own members. They hold no pointers into sim memory
  and never read the clock or sim state themselves: time and inputs are
  passed in.
- Validity methods are named `IsValid()`.

## Math types

- Class templates on `T`, constrained by `ArithmeticLike`, that work for both
  scalars and SIMD batches.
- Comparisons return masks, never `bool`.
- Named constructors are static methods (`Zero`, `Identity`, `UnitX`).
- Operators are members or hidden friends.

## Kinds and effects

- Each kind derives from its mirrored base and declares `kListName`,
  `Params`, `State`, and a `Command` or `Reading` type.
- Kinds take their settings as a `Params` struct. Reading settings from user
  config happens in `interface`.
- Error effects are stateless structs with `Params`, `State` and a static
  `Apply`, checked with `EffectLike`.
- See [sim-model.md](sim-model.md) for what kinds may model.
