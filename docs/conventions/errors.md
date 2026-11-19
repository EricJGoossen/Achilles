# Errors

- Every contract that can be violated is checked, preferring a concept, then
  a `static_assert`, then an `assert`.
- Sim internals use `assert`. Exposing raw sim state to the user does not
  make it user-facing.
- Only `interface` throws. All data entering the sim is validated there.
- A check too big for one expression becomes a validity method on the
  object, named `IsValid()`, which the object asserts.
- `interface` also calls a validity method and throws when the check is
  user-facing: when a user mistake could plausibly violate it.
