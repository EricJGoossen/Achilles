# repeat_while_step.hpp

## Role

`RepeatWhileStep<Body, Algorithm, Flag, Max>`: a Step that runs the step
list `Body` while `Algorithm`'s global field `Flag` is nonzero, at most
`Max` times. It's how an iterative solver stops early without any pass
branching on data.

## Connections

- Reads `Flag` through `SimContext::ViewFor<Algorithm>()` at index 0, so
  the flag must be a `GlobalIndexing` field.
- Runs `Body` through `SimContext::Step`, like any step list.
- The contact solver's `Iterate` (`contact_solver_step.hpp`) is the user.

## Invariants

- The flag is checked before every run, so a flag already clear runs
  nothing.
- `Body`'s last step is what writes the flag for the next check.
- With the flag never clearing it's `RepeatListT<Body, Max>`.

## Pitfalls

- The loop is data-dependent: ticks no longer all run the same passes.
  Cap `Max` to bound the worst case.

## When changing it

- `tests/engine_pass_repeat_while_step.cpp`.
