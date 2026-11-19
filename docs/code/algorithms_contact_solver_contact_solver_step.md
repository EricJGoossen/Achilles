# contact_solver_step.hpp

## Role

The stages of the soft-constraint contact solver, MuJoCo's primal CG: set
up rows, then iterate direction, line search and update, then hand the
constrained acceleration back to ABA. The iterations are step lists, not
loops.

## Connections

- Every stage derives from `SolverStep`, which runs one `engine::op_invoker::Step`
  over the solver's view with both the joint topology and the collision pair
  topology (`algorithms/row_sets/collision_pairs.hpp`).
- Reads ABA's factors and body state, the narrow phase's contacts and the
  joint limit rows through shared slots.
- `Solve` is part of the `SemiImplicitEuler` tick list; `InvWeightStep` is
  part of `Setup` (`algorithms/integrators.hpp`).
- `FinishStep` writes the constrained `qacc` into ABA's `qdd`, which the
  integrators then read.

## Invariants

- `Solve` runs after ABA and the collision pipeline in the same tick, and
  before sensing and integration.
- `kContactSolverIterations` and `kLineSearchIterations` set how many times
  the iteration lists repeat.
- `InvWeightStep` needs ABA to have run at the initial pose first.

## Pitfalls

- Each extra iteration adds a full direction, line search and update to
  every tick.
- Contact only exists in integrators whose tick list includes `Solve`.

## When changing it

- `contact_solver_ops.hpp` and `contact_solver_data.hpp`.
- `algorithms/integrators.hpp` if the stages or their order change.
- `tests/algorithms_contact_solver_contact_solver_step.cpp`.
