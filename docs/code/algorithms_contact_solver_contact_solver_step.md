# contact_solver_step.hpp

## Role

The stages of the soft-constraint contact solver, MuJoCo's primal Newton
or CG: set up rows, then iterate direction, line search and update, then
hand the constrained acceleration back to ABA. `SolveFor<Solver>` is each
solver's whole solve. Iterations past the first stop once converged
(`engine::pass::RepeatWhileStep` on `kContinue`).

## Connections

- Every stage derives from `SolverStep`, which runs one `engine::pass::Step`
  over the solver's view with both the joint topology and the collision pair
  topology (`algorithms/orderings/collision_pairs.hpp`).
- Reads ABA's factors and body state, the narrow phase's contacts and the
  joint limit rows through shared slots.
- `SolveFor<Solver>` is part of the `SemiImplicitEulerWith<Solver>` tick
  list, and `integrators.hpp` builds one tick per solver; `InvWeightStep`
  is part of `Setup` (`algorithms/integrators.hpp`).
- `interface::Simulation` picks the tick from `SimConfig::contact_solver`
  whenever the integrator or the config changes.
- Newton's `NewtonUpdateStep<true>` factors H once per tick, after the
  warmstart; the iterations after it reuse the factors.
- `FinishStep` writes the constrained `qacc` into ABA's `qdd`, which the
  integrators then read.

## Invariants

- `Solve` runs after ABA and the collision pipeline in the same tick, and
  before sensing and integration.
- At most `kContactSolverIterations` iterations, the first always; each runs
  `kLineSearchIterations` line-search steps.
- Every update ends in `ConvergedOp`, which writes `kContinue` for the next
  check.
- `InvWeightStep` needs ABA to have run at the initial pose first.

## Pitfalls

- Each extra iteration adds a full direction, line search and update to
  the tick.
- In floats the improvement rarely gets below about `1e-8`; a tolerance
  that low runs most solves to the cap.
- The Hessian drops cross terms between two moving bodies in contact, so
  Newton converges more slowly there, though still to the same answer.
- Contact only exists in integrators whose tick list includes `Solve`.

## When changing it

- `contact_solver_ops.hpp` and `contact_solver_data.hpp`.
- `algorithms/integrators.hpp` if the stages or their order change.
- `tests/algorithms_contact_solver_contact_solver_step.cpp`.
