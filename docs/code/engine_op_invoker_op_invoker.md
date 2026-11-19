# op_invoker.hpp

## Role

Calls an op on the field values a traversal points at. It turns an op's
`kInputs` and `kOutputs` into cursor loads and stores, so ops never see
rows, memory or traversals.

## Connections

- Constructed by `RunPass` in `algorithm_step.hpp`, once per pass, from the
  op and the algorithm's view.
- Called by a traversal (`traversals.hpp`) with each (target, source) index
  pair.
- Reads the op's contract from `engine/spec/op_contract.hpp`; resolves each
  argument through its field's indexing policy via `View::FieldCursor`.

## Invariants

- Arguments are passed in this order: inputs by const reference, then any
  traversal inputs, then outputs by pointer. `OpArgsMatchView` checks the op
  against this.
- Outputs are read-modify-write: an output starts from what the field holds.
  An output resolved to a broadcast index starts from zero and accumulates.
- Outputs are stored in `kOutputs` order, so a broadcast output that may
  share memory with a group output is listed after it.
- The op is borrowed, not copied, so its `operator()` must be const.

## Pitfalls

- An op that assumes its outputs start at zero breaks when a field holds
  last tick's value. Zero it in `Initialize` or an earlier op.
- Whole records are handed in place unless the op also reads the same field,
  in which case it gets a copy.

## When changing it

- `engine/spec/op_contract.hpp` and [algorithms.md](../conventions/algorithms.md)
  if the argument order or contract changes.
- `tests/engine_pass_op_invoker.cpp`; every overload and both output kinds
  need a test ([testing.md](../conventions/testing.md) §9.2).
