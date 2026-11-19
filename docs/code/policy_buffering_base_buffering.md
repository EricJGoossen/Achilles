# buffering_policy.hpp

## Role

Single and double buffering for a field's block. A double-buffered field
lets one side write the next value while another reads the current one,
swapped by flipping its group's parity.

## Connections

- Chosen per field by `using Buffering = DoubleBuffering<Group, Side>;` in
  its traits, resolved by `binding.hpp`.
- `sim_allocator.hpp` carves one `BufferParity` per group and replicates the
  first half into the second at init.
- `View` reads through `ReadOffset`; `BufferedScalar` writes the write half.
- `SimContext::FlipParity<Group>()` swaps a group.
- Groups in use today: ABA's joint torque (`JointTorqueSlot`, flipped by
  `aba::SwapTorqueStep`) and the attachment outputs (`AttachmentOutputSlot`,
  flipped by `attachment::ReadStep` and `InitStep`).

## Invariants

- Both sides of one double-buffered field name the same `SharedAs` slot, so
  they share one block; `Storage` makes their shapes compare equal.
- One parity per group, in the arena, so every copy of a view or context
  sees the same flip.
- A flip resets nothing. The step that flips decides what the new write
  half starts from: ABA zeroes it, the attachment outputs overwrite it.

## Pitfalls

- An input buffer (torque) flips before the passes that read it; an output
  buffer (attachment readings) flips after the passes that write it.
  Flipping on the wrong side makes readers see the wrong tick.

## When changing it

- `engine/view/view.hpp` and `sim_allocator.hpp`.
- `tests/engine_memory_buffering_policy.cpp`.
