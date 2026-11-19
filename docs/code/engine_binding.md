# binding.hpp

## Role

Reads a field's traits into the policies the engine uses: sharing,
ordering, indexing, buffering and layout. The one place optional traits get
their defaults and every declared policy is checked against its concept.

## Connections

- Reads `Traits<F>` from each algorithm's `*_data.hpp`.
- Used by `sim_allocator.hpp` to size and place blocks, by `view_factory.hpp`
  to build each field's `FieldBinding`, and by `sim_context.hpp` to collect
  orderings and buffer groups.
- `algorithms/attachment_kinds/sim_input.hpp` uses the same aliases to read
  another algorithm's field policies.

## Invariants

- Defaults: no `Ordering` means `LinearOrdering`, no `Indexing` means
  `RowIndexing`, no `Buffering` means `SingleBuffering`. `View`'s
  `FieldBinding` defaults must stay the same.
- `Layout` has no default; leaving it out is a compile error.
- `SlotId<Tag>()` keys shared slots and plans by a type's address. It only
  works because each tag is one type everywhere; a tag redeclared in two
  places would split a shared block.

## Pitfalls

- A trait spelled wrong (`Orderings` instead of `Ordering`) is silently
  ignored and the default is used.

## When changing it

- `engine/view/view.hpp` if a default changes.
- [engine.md](../conventions/engine.md) and
  [algorithms.md](../conventions/algorithms.md) if a trait is added or
  becomes required.
- `tests/engine_memory_binding.cpp`.
