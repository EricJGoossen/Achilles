# view.hpp

## Role

An algorithm's typed access to all its fields, over arena memory it doesn't
own. Each field addresses its own block through its own layout, indexing,
buffering and ordering, so fields with different policies sit side by side.

## Connections

- The view type comes from `view_factory.hpp`, which builds one
  `FieldBinding` per field from its traits through `engine/memory/binding.hpp`.
- `sim_allocator.hpp` constructs each view with its field bases, row counts,
  parities and row tables.
- `op_invoker.hpp` takes a `FieldCursor` per op argument and loads and
  stores through it in the loop.
- Interface handles (`attachment_handle.hpp`, `state_handle.hpp`) read and
  write single rows with `Load` and `Store`.

## Invariants

- Views stay trivially copyable: row tables are aggregates, and empty tables
  take no space.
- An index means a row for a scalar shape and a lane group for a batch shape.
  Callers must pass the matching unit.
- A broadcast input splats one row to every lane; a broadcast output starts
  from zero and stores as `row += sum of lanes`, read at store time.
- A field holding whole records hands them to ops in place, not copied.

## Pitfalls

- A view with fields over two orderings has two row counts. `Rows<F>()` is
  per field; the single-count shortcut only works when every field shares one
  ordering.
- Writes through a view bypass every check: nothing keeps a raw write
  consistent with the rest of the sim.

## When changing it

- `op_invoker.hpp` if cursor behavior changes.
- `engine/memory/binding.hpp` if `FieldBinding` defaults change.
- `tests/engine_view_view.cpp`.
