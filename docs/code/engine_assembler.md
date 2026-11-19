# assembler.hpp

## Role

Converts a domain type to and from its flat scalar leaves in memory, for one
instance or a whole SIMD batch at once. Every field's traits name an
`Assembler`; it is how a `Transform` or `Inertia` becomes rows of floats.

## Connections

- Each domain math type declares its own assembler alias next to itself (for
  example `Vector3Assembler` in `domain/math/vector3.hpp`).
- Field traits in each `*_data.hpp` pick one as `Assembler`.
- `View` asks the assembler which leaves a value has and the field's layout
  where each one sits.
- `sim_allocator.hpp` uses `PaddingSeed` for padding rows and the leaf order
  to populate fields from `.arow` data.

## Invariants

- The leaf order is the flattening order of the assembler's children. The
  `.arow` must give values in that order: a transform is translation, then
  rotation as `w, x, y, z`.
- A padding row holds the type's own `PaddingSeed()`, not zero: a zero
  quaternion is degenerate.
- `LeafAssembler<R>` holds a whole record per row; with several record types
  each sits in its own segment (`SegmentedLayout`).
- `Accumulate` sums only the lanes its mask names into the one row.

## Pitfalls

- Reordering an assembler's children silently changes what every `.arow`
  file's values mean.
- Non-arithmetic records read one instance at a time, never as a batch.

## When changing it

- [file-formats.md](../conventions/file-formats.md) if a leaf order changes.
- `tests/engine_assembler.cpp`.
