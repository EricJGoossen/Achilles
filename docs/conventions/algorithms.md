# Algorithms

The math that operates on the data. Each algorithm has its own folder with
1-3 files: `*_data`, `*_ops` and `*_step`.

## `*_data.hpp`: declarations only

- One `enum class <Algo>Field : uint8_t`, grouped by role (inputs, state,
  scratch, outputs) and ending in `kCount`, followed by
  `static_assert(FieldEnumLike)`.
- One `<Algo>FieldTraits<F>` specialization per field:
  - Required: `Assembler`, built from `conventions` types, and `Layout`.
  - Optional: `SharedAs`, `Ordering`, `Indexing` and `Buffering`.
  - `kName` only on fields loaded from `.arow` or read by sensors.
- Then `static_assert(FieldTraitsLike)`, and the view and topology aliases,
  each with its concept check.
- No functions, ops or metaprogramming.

## `*_ops.hpp`: math on one row group

- Each op is a struct with:
  - `FieldEnum`.
  - `kInputs` and `kOutputs` as `ArgData` arrays, each entry naming a field
    and whether it reads the target or parent row.
  - A `const operator()`: inputs by `const&`, then outputs by pointer, in
    the same order as the arrays.
- Seeding the base row uses the optional `kInitInputs`, `kInitOutputs` and
  `Initialize`.
- Each op is followed by `static_assert`s for `OpLike` and
  `OpArgsMatchView`, plus `OpHasInit` if it has `Initialize`.
- An op sees only its arguments. It has no traversal, no row indexing, no
  loops over rows, and no fields from another algorithm.
- The only state is immutable parameters set in the constructor.
- Math goes through domain types and stays branchless.
- Ops in one algorithm may share a common base.

## `*_step.hpp`: wiring only

- Each step is a struct with `FieldEnum` and a
  `static Step(sim_state, sim_config, dt)` templated on `SimContextLike`.
- The body fetches the view and topology, builds the ops from config, and
  makes one `engine::pass::Step(Ops<Pass<Op, Traversal>...>)` call.
- A step may make other engine calls, such as flipping a buffer's parity,
  but has no loops, conditions or math.
- Traversal aliases are chosen here.
- Steps in one algorithm may share a common base.
- The file ends with `using <Algo>Algorithm = engine::Algorithm<...>`,
  checked with `AlgorithmLike`.

## Sharing

- Algorithms never include each other.
- Algorithms never share data directly: shared data goes through the
  engine's `SharedAs` feature.
- Anything that depends on several algorithms goes in the shared top-level
  files below. Those are list definitions, shared definitions and aliases
  for the engine, with no logic.

## Top-level files

- `conventions.hpp`: concrete type aliases, the lane count and shape
  constants. Nothing else.
- `shared_slots.hpp`: only forward-declared tag structs, named
  `<Quantity>Slot` after the physical quantity, never after an algorithm.
  Every algorithm sharing a tag agrees on its assembler and ordering.
- Hosting list: one `TypeList` of every hosted algorithm. An algorithm comes
  before any that read its shared slots, because the first to name a slot
  owns it. Hosting says what memory exists, not what runs.
- Step lists: tick lists built only from `TypeList`, `ConcatT` and
  `RepeatListT`, plus their repeat counts. Each tick advances the clock
  exactly once, at the end.
- `row_sets/`: orderings for rows other than one per joint, such as
  collision pairs, shared by several algorithms, plus the field-name tags
  they read.
- Algorithm folders include only `conventions`, `shared_slots` and
  `row_sets/` from the top level, never the hosting or step lists. The
  top-level files may include any algorithm.

## Concrete types

`conventions.hpp` sets every concrete data type, so changing a scalar or
vector type across the sim is a one-line edit. `domain` and `engine` stay
templated on these types.

## Constants

- Shape constants, which size fields or set memory layout (contact slots,
  collision group count), go in `conventions.hpp`.
- Repeat counts, which say how many times a step list repeats a stage
  (solver iterations), go in the top-level step-list file, next to the list
  that repeats.
