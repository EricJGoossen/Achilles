# Code rules

## Types and templates

- No runtime polymorphism: no `virtual`, `std::function` or `std::variant`.
  The one exception is dispatch resolved at init, like the integrator: it is
  chosen once and stays fixed until the next init.
- Every template parameter that expects an interface has a concept.
- Every concept name ends in `Like`.
- Every concept has a concept archetype in its test file that minimally
  satisfies it, used at every point that takes the concept.

## Files and namespaces

- Objects over free functions, except for operators, `util` and the
  `interface` loaders.
- One object per file, except data structs without behavior, groups of very
  small objects, and template metaprogramming.
- A helper used by only one object lives inside it, as a static or private
  member, never as a free function in `detail`. A helper shared by several
  objects gets its own place, preferably as a method on a type.
- Each folder has its own namespace and each namespace its own folder; only
  `detail` is exempt.
- No two files share a name, even in different folders.
- A file gets a `.cpp` when it has more than 100 lines of non-trivial logic.
  Trivial logic is methods of 1-2 lines, getters, setters and methods that
  only forward.

## Naming and lint

- Naming follows `.clang-tidy`.
- Never use `NOLINT`. Change the code so it passes, or disable the check
  globally in `.clang-tidy`.

## Numerics

- All quantities are SI (m, kg, s, rad, N, N·m). Other units are converted in
  `interface` on load.
- The same seed gives identical results on the same build.

## The sim loop

- All memory is allocated at init, in the one arena block. No `malloc` in
  the loop.
- The loop never references memory created before init: everything it uses
  is copied in at init. Old memory may persist for resets but is never
  referenced.
- Algorithm operations are SIMD wherever possible: load a full lane, then
  operate on it.
- The loop is branchless, to allow a GPU port later. `if constexpr` and
  fixed-count loops are fine; any data-dependent branch needs Eric's
  permission.
