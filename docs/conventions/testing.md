# Testing

The contract for anything under `tests/`. If a test file doesn't follow it,
that's a bug in the test file, not a style nit. Section numbers are stable:
test comments cite them (for example "testing.md 9.2").

## 1. File organization

- One test file per production header, or per tight cluster of headers that
  can't be tested independently. Files with nothing to test, like pure alias
  lists, need none.
- The test file's name mirrors the header's path:
  `include/domain/spatial/inertia.hpp` -> `tests/domain_spatial_inertia.cpp`.
  Don't group unrelated modules into one file for convenience, and don't
  split one module's tests across files.
- `tests/CMakeLists.txt` globs every top-level `tests/*.cpp` file and builds
  one executable per file, named after its stem. Adding a file is enough to
  get it built and registered with CTest. The glob is non-recursive on
  purpose: a `.cpp` belongs at the top level of `tests/`, never under
  `tests/support/`.
- Shared test-only support code (fakes, concept archetypes, builders; see
  §3-4) lives under `tests/support/`, mirroring the `include/` path of the
  thing it stands in for: a shared `OpLike` fake used by more than one test
  file goes in `tests/support/engine_op_fakes.hpp`. `tests/support/` holds
  headers only. Never put test-only types in `include/` or `src/`.

## 2. Anatomy of a test file

Order, top to bottom:

1. `gtest/gtest.h` on its own, then standard library includes together
   with any other third-party headers (e.g. `<xsimd/xsimd.hpp>`), then
   project includes.
2. An anonymous `namespace { }` for file-local helpers, fixtures, and
   fakes that only this file needs (§4).
3. `TEST(Suite, Case)` bodies.

Naming:

- `Suite` is the behavior or invariant under test, not the function name:
  `ViewHeterogeneous`, `AssemblerRoundTrip`, not `TestAssembler`.
- `Case` states the scenario in one phrase: `TransformRoundTripScalar`,
  `RejectsWrongArity`. Someone reading a failure list without opening the
  file should be able to tell what broke.
- Group related cases under the same `Suite` rather than inventing a new
  suite per test.

Every `TEST` gets a one-line comment above it stating *why this case
exists* if that's not obvious from the name: what it would catch, not
what it does.

## 3. Real code vs. fakes: the decision rule

Default to real production types. Almost everything here is a deterministic
value type (`Vector3`, `Quaternion`, `Transform`, `Matrix`, `Inertia`,
concrete `Assembler`s, views). There is no I/O, no hardware, no clock, no
network, so the usual reasons to fake something don't exist. Constructing
the real type *is* the test setup, and using it is what catches a real
type/behavior mismatch a fake would paper over.

Reach for a fake (a minimal hand-written type, not a real domain type)
only when one of these is true:

- **You're testing generic code against a concept, not a concrete type.**
  Invokers, traversals, passes, views and assemblers are written against
  `OpLike`, `TraversalLike`, `TopologyLike`, `AssemblerLike` and so on.
  Testing them with the *smallest* type that satisfies the concept proves
  the generic code only relies on what the concept promises, rather than
  accidentally depending on something a real type happens to also provide.
  This is concept archetype testing (§4); use it for every concept that has
  a generic consumer.
- **The real type can't express the scenario.** For example, testing that
  `AssemblerLike` rejects a type whose read/write round-trip doesn't match
  its expected tuple needs a type built to violate that shape; no real
  `Assembler` does, by construction.
- **Building the real type is disproportionate to what's under test.**
  If reaching a specific edge case in real data requires threading
  through several unrelated subsystems, and the thing under test doesn't
  care about their correctness, a narrow fake removes that dependency.
  This is a judgment call, not a default. Don't reach for it just to
  avoid writing a constructor call.

Never fake a plain value type (`Vector3`, `Quaternion`, `Matrix`, ...) to
avoid constructing one. If a fake and a real type would look identical,
use the real type.

## 4. Concept archetypes: what they are, and where they live

A concept archetype (not the sim's `domain::archetype`) is a type that
implements *exactly* a concept's required interface and nothing more: no
convenience members, no extra constructors, no behavior beyond what makes
the `requires` clause true. Its job is to make generic code fail to compile
(or fail a `static_assert`) the moment it uses anything the concept doesn't
guarantee.

```cpp
// Exactly what TopologyLike requires. Nothing else.
struct TopologyArchetype {
  std::size_t Size() const { return 1; }
  std::size_t operator[](std::size_t) const { return 0; }
};
static_assert(TopologyLike<TopologyArchetype>);
```

Rules:

- Every project concept has a concept archetype, and it is used at every
  generic consumer of that concept.
- Name it `<Concept>Archetype`, dropping the `Like` suffix: `OpLike` ->
  `OpArchetype`, `AssemblerLike` -> `AssemblerArchetype`.
- `static_assert` the concept against the archetype immediately after
  defining it, in the same scope. An archetype that fails to satisfy its
  own concept is a broken test, not a broken concept.
- Where it lives depends only on reuse:
  - Used by exactly one test file: define it in that file's anonymous
    namespace (§2), right before the tests that use it.
  - Used by more than one test file: move it to
    `tests/support/<mirrored_path>.hpp` and include it from both. Don't
    create a shared archetype before a second use exists.
- An archetype is not a mock. It has no call expectations, no captured
  arguments and no behavior verification. If a test needs to assert "this
  was called with X", that's an instrumented fake (§5), and it almost never
  belongs here (§3).

## 5. Instrumented fakes

If, per §3, real types genuinely can't cover a scenario and the thing
under test needs to be observed (not just satisfied), write a fake that
adds the minimum instrumentation needed, such as a call counter or a
captured argument, on top of an otherwise archetype-shaped type. Keep the
instrumentation out of the archetype itself. If a concept check and a
behavior check both need the same shape, that's two types: an archetype
for the `static_assert`, and a separate instrumented fake for the behavior
test.

## 6. Compile-time checks vs. runtime checks

A concept's `static_assert` (against an archetype or a real type) belongs
next to the type it checks, at file scope. It doesn't need a `TEST`
wrapper, since it fires at compile time whether or not the binary runs.
Runtime behavior (round-trips, traversal order, numeric results) belongs in
`TEST` bodies with `EXPECT_*`. Don't put a concept `static_assert` inside a
`TEST` body: it adds nothing, and it makes the check look conditional when
it isn't.

## 7. Assertions

- `EXPECT_*`, not `ASSERT_*`, unless a failed check makes the rest of the
  test meaningless (e.g. a size check before indexing).
- Floating point: `EXPECT_FLOAT_EQ` for `float`, `EXPECT_DOUBLE_EQ` for
  `double`. Never `EXPECT_EQ` on floating point.
- Integers and enums: `EXPECT_EQ`.

## 8. Code style

Test files follow the same rules as `include/` and `src/` (see
[code-rules.md](code-rules.md)), not a separate test style. Two points are
easy to get wrong, and clang-tidy flags both under `tests/`:

- Numeric literal suffixes are uppercase: `0.0F`, `1e-5F`, not `0.0f`.
- Every `if`/`for`/`while`/`else` body gets `{}`, even a single statement.

## 9. Precondition and generic-path coverage

Two classes of bug pass a suite while looking covered: a precondition
`assert` nobody ever tripped, and a generic path exercised by only one of
its several shapes. Every individual test passes, so the gap is invisible
until the untested shape or violation happens in real use. Check for both
whenever you touch `include/` code, not just when adding a test file.

### 9.1 Every runtime `assert` gets a death test

A production `assert(cond && "message")` is a claim about a precondition.
An untested one is either dead code or a check that fires on the wrong
condition. For every such `assert`:

- Write exactly one `EXPECT_DEATH(<call that violates cond>, "<substring
  of message>")` proving it fires for a genuine violation. Matching a
  substring of the real message (not `""`) proves you tripped *that*
  assert and not another one that fired first.
- Use the smallest input that violates only that condition.
- A generic, widely repeated bounds guard (`i < size` on several
  near-identical accessors) needs one representative test for the pattern,
  not one per call site. Anything domain-specific (a physical-validity
  check, an invertibility precondition, a structural requirement) always
  gets its own.
- Death tests need asserts on, so the suite runs in a Debug build.

### 9.2 A generic path with more than one shape gets a test per shape

Templated engine code (invokers, traversals, passes, assemblers) often has
more than one distinct *shape*, and covering one proves nothing about the
others: two overloads implementing the same idea can share the same bug.
Before treating a generic type as covered, list its shapes and check each
has its own test:

- Every overload of the same operation.
- Both directions of a bidirectional thing (forward and backward
  traversal), unless the two are provably a trivial parameter flip of
  identical code. In that case say so in the test comment rather than
  skipping the second test silently.
- Overwrite (`=`) vs. accumulate (`+=`) outputs, wherever an op can express
  either. A suite built only from `=` ops will not catch a `+=` bug.
- Zero-length input, wherever a loop's exit condition isn't an explicit
  `size == 0` guard.

### 9.3 A regression test reproduces the failure at the level it happened

When a bug is fixed, its test must exercise the code path that was broken,
not a lower-level primitive one step removed from it. A direct test of an
op's `operator()` proves that function's logic, but says nothing about the
invoker that calls it. If the bug was in the invoker, the regression test
goes through the invoker. Before writing one, name the exact function whose
bug you're locking in, and confirm the test actually calls it.

### 9.4 Lane-generic code is tested at several lane sizes

Code that is generic over lane size is tested with at least 4, 8 and 32
lanes.

## 10. Thrown errors

Every throw in `interface` gets an `EXPECT_THROW` test naming its error
type, using the smallest bad input that triggers only that check. See
[errors.md](errors.md).
