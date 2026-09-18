# Test File Protocol

This document is the contract for anything under `tests/`. If a test file
doesn't follow it, that's a bug in the test file, not a style nit.

## 1. File organization

- One test file per production header (or tight cluster of headers that
  can't be tested independently), mirroring `include/`'s path in the
  filename: `include/engine/assembler.hpp` -> `tests/engine_assembler.cpp`,
  `include/domain/spatial/inertia.hpp` -> `tests/domain_spatial_inertia.cpp`.
  Don't group unrelated modules into one file for convenience, and don't
  split one module's tests across files.
- `tests/CMakeLists.txt` globs every top-level `tests/*.cpp` file
  (`CONFIGURE_DEPENDS`, same as the root `CMakeLists.txt`'s own source
  glob) and builds one executable per file, named after its stem. Adding a
  file is enough to get it built and registered with CTest -- no
  CMakeLists.txt edit needed. The glob is non-recursive and intentionally
  so: a `.cpp` belongs at the top level of `tests/`, never under
  `tests/support/` (which holds shared headers only, never its own test
  executables -- see below).
- Shared test-only support code (fakes, archetypes, builders — see §3-4)
  lives under `tests/support/`, mirroring the `include/` path of the thing
  it stands in for: a shared `OpLike` fake used by more than one test file
  goes in `tests/support/engine_op_fakes.hpp`. Never put test-only types in
  `include/` or `src/`.

## 2. Anatomy of a test file

Order, top to bottom:

1. `gtest/gtest.h` on its own, then standard library includes together
   with any other third-party headers (e.g. `<xsimd/xsimd.hpp>`), then
   project includes.
2. An anonymous `namespace { }` for file-local helpers, fixtures, and
   fakes that only this file needs (§4).
3. `TEST(Suite, Case)` bodies.

Naming:

- `Suite` is the behavior or invariant under test, not the function name —
  `PlanarViewHeterogeneous`, `AssemblerRoundTrip`, not `TestAssembler`.
- `Case` states the scenario in one phrase — `JointModelRoundTripScalar`,
  `RejectsWrongArity`. Someone reading a failure list without opening the
  file should be able to tell what broke.
- Group related cases under the same `Suite` rather than inventing a new
  suite per test.

Every `TEST` gets a one-line comment above it stating *why this case
exists* if that's not obvious from the name — what it would catch, not
what it does. Existing comments in this directory are the bar.

## 3. Real code vs. fakes — the decision rule

Default to real production types. This is a math/physics engine: almost
everything is a deterministic pure-function-over-data value type
(`Vector3`, `Quaternion`, `Transform`, `Matrix`, `Inertia`, `JointModel`,
concrete `Assembler`s, `PlanarView`). There is no I/O, no hardware, no
clock, no network — the usual reasons to fake something don't exist here.
Constructing the real type *is* the test setup, and using it is what
catches a real type/behavior mismatch a fake would paper over.

Reach for a fake (a minimal hand-written type, not a real domain type)
only when one of these is true:

- **You're testing generic/templated engine code against a concept, not
  a concrete type.** `RunPass`, `Step`, `PlanarView`, `Assembler` are
  written against `OpLike`, `TraversalLike`, `TopologyLike`,
  `AssemblerLike`, etc. Testing them with the *smallest* type that
  satisfies the concept — not a real `JointModel` or `ABATopology` — is
  what proves the generic code only relies on what the concept promises,
  rather than accidentally depending on something a real type happens to
  also provide. This is archetype testing (§4) — use it for every concept
  that has a generic consumer.
- **The real type can't express the scenario.** E.g. testing that
  `AssemblerLike` rejects a type whose `Read`/`Write` round-trip doesn't
  match `ExpectedTupleT` needs a type built to violate that shape —
  no real `Assembler` does, by construction.
- **Building the real type is disproportionate to what's under test.**
  If reaching a specific edge case in real data requires threading
  through several unrelated subsystems, and the thing under test doesn't
  care about their correctness, a narrow fake removes that dependency.
  This is a judgment call, not a default — don't reach for it just to
  avoid writing a constructor call.

Never fake a plain value type (`Vector3`, `Quaternion`, `Matrix`, ...) to
avoid constructing one. If a fake and a real type would look identical,
use the real type.

## 4. Archetypes: what they are, and where they live

An archetype is a type that implements *exactly* a concept's required
interface and nothing more — no convenience members, no extra
constructors, no behavior beyond what makes the `requires` clause true.
Its job is to make the generic code fail to compile (or fail a
`static_assert`) the moment it uses anything the concept doesn't
guarantee.

```cpp
// Exactly what TopologyLike requires. Nothing else.
struct TopologyArchetype {
  std::size_t Size() const { return 1; }
  std::size_t operator[](std::size_t) const { return 0; }
};
static_assert(achilles::domain::topology::TopologyLike<TopologyArchetype>);
```

Rules:

- Name it `<Concept>Archetype` (drop the `Like` suffix): `OpLikeArchetype`
  -> `OpArchetype`, `AssemblerLike` -> `AssemblerArchetype`,
  `TopologyLike` -> `TopologyArchetype`.
- `static_assert` the concept against the archetype immediately after
  defining it, in the same scope. An archetype that fails to satisfy its
  own concept is a broken test, not a broken concept.
- Where it lives is purely about reuse, decided by scope, not by which
  concept it's for:
  - Used by exactly one test file -> define it in that file's anonymous
    namespace (§2), right before the tests that use it.
  - Used by more than one test file -> promote it to
    `tests/support/<mirrored/path>.hpp` and include it from both. Don't
    preemptively create a shared archetype before a second use exists.
- An archetype is not a mock. It has no call expectations, no captured
  arguments, no behavior verification — it exists to be *minimal and
  concept-shaped*, not to observe how it was used. If a test needs to
  assert "this was called with X," that's a fake with instrumentation
  (§5), not an archetype, and it almost never belongs in this codebase
  (§3 — real types make this unnecessary in nearly every case here).
- Every project-level concept must have its own archetype to verify that
  generic code uses only the features promised by that concept.

## 5. Instrumented fakes

If, per §3, real types genuinely can't cover a scenario and the thing
under test needs to be observed (not just satisfied), write a fake that
adds the minimum instrumentation needed — a call counter, a captured
argument — on top of an otherwise-archetype-shaped type. Keep the
instrumentation out of the archetype itself; if a concept check and a
behavior check both need the same shape, that's two types (an archetype
for the `static_assert`, a separate instrumented fake for the behavior
test), not one type serving both jobs.

## 6. Compile-time checks vs. runtime checks

A concept's `static_assert` (against an archetype or a real type) belongs
next to the archetype/type it's checking, evaluated unconditionally at
file scope — it doesn't need a `TEST` wrapper, since it fires at compile
time regardless of whether the binary ever runs. Runtime behavior
(actual `Read`/`Write` round-trips, traversal order, numeric results)
belongs in `TEST` bodies with `EXPECT_*`. Don't put a `static_assert` for
concept satisfaction inside a `TEST` body — it adds nothing a file-scope
`static_assert` doesn't already give you, and it makes the check look
conditional when it isn't.

## 7. Assertions

- `EXPECT_*`, not `ASSERT_*`, unless a failed check makes the rest of the
  test meaningless (e.g. a null/size check before indexing) — matches
  `test_helpers.cpp`.
- Floating point: `EXPECT_FLOAT_EQ` for `float`, `EXPECT_DOUBLE_EQ` for
  `double`. Never `EXPECT_EQ` on floating point.
- Integers and enums: `EXPECT_EQ`.

## 8. Code style

Test files follow the same C++ style as `include/`/`src/`, not a separate
"test code" convention. Two points that are easy to get wrong by default
(and that clang-tidy's `*`-with-exclusions config under `tests/` will
flag):

- Numeric literal suffixes are uppercase: `0.0F`, `1e-5F`, not `0.0f`,
  `1e-5f`.
- Every `if`/`for`/`while`/`else` body gets `{}`, even a single statement
  (`if (x) { return; }`, not `if (x) return;`). No exceptions for
  one-liners.

## 9. Precondition and generic-path coverage

Two classes of bug have shipped past this suite while looking covered:
a precondition `assert()` nobody ever actually tripped, and a generic
engine path (an invoker, a traversal, a pass) exercised by only one of
its several distinct shapes. Both slip through the same way — every
individual test passes, so the gap is invisible until the untested shape
or the untested violation actually occurs in real use. Check for both
whenever you touch `include/` code, not just when adding a test file.

### 9.1 Every runtime `assert` gets a death test

A production `assert(cond && "message")` (not `static_assert`) is a claim
about a precondition. An untested one isn't a verified claim — it's
either dead code (if it sits behind a macro nothing ever defines, as one
did before being fixed) or a check that fires on the wrong condition
entirely, discovered only when someone finally violates it for real. For
every such `assert` in `include/`:

- Write exactly one `EXPECT_DEATH(<call that violates cond>, "<substring
  of message>")` proving it fires for a genuine violation. Matching on a
  substring of the real message (not `""`) also proves you tripped *that*
  assert and not some other one that happened to fire first.
- Use the smallest input that violates only that condition. If the input
  also happens to violate an earlier check in the same function, the
  test proves the wrong assert fired.
- A generic, widely-repeated bounds guard (`i < size` on half a dozen
  near-identical accessors) doesn't need one death test per call site —
  one representative test for the pattern is enough. Exhaustively testing
  every bounds check is noise, not coverage. Anything domain-specific
  (a physical-validity check, a block-invertibility precondition, a
  structural-shape requirement) always gets its own.

### 9.2 A generic path with more than one shape gets a test per shape

Templated engine code (`OpInvoker`, `Traversal`, `Pass`, `Assembler`, ...)
often has more than one distinct *shape* of interface, and covering one
proves nothing about the others — `OpInvoker` and `SingleOpInvoker`
share the exact same output-handling bug because they're two separate
`Invoke` overloads implementing the same idea, and a test of one said
nothing about the other. Before treating a generic type as covered,
enumerate its shapes and check each has its own test:

- Every overload of the same operation (a two-index and a one-index
  `Invoke`, say).
- Both directions of a bidirectional thing (forward/backward traversal),
  not just one — unless the two are provably a trivial parameter flip of
  identical code, in which case say so in the test comment rather than
  skipping the second test silently.
- Overwrite (`=`) vs. accumulate (`+=`) outputs, wherever `ArgData`/
  `use_target` lets an Op express either. A suite built only from
  `=`-style ops (the common case) will not catch a bug specific to `+=`.
- Zero-length/empty input, wherever a loop's exit condition isn't an
  explicit `size == 0` guard (a `j-- > 0` loop is correct at zero only by
  evaluation order, not by an explicit check, and that's exactly the kind
  of thing a later rewrite can silently break).

### 9.3 A regression test reproduces the failure at the level it happened

When a bug is fixed, its test must exercise the actual code path that was
broken, not a lower-level primitive one step removed from it. A direct
test of an `Op`'s `operator()` proves that function's own logic, but
tells you nothing about the `OpInvoker` plumbing that calls it — if the
bug was in that plumbing, the fix's regression test has to go through
`OpInvoker` (or whatever the real call path is), not around it. Before
writing a regression test, name the exact function whose bug you're
locking in, and confirm the test actually calls it — not something it
happens to call internally.

### 9.4 SIMD code should be tested on different lane sized

Whenever code is generic to lane size, it must be tested with multiple 
lane sizes. It must be tested with at least 4, 8, and 32.