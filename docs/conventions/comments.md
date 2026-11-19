# Comments

Comments are what humans read; docs are only for agents (see
[writing-docs.md](writing-docs.md)). Anything a human needs to understand the
code goes in a comment.

## Files

Every source file starts with a short comment saying what it holds and how
it fits into its folder. If the file has a doc, the comment links it:

```cpp
// The one init-time builder of the arena and the SimContext.
// Doc: docs/code/engine_allocation_sim_allocator.md
```

## Classes

Every class and struct gets a comment above it, except template
metaprogramming in `util/tmp`.

- Says what the object is for, in at most 100 words.
- Ends with a short example of how it is used.
- A concept's comment says what it guarantees.

## Methods and functions

Every method gets a comment above its declaration.

- A description of what it does and how to call it, in at most 50 words.
  The labeled lines below don't count toward the limit.
- Then one labeled line per item, in this order:
  - `<param>:` for each parameter, including template parameters. Output
    pointers say `(out)`.
  - `Returns:` what it returns.
  - `Requires:` the preconditions it asserts.
  - `Throws:` the cases where it throws, for `interface` only.
  - `Example:` a short example of how it is used.
- No Doxygen tags.
- Exempt: getters, setters, methods that only forward, and the parameters of
  an op's `operator()`, which its `kInputs`, `kOutputs` and field comments
  already describe.

```cpp
// Moves a parent body's velocity into this body's frame and adds the
// joint's own motion.
//
// x_up: transform from the parent body to this body.
// v_parent: the parent body's spatial velocity.
// qd: the joint's velocity, in joint coordinates.
// Returns: this body's spatial velocity.
// Requires: x_up is a proper rigid transform.
// Example: Velocity v = PropagateVelocity(x_up, v_parent, qd);
Velocity PropagateVelocity(
    const Transform& x_up, const Velocity& v_parent, const Velocity& qd
);
```

## Fields and enum values

- One line each, saying what it holds. Inline if it fits.
- Math uses the standard symbol beside the code's name:
  `kArticulatedInertia,  // I_A`.
- Constants and parameters name their quantity and unit:
  `kCruiseSpeed = 0.8F;  // m/s along the path`.

## Inside definitions

- Line comments inside a method explain how it works, only where the code
  is tricky or hard to follow. Never restate what an obvious line does.
- Template metaprogramming always gets these: an inline or one-line comment
  on each step that isn't obvious.

## Writing comments that last

- Explain why: intent, invariants, constraints and the reason for a choice.
- Describe the code as it is. No history: no "now", "used to", "the old
  version" or "before X existed".
- No references to branches, reviews, tickets or the bug being fixed.
- Refer to other code by symbol name or file path, never by line number.
- Examples show how something is used, never list where it is called from.
- No commented-out code.

## Style

- `//` only.
- Section dividers like `// --- Outputs ---` are fine for grouping.
- Test comments follow [testing.md](testing.md) §2.
