# policy

The interchangeable parts that define the sim's structure: anything where two
ways of doing it are conceivable (traversal, row access, ordering, layout,
indexing, buffering). Used polymorphically by the engine at compile time.

## Structure

- Everything is in its own subfolder. No files sit directly in `policy/`.
- Each policy type has its own subfolder, and each subfolder is a layer. It
  includes only `util`, `domain`, itself and lower layers, never higher ones
  or peers in the same layer.
- Every policy type has a concept. It lives in `base_<type>.hpp`
  (`base_traversal.hpp`), with any logic or types the type's policies share
  (like `Direction`), whether in a base class or not.
- A policy type has a base class only when its policies share logic, and
  then every policy of that type derives from it. Nothing is ever used
  polymorphically through a base.
- Every policy is named `<Kind><Type>` (`RowTraversal`, `AllRange`,
  `DoubleBuffering`) and has its own file named the same way
  (`row_traversal.hpp`).

| Layer | Folders |
|---|---|
| 0 | `layout`, `buffering`, `indexing`, `filter` |
| 1 | `ordering`, `source`, `range`, `scan` |
| 2 | `traversal` |

## Policies

- Every policy is an empty struct whose members are all `static`. Any
  runtime state it reads, such as buffer parity, is passed in, never stored.
- Each policy file ends with `static_assert(<Type>Like<Policy>)`. Templated
  policies are checked with one representative instantiation.
- Policies are chosen by type, in field traits or step aliases, and only
  configured by compile-time values like direction and stride.
- An alias that specializes a policy, like `TreeTraversal`, lives in that
  policy's file.
- Policies may name domain structure types like topology and layout. They
  know nothing of algorithms, fields or allocation. They compute indices and
  offsets, and never allocate.
