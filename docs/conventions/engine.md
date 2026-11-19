# engine

The plumbing that moves data: it invokes algorithms in the right order with
the data they ask for, without influencing what they compute.

## Structure

- Subfolders group related files. A file with no group sits directly in
  `engine/` (`algorithm.hpp`, `binding.hpp`).
- Exactly one level of subfolders.
- Each subfolder is a layer. It includes only `util`, `domain`, `policy`,
  `engine/`'s own files, itself and lower layers, never higher ones or peers
  in the same layer.

| Layer | Folders |
|---|---|
| 0 | `assembly`, `memory` |
| 1 | `spec` |
| 2 | `view` |
| 3 | `op_invoker` |
| 4 | `context` |
| 5 | `allocation` |

- A concept that applies to several objects gets its own
  `<topic>_contract.hpp`; one that applies to a single object lives in that
  object's file.

## Rules

- Engine code is generic over field enums, traits, ops and views. The only
  concrete types it names are domain structure types. Nothing here is
  specific to one algorithm or piece of memory.
- Every requirement on algorithm files is a concept in `spec/`, so a
  malformed algorithm fails to compile where it is declared.
- Optional traits have a documented default. Required traits have none, and
  omitting one is a compile error.
- The engine owns all control flow: traversal loops, ordering and running
  step lists.
- Init-time code (`allocation/`) may use standard containers. Loop-time code
  (`view`, `op_invoker`, and the step path in `context`) uses only arena
  memory.
- Algorithms only see typed cursors from views, never raw bytes.
