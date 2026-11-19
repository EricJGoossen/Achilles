# engine

The plumbing that moves data: it invokes algorithms in the right order with
the data they ask for, without influencing what they compute.

## Structure

- Everything is in its own subfolder. No files sit directly in `engine/`.
- Exactly one level of subfolders.
- Each subfolder is a layer. It includes only `util`, `domain`, `policy`,
  itself and lower layers, never higher ones or peers in the same layer.

| Layer | Folders |
|---|---|
| 0 | `assembly`, `memory` |
| 1 | `contract` |
| 2 | `binding` |
| 3 | `view`, `pass` |
| 4 | `context` |
| 5 | `allocation` |

## Rules

- Engine code is generic over field enums, traits, ops and views. The only
  concrete types it names are domain structure types. Nothing here is
  specific to one algorithm or piece of memory.
- Every requirement on algorithm files is a concept in `contract/`, so a
  malformed algorithm fails to compile where it is declared.
- Optional traits have a documented default. Required traits have none, and
  omitting one is a compile error.
- The engine owns all control flow: traversal loops, ordering and running
  step lists.
- Init-time code (`allocation/`) may use standard containers. Loop-time code
  (`view`, `pass`, and the step path in `context`) uses only arena memory.
- Algorithms only see typed cursors from views, never raw bytes.
