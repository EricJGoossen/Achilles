# Layers

Includes only point down this stack. Nothing includes `interface`.

`util` → `domain` → `policy` → `engine` → `algorithms` → `interface`

Inside `domain`, `policy` and `engine`, each subfolder is a layer. A folder
may include itself and lower layers, never higher ones or peers in the same
layer.

| Folder | Doc |
|---|---|
| `util` | [util.md](util.md) |
| `domain` | [domain.md](domain.md) |
| `policy` | [policy.md](policy.md) |
| `engine` | [engine.md](engine.md) |
| `algorithms` | [algorithms.md](algorithms.md) |
| `interface` | [interface.md](interface.md) |
