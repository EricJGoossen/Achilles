# interface

Everything the user touches: the `.arow` and YAML loaders, user config
parsing, the handles and the `Simulation` object. The top layer: it sees
every other layer, and nothing includes it.

## Rules

- User-facing objects stay minimal and mostly forward.
- `Simulation` is the only stateful user object. Its lifecycle is load, then
  `Init`, then `Step`; calls out of order throw.
- Handles are values holding a view and a row, cheap to copy, and every
  method forwards to the view.
- Handles are looked up by name and instance before the loop. An unknown
  name throws.
- Loaders are free functions. They validate everything entering the sim and
  convert units to SI. Each loader has its own error type, derived from
  `std::runtime_error`.
- Failures throw, never return `bool` or `optional`.
- Turning a user string into a value, like an integrator name into its enum,
  happens here.
- Raw sim state is exposed through state handles: unsynchronized, for
  debugging and logging, and writable only for quantities the state
  quantity list marks writable. Users never see an algorithm or a row.
- Everything a user touches lives in `interface/`.
- See [errors.md](errors.md) for what gets checked here.
