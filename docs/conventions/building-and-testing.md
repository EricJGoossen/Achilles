# Building and testing

## Building

- Never build unbounded; a full build has crashed WSL. Use at most:
  `(ulimit -v 3500000; nice -n 10 cmake --build <dir> -j6)`.
- Prefer building only the targets you need:
  `cmake --build <dir> --target <test_stem>`.

## Checks

- Format: `scripts/check-format.sh`.
- Tidy: `scripts/check-tidy.sh`.

## Running tests

- Each `tests/*.cpp` builds into its own executable named after its stem.
- Run one with `ctest --test-dir <dir> -R <test_stem>`.
- Writing tests follows [testing.md](testing.md).

## Examples

- Adding or changing examples follows [examples.md](examples.md).
