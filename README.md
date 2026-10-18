# Achilles

A C++20 articulated rigid-body dynamics engine. Achilles loads a jointed
scene from a YAML `.arow` file and runs forward dynamics on it with the
Articulated Body Algorithm (ABA). It integrates the result at a fixed
timestep and can draw the scene live in an OpenGL viewer.

The engine is built around compile-time contracts (C++20 concepts). Each
algorithm is written as small, typed **Ops** that the engine runs over the
joint tree in a planned memory layout, so adding a new algorithm doesn't
mean writing a new hand-rolled loop.

## Quick start

```bash
scripts/run-example.sh            # build, then open the viewer on the demo arm
scripts/run-example.sh --headless # same simulation, no window
```

This builds the `achilles` target into `build/` and runs it against
[examples/two_joint_arm.arow](examples/two_joint_arm.arow) with
[examples/sim_config.yaml](examples/sim_config.yaml). In the viewer,
left-drag to orbit, scroll to zoom, and close the window to stop.

## Building

### Dependencies

Fetched automatically by CMake (`FetchContent`):

| Library    | Version | Used for                         |
|------------|---------|----------------------------------|
| yaml-cpp   | 0.8.0   | `.arow` and config loading       |
| GLFW       | 3.4     | Window and input for the viewer  |
| GoogleTest | 1.14.0  | Test suite                       |

Needed on the system:

- CMake ≥ 3.16, Ninja, and a C++20 compiler (CI uses Clang)
- OpenGL and X11 development headers (`libgl1-mesa-dev`, `libx11-dev`,
  `libxrandr-dev`, `libxinerama-dev`, `libxcursor-dev`, `libxi-dev`,
  `libxext-dev`). You need these even for headless builds, because GLFW
  is always compiled.
- [xsimd](https://github.com/xtensor-stack/xsimd), installed as headers
- clang-format / clang-tidy 18, only for the lint targets

The easiest setup is the devcontainer in [.devcontainer/](.devcontainer/).
It has every dependency and the pinned LLVM tools, and it turns on the
pre-commit hook. CI uses the same image.

### Commands

```bash
cmake -S . -B build -G Ninja                     # Debug by default
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

CMake options:

- `BUILD_TESTS` (default `ON`): build the GoogleTest suite.
- `ENABLE_CCACHE` (default `ON`): use ccache if it's installed.

## Usage

```
achilles <scene.arow> [sim_config.yaml] [--headless]
```

The binary loads the scene, applies the optional config, and steps the
simulation at a fixed 1/120 s. It renders a frame per step unless you pass
`--headless`. You can also drive a simulation from code through
`achilles::interface::Simulation`
([include/interface/simulation.hpp](include/interface/simulation.hpp)):

```cpp
achilles::interface::Simulation sim;
sim.LoadArowFile("scene.arow");
sim.LoadConfigFile("sim_config.yaml");  // optional
sim.Init();
sim.SetHeadless(false);                 // headless is the default
while (sim.Step(1.0F / 120.0F)) {}
```

### Scene files (`.arow`)

A `.arow` file names an archetype and lists its joints. Each joint has a
`parent` (`null` for the root) and a `fields` map with its initial state:
joint subspace and activation mask, fixed joint transform, rigid-body
inertia, position, velocity, torque, and optional `visual_extents` /
`visual_color` for the viewer. The demo scene has a comment on every
field. The loader is in
[include/interface/archetype_loader.hpp](include/interface/archetype_loader.hpp).

### Simulation config

`sim_config.yaml` holds global parameters. For now that's
`base_acceleration`. For ordinary gravity on a fixed base, set it to
negative gravity:

```yaml
base_acceleration:
  angular: [0, 0, 0]
  linear: [0, 0, -9.8]
```

## Architecture

```
include/
  domain/      Math and spatial-algebra value types (Vector3/6, Quaternion,
               Matrix, spatial Transform/Inertia/Dual), archetypes, joint topology
  engine/      The generic runtime: field/op/algorithm contracts, memory
               (arena, allocator, bindings), topology layout and ordering,
               views, and the pass machinery (traversals, op invoker, sim context)
  algorithms/  The concrete algorithms and the registry that lists them
    aba/         Articulated Body Algorithm -> joint accelerations
    vi/          Velocity integration
    pi/          Position integration
    broad_phase/ Collision broad phase: sphere-swept-box shapes -> world
                 bounds -> filtered candidate-pair bitmask
    narrow_phase/ Collision narrow phase: candidate pairs -> per-pair contact
                 (signed distance, normal, point, active bit)
    viz/         Render-only state (world transforms, visuals); never stepped
  interface/   Loaders and the Simulation facade
  render/      The OpenGL/GLFW viewer
  util/        Buffers, SIMD helpers, template metaprogramming, I/O
src/           Implementations matching include/
tests/         One GoogleTest executable per header
```

The main pieces:

- **Ops** ([include/engine/op_contract.hpp](include/engine/op_contract.hpp)):
  a function object that states which fields it reads (`kInputs`) and
  writes (`kOutputs`), and whether each one comes from the target joint
  or its parent. The engine gathers those arguments and passes them to
  `operator()`. If an Op needs to seed state at the root, it can also
  define `Initialize`.
- **Traversals** ([include/engine/pass/traversals.hpp](include/engine/pass/traversals.hpp)):
  set the order Ops run in over the joint tree. The options are a forward
  or backward tree walk, a flat linear pass, or an all-pairs walk that
  checks one row against a whole SIMD lane group at a time. A field holds
  one value per row by default; naming
  `Indexing = topology::PairGroupIndexing<Lanes>` in its Traits gives it one
  value per (row, lane group of rows) pair instead -- e.g. a bitmask of which
  targets in that group overlap the row -- and
  `PairGroupIndexing<Lanes, Lanes>` one value per (row, row) pair, read a
  lane group at a time
  ([include/engine/topology/indexing_policy.hpp](include/engine/topology/indexing_policy.hpp)).
- **Registry** ([include/algorithms/registry.hpp](include/algorithms/registry.hpp)):
  the list of algorithms the simulation hosts, in the order they run.
  Each step runs ABA, then VI, then the collision broad and narrow
  phases, then PI -- so a contact solver can correct VI's velocity before
  PI integrates it.
  Shared fields are aliased from one algorithm to the next, so nothing is
  copied between them. Adding an algorithm takes one line here.

## Development

```bash
cmake --build build --target check-format         # clang-format (src/, include/)
cmake --build build --target check-tidy           # clang-tidy (src/, include/, tests/)
cmake --build build --target check-tidy-analyzer  # Clang Static Analyzer (slow)
```

- **Pre-commit hook:** [.githooks/pre-commit](.githooks/pre-commit) blocks
  commits that aren't clang-format clean. The devcontainer turns it on.
  Anywhere else, run `git config core.hooksPath .githooks`.
- **CI:** [.github/workflows/ci.yml](.github/workflows/ci.yml) runs format,
  tidy, tidy-analyzer, and build & test on every PR and on every push to
  `main`.
- **Tests:** read [tests/TESTING.md](tests/TESTING.md) before adding a
  test. Any new `tests/*.cpp` file is picked up and registered with CTest
  automatically.

## License

Apache License 2.0. See [LICENSE](LICENSE).
