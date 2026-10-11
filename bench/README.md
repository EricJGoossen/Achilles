# achilles vs MuJoCo benchmarks

Throughput and accuracy of achilles against MuJoCo 3.15.0 on the example
arm scenes, from identical initial conditions at the same `dt` (2 ms,
float32-rounded on both sides since `Simulation::Step` takes a `float`).

## Running it

Inside the devcontainer (builds/runs achilles):

```bash
bench/build.sh             # Release: build-bench/ (SSE2) + build-bench-native/ (AVX2)
bench/run_achilles.sh      # -> results/achilles_throughput.jsonl, results/traces/
```

On a host with MuJoCo (`pip install mujoco numpy matplotlib pyyaml`):

```bash
cd bench
python mujoco_throughput.py   # -> results/mujoco_throughput.jsonl
python compare_mujoco.py      # -> results/accuracy_summary.{json,md}, error_*.png
python report.py              # -> results/throughput_summary.md, throughput.png
```

`arow_to_mjcf.py` builds the MuJoCo model straight from each `.arow`, so
both engines simulate the same file. Batched runs hang `copies: N` arms off
a generated zero-DOF `bench_world` root (the loader forbids `copies` on a
root archetype); `achilles_bench trace ... wrapped` checks that the wrapper
leaves the arm's trajectory bit-identical.

## Setup measured (2026-10-06)

Intel Core Ultra 9 285H, WSL2 + devcontainer, single thread on both sides.
achilles: clang 14, `-O3 -DNDEBUG`, with and without `-march=native`.
MuJoCo: PyPI wheel 3.15.0, stepping loop in C (`mj_step(nstep=...)` or
`mujoco.rollout`), contacts disabled (the scenes have none).

MuJoCo is timed two ways: **merged** (one `MjModel` holding all N arms)
and **rollout** (one single-arm model, N independent states stepped in
turn: the "N separate simulations" baseline).

## Results

### Accuracy (`results/accuracy_summary.md`, `results/error_*.png`)

* **Forward dynamics:** at each of the 80k states achilles visited (4 scenes
  x 2 integrators x 10k steps), achilles' ABA `qdd` matches MuJoCo's `qacc`
  to a median relative error of about 1e-8 and at most 7e-7. The residual is
  float32 plumbing, not dynamics: `sim_config_loader.cpp` parses gravity into
  `std::array<float, 3>` (9.8f is 2e-8 relative off), and
  `RungeKutta4Step` builds its weights in float (`dt / 6.0F`).
* **Trajectories, engine vs engine** (end-effector error as % of reach):

  | scene | integrator | @1 s | @5 s | @20 s |
  |---|---|---|---|---|
  | two-link, small swing (non-chaotic) | euler / rk4 | 1e-6 / 4e-6 % | 7e-6 / 3e-5 % | 3e-5 / 1.2e-4 % |
  | two-link, horizontal release (chaotic) | euler / rk4 | 3e-6 / 1e-5 % | 2e-5 / 7e-5 % | diverged (chaos) |
  | three-link (chaotic) | euler / rk4 | 3e-6 / 1e-5 % | 2e-5 / 7e-5 % | diverged (chaos) |

  The chaotic scenes separate exponentially after about 10 s. MuJoCo
  against its own dt/100 reference does the same, so this is the Lyapunov
  exponent at work and not an engine error.
* **Against ground truth** (MuJoCo RK4 at dt/100): achilles Euler and MuJoCo
  Euler have the same error to 5 significant figures. achilles RK4 is about
  10x further from the truth than MuJoCo RK4, and the gap grows linearly in
  time. That pattern matches the float32 RK4 weights above.

### Throughput (`results/throughput_summary.md`, `results/throughput.png`)

Peak robot-steps/s, dt = 2 ms, single thread:

| scene / integrator | achilles AVX2 (best N) | MuJoCo merged | MuJoCo rollout |
|---|---|---|---|
| two-link / euler | 1.65 M (N=256-1024) | 4.48 M | 0.86 M |
| two-link / rk4 | 0.38 M (N=64) | 1.21 M | 0.23 M |
| three-link / euler | 1.09 M (N=256) | 2.93 M | 0.77 M |
| three-link / rk4 | 0.24 M (N=64) | 0.77 M | 0.21 M |

* Batching works. On the two-link arm (Euler), achilles goes from 0.40 M
  steps/s for one robot (best build) to 1.65 M at N=1024, a 4.1x gain.
* achilles beats MuJoCo's one-model-per-robot rollout by about 2.1x
  (two-link Euler, N=1024).
* MuJoCo with every robot merged into one model is still 2.6-4x faster than
  achilles at every N. On these scenes achilles is not faster than MuJoCo.
* Per-robot throughput drops at N=4096, where the working set no longer
  fits in cache. At N=1 the AVX2 build is slower than SSE2 because one
  robot gets padded out to 4 lanes.
