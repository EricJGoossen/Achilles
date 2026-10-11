# achilles vs MuJoCo benchmarks

Throughput and accuracy of achilles against MuJoCo 3.15.0 on serial-chain
arm scenes, from identical initial conditions at the same `dt` (2 ms,
float32-rounded on both sides).

## Running it

Inside the devcontainer (builds/runs achilles):

```bash
bench/build.sh             # Release: build-bench/ (float, SSE2),
                           #   build-bench-native/ (float, AVX2),
                           #   build-bench-native-double/ (double, AVX2)
bench/run_achilles.sh      # -> results/achilles_throughput.jsonl, results/traces/
```

With MuJoCo available (`pip install mujoco numpy matplotlib pyyaml`):

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
leaves the arm's trajectory bit-identical. A single robot (N = 1) runs the
scene as-is, since the wrapper would cost it a whole extra batch group the
MuJoCo model doesn't have.

`results/accuracy_summary_f64.{md,json}` is `compare_mujoco.py` run on
traces from `build-bench-native-double` (link it in as
`build-bench-native` in a scratch copy of the tree, as the scripts read
traces from there).

## Setup measured (2026-10-11)

Intel Core Ultra 9 285H, WSL2 + devcontainer, single thread on both sides.
achilles: clang 14, `-O3 -DNDEBUG`, with and without `-march=native`
(AVX2: 8 float / 4 double lanes; the project's default flags give SSE2,
4 float lanes), logging compiled out. MuJoCo: PyPI wheel 3.15.0, stepping
loop in C (`mj_step(nstep=...)` or `mujoco.rollout`), contacts disabled
(the scenes have none).

MuJoCo is timed two ways: **merged** (one `MjModel` holding all N arms)
and **rollout** (one single-arm model, N independent states stepped in
turn: the "N separate simulations" baseline). Speedups below are against
whichever of the two is faster at that N.

Scenes: `examples/two_joint_arm.arow`, `examples/three_joint_arm.arow`
(both released horizontally, chaotic) and `scenes/seven_joint_arm.arow`
(a 7-link planar chain -- see its header for why planar).

GCC: this code is about 3x slower built with GCC 12 than with clang 14
(measured on two_joint_arm), so the numbers here are clang only.

## Results

### Throughput (`results/throughput_summary.md`, `results/throughput.png`)

Robot-steps per second, best N for each engine:

| scene / integrator | achilles f32 AVX2 | achilles f64 AVX2 | MuJoCo merged | MuJoCo rollout |
|---|---|---|---|---|
| two-link / euler | 19.4 M (N=1024) | 9.4 M (N=256) | 6.9 M | 1.2 M |
| two-link / rk4 | 4.65 M (N=256) | 2.33 M (N=256) | 1.68 M | 0.32 M |
| three-link / euler | 12.5 M (N=256) | 6.3 M (N=256) | 4.25 M | 1.08 M |
| three-link / rk4 | 3.09 M (N=256) | 1.56 M (N=256) | 1.08 M | 0.29 M |
| seven-link / euler | 5.45 M (N=64) | 2.66 M (N=64) | 1.66 M | 0.77 M |
| seven-link / rk4 | 1.32 M (N=256) | 0.66 M (N=64) | 0.42 M | 0.20 M |

* **Float (the default build) is 2.2-3.9x faster than MuJoCo at every
  batch size from N = 16 up, on all three arms and both integrators**, and
  faster at N = 1 on the two- and three-link arms (2.1x, 1.6x). The one
  loss is a single seven-link arm on the AVX2 build (0.97x), where seven of
  every eight lanes are padding; the SSE2 build, half as wide, does that
  one at 1.21x.
* **Double, MuJoCo's own precision, is 1.2-2.0x faster** at every N from 4
  to 1024; at N = 1 it's 1.6-2.0x on the two- and three-link arms and even
  on the seven-link one (0.96-1.00x). At N = 4096 the two- and three-link
  arms are memory-bound and come out even (0.88-1.01x); the seven-link arm
  stays ahead (1.1-1.2x, against rollout -- MuJoCo can't compile a merged
  model of 4096 seven-link arms: "Could not allocate memory").
* Against N separate MuJoCo simulations (rollout), float achilles at its
  best N is 6.6-16x faster.

### Accuracy

Float (`results/accuracy_summary.md`), relative to MuJoCo in double:

* **Forward dynamics:** achilles' ABA `qdd` matches MuJoCo's `qacc` at every
  state of every trace to a relative error of at most 3.5e-5: float32
  round-off.
* **Trajectories:** end-effector within 6e-5 to 3e-4 % of reach of
  MuJoCo's at 1 s, 8e-4 to 2e-2 % at 5 s; the chaotic scenes then
  separate exponentially (MuJoCo against its own dt/100 reference does the
  same).
* **Against ground truth** (MuJoCo RK4 at dt/100): Euler error identical
  to MuJoCo Euler's. RK4 sits at 1e-3 to 4e-3 % of reach at 5 s, where
  MuJoCo's RK4 (double) is at 1e-4 to 1e-8 %: float RK4 is
  round-off-limited, not truncation-limited, at this dt.

Double (`results/accuracy_summary_f64.md`): ABA matches MuJoCo to 2e-14 to
2e-11 relative, trajectories agree to ~1e-12 % of reach at 5 s, and the
error against ground truth is the same as MuJoCo's to the digits reported,
for both integrators. The two engines compute the same dynamics; float is
the only accuracy cost.
