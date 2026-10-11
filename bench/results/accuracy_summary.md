| scene / integrator | max qacc rel err | tip err vs MuJoCo @1s / 5s / 20s (% reach) | achilles vs truth @5s | MuJoCo vs truth @5s | energy drift (% PE span) |
|---|---|---|---|---|---|
| two_joint_arm/euler | 6.1e-06 | 1.3e-04 / 5.4e-03 / 1.3e+02 | 7.3e+00 | 7.3e+00 | 1.10e+01 |
| two_joint_arm/rk4 | 5.3e-06 | 6.5e-05 / 4.3e-03 / 1.6e+02 | 4.3e-03 | 6.0e-06 | 4.55e-04 |
| three_joint_arm/euler | 6.8e-06 | 7.8e-05 / 1.8e-02 / 1.0e+02 | 2.9e+01 | 2.9e+01 | 9.56e+00 |
| three_joint_arm/rk4 | 1.1e-05 | 7.7e-05 / 9.9e-04 / 1.7e+02 | 9.4e-04 | 8.6e-05 | 3.22e-04 |
| asymmetric_two_joint_arm/euler | 2.7e-06 | 7.1e-05 / 7.8e-04 / 6.7e+01 | 5.1e+00 | 5.1e+00 | 7.05e+00 |
| asymmetric_two_joint_arm/rk4 | 3.2e-06 | 6.4e-05 / 1.3e-03 / 9.4e+01 | 1.3e-03 | 6.8e-07 | 4.48e-04 |
| two_joint_arm_small_swing/euler | 3.5e-05 | 2.7e-04 / 1.1e-03 / 5.8e-03 | 1.6e-01 | 1.6e-01 | 7.92e-02 |
| two_joint_arm_small_swing/rk4 | 1.3e-05 | 1.6e-04 / 1.0e-03 / 5.7e-03 | 1.0e-03 | 2.1e-08 | 1.06e-03 |
