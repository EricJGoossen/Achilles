| scene / integrator | max qacc rel err | tip err vs MuJoCo @1s / 5s / 20s (% reach) | achilles vs truth @5s | MuJoCo vs truth @5s | energy drift (% PE span) |
|---|---|---|---|---|---|
| two_joint_arm/euler | 3.3e-13 | 2.8e-13 / 4.5e-12 / 8.1e-05 | 7.3e+00 | 7.3e+00 | 1.26e+01 |
| two_joint_arm/rk4 | 2.3e-13 | 2.2e-13 / 4.4e-12 / 1.8e-04 | 6.0e-06 | 6.0e-06 | 8.31e-06 |
| three_joint_arm/euler | 1.0e-13 | 3.7e-13 / 1.2e-10 / 4.4e+01 | 2.9e+01 | 2.9e+01 | 8.05e+00 |
| three_joint_arm/rk4 | 1.9e-11 | 2.1e-13 / 3.0e-11 / 1.2e+02 | 8.6e-05 | 8.6e-05 | 2.96e-04 |
| asymmetric_two_joint_arm/euler | 9.4e-14 | 2.4e-13 / 6.8e-12 / 1.0e-05 | 5.1e+00 | 5.1e+00 | 6.49e+00 |
| asymmetric_two_joint_arm/rk4 | 2.2e-14 | 1.8e-13 / 3.5e-12 / 5.6e-02 | 6.8e-07 | 6.8e-07 | 1.19e-08 |
| two_joint_arm_small_swing/euler | 7.3e-14 | 2.8e-13 / 1.7e-12 / 9.0e-12 | 1.6e-01 | 1.6e-01 | 8.03e-02 |
| two_joint_arm_small_swing/rk4 | 9.6e-14 | 4.1e-13 / 1.8e-12 / 8.9e-12 | 2.1e-08 | 2.1e-08 | 3.45e-10 |
