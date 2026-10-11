| scene / integrator | max qacc rel err | tip err vs MuJoCo @1s / 5s / 20s (% reach) | achilles vs truth @5s | MuJoCo vs truth @5s | energy drift (% PE span) |
|---|---|---|---|---|---|
| two_joint_arm/euler | 7.3e-08 | 2.7e-06 / 1.8e-05 / 7.7e-01 | 7.3e+00 | 7.3e+00 | 1.26e+01 |
| two_joint_arm/rk4 | 6.9e-07 | 1.1e-05 / 7.0e-05 / 8.0e+01 | 7.0e-05 | 6.0e-06 | 8.08e-06 |
| three_joint_arm/euler | 7.9e-08 | 3.0e-06 / 1.7e-05 / 1.3e+02 | 2.9e+01 | 2.9e+01 | 1.19e+01 |
| three_joint_arm/rk4 | 2.5e-07 | 1.2e-05 / 7.2e-05 / 1.2e+02 | 1.3e-04 | 8.6e-05 | 1.06e-05 |
| asymmetric_two_joint_arm/euler | 2.4e-08 | 2.8e-06 / 1.8e-05 / 7.2e+00 | 5.1e+00 | 5.1e+00 | 6.39e+00 |
| asymmetric_two_joint_arm/rk4 | 2.2e-08 | 1.1e-05 / 6.9e-05 / 7.5e+01 | 6.9e-05 | 6.8e-07 | 1.29e-06 |
| two_joint_arm_small_swing/euler | 2.9e-08 | 9.6e-07 / 7.5e-06 / 2.9e-05 | 1.6e-01 | 1.6e-01 | 8.03e-02 |
| two_joint_arm_small_swing/rk4 | 2.9e-08 | 3.9e-06 / 3.0e-05 / 1.2e-04 | 3.0e-05 | 2.1e-08 | 1.92e-07 |
