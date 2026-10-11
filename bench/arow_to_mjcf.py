"""Translates a single-archetype achilles .arow scene into an equivalent MJCF.

Supports what the example scenes use: a tree of 1-DOF revolute joints, each
with a fixed parent->joint placement, a pivot-referenced rigid-body inertia,
and an initial rotation/velocity. Conventions (cross-checked against
tests/examples_two_joint_arm.cpp's closed-form model, and numerically by
compare_mujoco.py's qacc check):

  joint_subspace          6x6, row-major; rows = (rx, ry, rz, x, y, z) of
                          domain::spatial::Dual, columns = generalized slots.
  joint_velocity          indexed by generalized slot (column of S).
  rigid_body_inertia      (mass, h = mass * com, Ixx, Iyy, Izz, Ixy, Ixz, Iyz)
                          with I about the *joint origin*, in the joint frame.
  base_acceleration       -gravity (see algorithms/sim_config.hpp).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
import yaml


@dataclass
class Joint:
    name: str
    parent: str | None
    axis: np.ndarray
    slot: int
    pos: np.ndarray
    quat: np.ndarray
    mass: float
    com: np.ndarray
    inertia_com: np.ndarray  # (ixx, iyy, izz, ixy, ixz, iyz) about the CoM
    q0: float
    qd0: float
    children: list[str] = field(default_factory=list)


@dataclass
class Scene:
    name: str
    joints: list[Joint]  # depth-first order, root first
    gravity: np.ndarray

    def by_name(self, name: str) -> Joint:
        return next(j for j in self.joints if j.name == name)


def _flat(x):
    if isinstance(x, dict):
        out = []
        for v in x.values():
            out += _flat(v)
        return out
    if isinstance(x, (list, tuple)):
        out = []
        for v in x:
            out += _flat(v)
        return out
    return [float(x)]


def load_scene(arow_path: str, config_path: str) -> Scene:
    with open(arow_path) as f:
        doc = yaml.safe_load(f)
    with open(config_path) as f:
        cfg = yaml.safe_load(f)

    base = cfg["base_acceleration"]
    if any(abs(v) > 0 for v in _flat(base["angular"])):
        raise ValueError("nonzero angular base acceleration is not supported")
    gravity = -np.array(_flat(base["linear"]))

    joints: dict[str, Joint] = {}
    order: list[str] = []
    for spec in doc["joints"]:
        fl = spec["fields"]
        s = np.array(_flat(fl["joint_subspace"])).reshape(6, 6)
        active = [c for c in range(6) if np.any(s[:, c] != 0)]
        if len(active) != 1:
            raise ValueError(f"{spec['name']}: only 1-DOF joints supported")
        c = active[0]
        if np.any(s[3:, c] != 0):
            raise ValueError(f"{spec['name']}: only revolute joints supported")
        axis = s[:3, c] / np.linalg.norm(s[:3, c])

        ft = fl["fixed_joint_transform"]
        jp = fl["joint_position"]
        if any(abs(v) > 0 for v in _flat(jp["translation"])):
            raise ValueError(f"{spec['name']}: joint_position translation")
        w, x, y, z = _flat(jp["rotation"])
        q0 = 2.0 * math.atan2(float(np.dot([x, y, z], axis)), w)

        m, hx, hy, hz, ixx, iyy, izz, ixy, ixz, iyz = _flat(
            fl["rigid_body_inertia"]
        )
        com = np.array([hx, hy, hz]) / m
        i_o = np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]])
        i_c = i_o - m * (np.dot(com, com) * np.eye(3) - np.outer(com, com))

        joints[spec["name"]] = Joint(
            name=spec["name"],
            parent=spec.get("parent"),
            axis=axis,
            slot=c,
            pos=np.array(_flat(ft["translation"])),
            quat=np.array(_flat(ft["rotation"])),
            mass=m,
            com=com,
            inertia_com=np.array(
                [i_c[0, 0], i_c[1, 1], i_c[2, 2], i_c[0, 1], i_c[0, 2], i_c[1, 2]]
            ),
            q0=q0,
            qd0=_flat(fl["joint_velocity"])[c],
        )
        order.append(spec["name"])

    roots = []
    for name in order:
        p = joints[name].parent
        if p is None:
            roots.append(name)
        else:
            joints[p].children.append(name)

    dfs: list[Joint] = []

    def visit(n):
        dfs.append(joints[n])
        for ch in joints[n].children:
            visit(ch)

    for r in roots:
        visit(r)
    return Scene(name=doc["archetype"], joints=dfs, gravity=gravity)


def _v(a) -> str:
    return " ".join(repr(float(x)) for x in a)


def _body_xml(scene: Scene, j: Joint, prefix: str, indent: str) -> str:
    out = [
        f'{indent}<body name="{prefix}{j.name}" pos="{_v(j.pos)}" quat="{_v(j.quat)}">',
        f'{indent}  <joint name="{prefix}{j.name}" type="hinge" axis="{_v(j.axis)}"/>',
        f'{indent}  <inertial pos="{_v(j.com)}" mass="{j.mass!r}" '
        f'fullinertia="{_v(j.inertia_com)}"/>',
    ]
    for ch in j.children:
        out.append(_body_xml(scene, scene.by_name(ch), prefix, indent + "  "))
    out.append(f"{indent}</body>")
    return "\n".join(out)


def to_mjcf(
    scene: Scene, dt: float, integrator: str = "Euler", copies: int = 1
) -> str:
    """MJCF for `copies` independent instances of `scene` side by side."""
    roots = [j for j in scene.joints if j.parent is None]
    bodies = []
    for i in range(copies):
        prefix = f"r{i}_" if copies > 1 else ""
        for r in roots:
            bodies.append(_body_xml(scene, r, prefix, "    "))
    # dt is passed through float32 first: achilles' Simulation::Step takes a
    # float, so this keeps both engines on the exact same step size.
    return f"""<mujoco model="{scene.name}">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="{float(np.float32(dt))!r}" gravity="{_v(scene.gravity)}"
          integrator="{integrator}">
    <flag contact="disable"/>
  </option>
  <worldbody>
{chr(10).join(bodies)}
  </worldbody>
</mujoco>
"""


def initial_state(scene: Scene, copies: int = 1) -> tuple[np.ndarray, np.ndarray]:
    q = np.array([j.q0 for j in scene.joints] * copies)
    qd = np.array([j.qd0 for j in scene.joints] * copies)
    return q, qd


if __name__ == "__main__":
    import sys

    sc = load_scene(sys.argv[1], sys.argv[2])
    print(to_mjcf(sc, float(sys.argv[3]) if len(sys.argv) > 3 else 0.002))
