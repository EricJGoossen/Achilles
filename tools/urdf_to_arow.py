#!/usr/bin/env python3
"""Converts a URDF into an Achilles scene (.arow).

    tools/urdf_to_arow.py robot.urdf OUT_DIR [--archetype NAME] [--gaps FILE]
                          [--collision-group G --collision-mask M]

Writes OUT_DIR/<archetype>.arow and OUT_DIR/<archetype>.frames.json. No
dependencies beyond the standard library.

What becomes what:

- **Links on fixed joints are merged** into the moving body they're welded
  to, their inertias summed (parallel-axis theorem), as MuJoCo's
  `fusestatic` does. A merged link's name still resolves: the frames file
  maps every URDF link to the .arow joint that carries it and the link's
  pose in that joint's frame, so attachments can be placed on any link
  (hw-emulator's tools/elements_to_arow.py reads it).
- **revolute / continuous / prismatic** become one-DOF joints. Achilles turns
  a joint about (or slides it along) its own local x, so each such joint's
  frame is the URDF child frame turned to put x on the URDF axis; the frames
  file records that turn. `limit lower/upper` on a revolute or prismatic
  joint becomes `joint_limited` and its range.
- **floating** becomes a six-DOF free joint; **planar** isn't supported.
  The URDF's root link becomes a welded root.
- **Shapes.** Achilles gives each body one collision shape (a sphere-swept
  box) and one visual shape (box, sphere or cylinder). A body's first
  collision and first visual go on its own joint; every other shape goes on
  a welded, nearly massless carrier joint (`<link>__shape<i>`). Collision:
  box -> box, sphere -> sphere, cylinder -> a capsule (long) or a rounded
  slab (squat), mesh -> not representable. Visual: box, sphere, cylinder
  map exactly; mesh -> not drawn.
- **Collision filtering.** The root's welded children (a floor, walls) are
  group 1 and collide with group 2; everything under a floating joint is
  group 2 and collides with group 1, so a robot never collides with itself
  (Achilles has no per-pair exclusions; this covers MuJoCo's parent/child
  and `exclude` filtering for a single robot on a floor).

Every approximation and everything dropped (meshes, joint damping and
friction, mimic joints, effort and velocity limits, materials) is a sim gap:
printed as `[hwemu GAP] urdf_to_arow: ...` and, with --gaps, appended to that
JSONL file in hw-emulator's gap format, with process "model".
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import xml.etree.ElementTree as ET

# ---- small rigid-body math (w-first quaternions, as .arow writes them) ------


def qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def qconj(q):
    return (q[0], -q[1], -q[2], -q[3])


def qnorm(q):
    n = math.sqrt(sum(c * c for c in q))
    q = tuple(c / n for c in q)
    return q if q[0] >= 0 else tuple(-c for c in q)


def qrot(q, v):
    w = qmul(qmul(q, (0.0, *v)), qconj(q))
    return (w[1], w[2], w[3])


def rpy_quat(r, p, y):
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return qnorm((cr * cp * cy + sr * sp * sy,
                  sr * cp * cy - cr * sp * sy,
                  cr * sp * cy + sr * cp * sy,
                  cr * cp * sy - sr * sp * cy))


def quat_from_to(a, b):
    """The shortest rotation taking unit vector a onto unit vector b."""
    d = sum(x * y for x, y in zip(a, b))
    if d > 1 - 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    if d < -1 + 1e-12:
        # Half a turn about any axis perpendicular to a.
        axis = (0.0, 1.0, 0.0) if abs(a[0]) > 0.9 else (1.0, 0.0, 0.0)
        c = cross(a, axis)
        n = math.sqrt(sum(x * x for x in c))
        return (0.0, c[0] / n, c[1] / n, c[2] / n)
    c = cross(a, b)
    return qnorm((1 + d, *c))


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def mat_from_quat(q):
    w, x, y, z = q
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]]


class Pose:
    """A rigid transform: p_parent = R * p_child + t."""

    def __init__(self, t=(0.0, 0.0, 0.0), q=(1.0, 0.0, 0.0, 0.0)):
        self.t = tuple(float(c) for c in t)
        self.q = qnorm(q)

    def __mul__(self, o: "Pose") -> "Pose":
        r = qrot(self.q, o.t)
        return Pose((self.t[0] + r[0], self.t[1] + r[1], self.t[2] + r[2]), qmul(self.q, o.q))

    def inv(self) -> "Pose":
        qi = qconj(self.q)
        r = qrot(qi, self.t)
        return Pose((-r[0], -r[1], -r[2]), qi)

    def arow(self) -> str:
        return f"{{translation: {fmt(self.t)}, rotation: {fmt(self.q)}}}"

    def json(self):
        return {"translation": list(self.t), "rotation": list(self.q)}


def fmt(values) -> str:
    def one(v):
        v = float(v)
        if abs(v) < 1e-12:
            return "0"
        s = f"{v:.9g}"
        return s
    return "[" + ", ".join(one(v) for v in values) + "]"


def parse_origin(el) -> Pose:
    o = el.find("origin") if el is not None else None
    if o is None:
        return Pose()
    xyz = [float(v) for v in o.get("xyz", "0 0 0").split()]
    rpy = [float(v) for v in o.get("rpy", "0 0 0").split()]
    return Pose(xyz, rpy_quat(*rpy))


# ---- inertia ----------------------------------------------------------------


class Inertia:
    """Mass, center of mass and rotational inertia about the COM, in a frame."""

    def __init__(self, m=0.0, c=(0.0, 0.0, 0.0), I=None):
        self.m = m
        self.c = tuple(c)
        self.I = I or [[0.0] * 3 for _ in range(3)]

    def moved(self, pose: Pose) -> "Inertia":
        """The same body, expressed in the frame `pose` maps it into."""
        R = mat_from_quat(pose.q)
        c = qrot(pose.q, self.c)
        c = (c[0] + pose.t[0], c[1] + pose.t[1], c[2] + pose.t[2])
        RI = [[sum(R[i][k] * self.I[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
        I = [[sum(RI[i][k] * R[j][k] for k in range(3)) for j in range(3)] for i in range(3)]
        return Inertia(self.m, c, I)

    def __add__(self, o: "Inertia") -> "Inertia":
        m = self.m + o.m
        if m <= 0:
            return Inertia()
        c = tuple((self.m * a + o.m * b) / m for a, b in zip(self.c, o.c))
        I = [[0.0] * 3 for _ in range(3)]
        for part in (self, o):
            d = [part.c[i] - c[i] for i in range(3)]
            dd = sum(x * x for x in d)
            for i in range(3):
                for j in range(3):
                    I[i][j] += part.I[i][j] + part.m * ((dd if i == j else 0.0) - d[i] * d[j])
        return Inertia(m, c, I)

    def arow(self) -> list[float]:
        """.arow's rigid_body_inertia: mass, h = m*c, then the inertia about
        the frame's origin as Ixx, Iyy, Izz, Ixy, Ixz, Iyz."""
        m, c = self.m, self.c
        cc = sum(x * x for x in c)
        J = [[self.I[i][j] + m * ((cc if i == j else 0.0) - c[i] * c[j]) for j in range(3)] for i in range(3)]
        return [m, m * c[0], m * c[1], m * c[2], J[0][0], J[1][1], J[2][2], J[0][1], J[0][2], J[1][2]]


def parse_inertial(link) -> Inertia:
    el = link.find("inertial")
    if el is None:
        return Inertia()
    pose = parse_origin(el)
    m = float(el.find("mass").get("value", "0")) if el.find("mass") is not None else 0.0
    i = el.find("inertia")
    g = (lambda k: float(i.get(k, "0"))) if i is not None else (lambda k: 0.0)
    I = [[g("ixx"), g("ixy"), g("ixz")], [g("ixy"), g("iyy"), g("iyz")], [g("ixz"), g("iyz"), g("izz")]]
    return Inertia(m, (0.0, 0.0, 0.0), I).moved(pose)


# ---- gaps -------------------------------------------------------------------


class Gaps:
    def __init__(self, path: str | None):
        self.path = path
        self.seen: set[str] = set()

    def __call__(self, key: str, message: str) -> None:
        if key in self.seen:
            return
        self.seen.add(key)
        print(f"[hwemu GAP] urdf_to_arow: {message}", file=sys.stderr)
        if self.path:
            os.makedirs(os.path.dirname(os.path.abspath(self.path)), exist_ok=True)
            with open(self.path, "a") as f:
                f.write(json.dumps({"process": "model", "source": "urdf_to_arow", "key": key,
                                    "message": message, "time": 0}) + "\n")


# ---- shapes -----------------------------------------------------------------

Y_TO_Z = (math.cos(math.pi / 4), math.sin(math.pi / 4), 0.0, 0.0)  # +90 deg about x: y -> z


def collision_shape(link_name, geom_el, gaps):
    """(pose in link frame, half extents, radius) or None."""
    pose = parse_origin(geom_el)
    g = geom_el.find("geometry")
    shape = g[0] if g is not None and len(g) else None
    if shape is None:
        return None
    if shape.tag == "box":
        s = [float(v) / 2 for v in shape.get("size").split()]
        return pose, s, 0.0
    if shape.tag == "sphere":
        return pose, [0.0, 0.0, 0.0], float(shape.get("radius"))
    if shape.tag == "cylinder":
        r, L = float(shape.get("radius")), float(shape.get("length"))
        s = min(r, L / 2)
        if s >= r:  # long: a capsule along z, as long overall as the cylinder
            gaps(f"cylinder-capsule:{link_name}",
                 f"{link_name}: collision cylinder (r {r:g}, length {L:g}) approximated as a capsule "
                 f"(rounded ends)")
            return pose, [0.0, 0.0, L / 2 - r], r
        gaps(f"cylinder-slab:{link_name}",
             f"{link_name}: collision cylinder (r {r:g}, length {L:g}) approximated as a rounded square slab "
             f"(corners reach {math.sqrt(2) * (r - s) + s:.3f} m, not {r:g})")
        return pose, [r - s, r - s, 0.0], s
    if shape.tag == "mesh":
        gaps(f"mesh-collision:{link_name}",
             f"{link_name}: collision mesh {shape.get('filename')} isn't representable in Achilles; dropped "
             f"(give the link a primitive collision)")
        return None
    gaps(f"geom:{link_name}:{shape.tag}", f"{link_name}: unknown collision geometry <{shape.tag}>; dropped")
    return None


def visual_shape(link_name, geom_el, gaps):
    """(pose, half extents, shape code, rgb) or None."""
    pose = parse_origin(geom_el)
    g = geom_el.find("geometry")
    shape = g[0] if g is not None and len(g) else None
    rgb = [0.7, 0.7, 0.7]
    mat = geom_el.find("material")
    if mat is not None:
        col = mat.find("color")
        if col is not None:
            rgba = [float(v) for v in col.get("rgba").split()]
            rgb = rgba[:3]
            if len(rgba) > 3 and rgba[3] < 1:
                gaps(f"alpha:{link_name}", f"{link_name}: visual transparency (alpha {rgba[3]:g}) not drawn")
        elif mat.get("name"):
            gaps(f"material:{link_name}", f"{link_name}: named material '{mat.get('name')}' without a color; "
                                          f"drawn grey")
    if shape is None:
        return None
    if shape.tag == "box":
        return pose, [float(v) / 2 for v in shape.get("size").split()], 0, rgb
    if shape.tag == "sphere":
        r = float(shape.get("radius"))
        return pose, [r, r, r], 1, rgb
    if shape.tag == "cylinder":
        r, L = float(shape.get("radius")), float(shape.get("length"))
        # Achilles's unit cylinder is about y; the URDF's is about z.
        return pose * Pose(q=Y_TO_Z), [r, L / 2, r], 2, rgb
    if shape.tag == "mesh":
        gaps(f"mesh-visual:{link_name}", f"{link_name}: visual mesh {shape.get('filename')} not drawn "
                                         f"(Achilles draws boxes, spheres and cylinders)")
        return None
    gaps(f"geom-visual:{link_name}:{shape.tag}", f"{link_name}: unknown visual geometry <{shape.tag}>; not drawn")
    return None


# ---- the conversion ---------------------------------------------------------

SUBSPACE_FIXED = [0] * 36
SUBSPACE_FREE = [1 if i % 7 == 0 else 0 for i in range(36)]
SUBSPACE_ROT_X = [1] + [0] * 35
SUBSPACE_LIN_X = [0, 0, 0, 1] + [0] * 32
CARRIER_INERTIA = Inertia(1e-6, (0, 0, 0), [[1e-12, 0, 0], [0, 1e-12, 0], [0, 0, 1e-12]])


class Body:
    def __init__(self, name, parent, kind, fixed_transform: Pose, position: Pose):
        self.name = name
        self.parent = parent
        self.kind = kind  # fixed | free | revolute | prismatic
        self.fixed_transform = fixed_transform
        self.position = position
        self.inertia = Inertia()
        self.collision = None  # (pose, extents, radius)
        self.visual = None     # (pose, extents, shape, rgb)
        self.limit = None      # (lower, upper)
        self.group = 2
        self.comment = ""


def convert(urdf_path, archetype, gaps, group_mask=None):
    root_el = ET.parse(urdf_path).getroot()
    links = {l.get("name"): l for l in root_el.findall("link")}
    joints = root_el.findall("joint")
    child_joint = {j.find("child").get("link"): j for j in joints}
    children: dict[str, list] = {}
    for j in joints:
        children.setdefault(j.find("parent").get("link"), []).append(j)
    roots = [n for n in links if n not in child_joint]
    if len(roots) != 1:
        raise SystemExit(f"urdf_to_arow: the URDF needs exactly one root link, not {roots}")
    root = roots[0]

    if root_el.findall("transmission"):
        gaps("transmission", "URDF <transmission> elements ignored (hw-emulator's devices.yaml binds the motors)")
    for el in root_el.findall("gazebo"):
        gaps("gazebo", "URDF <gazebo> extensions ignored")
        break

    bodies: list[Body] = []
    body_of: dict[str, Body] = {}
    frames: dict[str, dict] = {}  # URDF link -> {joint, pose (link in joint frame)}
    joint_map: dict[str, str] = {}  # URDF joint -> .arow joint

    def add_link(link_name: str, body: Body, link_in_body: Pose, floating: bool) -> None:
        link = links[link_name]
        frames[link_name] = {"joint": body.name, "pose": link_in_body.json()}
        body.inertia = body.inertia + parse_inertial(link).moved(link_in_body)
        group = 2 if floating else 1
        for el in link.findall("collision"):
            s = collision_shape(link_name, el, gaps)
            if s is None:
                continue
            pose, ext, rad = s
            target = free_slot(body, link_name, "collision")
            target.collision = (link_in_body * pose, ext, rad)
            target.group = group
        for el in link.findall("visual"):
            s = visual_shape(link_name, el, gaps)
            if s is None:
                continue
            pose, ext, shape, rgb = s
            target = free_slot(body, link_name, "visual")
            target.visual = (link_in_body * pose, ext, shape, rgb)
            target.group = group
        for j in children.get(link_name, []):
            visit_joint(j, body, link_in_body, floating)

    carriers: dict[str, list[Body]] = {}  # per body

    def carrier(body: Body, link_name: str) -> Body:
        mine = carriers.setdefault(body.name, [])
        c = Body(f"{link_name}__shape{sum(1 for x in mine if x.name.startswith(link_name + '__')) + 1}",
                 body.name, "fixed", Pose(), Pose())
        c.inertia = CARRIER_INERTIA
        c.comment = f"carries more shapes of {link_name} (Achilles has one collision and one visual per body)"
        mine.append(c)
        bodies.append(c)
        return c

    def free_slot(body: Body, link_name: str, slot: str) -> Body:
        """The body itself, or one of its carriers, with `slot` still empty."""
        for b in [body] + carriers.get(body.name, []):
            if getattr(b, slot) is None:
                return b
        return carrier(body, link_name)

    def visit_joint(j, parent_body: Body, parent_link_in_body: Pose, floating: bool) -> None:
        name, jtype = j.get("name"), j.get("type")
        child = j.find("child").get("link")
        origin = parent_link_in_body * parse_origin(j)  # child link frame in parent body frame (at q = 0)
        for tag in ("mimic", "calibration", "safety_controller"):
            if j.find(tag) is not None:
                gaps(f"{tag}:{name}", f"{name}: URDF <{tag}> ignored")
        dyn = j.find("dynamics")
        if dyn is not None:
            for k in ("damping", "friction"):
                if float(dyn.get(k, "0")) != 0:
                    gaps(f"dynamics-{k}", f"joint {k} (URDF <dynamics {k}>) isn't modelled by Achilles; ignored "
                                          f"(first on {name})")
        if jtype == "fixed":
            joint_map[name] = parent_body.name
            add_link(child, parent_body, origin, floating)
            return
        axis = (1.0, 0.0, 0.0)
        ax_el = j.find("axis")
        if ax_el is not None:
            axis = tuple(float(v) for v in ax_el.get("xyz").split())
            n = math.sqrt(sum(a * a for a in axis))
            axis = tuple(a / n for a in axis)
        if jtype in ("revolute", "continuous", "prismatic"):
            turn = Pose(q=quat_from_to((1.0, 0.0, 0.0), axis))  # joint frame in child link frame
            body = Body(name, parent_body.name, "prismatic" if jtype == "prismatic" else "revolute",
                        origin * turn, Pose())
            lim = j.find("limit")
            if jtype != "continuous" and lim is not None and ("lower" in lim.attrib or "upper" in lim.attrib):
                body.limit = (float(lim.get("lower", "0")), float(lim.get("upper", "0")))
            if lim is not None and (lim.get("effort") or lim.get("velocity")):
                gaps("limit-effort-velocity", "URDF joint effort and velocity limits ignored (the motor models "
                                              "in devices.yaml limit current and speed)")
            body.comment = f"URDF {jtype} {name}: {parent_body.name} -> {child}, axis {fmt(axis)}"
            bodies.append(body)
            body_of[child] = body
            joint_map[name] = body.name
            add_link(child, body, turn.inv(), floating)
            return
        if jtype == "floating":
            body = Body(name, parent_body.name, "free", Pose(), origin)
            body.comment = f"URDF floating {name}: {parent_body.name} -> {child}"
            bodies.append(body)
            body_of[child] = body
            joint_map[name] = body.name
            add_link(child, body, Pose(), True)
            return
        raise SystemExit(f"urdf_to_arow: joint {name}: type {jtype} isn't supported")

    rb = Body(root, None, "fixed", Pose(), Pose())
    rb.comment = f"the URDF root, {root}, welded to the world"
    bodies.append(rb)
    body_of[root] = rb
    # The root's own shapes (a floor) go on carriers, so they're the root's
    # children and their pairs with the robot aren't skipped as parent/child.
    root_link = links[root]
    frames[root] = {"joint": root, "pose": Pose().json()}
    rb.inertia = parse_inertial(root_link)
    if rb.inertia.m <= 0:
        rb.inertia = Inertia(1.0, (0, 0, 0), [[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]])
    rb.collision = rb.visual = False  # keep the root's own slots empty
    for el in root_link.findall("collision"):
        s = collision_shape(root, el, gaps)
        if s:
            c = free_slot(rb, root, "collision")
            c.collision, c.group = s, 1
    for el in root_link.findall("visual"):
        s = visual_shape(root, el, gaps)
        if s:
            c = free_slot(rb, root, "visual")
            c.visual, c.group = s, 1
    rb.collision = rb.visual = None
    for j in children.get(root, []):
        visit_joint(j, rb, Pose(), False)

    # Order bodies parent-first (carriers were appended as found).
    order, placed = [], set()
    pending = list(bodies)
    while pending:
        progressed = False
        for b in list(pending):
            if b.parent is None or b.parent in placed:
                order.append(b)
                placed.add(b.name)
                pending.remove(b)
                progressed = True
        if not progressed:
            raise SystemExit("urdf_to_arow: internal error ordering bodies")
    return order, frames, joint_map


def write_arow(bodies, archetype, out_path, src):
    any_limit = any(b.limit for b in bodies)
    lines = [
        f"# Generated by Achilles tools/urdf_to_arow.py from {src}. Don't edit:",
        "# regenerate it. Joints turn about (slide along) their own local x; each",
        "# joint's fixed_joint_transform turns that onto the URDF axis.",
        f"archetype: {archetype}",
        "joints:",
    ]
    for b in bodies:
        if b.kind == "fixed":
            sub, mask = SUBSPACE_FIXED, 0
        elif b.kind == "free":
            sub, mask = SUBSPACE_FREE, 63
        elif b.kind == "revolute":
            sub, mask = SUBSPACE_ROT_X, 1
        else:
            sub, mask = SUBSPACE_LIN_X, 1
        inertia = b.inertia
        if inertia.m <= 0:
            inertia = CARRIER_INERTIA
        col_pose, col_ext, col_rad = b.collision or (Pose(), [0, 0, 0], 0.0)
        vis_pose, vis_ext, vis_shape, vis_rgb = b.visual or (Pose(), [0, 0, 0], 0, [0, 0, 0])
        has_col = b.collision is not None
        lines += [
            f"  # {b.comment}" if b.comment else None,
            f"  - name: {b.name}",
            f"    parent: {b.parent if b.parent else 'null'}",
            "    fields:",
            f"      joint_subspace: {fmt(sub)}",
            f"      joint_activation_mask: [{mask}]",
            f"      fixed_joint_transform: {b.fixed_transform.arow()}",
            f"      rigid_body_inertia: {fmt(inertia.arow())}",
            f"      joint_position: {b.position.arow()}",
            "      joint_velocity: [0, 0, 0, 0, 0, 0]",
            f"      visual_pose: {vis_pose.arow()}",
            f"      visual_extents: {fmt(vis_ext)}",
            f"      visual_shape: [{vis_shape}]",
            f"      visual_color: {fmt(vis_rgb)}",
            f"      collision_pose: {col_pose.arow()}",
            f"      collision_extents: {fmt(col_ext)}",
            f"      collision_radius: [{col_rad:.9g}]",
            f"      collision_margin: [{0.001 if has_col else 0}]",
            f"      collision_group: [{b.group if has_col else 0}]",
            f"      collision_mask: [{(2 if b.group == 1 else 1) if has_col else 0}]",
        ]
        if any_limit:
            lo, hi = b.limit or (0.0, 0.0)
            lines += [
                f"      joint_limited: [{1 if b.limit else 0}]",
                f"      joint_range_lower: {fmt([lo, 0, 0, 0, 0, 0])}",
                f"      joint_range_upper: {fmt([hi, 0, 0, 0, 0, 0])}",
            ]
    with open(out_path, "w") as f:
        f.write("\n".join(l for l in lines if l is not None) + "\n")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("urdf")
    ap.add_argument("out_dir")
    ap.add_argument("--archetype", default=None, help="the archetype's name (default: the URDF robot's name)")
    ap.add_argument("--gaps", default=None, help="append gaps to this JSONL file")
    args = ap.parse_args()
    gaps = Gaps(args.gaps)
    archetype = args.archetype or ET.parse(args.urdf).getroot().get("name", "robot")
    bodies, frames, joint_map = convert(args.urdf, archetype, gaps)
    os.makedirs(args.out_dir, exist_ok=True)
    arow = os.path.join(args.out_dir, f"{archetype}.arow")
    write_arow(bodies, archetype, arow, os.path.relpath(args.urdf, args.out_dir))
    with open(os.path.join(args.out_dir, f"{archetype}.frames.json"), "w") as f:
        json.dump({"archetype": archetype, "links": frames, "joints": joint_map}, f, indent=1)
    moving = sum(1 for b in bodies if b.kind != "fixed")
    print(f"{arow}: {len(bodies)} joints ({moving} moving), {len(frames)} URDF links")
    return 0


if __name__ == "__main__":
    sys.exit(main())
