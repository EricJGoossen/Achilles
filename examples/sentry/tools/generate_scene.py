#!/usr/bin/env python3
"""Writes the sentry demo's scene files: field.arow, sentry.arow, sensors/*.arow and
sentry_geometry.hpp.

The field is the ARC 2026 3v3 Battlefield as the rules manual draws it ("2026 ARCC Rules:
Events", section 3.2: Figure 3-3's plan and side view, Figure 3-6's High Ground), mirrored
about x = 6 as the manual says. The sentry is robot/sentry_coarse.urdf (the team's CAD export):
its joint frames, axes, masses and inertias, drawn with the primitive shapes in
robot/primitive_collisions.yaml. sentry_geometry.hpp hands sentry_sim.cpp's wheel model the
same numbers.

Run from anywhere: python3 examples/sentry/tools/generate_scene.py
"""

import math
import os
import xml.etree.ElementTree as ET

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.dirname(HERE)
URDF = os.path.join(OUT, "robot", "sentry_coarse.urdf")
PRIMITIVES = os.path.join(OUT, "robot", "primitive_collisions.yaml")

# --- Field (rules manual, mm there, m here; x along the 12 m length, y along the 8 m width,
# origin at the red side's corner as Otto and the MCB have it) ---
FIELD_LENGTH = 12.0
FIELD_WIDTH = 8.0
# Where the field frame sits in the simulation's: the field's centre on the origin, which the
# viewer orbits. Everything is attached under the field's world joint, and sentry_sim.cpp reads
# poses relative to it, so the field frame itself is unchanged.
FIELD_ORIGIN = (-FIELD_LENGTH / 2, -FIELD_WIDTH / 2, 0.0)
BOUNDARY_HEIGHT = 0.420  # Figure 3-3 side view
BOUNDARY_THICKNESS = 0.1
WALL_BASE_HEIGHT = 0.420  # the walls' bases, as tall as the boundary
WALL_THICKNESS = 0.170
PANEL_THICKNESS = 0.04  # not dimensioned; thin panels on the wall bases
NEAR_WALL_X = 2.250
NEAR_WALL_TOP = 0.420 + 0.987  # panel above the base (side view: 987)
FAR_WALL_X = NEAR_WALL_X + WALL_THICKNESS + 1.000
FAR_WALL_TOP = 0.420 + 0.250  # post above the base (side view: 250)
NEAR_WALL_Y = (FIELD_WIDTH - 2.0 - 0.8 - 3.0254, FIELD_WIDTH - 2.0 - 0.8)
FAR_WALL_Y = (FIELD_WIDTH - 2.0 - 2.0254, FIELD_WIDTH - 2.0)
# High Ground (Figure 3-6): a deck 210 mm up, two 1 m rows along the north wall, 784 mm
# 15-degree ramps at the ends and into the gap in the front row, and a 2 m step at the back
# centre 410 mm up (standing 19 mm proud of the deck's front).
DECK_HEIGHT = 0.210
STEP_HEIGHT = 0.410
RAMP_LENGTH = 0.784
HIGH_GROUND_X0 = 3.000 - RAMP_LENGTH  # Figure 3-3: (2216.3)
CENTRE_BLOCK_HEIGHT = 0.420  # the block by the bumpy roads: two decks' worth (side view)

# (name, x0, y0, x1, y1, z0, z1, shape, colour): the red half's pieces, mirrored about x = 6.
# shape: "box", or "ramp+" / "ramp-" for a slope rising toward +x / -x from z0 to z1.
GREY = (0.55, 0.58, 0.65)
DECK = (0.62, 0.62, 0.66)
MIRRORED = [
    ("near_wall", NEAR_WALL_X, NEAR_WALL_Y[0], NEAR_WALL_X + WALL_THICKNESS, NEAR_WALL_Y[1],
     0.0, WALL_BASE_HEIGHT, "box", GREY),
    ("near_wall_panel", NEAR_WALL_X + (WALL_THICKNESS - PANEL_THICKNESS) / 2, NEAR_WALL_Y[0],
     NEAR_WALL_X + (WALL_THICKNESS + PANEL_THICKNESS) / 2, NEAR_WALL_Y[1],
     WALL_BASE_HEIGHT, NEAR_WALL_TOP, "box", (0.35, 0.37, 0.42)),
    ("far_wall", FAR_WALL_X, FAR_WALL_Y[0], FAR_WALL_X + WALL_THICKNESS, FAR_WALL_Y[1],
     0.0, WALL_BASE_HEIGHT, "box", GREY),
    ("far_wall_post", FAR_WALL_X + (WALL_THICKNESS - PANEL_THICKNESS) / 2, FAR_WALL_Y[0],
     FAR_WALL_X + (WALL_THICKNESS + PANEL_THICKNESS) / 2, FAR_WALL_Y[1],
     WALL_BASE_HEIGHT, FAR_WALL_TOP, "box", (0.35, 0.37, 0.42)),
    ("deck_back", 3.0, 7.0, 5.0, 8.0, 0.0, DECK_HEIGHT, "box", DECK),
    ("deck_wing", 3.0, 6.0, 4.0, 7.0, 0.0, DECK_HEIGHT, "box", DECK),
    ("ramp_end", HIGH_GROUND_X0, 6.0, 3.0, 8.0, 0.0, DECK_HEIGHT, "ramp+", (0.75, 0.75, 0.78)),
    ("ramp_inner", 4.0, 6.0, 4.0 + RAMP_LENGTH, 7.0, 0.0, DECK_HEIGHT, "ramp-", (0.75, 0.75, 0.78)),
]
CENTRED = [
    ("step", 5.0, 7.0 - 0.019, 7.0, 8.0, 0.0, STEP_HEIGHT, "box", (0.5, 0.5, 0.55)),
    ("centre_block", 5.0, 0.70, 7.0, 1.70, 0.0, CENTRE_BLOCK_HEIGHT, "box", DECK),
]
# Floor markings and the bumpy roads: drawn, not collided with (the chassis is planar and the
# lidars sit far above them).
FLOOR = 0.004
BUMPS = 0.020
MIRRORED_ZONES = [
    ("resupply_zone", 0.0, 3.0, 1.5, 5.0, FLOOR, None),  # coloured per team below
    ("bumpy_road_a", 0.431, 1.776, 5.233, 2.174, BUMPS, (0.13, 0.13, 0.15)),
    ("bumpy_road_b", 0.431, 0.980, 4.829, 1.776, BUMPS, (0.13, 0.13, 0.15)),
    ("bumpy_road_c", 0.431, 0.184, 4.434, 0.980, BUMPS, (0.13, 0.13, 0.15)),
]
CENTRED_ZONES = [("capture_point", 5.0, 3.0, 7.0, 5.0, FLOOR, (0.35, 0.8, 0.45))]
TEAM_COLOURS = {"red": (0.8, 0.3, 0.3), "blue": (0.3, 0.45, 0.85)}

# --- Sentry: what the URDF doesn't say ---
URDF_NAMES = {  # URDF link -> the joint names sentry_sim.cpp and the sensors use
    "base_link": "chassis",
    "fl_wheel": "left_front",
    "fr_wheel": "right_front",
    "bl_wheel": "left_back",
    "br_wheel": "right_back",
    "turret_major": "turret_major",
    "turret_minor_frame": "widow_yaw",
    "turret_minor_firing_system": "widow_pitch",
}
LINK_COLOURS = {
    "base_link": (0.75, 0.25, 0.25),
    "fl_wheel": (0.25, 0.25, 0.28),
    "turret_major": (0.45, 0.47, 0.55),
    "turret_minor_frame": (0.35, 0.35, 0.38),
    "turret_minor_firing_system": (0.78, 0.78, 0.8),
}
SKIPPED_PRIMITIVES = {"firing_overall_bounding_box"}  # encloses the two beside it
SHAPE_PREFIXES = ("chassis_", "major_", "minor_yaw_", "firing_")  # dropped from visual joints' names
CHASSIS_COLLISION = "chassis_main_structure"  # the one collision shape (see generate_sentry)
COLLISION_RADIUS = 0.02
CHASSIS_GEARBOX_RATIO = 17.0 / 268.0  # output turns per rotor turn
M3508_ROTOR_INERTIA = 1.2e-5  # kg m^2, estimate
TURRET_MAJOR_RATIO = (187.0 / 3591.0) * (30.0 / 95.0)  # PULLEY_RATIO
GM6020_ROTOR_INERTIA = 2.4e-4  # kg m^2, estimate
INITIAL_X, INITIAL_Y = 0.75, 4.0  # INITIAL_CHASSIS_POSITION_X/Y, facing +x

# The lidars, where Otto's sentry config mounts them (lidar_position_to_chassis_frame and
# lidar_orientation_to_chassis_frame: the LD19's zero direction, as a yaw from forward), on top
# of turret major's turntable (the URDF doesn't have them; the height is an estimate). Otto's
# "chassis" frame is turret major's: the sentry's MCB reports turret major's pose as the chassis
# odometry it sends the Jetson (SentryTransformAdapter::getVisionWorldToChassis), so the lidars
# ride on turret major and don't spin when the chassis beyblades. Each LD19's head spins on its
# own joint (see sentry_sim.cpp).
LIDAR_HEIGHT = 0.204  # above turret major's frame
LIDARS = [("lidar_front", (0.135, 0.0), -1.5707), ("lidar_rear", (-0.135, 0.0), 1.5707)]

# Turret major's three IMUs (sentry_drivers.hpp turretMajorImuTransforms: mm and rad).
MAJOR_IMUS = [
    ("major_ism330_0", "ism330", (-76.7e-3, -116.14e-3, 0.0), 0.0),
    ("major_ism330_1", "ism330", (-76.7e-3, 116.04e-3, 0.0), math.pi),
    ("major_mpu6500", "mpu6500", (-14.97e-3, -115.5e-3, 0.0), math.pi / 2),
]

WELDED = "[" + ", ".join(["0"] * 36) + "]"
REST = "{translation: [0, 0, 0], rotation: [1, 0, 0, 0]}"
VISUAL_ONLY_INERTIA = [1e-3, 0, 0, 0, 1e-6, 1e-6, 1e-6, 0, 0, 0]


# --- Small rotation helpers (quaternions [w, x, y, z], matrices as row lists) ---

def quat_axis(axis, angle):
    s = math.sin(angle / 2)
    return [math.cos(angle / 2), axis[0] * s, axis[1] * s, axis[2] * s]


def quat_z(angle):
    return quat_axis((0, 0, 1), angle)


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return [
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ]


def quat_to_matrix(q):
    w, x, y, z = q
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ]


def matrix_to_quat(m):
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        q = [0.25 * s, (m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s]
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
        q = [(m[2][1] - m[1][2]) / s, 0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s]
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
        q = [(m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s]
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
        q = [(m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s]
    return q if q[0] >= 0 else [-v for v in q]


def rpy_matrix(rpy):
    """URDF's fixed-axis roll, pitch, yaw: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    roll, pitch, yaw = rpy
    q = quat_mul(quat_axis((0, 0, 1), yaw), quat_mul(quat_axis((0, 1, 0), pitch), quat_axis((1, 0, 0), roll)))
    return quat_to_matrix(q)


def mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def mat_vec(m, v):
    return [sum(m[i][k] * v[k] for k in range(3)) for i in range(3)]


def transpose(m):
    return [[m[j][i] for j in range(3)] for i in range(3)]


# --- .arow writing ---

def subspace(columns):
    """A joint subspace from up to six DOF columns, each [roll, pitch, yaw, x, y, z].

    The .arow field is the 6x6 matrix S row-major: one row per spatial component, one column
    per degree of freedom.
    """
    matrix = [[0] * 6 for _ in range(6)]
    for dof, column in enumerate(columns):
        for row, value in enumerate(column):
            matrix[row][dof] = value
    return "[" + ", ".join(str(v) for row in matrix for v in row) + "]"


def fmt(values):
    return "[" + ", ".join(f"{v:.6g}" for v in values) + "]"


def pose(translation, rotation):
    return f"{{translation: {fmt(translation)}, rotation: {fmt(rotation)}}}"


def joint(name, parent, sub, mask, translation, rotation, inertia, position=REST, extra=None):
    lines = [
        f"  - name: {name}",
        f"    parent: {parent}",
        "    fields:",
        f"      joint_subspace: {sub}",
        f"      joint_activation_mask: [{mask}]",
        f"      fixed_joint_transform: {pose(translation, rotation)}",
        f"      rigid_body_inertia: {fmt(inertia)}",
        f"      joint_position: {position}",
        "      joint_velocity: [0, 0, 0, 0, 0, 0]",
    ]
    for key, value in (extra or {}).items():
        lines.append(f"      {key}: {value}")
    return "\n".join(lines)


# The viewer draws every shape outward along its visual frame's +y (viz_data.hpp: a box from
# its frame's origin, a cylinder's axis along it). A shape authored z-up -- a URDF cylinder's
# axis, a box's height -- gets this quarter turn about x to stand it up.
Z_UP = quat_axis((1, 0, 0), math.pi / 2)


def visual(shape, centre, rotation, half, colour):
    """Fields drawing a box or cylinder centred at `centre`, turned by `rotation`, with
    half-extents `half` along its own x, y, z (a cylinder: radius, radius, half-length)."""
    up = mat_vec(quat_to_matrix(rotation), (0, 0, half[2]))
    base = [c - u for c, u in zip(centre, up)]
    return {
        "visual_shape": "[2]" if shape == "cylinder" else "[0]",
        "visual_pose": pose(base, quat_mul(rotation, Z_UP)),
        "visual_extents": fmt((half[0], half[2], half[1])),
        "visual_color": fmt(colour),
    }


def no_visual():
    """Visual fields drawing nothing. The archetype loader takes an archetype's field list from
    its first joint and ignores fields that joint lacks, so a first joint with nothing to draw
    still declares them."""
    return {
        "visual_shape": "[0]",
        "visual_pose": REST,
        "visual_extents": "[0, 0, 0]",
        "visual_color": "[0, 0, 0]",
    }


def no_collision():
    return {
        "collision_pose": REST,
        "collision_extents": "[0, 0, 0]",
        "collision_radius": "[0]",
        "collision_margin": "[0]",
        "collision_group": "[0]",
        "collision_mask": "[0]",
    }


def write(path, text):
    with open(os.path.join(OUT, path), "w") as f:
        f.write(text)


# --- Field ---

def mirror(piece):
    name, x0, y0, x1, y1, z0, z1, shape, colour = piece
    flipped = {"box": "box", "ramp+": "ramp-", "ramp-": "ramp+"}[shape]
    return (name, FIELD_LENGTH - x1, y0, FIELD_LENGTH - x0, y1, z0, z1, flipped, colour)


def field_pieces():
    """Every solid piece: (name, x0, y0, x1, y1, z0, z1, shape, colour)."""
    pieces = []
    for piece in MIRRORED:
        pieces.append((piece[0] + "_red",) + piece[1:])
        pieces.append((piece[0] + "_blue",) + mirror(piece)[1:])
    pieces += CENTRED
    t, h = BOUNDARY_THICKNESS, BOUNDARY_HEIGHT
    pieces += [
        ("boundary_south", -t, -t, FIELD_LENGTH + t, 0.0, 0.0, h, "box", GREY),
        ("boundary_north", -t, FIELD_WIDTH, FIELD_LENGTH + t, FIELD_WIDTH + t, 0.0, h, "box", GREY),
        ("boundary_west", -t, 0.0, 0.0, FIELD_WIDTH, 0.0, h, "box", GREY),
        ("boundary_east", FIELD_LENGTH, 0.0, FIELD_LENGTH + t, FIELD_WIDTH, 0.0, h, "box", GREY),
    ]
    return pieces


def field_zones():
    """Floor markings and bumpy roads: (name, x0, y0, x1, y1, height, colour)."""
    zones = []
    for name, x0, y0, x1, y1, height, colour in MIRRORED_ZONES:
        for team, (a, b) in (("red", (x0, x1)), ("blue", (FIELD_LENGTH - x1, FIELD_LENGTH - x0))):
            zones.append((f"{name}_{team}", a, y0, b, y1, height, colour or TEAM_COLOURS[team]))
    return zones + CENTRED_ZONES


def field_boxes():
    """Every solid piece's footprint, (name, centre x, centre y, half x, half y), for plots."""
    return [(n, (x0 + x1) / 2, (y0 + y1) / 2, (x1 - x0) / 2, (y1 - y0) / 2)
            for n, x0, y0, x1, y1, *_ in field_pieces()]


RAMP_SLAB = 0.15  # a ramp is a tilted slab whose top face is the slope


def piece_shape(x0, y0, x1, y1, z0, z1, shape):
    """(centre, rotation, half-extents) of a piece's box, in a frame at its footprint's centre."""
    hy = (y1 - y0) / 2
    if shape == "box":
        return [0.0, 0.0, (z0 + z1) / 2], [1, 0, 0, 0], [(x1 - x0) / 2, hy, (z1 - z0) / 2]
    run, rise = x1 - x0, z1 - z0
    angle = math.atan2(rise, run)
    tilt = -angle if shape == "ramp+" else angle  # about +y: rising toward +x is a negative turn
    normal = mat_vec(quat_to_matrix(quat_axis((0, 1, 0), tilt)), (0, 0, 1))
    top_middle = [0.0, 0.0, z0 + rise / 2]
    centre = [t - n * RAMP_SLAB / 2 for t, n in zip(top_middle, normal)]
    return centre, quat_axis((0, 1, 0), tilt), [math.hypot(run, rise) / 2, hy, RAMP_SLAB / 2]


def generate_field():
    out = [
        "# GENERATED by tools/generate_scene.py -- edit that, not this.",
        "#",
        "# The ARC 2026 3v3 Battlefield, from the rules manual (2026 ARCC Rules: Events, 3.2:",
        "# Figure 3-3's plan and side view, Figure 3-6's High Ground), mirrored about x = 6. World",
        "# frame (the world joint's) = Otto's and the MCB's field frame: origin at the red side's",
        "# corner, +x along the 12 m length, +y along the 8 m width, +z up. The world joint itself",
        "# sits at (-6, -4, 0), so the viewer, which orbits the origin, centres on the field. Every",
        "# piece is welded to the massless world root at its true height: the 420 mm boundary, the walls' 420 mm bases with their panel",
        "# (near) and post (far), the 420 mm centre block, and the High Ground's 210 mm deck, 410 mm",
        "# step and 15-degree ramps (tilted slabs). The sentry's chassis is planar, so it can't",
        "# climb the ramps: like every other piece they are obstacles to it. Floor markings and",
        "# the bumpy roads are drawn but not collided with.",
        "#",
        "# Collision groups: the field is group 1 and collides with group 2 (the sentry).",
        "archetype: field",
        "includes:",
        "  - sentry.arow",
        "joints:",
        joint("world", "null", WELDED, 0, FIELD_ORIGIN, (1, 0, 0, 0), [1, 0, 0, 0, 0.1, 0.1, 0.1, 0, 0, 0],
              extra={**no_visual(), **no_collision()}),
    ]
    for name, x0, y0, x1, y1, z0, z1, shape, colour in field_pieces():
        centre, rotation, half = piece_shape(x0, y0, x1, y1, z0, z1, shape)
        fields = visual("box", centre, rotation, half, colour)
        fields.update({
            "collision_pose": pose(centre, rotation),
            "collision_extents": fmt(half),
            "collision_radius": "[0]",
            "collision_margin": "[0.005]",
            "collision_group": "[1]",
            "collision_mask": "[2]",
        })
        out.append(joint(name, "world", WELDED, 0, ((x0 + x1) / 2, (y0 + y1) / 2, 0), (1, 0, 0, 0),
                         [1, 0, 0, 0, 0.1, 0.1, 0.1, 0, 0, 0], extra=fields))
    for name, x0, y0, x1, y1, height, colour in field_zones():
        fields = visual("box", (0, 0, height / 2), [1, 0, 0, 0], ((x1 - x0) / 2, (y1 - y0) / 2, height / 2), colour)
        fields.update(no_collision())
        out.append(joint(name, "world", WELDED, 0, ((x0 + x1) / 2, (y0 + y1) / 2, 0), (1, 0, 0, 0),
                         [1, 0, 0, 0, 0.1, 0.1, 0.1, 0, 0, 0], extra=fields))
    write("field.arow", "\n".join(out) + "\n")


# --- Sentry ---

def floats(text):
    return [float(v) for v in text.split()]


def load_urdf():
    """links: name -> (mass, com, inertia matrix about the com, in the link frame);
    joints: child link -> (name, parent link, origin xyz, origin rpy, axis)."""
    root = ET.parse(URDF).getroot()
    links = {}
    for link in root.findall("link"):
        inertial = link.find("inertial")
        if inertial is None:
            continue
        origin = inertial.find("origin")
        i = inertial.find("inertia").attrib
        tensor = [
            [float(i["ixx"]), float(i["ixy"]), float(i["ixz"])],
            [float(i["ixy"]), float(i["iyy"]), float(i["iyz"])],
            [float(i["ixz"]), float(i["iyz"]), float(i["izz"])],
        ]
        rotation = rpy_matrix(floats(origin.get("rpy", "0 0 0")))
        links[link.get("name")] = (
            float(inertial.find("mass").get("value")),
            floats(origin.get("xyz", "0 0 0")),
            mat_mul(mat_mul(rotation, tensor), transpose(rotation)),
        )
    joints = {}
    for j in root.findall("joint"):
        origin = j.find("origin")
        axis = j.find("axis")
        joints[j.find("child").get("link")] = (
            j.get("name"),
            j.find("parent").get("link"),
            floats(origin.get("xyz", "0 0 0")),
            floats(origin.get("rpy", "0 0 0")),
            floats(axis.get("xyz")) if axis is not None else None,
        )
    return links, joints


def load_primitives():
    """link -> [(name, shape, half-extents (x, y, z), centre, rotation matrix)] in the link frame."""
    with open(PRIMITIVES) as f:
        spec = yaml.safe_load(f)["links"]
    out = {}
    for link, data in spec.items():
        shapes = []
        for p in data.get("primitives", []):
            if p["name"] in SKIPPED_PRIMITIVES:
                continue
            origin = p.get("origin") or p["robot_aligned_origin"]
            dims = p["dimensions"]
            if p["type"] == "box":
                half = [v / 2 for v in dims["size"]]
            else:
                half = [dims["radius"], dims["radius"], dims["length"] / 2]
            shapes.append((p["name"], p["type"], half, origin["xyz"], rpy_matrix(origin["rpy"])))
        out[link] = shapes
    return out


def spatial_inertia(mass, com, tensor, frame):
    """rigid_body_inertia about a joint origin, in the joint's frame -- [m, m*c, Ixx, Iyy, Izz, Ixy,
    Iyz, Ixz] -- from a URDF inertial (the link frame) and the joint frame's rotation in it."""
    r = transpose(frame)
    c = mat_vec(r, com)
    i = mat_mul(mat_mul(r, tensor), frame)
    cx, cy, cz = c
    ixx = i[0][0] + mass * (cy * cy + cz * cz)
    iyy = i[1][1] + mass * (cx * cx + cz * cz)
    izz = i[2][2] + mass * (cx * cx + cy * cy)
    ixy = i[0][1] - mass * cx * cy
    iyz = i[1][2] - mass * cy * cz
    ixz = i[0][2] - mass * cx * cz
    return [mass, mass * cx, mass * cy, mass * cz, ixx, iyy, izz, ixy, iyz, ixz]


def wheel_directions():
    """Each wheel's (name, rolling direction) in the chassis frame, x forward, y left.

    The rolling direction is the way the wheel carries the chassis when the firmware calls its
    velocity positive: Holonomic4MotorChassisSubsystem::calculateOutput commands LF = x - y,
    RF = -x - y, LB = x + y, RB = y - x (and -r on every wheel), the same directions as
    FourWheelEKFOdometry::WHEEL_CONFIGS' wheelOrientationToForwardRadians. The URDF's wheel
    axes agree: each is z x (rolling direction).
    """
    s = 1 / math.sqrt(2)
    return [("left_front", (s, -s)), ("right_front", (-s, -s)), ("left_back", (s, s)), ("right_back", (-s, s))]


def sentry_frames(joints):
    """URDF link -> (parent joint name, translation, joint-frame rotation in the link frame,
    subspace). The chassis is planar; wheels turn their frame so the axle is local x; turret
    joints keep the URDF's frames (yaw about z, pitch about y)."""
    rolling = dict(wheel_directions())
    frames = {}
    for link, name in URDF_NAMES.items():
        if link == "base_link":
            footprint = joints["base_link"][2]
            frames[link] = (None, footprint, [[1, 0, 0], [0, 1, 0], [0, 0, 1]], None)
            continue
        _, parent, xyz, rpy, axis = joints[link]
        assert rpy == [0.0, 0.0, 0.0], f"{link}: rotated joint origins aren't handled"
        if name in rolling:
            dx, dy = rolling[name]
            assert abs(axis[0] + dy) < 1e-4 and abs(axis[1] - dx) < 1e-4, f"{link}: axis {axis} vs firmware"
            frame = quat_to_matrix(quat_z(math.atan2(axis[1], axis[0])))
            sub = subspace([[1, 0, 0, 0, 0, 0]])
        elif axis == [0.0, 0.0, 1.0]:
            frame, sub = [[1, 0, 0], [0, 1, 0], [0, 0, 1]], subspace([[0, 0, 1, 0, 0, 0]])
        elif axis == [0.0, 1.0, 0.0]:
            frame, sub = [[1, 0, 0], [0, 1, 0], [0, 0, 1]], subspace([[0, 1, 0, 0, 0, 0]])
        else:
            raise ValueError(f"{link}: unhandled axis {axis}")
        frames[link] = (URDF_NAMES[parent], xyz, frame, sub)
    return frames


# Reflected rotor inertia on each driven axis (the joint frame's index): the URDF's links don't
# carry the motors' rotors spinning at the gear ratio.
ROTOR_INERTIA = {
    "wheel": (0, M3508_ROTOR_INERTIA / CHASSIS_GEARBOX_RATIO ** 2),
    "turret_major": (2, M3508_ROTOR_INERTIA / TURRET_MAJOR_RATIO ** 2),
    "widow_yaw": (2, GM6020_ROTOR_INERTIA),
    "widow_pitch": (1, GM6020_ROTOR_INERTIA),
}


def generate_sentry():
    links, joints = load_urdf()
    primitives = load_primitives()
    frames = sentry_frames(joints)
    wheel_names = dict(wheel_directions())
    out = [
        "# GENERATED by tools/generate_scene.py -- edit that, not this.",
        "#",
        "# The sentry, from robot/sentry_coarse.urdf (joint frames, axes, masses, inertias) drawn",
        "# with robot/primitive_collisions.yaml's shapes, attached to field.arow's world. Every",
        "# frame here is x forward, y left, z up, as the MCB's and Otto's chassis and turret",
        "# frames are. URDF links -> joints: base_link -> chassis, {fl,fr,bl,br}_wheel ->",
        "# {left,right}_{front,back}, turret_major, turret_minor_frame -> widow_yaw,",
        "# turret_minor_firing_system -> widow_pitch.",
        "#",
        "# - chassis: a planar joint (yaw, x, y) at base_link's height. The arena is flat and the",
        "#   firmware's odometry is 2D; the wheels don't touch anything -- sentry_sim.cpp's",
        "#   omni-wheel model turns each wheel's slip into a traction force on the chassis. Its",
        "#   one collision shape is the chassis's main box: the lidars scan every body's shape, and",
        "#   anything taller would sit in their beams.",
        "# - four wheels, each a revolute joint about its axle (local x), the axle turned so that",
        "#   positive rotation rolls the chassis the way the firmware calls that wheel's velocity",
        "#   positive. Inertias add the motors' rotors, reflected through their gearing.",
        "# - turret_major (yaw), widow_yaw (yaw), widow_pitch (pitch), where the URDF puts them.",
        "# - two lidar spinners on turret major: each LD19's rotating head, spun at 10 Hz by",
        "#   sentry_sim.cpp. Not in the URDF; placed as Otto's config says.",
        "# - <link>_<shape>: welded, visual-only children for a link's other primitives.",
        "archetype: sentry",
        "attach:",
        "  archetype: field",
        "  joint: world",
        "includes:",
    ]
    out += [f"  - {path}" for path in generate_sensors()]
    out.append("joints:")
    geometry = {}
    for link, name in URDF_NAMES.items():
        parent, translation, frame, sub = frames[link]
        mass, com, tensor = links[link]
        inertia = spatial_inertia(mass, com, tensor, frame)
        rotor = ROTOR_INERTIA.get("wheel" if name in wheel_names else name)
        if rotor:
            inertia[4 + rotor[0]] += rotor[1]
        colour = LINK_COLOURS.get(link, LINK_COLOURS["fl_wheel"])
        to_joint = transpose(frame)
        shapes = [(n, t, h, mat_vec(to_joint, c), matrix_to_quat(mat_mul(to_joint, r)))
                  for n, t, h, c, r in primitives[link]]
        main, rest = shapes[0], shapes[1:]
        if link == "base_link":
            main = next(s for s in shapes if s[0] == CHASSIS_COLLISION)
            rest = [s for s in shapes if s is not main]
        fields = visual(main[1], main[3], main[4], main[2], colour)
        if link == "base_link":
            fields.update({
                "collision_pose": pose(main[3], main[4]),
                "collision_extents": fmt([h - COLLISION_RADIUS for h in main[2]]),
                "collision_radius": f"[{COLLISION_RADIUS}]",
                "collision_margin": "[0.005]",
                "collision_group": "[2]",
                "collision_mask": "[1]",
            })
            out.append(joint(
                name, "null", subspace([[0, 0, 1, 0, 0, 0], [0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 1, 0]]), 7,
                translation, (1, 0, 0, 0), inertia,
                position=f"{{translation: [{INITIAL_X}, {INITIAL_Y}, 0], rotation: [1, 0, 0, 0]}}",
                extra=fields,
            ))
        else:
            fields.update(no_collision())
            out.append(joint(name, parent, sub, 1, translation, matrix_to_quat(frame), inertia, extra=fields))
        for shape_name, kind, half, centre, rotation in rest:
            extra = visual(kind, centre, rotation, half, colour)
            extra.update(no_collision())
            short = next((shape_name[len(p):] for p in SHAPE_PREFIXES if shape_name.startswith(p)), shape_name)
            out.append(joint(f"{name}_{short}", name, WELDED, 0, (0, 0, 0), (1, 0, 0, 0), VISUAL_ONLY_INERTIA,
                             extra=extra))
        if name in wheel_names:
            geometry["wheel_radius"] = primitives[link][0][2][0]
            geometry["wheel_offset"] = abs(translation[0])
            geometry["wheel_axle_inertia"] = inertia[4]
    yaw = subspace([[0, 0, 1, 0, 0, 0]])
    for name, (px, py), lidar_yaw in LIDARS:
        extra = visual("cylinder", (0, 0, 0.0175), [1, 0, 0, 0], (0.02, 0.02, 0.0175), (0.1, 0.1, 0.12))
        extra.update(no_collision())
        out.append(joint(
            name, "turret_major", yaw, 1, (px, py, LIDAR_HEIGHT), quat_z(lidar_yaw),
            [0.05, 0, 0, 0, 1e-4, 1e-4, 1e-4, 0, 0, 0], extra=extra,
        ))
    write("sentry.arow", "\n".join(out) + "\n")
    geometry["mass"] = sum(links[link][0] for link in URDF_NAMES)
    return geometry


def generate_geometry_header(geometry):
    write("sentry_geometry.hpp", "\n".join([
        "// GENERATED by tools/generate_scene.py from robot/sentry_coarse.urdf -- edit that, not",
        "// this. The numbers sentry_sim.cpp's omni-wheel model needs, as sentry.arow has them.",
        "#pragma once",
        "",
        "namespace sentry::geometry {",
        "",
        f"inline constexpr float kWheelRadius = {geometry['wheel_radius']:.6g}F;  // m",
        "// Each wheel's distance from the chassis centre along x and along y (they sit on the",
        "// diagonals).",
        f"inline constexpr float kWheelOffset = {geometry['wheel_offset']:.6g}F;  // m",
        "// About the axle, the motor's rotor reflected through the gearbox included.",
        f"inline constexpr float kWheelAxleInertia = {geometry['wheel_axle_inertia']:.6g}F;  // kg m^2",
        "// Everything the wheels carry.",
        f"inline constexpr float kRobotMass = {geometry['mass']:.6g}F;  // kg",
        "",
        "}  // namespace sentry::geometry",
        "",
    ]))


def sensor(name, kind, joint_name, config_lines, comment):
    """Writes sensors/<name>.arow (one sensor per file) and returns its include path."""
    lines = ["# GENERATED by tools/generate_scene.py -- edit that, not this."]
    lines += [f"# {line}" if line else "#" for line in comment]
    lines += [f"sensor: {name}", f"kind: {kind}", f"attach: {{archetype: sentry, joint: {joint_name}}}", "config:"]
    lines += [f"  {line}" for line in config_lines]
    write(f"sensors/{name}.arow", "\n".join(lines) + "\n")
    return f"sensors/{name}.arow"


def generate_sensors():
    # Every sensor on the sentry. sentry_sim.cpp turns their readings into what the real parts
    # put on the wire: DJI motor feedback frames, IMU register bursts, turret-MCB CAN frames,
    # LD19 serial packets. Encoders are ideal shaft angles -- the DJI and encoder emulation
    # quantizes them itself; the IMUs carry their chips' error models.
    out = []
    for name, _ in wheel_directions():
        out.append(sensor(f"{name}_shaft", "encoder", name, ["axis: [1, 0, 0]"], [f"{name} wheel shaft (truth)."]))
    for joint_name, axis in (("turret_major", "[0, 0, 1]"), ("widow_yaw", "[0, 0, 1]"), ("widow_pitch", "[0, 1, 0]")):
        out.append(sensor(f"{joint_name}_shaft", "encoder", joint_name, [f"axis: {axis}"], [f"{joint_name} output shaft (truth)."]))
    for name, _, _ in LIDARS:
        out.append(sensor(f"{name}_head", "encoder", name, ["axis: [0, 0, 1]", "multi_turn: false"], [f"{name}'s head angle."]))
        out.append(sensor(
            name, "lidar", name,
            [
                "period: 0.002",
                "horizontal_samples: 9",
                f"horizontal_fov: {math.radians(7.2):.6f}",
                "min_range: 0.02",
                "max_range: 12",
                "range_noise: 0.008",
                "resolution: 0.001",
            ],
            [
                f"{name}: an LD19's rotating head. 4500 samples/s at 10 rev/s is 9 samples",
                "every 2 ms across the 7.2 deg the head turns in that time; sentry_sim.cpp",
                "packs them, 12 to a packet, into LD19 serial frames.",
            ],
        ))
    # Turret major IMUs (on the main MCB): MEMS-grade error models.
    for name, chip, (x, y, z), yaw in MAJOR_IMUS:
        mount = f"mount: {{translation: {fmt((x, y, z))}, rotation: {fmt(quat_z(yaw))}}}"
        gyro_bias = "[0.004, -0.003, 0.005]" if chip == "ism330" else "[-0.006, 0.004, 0.003]"
        out.append(sensor(
            f"{name}_gyro", "gyroscope", "turret_major",
            [mount, f"bias: {gyro_bias}", "noise_density: 0.0012", "bias_instability: 0.00005"],
            [f"{name} ({chip}) gyro, mounted as turretMajorImuTransforms says."],
        ))
        out.append(sensor(
            f"{name}_accel", "accelerometer", "turret_major",
            [mount, "bias: [0.03, -0.02, 0.05]", "noise_density: 0.0015"],
            [f"{name} ({chip}) accelerometer."],
        ))
    for name, joint_name in (("chassis_mcb", "chassis"), ("widow_mcb", "widow_pitch")):
        out.append(sensor(
            f"{name}_gyro", "gyroscope", joint_name,
            ["bias: [0.003, 0.004, -0.004]", "noise_density: 0.0012", "bias_instability: 0.00005"],
            [f"The {name.replace('_', ' ')} (a turret MCB) IMU's gyro, in its mounted frame."],
        ))
        out.append(sensor(
            f"{name}_accel", "accelerometer", joint_name,
            ["bias: [0.02, 0.03, -0.04]", "noise_density: 0.0015"],
            [f"The {name.replace('_', ' ')} IMU's accelerometer."],
        ))
    return out


if __name__ == "__main__":
    generate_field()
    generate_geometry_header(generate_sentry())
    print("wrote field.arow, sentry.arow, sensors/ and sentry_geometry.hpp")
