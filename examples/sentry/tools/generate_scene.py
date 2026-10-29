#!/usr/bin/env python3
"""Writes the sentry demo's scene files: field.arow, sentry.arow and sensors/*.arow.

The field is the ARC 2026 arena as Project Otto's fixed map describes it
(project_otto/autonav/map/_occupancy_grid.py, ARC2026Field): every obstacle
rectangle there becomes a box here, mirrored across x = 6 the same way, plus
the boundary walls. The sentry's dimensions are aruw-mcb's
(robot/sentry/sentry_chassis_constants.hpp, sentry_turret_constants.hpp)
where the firmware states them and estimates elsewhere; see the README.

Run from anywhere: python3 examples/sentry/tools/generate_scene.py
"""

import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.dirname(HERE)

# --- Field (Otto's ARC2026Field, meters; x along the 12 m length, y along the 8 m width) ---
FIELD_LENGTH = 12.0
FIELD_WIDTH = 8.0
WALL_HEIGHT = 0.5  # every obstacle; the lidars sit well below it
BOUNDARY_THICKNESS = 0.1

WIDE_PLATFORM = ((3.0 - 0.784, 6.0), (5.0, FIELD_WIDTH))
THIN_PLATFORM = ((5.0, 7.0 - 0.02), (7.0, FIELD_WIDTH))
WALL_THICKNESS = 0.420 - 0.250
NEAR_WALL = ((2.250, FIELD_WIDTH - 2 - 0.8 - 3.0254), (2.250 + WALL_THICKNESS, FIELD_WIDTH - 2 - 0.8))
FAR_WALL_X = 2.250 + WALL_THICKNESS + 1
FAR_WALL = ((FAR_WALL_X, FIELD_WIDTH - 2 - 2.0254), (FAR_WALL_X + WALL_THICKNESS, FIELD_WIDTH - 2))
ENTRANCE_BLOCK = ((2.5, 3.5), (FAR_WALL_X + WALL_THICKNESS, 4.0))
BLOCK = ((5.0, 0.7), (7.0, 1.7))
MIRRORED_OBSTACLES = [
    ("wide_platform", WIDE_PLATFORM),
    ("thin_platform", THIN_PLATFORM),
    ("near_wall", NEAR_WALL),
    ("far_wall", FAR_WALL),
    ("entrance_block", ENTRANCE_BLOCK),
    ("block", BLOCK),
]

# --- Sentry (aruw-mcb SENTRY_ACHLYS) ---
WHEEL_RADIUS = 0.0762  # WHEEL_RADIUS (FUDGE_FACTOR 1)
WHEELBASE_RADIUS = 0.23864  # chassis center to each wheel
WHEEL_OFFSET = WHEELBASE_RADIUS / math.sqrt(2)  # per axis, wheels on the diagonals
CHASSIS_GEARBOX_RATIO = 17.0 / 268.0  # output turns per rotor turn
M3508_ROTOR_INERTIA = 1.2e-5  # kg m^2, estimate
TURRET_MAJOR_RATIO = (187.0 / 3591.0) * (30.0 / 95.0)  # PULLEY_RATIO
GM6020_ROTOR_INERTIA = 2.4e-4  # kg m^2, estimate
INITIAL_X, INITIAL_Y = 0.75, 4.0  # INITIAL_CHASSIS_POSITION_X/Y, facing +x

CHASSIS_MASS = 14.0
CHASSIS_HALF = (0.24, 0.24, 0.07)  # x, y, z half-extents of the collision box
TURRET_MAJOR_HEIGHT = 0.20  # above the chassis frame (axle height)
WIDOW_YAW_HEIGHT = 0.10  # above turret major
WIDOW_PITCH_HEIGHT = 0.06  # above the widow yaw joint

# The lidars, where Otto's sentry config mounts them (lidar_position_to_chassis_frame and
# lidar_orientation_to_chassis_frame: the LD19's zero direction, as a yaw from forward), at an
# estimated height. Otto's "chassis" frame is turret major's: the sentry's MCB reports turret
# major's pose as the chassis odometry it sends the Jetson
# (SentryTransformAdapter::getVisionWorldToChassis), so the lidars ride on turret major and
# don't spin when the chassis beyblades. Each LD19's head spins on its own joint (see
# sentry_sim.cpp).
LIDAR_HEIGHT = 0.06  # above turret major's frame
LIDARS = [("lidar_front", (0.135, 0.0), -1.5707), ("lidar_rear", (-0.135, 0.0), 1.5707)]

# Turret major's three IMUs (sentry_drivers.hpp turretMajorImuTransforms: mm and rad).
MAJOR_IMUS = [
    ("major_ism330_0", "ism330", (-76.7e-3, -116.14e-3, 0.0), 0.0),
    ("major_ism330_1", "ism330", (-76.7e-3, 116.04e-3, 0.0), math.pi),
    ("major_mpu6500", "mpu6500", (-14.97e-3, -115.5e-3, 0.0), math.pi / 2),
]

WELDED = "[" + ", ".join(["0"] * 36) + "]"
REST = "{translation: [0, 0, 0], rotation: [1, 0, 0, 0]}"
UPRIGHT = "[0.70710678, 0.70710678, 0, 0]"


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


def quat_z(angle):
    return [math.cos(angle / 2), 0.0, 0.0, math.sin(angle / 2)]


def fmt(values):
    return "[" + ", ".join(f"{v:.6g}" for v in values) + "]"


def box_inertia(mass, half, com=(0.0, 0.0, 0.0)):
    """rigid_body_inertia about the joint origin: [m, m*c, Ixx, Iyy, Izz, Ixy, Iyz, Ixz]."""
    a, b, c = (2 * h for h in half)
    ixx = mass * (b * b + c * c) / 12
    iyy = mass * (a * a + c * c) / 12
    izz = mass * (a * a + b * b) / 12
    cx, cy, cz = com
    ixx += mass * (cy * cy + cz * cz)
    iyy += mass * (cx * cx + cz * cz)
    izz += mass * (cx * cx + cy * cy)
    return [mass, mass * cx, mass * cy, mass * cz, ixx, iyy, izz, -mass * cx * cy, -mass * cy * cz, -mass * cx * cz]


def joint(name, parent, sub, mask, translation, rotation, inertia, position=REST, extra=None):
    lines = [
        f"  - name: {name}",
        f"    parent: {parent}",
        "    fields:",
        f"      joint_subspace: {sub}",
        f"      joint_activation_mask: [{mask}]",
        f"      fixed_joint_transform: {{translation: {fmt(translation)}, rotation: {fmt(rotation)}}}",
        f"      rigid_body_inertia: {fmt(inertia)}",
        f"      joint_position: {position}",
        "      joint_velocity: [0, 0, 0, 0, 0, 0]",
    ]
    for key, value in (extra or {}).items():
        lines.append(f"      {key}: {value}")
    return "\n".join(lines)


def no_collision(visual_extents=(0, 0, 0), color=(0, 0, 0)):
    return {
        "visual_extents": fmt(visual_extents),
        "visual_color": fmt(color),
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


def field_boxes():
    """(name, center x, center y, half x, half y) for every obstacle and boundary wall."""
    boxes = []
    for name, ((x0, y0), (x1, y1)) in MIRRORED_OBSTACLES:
        for suffix, (a, b) in (("red", (x0, x1)), ("blue", (FIELD_LENGTH - x1, FIELD_LENGTH - x0))):
            boxes.append((f"{name}_{suffix}", (a + b) / 2, (y0 + y1) / 2, (b - a) / 2, (y1 - y0) / 2))
    t = BOUNDARY_THICKNESS
    boxes += [
        ("boundary_south", FIELD_LENGTH / 2, -t / 2, FIELD_LENGTH / 2 + t, t / 2),
        ("boundary_north", FIELD_LENGTH / 2, FIELD_WIDTH + t / 2, FIELD_LENGTH / 2 + t, t / 2),
        ("boundary_west", -t / 2, FIELD_WIDTH / 2, t / 2, FIELD_WIDTH / 2),
        ("boundary_east", FIELD_LENGTH + t / 2, FIELD_WIDTH / 2, t / 2, FIELD_WIDTH / 2),
    ]
    return boxes


def generate_field():
    out = [
        "# GENERATED by tools/generate_scene.py -- edit that, not this.",
        "#",
        "# The ARC 2026 arena, from Project Otto's fixed map (ARC2026Field): each obstacle",
        "# rectangle there is a box here, mirrored across x = 6 as Otto mirrors it, inside four",
        "# boundary walls. World frame = Otto's and the MCB's field frame: origin at the red",
        "# side's corner, +x along the 12 m length, +y along the 8 m width, +z up. Every box is",
        f"# {WALL_HEIGHT} m tall and welded to the massless world root. Boxes are authored +y up",
        "# for the viewer (see examples/maze/maze.arow), hence the quarter turn about x.",
        "#",
        "# Collision groups: the field is group 1 and collides with group 2 (the sentry).",
        "archetype: field",
        "includes:",
        "  - sentry.arow",
        "joints:",
        joint("world", "null", WELDED, 0, (0, 0, 0), (1, 0, 0, 0), [1, 0, 0, 0, 0.1, 0.1, 0.1, 0, 0, 0],
              extra=no_collision()),
    ]
    for name, cx, cy, hx, hy in field_boxes():
        h = WALL_HEIGHT / 2
        out.append(joint(
            name, "world", WELDED, 0, (cx, cy, 0), [float(v) for v in UPRIGHT.strip("[]").split(",")],
            [1, 0, 0, 0, 0.1, 0.1, 0.1, 0, 0, 0],
            extra={
                "visual_extents": fmt((hx, h, hy)),
                "visual_color": "[0.55, 0.6, 0.7]",
                "collision_pose": f"{{translation: [0, {h:.6g}, 0], rotation: [1, 0, 0, 0]}}",
                "collision_extents": fmt((hx, h, hy)),
                "collision_radius": "[0]",
                "collision_margin": "[0.005]",
                "collision_group": "[1]",
                "collision_mask": "[2]",
            },
        ))
    write("field.arow", "\n".join(out) + "\n")


def wheel_directions():
    """Each wheel's (name, position, rolling direction) in the chassis frame, x forward, y left.

    The rolling direction is the way the wheel carries the chassis when the firmware calls its
    velocity positive: Holonomic4MotorChassisSubsystem::calculateOutput commands LF = x - y,
    RF = -x - y, LB = x + y, RB = y - x (and -r on every wheel), the same directions as
    FourWheelEKFOdometry::WHEEL_CONFIGS' wheelOrientationToForwardRadians.
    """
    a = WHEEL_OFFSET
    s = 1 / math.sqrt(2)
    return [
        ("left_front", (a, a), (s, -s)),
        ("right_front", (a, -a), (-s, -s)),
        ("left_back", (-a, a), (s, s)),
        ("right_back", (-a, -a), (-s, s)),
    ]


def generate_sentry():
    chassis_inertia = box_inertia(CHASSIS_MASS, CHASSIS_HALF, (0, 0, 0.05))
    out = [
        "# GENERATED by tools/generate_scene.py -- edit that, not this.",
        "#",
        "# The sentry (aruw-mcb SENTRY_ACHLYS), attached to field.arow's world. Every frame here",
        "# is x forward, y left, z up, as the MCB's and Otto's chassis and turret frames are.",
        "#",
        "# - chassis: a planar joint (yaw, x, y) at axle height. The arena is flat and the",
        "#   firmware's odometry is 2D; the wheels don't touch anything -- sentry_sim.cpp's",
        "#   omni-wheel model turns each wheel's slip into a traction force on the chassis.",
        "# - four wheels on the diagonals, each a revolute joint about its axle (local x), the",
        "#   axle turned so that positive rotation rolls the chassis the way the firmware calls",
        "#   that wheel's velocity positive. Their inertia includes the M3508 rotor's, reflected",
        "#   through the chassis gearbox.",
        "# - turret_major: yaw about z over the chassis center, driven by an M3508 through the",
        "#   pulley (reflected rotor inertia included); widow_yaw on top of it (offset 0, as",
        "#   TURRET_MINOR_OFFSET says); widow_pitch about y. The RealSense rides on widow_pitch.",
        "# - two lidar spinners on turret major: each LD19's rotating head, spun at 10 Hz by",
        "#   sentry_sim.cpp.",
        "#",
        "# Masses and heights not in the firmware are estimates; see the README.",
        "archetype: sentry",
        "attach:",
        "  archetype: field",
        "  joint: world",
        "includes:",
    ]
    out += [f"  - {path}" for path in generate_sensors()]
    out += [
        "joints:",
        joint(
            "chassis", "null", subspace([[0, 0, 1, 0, 0, 0], [0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 1, 0]]), 7,
            (0, 0, WHEEL_RADIUS), (1, 0, 0, 0), chassis_inertia,
            position=f"{{translation: [{INITIAL_X}, {INITIAL_Y}, 0], rotation: [1, 0, 0, 0]}}",
            extra={
                "visual_extents": fmt((CHASSIS_HALF[0], CHASSIS_HALF[2], CHASSIS_HALF[1])),
                "visual_color": "[0.85, 0.2, 0.2]",
                "collision_pose": "{translation: [0, 0, 0.05], rotation: [1, 0, 0, 0]}",
                "collision_extents": fmt((CHASSIS_HALF[0] - 0.02, CHASSIS_HALF[1] - 0.02, CHASSIS_HALF[2] - 0.02)),
                "collision_radius": "[0.02]",
                "collision_margin": "[0.005]",
                "collision_group": "[2]",
                "collision_mask": "[1]",
            },
        ),
    ]
    axle = subspace([[1, 0, 0, 0, 0, 0]])
    wheel_axle_inertia = 1.0e-3 + M3508_ROTOR_INERTIA / (CHASSIS_GEARBOX_RATIO ** 2)
    for name, (px, py), (dx, dy) in wheel_directions():
        # The axle u = z x d: positive rotation about it rolls the wheel along d.
        ux, uy = -dy, dx
        rotation = quat_z(math.atan2(uy, ux))
        out.append(joint(
            name, "chassis", axle, 1, (px, py, 0), rotation,
            [0.6, 0, 0, 0, wheel_axle_inertia, 1.5e-3, 1.5e-3, 0, 0, 0],
            extra=no_collision((0.02, WHEEL_RADIUS, 0.02), (0.15, 0.15, 0.17)),
        ))
    yaw = subspace([[0, 0, 1, 0, 0, 0]])
    pitch = subspace([[0, 1, 0, 0, 0, 0]])
    major_yaw_inertia = 0.045 + M3508_ROTOR_INERTIA / (TURRET_MAJOR_RATIO ** 2)
    out.append(joint(
        "turret_major", "chassis", yaw, 1, (0, 0, TURRET_MAJOR_HEIGHT), (1, 0, 0, 0),
        [4.0, 0, 0, 0, 0.03, 0.03, major_yaw_inertia, 0, 0, 0],
        extra=no_collision((0.15, 0.05, 0.15), (0.3, 0.3, 0.35)),
    ))
    out.append(joint(
        "widow_yaw", "turret_major", yaw, 1, (0, 0, WIDOW_YAW_HEIGHT), (1, 0, 0, 0),
        [1.0, 0, 0, 0, 0.005, 0.005, 0.008 + GM6020_ROTOR_INERTIA, 0, 0, 0],
        extra=no_collision((0.05, 0.03, 0.05), (0.3, 0.3, 0.35)),
    ))
    out.append(joint(
        "widow_pitch", "widow_yaw", pitch, 1, (0, 0, WIDOW_PITCH_HEIGHT), (1, 0, 0, 0),
        box_inertia(1.45, (0.1, 0.05, 0.05), (0.02, 0, 0))[:5] + [0.006 + GM6020_ROTOR_INERTIA, 0.008, 0, 0, 0],
        extra=no_collision((0.12, 0.04, 0.05), (0.2, 0.2, 0.25)),
    ))
    for name, (px, py), lidar_yaw in LIDARS:
        out.append(joint(
            name, "turret_major", yaw, 1, (px, py, LIDAR_HEIGHT), quat_z(lidar_yaw),
            [0.05, 0, 0, 0, 1e-4, 1e-4, 1e-4, 0, 0, 0],
            extra=no_collision((0.02, 0.02, 0.02), (0.1, 0.1, 0.1)),
        ))
    write("sentry.arow", "\n".join(out) + "\n")


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
    for name, _, _ in wheel_directions():
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
    generate_sentry()
    print("wrote field.arow, sentry.arow and sensors/")
