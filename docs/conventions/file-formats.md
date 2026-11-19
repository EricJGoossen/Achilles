# File formats

Both formats are YAML. All values are SI: meters, kilograms, seconds and
radians. `interface` loads and validates them, and throws on bad input.

## `.arow` files

Each file declares exactly one archetype, one sensor or one actuator. Paths
in `includes` are relative to the including file.

### Archetype

```yaml
archetype: wheeled_robot   # unique across every included file
copies: 1                  # optional, default 1
attach:                    # omitted only on the scene
  archetype: scene
  joint: robot_mount
includes:                  # optional
  - ../actuators/left_front_motor.arow
joints:
  - name: chassis
    parent: null           # null: this archetype's local root
    fields:
      joint_position: {translation: [0, 0.1, 0], rotation: [1, 0, 0, 0]}
```

- Exactly one archetype has no `attach`: the scene. It has one copy.
- The attach graph is a tree, and each archetype's joints form one tree.
- Field names match a field's `kName`.
- A field is given on every joint of an archetype or on none.
- A value is a flat list or nested lists and maps of numbers, flattened in
  order. Its leaves follow the field's assembler order. A transform is
  `translation [x, y, z]`, then `rotation [w, x, y, z]`.
- Every copy shares a joint's field value, unless it's written
  `{per_copy: [v_0, v_1, ...]}`: then copy `i` gets `v_i`, and the list has
  exactly `copies` entries of one leaf count. Sibling bodies that differ
  only in their values, like the rollers around a wheel, can then be one
  archetype's copies and share SIMD lanes.

### Sensor or actuator

```yaml
sensor: left_front_encoder   # or actuator:; unique across both
kind: encoder                # a registered kind's kListName
attach: {archetype: wheeled_robot, joint: left_front}
config:                      # optional; each a number or list of numbers
  position_resolution: 0.0061359
```

- Every instance of the attached archetype gets one.
- An actuator's `axis` is in its joint's frame.

### Layering

- A scene lives in its example's folder. Shared robots, sensors and
  actuators live in `examples/shared/`.
- A robot includes either both its sensors and its actuators, or neither.
  When it includes neither, the scene includes them.

## `sim_config.yaml`

```yaml
world_base_transform: {translation: [0, 0, 0], rotation: [1, 0, 0, 0]}
base_velocity: {angular: [0, 0, 0], linear: [0, 0, 0]}
base_acceleration: {angular: [0, 0, 0], linear: [0, 9.81, 0]}
```

- Every key is optional and defaults to zero, or identity for the
  transform.
- `base_acceleration` is minus gravity: `[0, 9.81, 0]` is gravity along -y.
