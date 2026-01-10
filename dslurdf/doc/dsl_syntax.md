# DURDF Syntax Specification

**DURDF** (Domain Specific Language for URDF) is a YAML-based DSL for ROS2 robot descriptions.

**Status**: This document reflects the **current implementation** (v0.1.0)
**See also**: [dsl_syntax_futures.md](dsl_syntax_futures.md) for planned features

---

## Overview

DURDF achieves 43-60% code reduction through:
- **Flat syntax**: Geometry as top-level properties
- **Smart defaults**: Auto-collision, fixed joints
- **Hierarchy-first**: TF tree visible at top of file
- **Array shorthand**: Concise geometry definitions

---

## File Structure

```yaml
robot: <name>                # Required: Robot name

materials:                   # Optional: Custom material definitions
  <material_name>: [r, g, b, a]

hierarchy:                   # Required: Parent-child tree structure
  <root_link>:
    <child_link>:
      - <grandchild1>
      - <grandchild2>

joints:                      # Optional: Joint properties
  <child_link>:              # Keyed by child link name
    type: <joint_type>
    xyz: [x, y, z]
    axis: [x, y, z]

links:                       # Required: Link geometries
  <link_name>:
    <geometry>: [params]
    material: <name>
    rpy: [r, p, y]
```

---

## 1. Robot Declaration

```yaml
robot: my_robot_name
```

**Rules:**
- Must be first line
- Alphanumeric with underscores
- Used as `<robot name="...">` in URDF

---

## 2. Materials Section (Optional)

Define custom materials once, reference by name:

```yaml
materials:
  clear: [0.9, 0.9, 0.9, 0.3]     # RGBA values (0-1)
  black: [0, 0, 0, 1]
  my_color: [0.5, 0.2, 0.8, 1.0]
```

**Format:**
- `[r, g, b, a]` where each value is 0.0 to 1.0
- `r` = red, `g` = green, `b` = blue, `a` = alpha (transparency)

**Usage:**
```yaml
links:
  base_link:
    material: clear           # Reference by name
```

---

## 3. Hierarchy Section (Required)

Defines the TF tree (parent-child relationships):

### Format 1: Nested Dictionary

```yaml
hierarchy:
  base_footprint:
    base_link:
      left_wheel:
      right_wheel:
```

### Format 2: List of Children

```yaml
hierarchy:
  base_footprint:
    base_link:
      - left_wheel
      - right_wheel
      - lidar
```

### Format 3: Mixed (Nested with Lists)

```yaml
hierarchy:
  base_footprint:
    base_link:              # Nested child
      - left_wheel          # List of children
      - right_wheel
      lidar:                # Nested child with descendants
        - camera
```

**Rules:**
- Root link has no parent (e.g., `base_footprint`)
- Indentation shows parent-child relationships
- Each child automatically gets a joint to its parent
- Joint names auto-generated: `{parent}_to_{child}`

---

## 4. Joints Section (Optional)

Specify joint properties for children in hierarchy:

```yaml
joints:
  base_link:                 # Child link name (NOT joint name)
    xyz: [0, 0, 0.05]       # Position relative to parent

  left_wheel:
    type: continuous        # Joint type
    xyz: [0, 0.10, -0.015]
    axis: [0, 1, 0]         # Rotation axis

  right_wheel:
    type: continuous
    xyz: [0, -0.10, -0.015]
    axis: [0, 1, 0]
```

### Joint Properties

| Property | Type | Default | Description |
|----------|------|---------|-------------|
| `type` | string | `fixed` | `fixed`, `continuous`, `revolute`, `prismatic` |
| `xyz` | `[x, y, z]` | `[0, 0, 0]` | Position offset from parent |
| `rpy` | `[r, p, y]` | `[0, 0, 0]` | Rotation offset (radians) |
| `axis` | `[x, y, z]` | - | Rotation/translation axis (required for continuous/revolute/prismatic) |

**Notes:**
- If omitted, joint defaults to `type: fixed` with no offset
- Joint name auto-generated from hierarchy
- `axis` required for `continuous` and `revolute` joints

---

## 5. Links Section (Required)

Define geometry and visual properties:

```yaml
links:
  base_footprint: {}         # Empty link (reference frame only)

  base_link:
    octagon: [0.15, 0.003]   # Geometry
    material: clear          # Material reference

  left_wheel:
    cylinder: [0.035, 0.02]
    rpy: [1.5708, 0, 0]     # Visual rotation (radians)
    material: black
```

### Geometry Types

#### Cylinder
```yaml
cylinder: [radius, length]
```
- `radius`: Cylinder radius (meters)
- `length`: Cylinder height (meters)

#### Box
```yaml
box: [x, y, z]
```
- `x, y, z`: Dimensions in meters

#### Sphere
```yaml
sphere: [radius]
```
- `radius`: Sphere radius (meters)

#### Octagon (8-sided prism)
```yaml
octagon: [radius, length]
```
- `radius`: Distance from center to vertex (circumradius)
- `length`: Height of prism (meters)
- Currently approximated as cylinder in URDF with comment

#### Hexagon (6-sided prism)
```yaml
hexagon: [radius, length]
```
- `radius`: Distance from center to vertex (circumradius)
- `length`: Height of prism (meters)
- Currently approximated as cylinder in URDF with comment

### Link Properties

| Property | Type | Required | Description |
|----------|------|----------|-------------|
| Geometry | See above | No* | One of: cylinder, box, sphere, octagon, hexagon |
| `material` | string | No | Material name (from materials section) |
| `rpy` | `[r, p, y]` | No | Visual geometry rotation in radians |

\* Geometry can be omitted for reference frames (e.g., `base_footprint: {}`)

### Visual Rotation (rpy)

Rotates the visual (and collision) geometry:

```yaml
left_wheel:
  cylinder: [0.035, 0.02]
  rpy: [1.5708, 0, 0]        # Rotate 90° around X-axis
```

**Common rotations (radians):**
- 90° = 1.5708
- 180° = 3.1416
- 270° = 4.7124

---

## 6. Smart Defaults

### Auto-Collision

Collision geometry is automatically generated identical to visual geometry:

```yaml
base_link:
  cylinder: [0.15, 0.05]
```

Generates:
```xml
<visual>
  <geometry>
    <cylinder radius="0.15" length="0.05"/>
  </geometry>
</visual>
<collision>
  <geometry>
    <cylinder radius="0.15" length="0.05"/>  <!-- Auto-generated -->
  </geometry>
</collision>
```

### Fixed Joints

If `type` omitted in joints section, defaults to `fixed`:

```yaml
joints:
  sensor:
    xyz: [0.1, 0, 0.2]      # Defaults to fixed joint
```

### Empty Links

Reference frames with no physical geometry:

```yaml
links:
  base_footprint: {}         # Valid - no geometry
```

---

## Complete Example

```yaml
robot: simple_bot

materials:
  clear: [0.9, 0.9, 0.9, 0.3]
  black: [0, 0, 0, 1]

hierarchy:
  base_footprint:
    base_link:
      - left_wheel
      - right_wheel

joints:
  base_link:
    xyz: [0, 0, 0.05]

  left_wheel:
    type: continuous
    xyz: [0, 0.10, -0.015]
    axis: [0, 1, 0]

  right_wheel:
    type: continuous
    xyz: [0, -0.10, -0.015]
    axis: [0, 1, 0]

links:
  base_footprint: {}

  base_link:
    octagon: [0.15, 0.003]
    material: clear

  left_wheel:
    cylinder: [0.035, 0.02]
    rpy: [1.5708, 0, 0]
    material: black

  right_wheel:
    cylinder: [0.035, 0.02]
    rpy: [1.5708, 0, 0]
    material: black
```

**Result**: 49 lines (vs 80 lines URDF = 39% reduction)

---

## Validation & Warnings

The compiler tracks and warns about:

### Unused Top-Level Sections
```
Warning: Section 'params' not recognized (ignored)
```

### Unused Link Properties
```
Warning: Link 'wheel' has unused properties: ['mass', 'inertia']
```

### Joints Without Hierarchy
```
Warning: joints section for 'sensor' has no matching child in hierarchy
```

---

## Best Practices

### 1. Hierarchy First
Put hierarchy at top so robot structure is immediately visible:

```yaml
robot: my_robot

hierarchy:          # ← Robot structure visible here
  base:
    - wheel1
    - wheel2

joints:             # ← Details below
  ...
```

### 2. Use Comments
YAML comments explain intent:

```yaml
joints:
  base_link:
    xyz: [0, 0, 0.05]    # 5cm above ground
```

### 3. Empty Links for Frames
Use `{}` for reference frames:

```yaml
links:
  base_footprint: {}     # Ground projection
```

### 4. Consistent Units
Always use meters and radians:

```yaml
cylinder: [0.15, 0.05]   # 15cm radius, 5cm height (in meters)
rpy: [1.5708, 0, 0]      # 90° in radians
```

---

## Limitations (Current Implementation)

**Not yet implemented:**
- ❌ Parameters/variables (`params:` section)
- ❌ Templates (`templates:` section)
- ❌ Mirroring operators
- ❌ Degree angle notation (`90deg`)
- ❌ Axis shortcuts (`axis: y`)
- ❌ Dictionary geometry format (`{radius: 0.15, length: 0.05}`)
- ❌ Mesh files
- ❌ Mass/inertia specifications
- ❌ Explicit collision overrides

**See**: [dsl_syntax_futures.md](dsl_syntax_futures.md) for planned features

---

## Conversion to URDF

Use the `durdf` command-line tool:

```bash
durdf input.durdf output.urdf
```

Or programmatically:

```python
from dslurdf import DurdfLoader, UrdfGenerator

loader = DurdfLoader()
generator = UrdfGenerator()

data = loader.load("robot.durdf")
urdf_xml = generator.generate(data)

with open("robot.urdf", "w") as f:
    f.write(urdf_xml)
```

---

## See Also

- [dsl_syntax_futures.md](dsl_syntax_futures.md) - Planned future features
- [design_philosophy.md](design_philosophy.md) - Design principles
- [dsl_approach.md](dsl_approach.md) - Approach comparison
- [../examples/](../examples/) - Example DURDF files

---

**Version**: 0.1.0 (Current Implementation)
**Last Updated**: 2026-01-09
**Status**: Reflects actual working implementation
