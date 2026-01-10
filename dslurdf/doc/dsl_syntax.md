# DURDF Syntax Specification

**DURDF** (Domain Specific Language for URDF) is a YAML-based DSL for ROS2 robot descriptions.

**Status**: This document reflects the **current implementation** (v0.1.0)
**See also**: [dsl_syntax_futures.md](dsl_syntax_futures.md) for planned features

---

## Overview

DURDF achieves 30-60% code reduction through:
- **Flat syntax**: Geometry as top-level properties
- **Smart defaults**: Auto-collision, fixed joints
- **Hierarchy-first**: TF tree visible at top of file
- **Array shorthand**: Concise geometry definitions
- **Constants**: Reusable named values with `$variable_name` syntax
- **Degree notation**: Intuitive `90deg` instead of radians

---

## File Structure

```yaml
robot: <name>                # Required: Robot name

constants:                   # Optional: Named constants for reuse
  <name>: <value>

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

## 2. Constants Section (Optional)

Define named constants that can be reused throughout the file:

```yaml
constants:
  base_radius: 0.15
  wheel_radius: 0.030
  wheel_width: 0.0175
  wheel_spacing: 0.125
  ninety_deg: 1.5708
```

**Usage:**

Reference constants using `$variable_name` syntax:

```yaml
links:
  base_link:
    cylinder: [$base_radius, 0.003]

  left_wheel:
    cylinder: [$wheel_radius, $wheel_width]
    rpy: [$ninety_deg, 0, 0]

joints:
  left_wheel:
    xyz: [0, $wheel_spacing, 0]
```

**Rules:**
- Constants are simple name-value pairs
- Values are typically numeric (dimensions, angles, etc.)
- Reference with `$name` anywhere in materials, hierarchy, joints, or links sections
- Substitution happens at load time (before URDF generation)
- Unknown constant references will raise an error

**Benefits:**
- Change dimensions in one place
- Self-documenting code with meaningful names
- Consistent values across related components

---

## 3. Materials Section (Optional)

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

## 4. Hierarchy Section (Required)

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

## 5. Joints Section (Optional)

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

## 6. Links Section (Required)

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

#### Mesh
```yaml
# Simple format with absolute path
mesh: "file:///absolute/path/to/model.obj"

# Or with ROS package URI (recommended)
mesh: "package://robot_name/meshes/base.stl"

# Dictionary format (with optional scale)
mesh:
  filename: "file:///absolute/path/to/model.dae"
  scale: [1, 1, 1]
```
- `filename`: Path to mesh file (typically .obj, .dae, or .stl)
  - **IMPORTANT**: Use absolute paths or ROS package URIs - relative paths won't work in visualizers
  - ROS package URI format: `package://package_name/path/to/mesh.stl`
  - Absolute path format: `file:///absolute/path/to/mesh.obj`
  - Relative paths like `"cube.obj"` will fail in RViz/Gazebo
- `scale`: Optional scaling factor [x, y, z] (defaults to [1, 1, 1] if not specified)
- Supported formats:
  - **OBJ** (with .mtl for colors) - Best for Autodesk Fusion exports
  - **DAE** (Collada with embedded materials) - Best for ROS/colored meshes
  - **STL** (geometry only, no colors)

**Format Options:**
1. **Simple string**: Just the filename (scale defaults to [1, 1, 1])
2. **Dictionary**: Filename with optional scale parameter

**Note on materials**:
- Mesh geometries use colors/materials defined in the mesh file itself
- For OBJ files: The `.mtl` file must be in the same directory as the `.obj` file
- For DAE files: Materials are embedded in the file
- Any `material:` property specified in DURDF for a mesh link will be ignored
- To see colors in visualizers, always use absolute paths or package URIs

### Link Properties

| Property | Type | Required | Description |
|----------|------|----------|-------------|
| Geometry | See above | No* | One of: cylinder, box, sphere, octagon, hexagon, mesh |
| `material` | string | No | Material name (from materials section) |
| `rpy` | `[r, p, y]` | No | Visual geometry rotation in radians |

\* Geometry can be omitted for reference frames (e.g., `base_footprint: {}`)

### Visual Rotation (rpy)

Rotates the visual (and collision) geometry:

```yaml
left_wheel:
  cylinder: [0.035, 0.02]
  rpy: [90deg, 0, 0]         # Rotate 90° around X-axis
```

**Degree notation:**
Use the `deg` suffix for intuitive angle specification:
- `90deg` = 90 degrees (π/2 radians)
- `180deg` = 180 degrees (π radians)
- `45.5deg` = 45.5 degrees
- `-90deg` = -90 degrees

You can also use radians directly:
```yaml
rpy: [1.5708, 0, 0]          # Same as [90deg, 0, 0]
```

---

## 7. Smart Defaults

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

constants:
  base_height: 0.05
  base_radius: 0.15
  base_thickness: 0.003
  wheel_radius: 0.035
  wheel_width: 0.02
  wheel_spacing: 0.10
  wheel_drop: -0.015

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
    xyz: [0, 0, $base_height]

  left_wheel:
    type: continuous
    xyz: [0, $wheel_spacing, $wheel_drop]
    axis: [0, 1, 0]

  right_wheel:
    type: continuous
    xyz: [0, -$wheel_spacing, $wheel_drop]
    axis: [0, 1, 0]

links:
  base_footprint: {}

  base_link:
    octagon: [$base_radius, $base_thickness]
    material: clear

  left_wheel:
    cylinder: [$wheel_radius, $wheel_width]
    rpy: [90deg, 0, 0]
    material: black

  right_wheel:
    cylinder: [$wheel_radius, $wheel_width]
    rpy: [90deg, 0, 0]
    material: black
```

**Result**: 53 lines (vs 80 lines URDF = 34% reduction)
**Note**: Degree notation and constants improve readability

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

### 1. Use Constants for Dimensions
Define dimensions once, reuse with meaningful names:

```yaml
constants:
  wheel_radius: 0.035
  wheel_spacing: 0.10

links:
  left_wheel:
    cylinder: [$wheel_radius, 0.02]

joints:
  left_wheel:
    xyz: [0, $wheel_spacing, 0]
```

### 2. Hierarchy First
Put hierarchy at top so robot structure is immediately visible:

```yaml
robot: my_robot

constants:          # ← Dimensions defined once
  ...

hierarchy:          # ← Robot structure visible here
  base:
    - wheel1
    - wheel2

joints:             # ← Details below
  ...
```

### 3. Use Comments
YAML comments explain intent:

```yaml
joints:
  base_link:
    xyz: [0, 0, 0.05]    # 5cm above ground
```

### 4. Empty Links for Frames
Use `{}` for reference frames:

```yaml
links:
  base_footprint: {}     # Ground projection
```

### 5. Consistent Units
Always use meters for distances. For angles, prefer degree notation:

```yaml
cylinder: [0.15, 0.05]   # 15cm radius, 5cm height (in meters)
rpy: [90deg, 0, 0]       # 90° - intuitive and readable
```

---

## Limitations (Current Implementation)

**Not yet implemented:**
- ❌ Templates (`templates:` section)
- ❌ Mirroring operators
- ❌ Axis shortcuts (`axis: y`)
- ❌ Dictionary geometry format (`{radius: 0.15, length: 0.05}`) for primitive shapes
- ❌ Mass/inertia specifications
- ❌ Explicit collision overrides
- ❌ Constant expressions (e.g., `$width * 2`) - only simple substitution is supported

**Implemented:**
- ✅ Constants with `$variable_name` substitution
- ✅ Degree notation (`90deg`, `180deg`, etc.)
- ✅ Mesh files (with optional scale parameter)

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
