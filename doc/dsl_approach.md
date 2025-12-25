# Semantic YAML DSL Approach

The Semantic YAML DSL is a domain-specific language designed to achieve maximum conciseness (60% reduction) through templates, smart defaults, and semantic shortcuts.

---

## Overview

**Goal**: Create a YAML-based language that compiles to URDF while being dramatically more concise and readable.

**Key Achievement**: 47-60% code reduction through:
- Flat syntax (geometry as top-level fields)
- Explicit hierarchy section
- Smart defaults (collision=visual, joints=fixed, etc.)
- Templates and mirroring for symmetric components
- Variable substitution with expressions

---

## Design Principles

### 1. Flat Syntax
Geometry types as top-level fields, not nested XML blocks:

```yaml
# DSL (flat)
base_link:
  cylinder: {radius: 0.14, length: 0.003}
  material: yellow
  mass: 1.0

# vs URDF (nested)
<link name="base_link">
  <visual>
    <geometry>
      <cylinder radius="0.14" length="0.003"/>
    </geometry>
    <material name="yellow"/>
  </visual>
  <inertial>
    <mass value="1.0"/>
  </inertial>
</link>
```

### 2. Hierarchy First
TF tree structure visible at the top of the file:

```yaml
robot: my_robot

hierarchy:
  base_footprint:
    - base_link:
        - left_wheel
        - right_wheel
        - lidar
```

This shows the entire robot structure at a glance, separate from geometry details.

### 3. Smart Defaults
Eliminate boilerplate through sensible defaults:

- **Collision = Visual**: Unless explicitly overridden
- **Inertia**: Auto-calculated from geometry + mass
- **Origin**: Defaults to `[0, 0, 0]`
- **Joint Type**: Defaults to `fixed`
- **Rotation**: Defaults to `[0, 0, 0]`

```yaml
# Only specify what's different from defaults
sensor:
  cylinder: {radius: 0.03, length: 0.04}
  # Implicit: fixed joint, origin [0,0,0], collision=visual
```

### 4. Templates & Mirroring
Eliminate repetition for symmetric components:

```yaml
templates:
  wheel:
    cylinder: [{wheel_radius}, {wheel_width}]
    material: black
    joint_type: continuous
    axis: [0, 1, 0]

# Use template with mirroring
wheels:
  template: wheel
  mirror_y:
    names: [left_wheel, right_wheel]
    origin: {xyz: [0, {wheel_base/2}, {-wheel_radius}]}
```

This creates both wheels automatically with mirrored Y positions.

### 5. Variable Substitution
Parameters defined once, reused with arithmetic:

```yaml
params:
  wheel_base: 0.24
  wheel_radius: 0.05

# Later, use with expressions
origin: {xyz: [0, {wheel_base/2}, {-wheel_radius}]}
```

Supports: `+`, `-`, `*`, `/`, parentheses for grouping.

### 6. Named Material Library
Predefined semantic names:

```yaml
# Instead of rgba="0.5 0.5 0.5 1.0"
material: gray

# Instead of rgba="0.7 0.7 0.7 1.0"
material: steel
```

Library includes: red, blue, green, yellow, orange, black, white, gray, steel, aluminum, coral, plum, charcoal, gold, etc.

---

## Syntax Specification

### File Structure

```yaml
robot: <name>

hierarchy:
  <root_link>:
    - <child_link>:
        - <grandchild_link>

params:
  <param_name>: <value>

templates:
  <template_name>:
    <link_properties>

links:
  <link_name>:
    <geometry>
    <properties>
```

### Geometry Specifications

```yaml
# Cylinder
cylinder: {radius: <r>, length: <l>}
cylinder: [<r>, <l>]  # Shorthand

# Box
box: {size: [<x>, <y>, <z>]}
box: [<x>, <y>, <z>]  # Shorthand

# Sphere
sphere: {radius: <r>}
sphere: <r>  # Shorthand

# Mesh
mesh: {filename: <path>, scale: [<x>, <y>, <z>]}

# Polygon primitives
hexagon: {radius: <r>, length: <l>}
octagon: {radius: <r>, length: <l>}
polygon: {sides: <n>, radius: <r>, length: <l>}
```

### Joint Types

```yaml
# Fixed (default, can be omitted)
joint_type: fixed

# Continuous (for wheels)
joint_type: continuous
axis: [0, 1, 0]

# Revolute (with limits)
joint_type: revolute
axis: [0, 1, 0]
limits: {lower: -1.57, upper: 1.57}
```

### Origins and Transforms

```yaml
# Flat syntax - position and rotation as top-level fields
top_link:
  cylinder: [0.05, 0.2]
  xyz: [0, 0, 0.1]
  rpy: [1.57, 0, 0]

# Position only
sensor_link:
  box: [0.1, 0.1, 0.05]
  xyz: [0.2, 0, 0.15]

# Rotation only
rotated_link:
  cylinder: [0.03, 0.1]
  rpy: [0, 1.57, 0]
```

### Mirroring

```yaml
# Y-axis mirroring (left/right)
mirror_y:
  names: [left_wheel, right_wheel]
  origin: {xyz: [0, {wheel_base/2}, 0]}

# Creates two instances with Y positions at ±wheel_base/2
```

---

## Example: Simple Robot

See [output/dsl/simple_robot.yaml](../output/dsl/simple_robot.yaml) for complete example.

**Line count comparison:**
- Original URDF: 86 lines
- DSL version: 46 lines (47% reduction)

**Key features demonstrated:**
- Hierarchy section shows structure
- Parameters defined and reused
- Template for wheels with mirroring
- Smart defaults eliminate boilerplate

---

## Formal Grammar (BNF)

Complete BNF grammar specification is available in the archived documentation: `dslurdf/doc/dsl_syntax.md` (363 lines).

Key production rules:
```bnf
<robot-spec> ::= <robot-decl> <hierarchy-section>
                 [<params-section>] [<templates-section>] <links-section>

<geometry-spec> ::= <cylinder-spec> | <box-spec> | <sphere-spec>
                  | <mesh-spec> | <polygon-spec>

<expression> ::= <term> | <expression> "+" <term> | <expression> "-" <term>

<term> ::= <factor> | <term> "*" <factor> | <term> "/" <factor>

<factor> ::= NUMBER | "{" IDENTIFIER "}" | "(" <expression> ")"
```

---

## Future Extensions

Potential additions not in core specification (see `dslurdf/doc/dsl_syntax_futures.md`):

### Explicit Collision Override
```yaml
base_link:
  box: [0.4, 0.3, 0.1]  # Visual
  collision:
    box: [0.42, 0.32, 0.12]  # Larger for safety margin
```

### Gazebo Simulation Config
```yaml
gazebo:
  materials:
    base_link: {mu1: 0.5, mu2: 0.5}
  sensors:
    lidar: {update_rate: 10, ray_count: 360}
```

### Conditional Logic
```yaml
if: {has_lidar}
then:
  lidar_link:
    cylinder: [0.03, 0.04]
```

### Advanced Mirroring
```yaml
mirror_x: ...    # Front/back
mirror_z: ...    # Top/bottom
mirror_plane: {normal: [1, 1, 0]}  # Arbitrary plane
```

---

## Comparison Results

### Dome1 Robot
- **Original**: 326 lines URDF/Xacro
- **DSL**: ~130 lines
- **Reduction**: 60%

### Simple Robot
- **Original**: 86 lines URDF
- **DSL**: 46 lines
- **Reduction**: 47%

### Benefits Beyond Line Count
1. **Readability**: Hierarchy immediately visible
2. **Maintainability**: Change parameters in one place
3. **Teachability**: Clearer structure for learning
4. **Consistency**: Templates enforce uniform patterns

---

## Implementation Status

**Completed:**
- ✅ Syntax specification finalized
- ✅ BNF grammar documented
- ✅ Example robots created
- ✅ Design principles documented

**Next Steps:**
1. Implement YAML→URDF compiler
2. Add schema validation
3. Implement template expansion
4. Add expression evaluator
5. Generate proper URDF XML output

---

## References

- **Current Project State**: [current.md](current.md)
- **Design Philosophy**: [design_philosophy.md](design_philosophy.md)
- **Full Grammar**: `dslurdf/doc/dsl_syntax.md`
- **Future Extensions**: `dslurdf/doc/dsl_syntax_futures.md`
- **Example Robot**: [output/dsl/simple_robot.yaml](../output/dsl/simple_robot.yaml)
