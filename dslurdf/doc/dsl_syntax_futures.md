# URDF DSL Future Extensions

Potential syntax extensions and features not yet implemented in the core DSL specification.

**Related:** See [dsl_syntax.md](dsl_syntax.md) for current specification.

---

## Explicit Collision Geometry

```bnf
<collision-spec> ::= "collision:" <geometry-spec>
```

### Current Behavior
Collision geometry is implicitly set equal to visual geometry (smart default).

### Proposed Extension
Allow explicit collision geometry overrides when needed:

```yaml
base_link:
  # Visual geometry
  box: {size: [0.4, 0.3, 0.1]}
  material: gray
  mass: 5.0

  # Override collision with simplified geometry
  collision:
    box: {size: [0.42, 0.32, 0.12]}  # Slightly larger for safety margin
```

### Use Cases
- Simplified collision meshes for performance
- Safety margins around visual geometry
- Different collision shapes for complex visual meshes

---

## Gazebo Simulation Configuration

```bnf
<gazebo-section> ::= "gazebo:" <gazebo-config>

<gazebo-config> ::= {<gazebo-materials> | <gazebo-friction> | <gazebo-sensors> | <gazebo-controller>}

<gazebo-materials> ::= "materials:" {<link-material-def>}

<link-material-def> ::= IDENTIFIER ":" "{" <rgba-properties> "}"

<rgba-properties> ::= "ambient:" <rgba-value> ["," "diffuse:" <rgba-value>] ["," "specular:" <rgba-value>]

<gazebo-friction> ::= "friction:" {<link-friction-def>}

<link-friction-def> ::= IDENTIFIER ":" "{" "mu1:" NUMBER "," "mu2:" NUMBER "}"

<gazebo-sensors> ::= "sensors:" {<sensor-def>}

<sensor-def> ::= IDENTIFIER ":" "{" <sensor-properties> "}"

<gazebo-controller> ::= "controller:" "{" <controller-properties> "}"
```

### Description
Gazebo-specific configuration for simulation physics, sensors, and controllers.

### Example

```yaml
robot: simulation_bot

hierarchy:
  base_link:
    - left_wheel
    - right_wheel
    - laser

# ... links definitions ...

# Gazebo simulation configuration
gazebo:
  # Material appearance for simulation
  materials:
    base_link:
      ambient: [0.8392, 0.6314, 0.0, 1.0]
      diffuse: [0.8392, 0.6314, 0.0, 1.0]
      specular: [0.99, 0.99, 0.99, 1.0]

  # Friction properties
  friction:
    left_wheel: {mu1: 1.0, mu2: 1.0}
    right_wheel: {mu1: 1.0, mu2: 1.0}
    caster: {mu1: 0.01, mu2: 0.01}

  # Sensors
  sensors:
    imu:
      link: imu_link
      type: imu
      update_rate: 20
      plugin:
        name: imu_plugin
        filename: libgazebo_ros_imu_sensor.so
        topic: imu/data

    lidar:
      link: laser
      type: gpu_ray
      update_rate: 10
      range: [0.08, 12.0]
      plugin:
        name: laser_controller
        filename: libgazebo_ros_ray_sensor.so
        topic: scan

  # Differential drive controller
  controller:
    type: differential_drive
    plugin:
      name: diff_drive_controller
      filename: libgazebo_ros_diff_drive.so
      left_joint: left_wheel_joint
      right_joint: right_wheel_joint
      wheel_separation: 0.3
      wheel_diameter: 0.1
```

### Use Cases
- Simulation-specific material properties (ambient, diffuse, specular)
- Friction coefficients for realistic physics
- Sensor plugin configuration (IMU, lidar, cameras)
- Controller configuration (differential drive, joint controllers)

---

## Conditional Logic

```bnf
<conditional> ::= "if:" <condition> "then:" <properties> ["else:" <properties>]

<condition> ::= <boolean-expr>

<boolean-expr> ::= IDENTIFIER "==" <value>
                 | IDENTIFIER "!=" <value>
                 | IDENTIFIER ">" <value>
                 | IDENTIFIER "<" <value>
```

### Description
Enable/disable features based on configuration parameters.

### Example

```yaml
params:
  has_lidar: true
  has_camera: false
  robot_variant: "indoor"

links:
  base_link:
    box: {size: [0.4, 0.3, 0.1]}
    material: gray

  # Conditional sensor inclusion
  if: {has_lidar}
  then:
    laser:
      origin: {xyz: [0.1, 0, 0.3]}
      cylinder: {radius: 0.03, length: 0.04}
      material: red

  if: {has_camera}
  then:
    camera_link:
      origin: {xyz: [0.15, 0, 0.2]}
      box: {size: [0.03, 0.09, 0.03]}
      material: blue

  # Variant-specific configuration
  if: {robot_variant == "outdoor"}
  then:
    mudguards:
      template: mudguard
      instances:
        - {name: front_mudguard, origin: {xyz: [0.2, 0, -0.05]}}
        - {name: rear_mudguard, origin: {xyz: [-0.2, 0, -0.05]}}
```

### Use Cases
- Multi-variant robot configurations
- Optional sensor packages
- Simulation vs. hardware differences
- Feature toggles

---

## Iteration and Loops

```bnf
<loop> ::= "for:" IDENTIFIER "in:" <range> <properties>

<range> ::= "[" NUMBER "," NUMBER ["," NUMBER] "]"    # [start, end, step]
          | "[" <value-list> "]"                      # Explicit list

<value-list> ::= <value> {"," <value>}
```

### Description
Generate repeated structures with parameterized variations.

### Example

```yaml
params:
  leg_count: 6
  leg_angles: [0, 60, 120, 180, 240, 300]  # Degrees

templates:
  leg:
    box: {size: [0.05, 0.02, 0.3]}
    material: black
    joint_type: revolute
    limits: {lower: -45deg, upper: 45deg}

links:
  # Generate hexapod legs with loop
  for: i in [0, 1, 2, 3, 4, 5]
  legs:
    name: leg_{i}
    template: leg
    origin:
      xyz: [0, 0, 0]
      rpy: [0, 0, {leg_angles[i]}deg]

  # Alternative: range-based loop
  for: n in [1, 6, 1]  # start=1, end=6, step=1
  support_posts:
    name: post_{n}
    cylinder: {radius: 0.005, length: 0.1}
    origin: {xyz: [{0.1 * cos(60*n)}, {0.1 * sin(60*n)}, 0]}
```

### Use Cases
- Multi-legged robots (hexapods, quadrupeds)
- Repeated sensor arrays
- Parametric structures
- Modular assemblies

---

## File Inclusion

```bnf
<include> ::= "include:" STRING

<import> ::= "import:" STRING "as:" IDENTIFIER
```

### Description
Compose robots from multiple files and reusable modules.

### Example

**File: `modules/mobile_base.yaml`**
```yaml
# Reusable mobile base module
params:
  base_width: 0.3
  wheel_radius: 0.05

templates:
  drive_wheel:
    cylinder: {radius: {wheel_radius}, length: 0.02}
    material: rubber
    joint_type: continuous

links:
  base_link:
    box: {size: [0.4, {base_width}, 0.1]}
    material: aluminum
```

**File: `modules/sensor_suite.yaml`**
```yaml
# Sensor package module
links:
  laser:
    cylinder: {radius: 0.03, length: 0.04}
    material: red

  camera_link:
    box: {size: [0.03, 0.09, 0.03]}
    material: blue
```

**Main Robot File: `my_robot.yaml`**
```yaml
robot: custom_robot

# Import modules
include: modules/mobile_base.yaml
include: modules/sensor_suite.yaml

# Or with namespacing
import: modules/mobile_base.yaml as base
import: modules/sensor_suite.yaml as sensors

hierarchy:
  base_footprint:
    - base.base_link:
        - sensors.laser
        - sensors.camera_link
        - left_wheel
        - right_wheel

# Override or extend imported definitions
params:
  base.base_width: 0.4  # Override imported parameter

links:
  wheels:
    template: base.drive_wheel
    mirror_y:
      origin: {xyz: [0, {base.base_width/2}, 0]}
      names: [left_wheel, right_wheel]
```

### Use Cases
- Modular robot composition
- Shared component libraries
- Team collaboration
- Version control of robot modules
- Standard sensor/actuator packages

---

## Advanced Mirroring

```bnf
<mirror-spec> ::= "mirror_x:" <mirror-config>
                | "mirror_y:" <mirror-config>
                | "mirror_z:" <mirror-config>
                | "mirror_plane:" <plane-spec>

<plane-spec> ::= "{" "normal:" <vector-expr> "," "point:" <vector-expr> "}"
```

### Description
Extend mirroring beyond Y-axis to support arbitrary symmetry planes.

### Example

```yaml
templates:
  arm_segment:
    box: {size: [0.1, 0.05, 0.3]}
    material: silver

links:
  # Mirror across X-axis
  arms_x:
    template: arm_segment
    mirror_x:
      origin: {xyz: [0.2, 0, 0]}
      names: [front_arm, rear_arm]

  # Mirror across arbitrary plane
  arms_diagonal:
    template: arm_segment
    mirror_plane:
      normal: [1, 1, 0]  # 45-degree diagonal
      point: [0, 0, 0]
      names: [arm_a, arm_b]
```

---

## Type Annotations and Validation

```bnf
<type-annotation> ::= "type:" <robot-type>

<robot-type> ::= "mobile_platform" <platform-config>
               | "manipulator" <manipulator-config>
               | "humanoid" <humanoid-config>
               | "legged" <legged-config>

<platform-config> ::= "{" "drive_type:" <drive-type> "," "wheels:" NUMBER "}"

<drive-type> ::= "differential" | "ackermann" | "mecanum" | "skid_steer"
```

### Description
Optional type declarations for validation and smart defaults (originally considered, may be reconsidered).

### Example

```yaml
robot: validated_bot

type: mobile_platform
  drive_type: differential
  wheels: 2
  sensors: {imu: 1, lidar: 1}

# Compiler validates:
# - Exactly 2 wheels with continuous joints exist
# - Required frames: base_link, base_footprint
# - Sensor links match declared count
```

---

## Semantic Robot Archetypes

```bnf
<archetype-spec> ::= "archetype:" <archetype-type> [<archetype-config>]

<archetype-type> ::= "differential_drive_base"
                   | "stacked_plates"
                   | "hexapod_base"
                   | "sensor_tower"
                   | "quadruped_base"

<archetype-config> ::= "{" <archetype-properties> "}"

<archetype-properties> ::= {<semantic-param> | <override-spec>}

<semantic-param> ::= "wheel_base:" <expression>
                   | "wheel_ground_clearance:" <expression>
                   | "plate_spacing:" <expression>
                   | "plate_count:" NUMBER
                   | "leg_count:" NUMBER
                   | "mounting_height:" <expression>
```

### Description

Semantic archetypes provide higher-level abstractions for common robot structural patterns. Instead of explicitly defining every component, archetypes recognize semantic parameter names (like `wheel_base`, `wheel_ground_clearance`, `plate_spacing`) and automatically generate appropriate structures, mounting hardware, and positioning.

This is a **Tier 4 enhancement** - an optional layer on top of the explicit DSL that provides:
- Auto-generation of common structural patterns
- Semantic understanding of robot characteristics
- Automatic mounting superstructure for sensors
- Convention-based positioning and spacing

### Key Principles

1. **Optional and Layered**: Archetypes are syntactic sugar over the explicit DSL
2. **Override Capability**: Any auto-generated component can be manually overridden
3. **Escape Hatches**: Always allow fallback to explicit specification
4. **~80% Coverage**: Designed for common patterns, not edge cases

### Example 1: Differential Drive Base with Stacked Plates

```yaml
robot: mobile_robot

# High-level archetype declaration
archetype: differential_drive_base
  chassis: stacked_plates
  plate_count: 2
  plate_spacing: 0.060
  plate_diameter: 0.280
  wheel_base: 0.215
  wheel_diameter: 0.0625
  wheel_ground_clearance: 0.005

# Compiler auto-generates:
# - base_footprint (ground projection)
# - base_link (robot center)
# - bottom_plate, top_plate (with correct Z offsets)
# - support_posts (connecting plates at 120° intervals)
# - left_wheel, right_wheel (positioned at ±wheel_base/2, ground contact)
# - Appropriate joints (fixed for structure, continuous for wheels)

hierarchy:
  base_footprint:
    - base_link:
        - bottom_plate
        - top_plate
        - support_post_1
        - support_post_2
        - support_post_3
        - left_wheel
        - right_wheel

# Custom sensors with auto-mounted superstructure
links:
  laser:
    origin: [0, 0, 0.150]  # Place at desired height
    cylinder: [0.03, 0.02]
    material: red
    # Compiler auto-generates mounting tower from top_plate to laser

  imu_link:
    origin: [-0.040, 0, {bottom_plate_height}]  # Mount on bottom plate
    box: [0.020, 0.020, 0.010]
    material: orange
    # No mounting needed - directly attached to bottom_plate
```

**What the archetype generates automatically:**
- Plate geometries at correct Z heights
- Support posts at 120° intervals connecting plates
- Wheels positioned at ±wheel_base/2 with ground contact (Z calculated from wheel_diameter and clearance)
- Mounting tower for laser (connects top_plate to sensor)
- Appropriate materials and masses based on standard densities

### Example 2: Hexapod Base with Repeated Leg Pattern

```yaml
robot: hexapod

archetype: hexapod_base
  leg_count: 6
  body_radius: 0.15
  body_height: 0.05
  leg_length: 0.25
  leg_spread_angle: 60deg  # Angle between adjacent legs

# Compiler auto-generates:
# - Hexagonal body (or specified geometry)
# - 6 leg mount points at 60° intervals
# - Leg coordinate frames with correct rotations
# - Hip joint attachment points

hierarchy:
  base_link:
    - leg_0
    - leg_1
    - leg_2
    - leg_3
    - leg_4
    - leg_5

templates:
  leg_segment:
    box: [0.05, 0.02, {leg_length}]
    material: black
    joint_type: revolute
    limits: {lower: -45deg, upper: 45deg}

links:
  # Archetype auto-positions leg mounts
  legs:
    template: leg_segment
    # Mounting positions inferred from leg_count and body_radius
    # Rotations auto-calculated at 0°, 60°, 120°, 180°, 240°, 300°
```

### Example 3: Sensor Tower with Auto-Generated Mounting

```yaml
robot: sensor_platform

archetype: differential_drive_base
  chassis: stacked_plates
  plate_count: 2

links:
  # Sensor placement with semantic mounting
  lidar:
    mount_to: top_plate
    mounting_height: 0.08  # Height above top_plate
    origin: [-0.04, 0, auto]  # Z auto-calculated
    cylinder: [0.03, 0.02]
    material: red
    # Compiler generates:
    # - Mounting tower (cylinder connecting top_plate to sensor)
    # - Appropriate Z coordinate for sensor origin

  camera:
    mount_to: lidar
    mounting_offset: [0.05, 0, 0.03]  # Relative to lidar
    box: [0.03, 0.09, 0.03]
    material: blue
```

**What happens:**
1. `mount_to: top_plate` + `mounting_height: 0.08` → creates a mounting tower link
2. `origin: [-0.04, 0, auto]` → Z is calculated as `top_plate.z + mounting_height + tower_height/2`
3. Tower geometry automatically sized: radius from sensor footprint, height = mounting_height

### Recognized Semantic Parameters

**Differential Drive:**
- `wheel_base` → Auto-position wheels at ±wheel_base/2 on Y-axis
- `wheel_ground_clearance` → Auto-calculate wheel Z for ground contact
- `wheel_diameter`, `wheel_width` → Wheel geometry
- `caster_offset` → Auto-position front/rear casters

**Stacked Plates:**
- `plate_count` → Number of horizontal plates
- `plate_spacing` → Vertical distance between plates
- `plate_diameter` or `plate_size` → Plate geometry
- `support_post_count` → Auto-generate support posts (default: 3)

**Legged Robots:**
- `leg_count` → Number of legs
- `leg_spread_angle` → Angular spacing
- `body_radius` → Radial distance for leg mounting
- `leg_length` → Individual leg segment length

**Sensor Mounting:**
- `mount_to: <link_name>` → Attach sensor to specified link
- `mounting_height` → Height of mounting structure
- `mounting_offset` → Relative position offset

### Override and Escape Hatches

Any auto-generated component can be explicitly overridden:

```yaml
archetype: differential_drive_base
  wheel_base: 0.215

# Override auto-generated wheels with custom design
links:
  left_wheel:
    # Custom wheel design replaces archetype default
    cylinder: [0.04, 0.03]  # Different size
    material: rubber        # Different material
    origin: [0.05, 0.1075, -0.03]  # Custom positioning
    joint_type: continuous
    axis: y

  # right_wheel still uses archetype default
```

### What Works Well (Strengths)

1. **Differential drive mobile bases** - Well-defined patterns (TurtleBot, ROSbot, etc.)
2. **Stacked plate chassis** - Common structural pattern
3. **Legged robots** - Regular geometric patterns (hexapods, quadrupeds)
4. **Sensor towers** - Repeating mounting structures
5. **Support structures** - Posts, brackets, mounting plates

**Coverage: ~80% of educational and research robots**

### What Doesn't Work Well (Limitations)

1. **Custom geometries** - Unique shapes require explicit specification
2. **Complex manipulators** - Irregular kinematic chains
3. **Irregular structures** - Non-standard layouts
4. **Artistic designs** - Creative/aesthetic robot designs

**For these cases:** Use explicit DSL syntax or override archetype components

### Use Cases

- **Rapid prototyping** - Quick robot variants for testing
- **Educational robots** - Standard platforms (TurtleBot-like, hexapods)
- **Multi-variant fleets** - Generate similar robots with different parameters
- **Sensor configurations** - Easy swap of sensor packages
- **Modular platforms** - Mix archetype bases with custom payloads

### Implementation Notes

Archetypes are implemented as:
1. **Macro expansion** - Archetype declarations expand to explicit DSL at compile time
2. **Parameter validation** - Semantic parameters are checked for consistency
3. **Conflict resolution** - Explicit specifications always override archetype defaults
4. **Warnings** - Compiler warns about unused archetype parameters

### Relationship to Tier 3 Semantic Recognition

Tier 3 focused on recognizing semantic parameters in explicit syntax:
```yaml
# Tier 3: Recognize wheel_base and auto-infer mirror positioning
wheels:
  template: wheel
  mirror_y: ground  # Infers Y from wheel_base, Z from ground contact
```

Tier 4 archetypes go further - generate entire structures:
```yaml
# Tier 4: Generate complete robot from archetype
archetype: differential_drive_base
  wheel_base: 0.215
# → Generates base_link, plates, wheels, support posts, joints
```

**Both layers complement each other** - Tier 3 simplifies explicit syntax, Tier 4 provides structural templates.

---

## Notes

These extensions are **not part of the core specification** but represent potential future directions based on:
- Community feedback
- Common use cases
- Advanced robotic system requirements
- Lessons learned from implementation

**Future Extensions Summary:**
1. **Explicit Collision Geometry** - Override visual geometry for collision detection
2. **Gazebo Simulation Configuration** - Simulation-specific materials, friction, sensors, controllers
3. **Conditional Logic** - Enable/disable features based on parameters
4. **Iteration and Loops** - Generate repeated structures (hexapod legs, sensor arrays)
5. **File Inclusion** - Modular robot composition from reusable component libraries
6. **Advanced Mirroring** - X-axis, Z-axis, and arbitrary plane mirroring
7. **Type Annotations** - Optional validation and smart defaults based on robot type
8. **Semantic Robot Archetypes** (Tier 4) - High-level abstractions for common patterns (differential drive, stacked plates, hexapods, sensor towers)

Priority for implementation should be based on user demand and real-world needs.

---

**Version:** 1.0 (Future Extensions)
**Last Updated:** 2025-12-20
**Status:** Proposed, not implemented
