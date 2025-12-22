# Dome1 Robot - PyURDF vs URDF Comparison

## Overview

The Dome1 robot is a two-tier platform robot with sensors:
- 2 circular plates (bottom and top) at different heights
- 3 support posts connecting the plates
- 2 drive wheels (differential drive)
- Laser sensor with support tower
- IMU sensor
- Front directional marker
- Total: 12 links, 11 joints

## File Comparison

### Original URDF/Xacro Approach

**Files:**
- `dome1.urdf.xacro` - 326 lines

**Structure:**
```xml
<!-- 60+ property declarations -->
<xacro:property name="plate_diameter" value="0.280" />
<xacro:property name="wheel_base" value="0.215" />
...

<!-- 15 material definitions -->
<material name="coral">
  <color rgba="0.91 0.45 0.38 1.0"/>
</material>
...

<!-- Links and joints -->
<link name="base_footprint">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <cylinder radius="${plate_diameter/2}" length="${plate_thickness}"/>
    </geometry>
    <material name="plum"/>
  </visual>
</link>

<!-- Macros -->
<xacro:macro name="support_post" params="name x_pos y_pos">
  <link name="${name}">
    <visual>...</visual>
    <collision>...</collision>
  </link>
  <joint name="base_link_to_${name}" type="fixed">
    <parent link="base_link"/>
    <child link="${name}"/>
    <origin xyz="${x_pos} ${y_pos} ${(top_plate_height - bottom_plate_height)/2}" rpy="0 0 0"/>
  </joint>
</xacro:macro>

<xacro:support_post name="support_post_1" x_pos="${support_post_1_x}" y_pos="${support_post_1_y}"/>
<xacro:support_post name="support_post_2" x_pos="${support_post_2_x}" y_pos="${support_post_y_spread}"/>
<xacro:support_post name="support_post_3" x_pos="${support_post_2_x}" y_pos="${-support_post_y_spread}"/>
```

### PyURDF Approach

**Files:**
1. `dome1_params.py` - 80 lines (all parameters including materials)
2. `dome1_build.py` - 88 lines (build script)
3. **Total: 168 lines**

**Structure:**
```python
# dome1_params.py
class Dome1Params:
    plate_diameter = 0.280
    wheel_base = 0.215
    # ... organized groups

    support_posts = [
        ("support_post_1", 0.1, 0.0),
        ("support_post_2", -0.1, 0.070),
        ("support_post_3", -0.1, -0.070),
    ]

    materials = {
        'coral': [0.91, 0.45, 0.38, 1.0],
        # ... more materials
    }

# dome1_build.py
import urdfbuilder as ub
from dome1_params import Dome1Params as p

robot = ub.urdf("dome_x1")

# Base footprint - visible plate on ground
base_footprint = (robot.link("base_footprint")
                  .cylinder(p.plate_diameter/2, p.plate_thickness, material="plum"))

# Base link - sphere marker
base_link = (base_footprint.link("base_link")
             .sphere(p.base_link_marker_radius, material="gold")
             .joint_origin([-p.base_link_x_offset, 0, p.bottom_plate_height]))

# Plates
bottom_plate = (base_link.link("bottom_plate")
                .cylinder(p.plate_diameter/2, p.plate_thickness, material="coral")
                .joint_origin([-p.base_link_x_offset, 0, 0]))

top_plate = (base_link.link("top_plate")
             .cylinder(p.plate_diameter/2, p.plate_thickness, material="charcoal")
             .joint_origin([-p.base_link_x_offset, 0, z_between_plates]))

# Support posts - Python loop instead of macro
for name, x_pos, y_pos in p.support_posts:
    post = (base_link.link(name)
            .cylinder(p.support_post_diameter/2, p.support_post_height,
                     material="mustard")
            .joint_origin([x_pos, y_pos, z_between_plates/2])
            .collision(geometry=("cylinder", p.support_post_diameter/2,
                                           p.support_post_diameter)))

# Wheels - Python loop
for side, reflect in [("left", 1), ("right", -1)]:
    y_pos = reflect * p.wheel_base/2
    wheel = (base_link.link(f"{side}_wheel")
             .cylinder(p.wheel_diameter/2, p.wheel_width, material="steel")
             .origin([0, 0, 0], [math.pi/2, 0, 0])
             .continuous_joint(axis="y")
             .joint_origin([p.wheel_x_offset, y_pos, z_pos]))

# Sensors
laser = (base_link.link("laser")
         .cylinder(p.laser_radius, p.laser_height, material="red")
         .origin([0, 0, 0], [0, 0, math.pi/2])
         .joint_origin([p.laser_x_offset, 0, p.laser_z_offset],
                      [0, 0, -math.pi/2]))

imu_link = (base_link.link("imu_link")
            .box([p.imu_length, p.imu_width, p.imu_height], material="orange")
            .joint_origin([p.imu_x_offset, 0, p.imu_z_offset]))
```

## Key Differences

### 1. Line Count
- **URDF/Xacro**: 326 lines
- **PyURDF**: 168 lines
- **Reduction**: 48% fewer lines

### 2. Material Definitions
- **URDF/Xacro**: 15 separate `<material>` blocks (~60 lines)
- **PyURDF**: Single dictionary in params file (~17 lines)
- Materials referenced by name string in both approaches

### 3. Repeated Components
- **URDF/Xacro**: Macros with parameter expansion
  ```xml
  <xacro:macro name="support_post" params="name x_pos y_pos">
    ... 20 lines ...
  </xacro:macro>
  <xacro:support_post name="support_post_1" x_pos="..." y_pos="..."/>
  ```

- **PyURDF**: Python loops with data structures
  ```python
  support_posts = [("support_post_1", 0.1, 0.0), ...]

  for name, x_pos, y_pos in p.support_posts:
      post = base_link.link(name).cylinder(...).joint_origin([x_pos, y_pos, z])
  ```

### 4. Complex Positioning
- **URDF/Xacro**: Expression in XML attributes
  ```xml
  <origin xyz="${x_pos} ${y_pos} ${(top_plate_height - bottom_plate_height)/2}" .../>
  ```

- **PyURDF**: Natural Python expressions
  ```python
  z_between_plates = p.top_plate_height - p.bottom_plate_height
  .joint_origin([x_pos, y_pos, z_between_plates/2])
  ```

### 5. Geometry Types Used
- **Cylinder**: Plates, posts, wheels, laser, laser tower
- **Sphere**: Base link marker
- **Box**: IMU sensor, front marker

All geometry types supported in both approaches.

## Code Patterns Demonstrated

### Pattern 1: Data-Driven Component Creation
```python
# Define data
support_posts = [
    ("support_post_1", 0.1, 0.0),
    ("support_post_2", -0.1, 0.070),
    ("support_post_3", -0.1, -0.070),
]

# Create components from data
for name, x, y in support_posts:
    post = base_link.link(name).cylinder(...).joint_origin([x, y, z])
```

### Pattern 2: Hierarchical Assembly
```python
# Clear parent-child relationships through chaining
base_footprint = robot.link("base_footprint").cylinder(...)
base_link = base_footprint.link("base_link").sphere(...)
bottom_plate = base_link.link("bottom_plate").cylinder(...)
top_plate = base_link.link("top_plate").cylinder(...)
laser_tower = top_plate.link("laser_tower").cylinder(...)
```

### Pattern 3: Calculated Positions
```python
# Python variables for intermediate calculations
z_between_plates = p.top_plate_height - p.bottom_plate_height
z_pos = -p.wheel_diameter/2 - p.wheel_ground_clearance

# Use in positioning
.joint_origin([x, y, z_between_plates/2])
```

### Pattern 4: Multiple Origins (Visual + Joint)
```python
# Laser with rotation in visual and joint
laser = (base_link.link("laser")
         .cylinder(p.laser_radius, p.laser_height, material="red")
         .origin([0, 0, 0], [0, 0, math.pi/2])         # Visual rotation
         .joint_origin([p.laser_x_offset, 0, p.laser_z_offset],
                      [0, 0, -math.pi/2]))              # Joint rotation
```

## Benefits Summary

✅ **Much less code** - 48% reduction (326 → 168 lines)
✅ **Cleaner data structures** - Lists/dicts vs macro invocations
✅ **Better material handling** - Dictionary vs 15 separate blocks
✅ **Natural expressions** - Python math vs `${}` syntax
✅ **IDE support** - Autocomplete, refactoring, navigation
✅ **Easier to extend** - Add posts/sensors by adding to list
✅ **More readable** - Clear hierarchy through indentation

## Complexity Comparison

**URDF/Xacro:**
- 60+ `<xacro:property>` declarations
- 15 `<material>` definitions
- 2 `<xacro:macro>` definitions
- Multiple `<xacro:macro>` invocations
- Expression syntax: `${variable}`, `${expr + expr}`

**PyURDF:**
- 1 parameter class with grouped attributes
- 1 material dictionary
- Standard Python loops (for name, x, y in list)
- Standard Python expressions

## Implementation Notes

The `dome1_build.py` demonstrates the intended API with a simpler robot than Floora. Key features:
- Multiple geometry types (cylinder, sphere, box)
- Data-driven component generation (support posts list)
- Hierarchical assembly (laser_tower child of top_plate)
- Complex positioning with intermediate variables
- Multiple origins (visual rotation + joint rotation)
