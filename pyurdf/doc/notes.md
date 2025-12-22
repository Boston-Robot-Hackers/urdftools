## PyURDF Package Structure

```
pyurdf/
├── __init__.py           # Package initialization
├── urdfbuilder.py        # Core fluent API (STUB - shows intended interface)
├── robot_params.py       # Example robot parameters
└── robot_build.py        # Example robot build script
```

## Improvements

- what is the best way to remove duplication? Is it by simply defining a regular python helper or is there something better to do?
- would it make sense to put the "constants" into a separate python file to make this one shorter?

### Answer: Multiple approaches for removing duplication

**Option 1: Simple Python Function**
```python
def add_wheel(parent, name, y_offset):
    wheel = parent.link(name)
    wheel.cylinder(radius=wheel_radius, length=wheel_width)
    wheel.material("gray")
    wheel.joint_type("continuous")
    wheel.axis("y")
    wheel.origin(xyz=[0, y_offset, wheel_z_offset], rpy=[90, 0, 0])
    return wheel

# Use it:
left_wheel = add_wheel(base_link, "left_wheel", wheel_y_offset)
right_wheel = add_wheel(base_link, "right_wheel", -wheel_y_offset)
```

**Option 2: For loop (for symmetric parts)**
```python
for side, y_sign in [("left", 1), ("right", -1)]:
    wheel = base_link.link(f"{side}_wheel")
    wheel.cylinder(radius=wheel_radius, length=wheel_width)
    wheel.material("gray")
    wheel.joint_type("continuous")
    wheel.axis("y")
    wheel.origin(xyz=[0, y_sign * wheel_y_offset, wheel_z_offset], rpy=[90, 0, 0])
```

**Option 3: Library-provided mirroring**
```python
# The library could provide a mirror method
wheel_template = {
    "geometry": ("cylinder", wheel_radius, wheel_width),
    "material": "gray",
    "joint_type": "continuous",
    "axis": "y"
}

base_link.mirror_y(
    name="wheel",
    template=wheel_template,
    origin=[0, wheel_y_offset, wheel_z_offset],
    rpy=[90, 0, 0]
)
# Creates: left_wheel and right_wheel automatically
```

**Option 4: Chained/Fluent API**
```python
# More compact chaining style
left_wheel = (base_link.link("left_wheel")
              .cylinder(wheel_radius, wheel_width)
              .material("gray")
              .joint_type("continuous")
              .axis("y")
              .origin([0, wheel_y_offset, wheel_z_offset], [90, 0, 0]))
```

### Separating Constants

**Yes, separating constants makes sense for larger robots:**

```python
# robot_params.py
class RobotParams:
    # Base dimensions
    base_radius = 0.15
    base_thickness = 0.012
    base_mass = 2.5

    # Wheel dimensions
    wheel_radius = 0.037
    wheel_width = 0.020
    wheel_y_offset = 0.16
    wheel_z_offset = 0.05

    # Sensor positions
    lidar_z_offset = 0.12

# robot_build.py
import urdfbuilder as ub
from robot_params import RobotParams as p

robot = ub.urdf("MyRobot")
base_link = robot.link("base_footprint").link("base_link")
base_link.cylinder(p.base_radius, p.base_thickness).mass(p.base_mass)
# ...
```


## Example - Refactored with Fluent API

**File: pyurdf/robot_params.py**
```python
class RobotParams:
    """Physical parameters for MyRobot"""

    # Structural dimensions
    distance_between_plates = 0.5
    rod_offset = 0.2

    # Base dimensions
    base_radius = 0.15
    base_thickness = 0.012
    base_mass = 2.5

    # Wheel dimensions
    wheel_radius = 0.037
    wheel_width = 0.020
    wheel_y_offset = 0.16
    wheel_z_offset = 0.05

    # Sensor positions
    lidar_z_offset = 0.12
```

**File: pyurdf/robot_build.py**
```python
import urdfbuilder as ub
from robot_params import RobotParams as p

# Create robot with base hierarchy
robot = ub.urdf("MyRobot")
base_footprint = robot.link("base_footprint")

# Base link with chained methods
base_link = (base_footprint.link("base_link")
             .cylinder(p.base_radius, p.base_thickness, material="bamboo")
             .mass(p.base_mass))

# Left wheel - fluent API chaining
left_wheel = (base_link.link("left_wheel")
              .cylinder(p.wheel_radius, p.wheel_width, material="gray")
              .continuous_joint(axis="y")
              .origin([0, p.wheel_y_offset, p.wheel_z_offset], [90, 0, 0]))

# Right wheel - same pattern, mirrored y position
right_wheel = (base_link.link("right_wheel")
               .cylinder(p.wheel_radius, p.wheel_width, material="gray")
               .continuous_joint(axis="y")
               .origin([0, -p.wheel_y_offset, p.wheel_z_offset], [90, 0, 0]))

# Lidar sensor
lidar = (base_link.link("base_scan")
         .mesh("package://my_robot/meshes/lidar.stl")
         .origin([0, 0, p.lidar_z_offset]))

# Generate URDF
robot.save("my_robot.urdf")
```

### Combined Methods for Better Ergonomics

1. **Geometry + Material**: `.cylinder(r, l, material=None)`
   - Optionally include material in geometry definition
   - Cleaner than separate calls for common case

2. **Joint Type + Axis**:
   - `.continuous_joint(axis)` - for continuous joints (wheels)
   - `.revolute_joint(axis, limits)` - for revolute with limits
   - `.fixed_joint()` - default, no parameters needed

3. **Origin**: `.origin(xyz, rpy=None)`
   - Already combined xyz and rotation
   - rpy optional for position-only transforms

4. **Collision Override**: `.collision(geometry, origin=None)`
   - When collision differs from visual

### Benefits of This Approach

✅ **Much more concise** - 27 lines vs 40+ lines
✅ **Clear hierarchy** - chaining shows parent-child relationships
✅ **Type-safe** - IDE autocomplete works
✅ **Standard Python** - no new syntax to learn
✅ **Separated concerns** - params in one file, build logic in another
✅ **Easy to debug** - can print intermediate results
✅ **Flexible** - can still use imperative style when needed
