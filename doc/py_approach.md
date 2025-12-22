# Python Builder API Approach

The Python Builder API provides a fluent interface for constructing URDF descriptions using standard Python, achieving 21-48% code reduction while leveraging full IDE support and familiar programming patterns.

---

## Overview

**Goal**: Provide a Python library that makes URDF creation feel like natural Python programming.

**Key Achievement**: 41-50% code reduction through:
- Builder pattern with optional method chaining
- Combined methods (geometry + material, joint type + axis)
- Python loops and functions for duplication
- Separation of parameters from build logic
- Full IDE support (autocomplete, debugging, refactoring)

---

## Design Principles

### 1. Builder Pattern (Not Fluent API)

Methods return `self` to enable chaining, but **one operation per line** is encouraged for clarity:

```python
# Preferred: Clear, debuggable
base_link = base_footprint.link("base_link")
base_link.cylinder(0.15, 0.05)
base_link.mass(2.0)

# Also valid: Chained (use for simple cases)
base_link = (base_footprint.link("base_link")
             .cylinder(0.15, 0.05)
             .mass(2.0))
```

**Why not forced chaining?**
- Can set breakpoints on individual operations
- Can print intermediate state for debugging
- Easier to comment out specific operations
- More readable for complex configurations

### 2. Combined Methods

Frequently-used operations combined into single method calls:

```python
# Geometry + Material (optional parameter)
.cylinder(radius, length, material="gray")
.box(size, material="blue")
.sphere(radius, material="red")

# Joint Type + Axis
.continuous_joint(axis="y")
.revolute_joint(axis="z", limits={"lower": -1.57, "upper": 1.57})
.fixed_joint()  # Explicit, though fixed is default

# Origin with optional rotation
.origin([x, y, z], [roll, pitch, yaw])
.origin([x, y, z])  # Rotation omitted = [0, 0, 0]
```

### 3. Separate Parameters (Optional)

For complex robots, parameters can live in separate file:

```python
# robot_params.py
class RobotParams:
    base_radius = 0.15
    wheel_radius = 0.05
    wheel_separation = 0.24

# robot_build.py
from robot_params import RobotParams as p

base_link.cylinder(p.base_radius, p.base_height)
```

For simple robots, inline parameters work fine:

```python
# All in one file
base_radius = 0.15
wheel_radius = 0.05

base_link.cylinder(base_radius, 0.05)
```

### 4. Python Patterns for Duplication

Use standard Python instead of macros:

```python
# Loop for symmetric parts
for side, reflect in [("left", 1), ("right", -1)]:
    y_pos = reflect * wheel_separation / 2
    wheel = base_link.link(f"{side}_wheel")
    wheel.cylinder(wheel_radius, wheel_width)
    wheel.continuous_joint(axis="y")
    wheel.joint_origin([0, y_pos, -0.025])

# Data-driven component creation
support_posts = [
    ("post_1", 0.1, 0.0),
    ("post_2", -0.1, 0.07),
    ("post_3", -0.1, -0.07),
]

for name, x, y in support_posts:
    post = base_link.link(name)
    post.cylinder(post_radius, post_height)
    post.joint_origin([x, y, z])

# Helper functions for complex patterns
def add_caster(parent, name, x, y):
    bearing = parent.link(f"{name}_bearing")
    bearing.cylinder(bearing_radius, bearing_height)
    bearing.continuous_joint(axis="z")
    bearing.joint_origin([x, y, z_offset])

    wheel = bearing.link(f"{name}_wheel")
    wheel.sphere(caster_wheel_radius)
    return bearing
```

---

## API Reference

### Core Classes

#### Robot
```python
robot = ub.urdf(name)           # Create robot
link = robot.link(name)          # Create root-level link
robot.save(filename)             # Save to URDF file
urdf_str = robot.to_urdf()       # Generate URDF string
```

#### Link
```python
# Create child links
child = parent.link(name)

# Geometry (with optional material)
link.cylinder(radius, length, material=None)
link.box(size, material=None)              # size = [x, y, z]
link.sphere(radius, material=None)
link.mesh(filename, scale=None, material=None)

# Properties
link.material(name)              # Set material separately
link.mass(value)
link.inertia(tensor_dict)        # {ixx, ixy, ixz, iyy, iyz, izz}

# Origins
link.origin(xyz, rpy=None)       # Visual geometry origin
link.joint_origin(xyz, rpy=None) # Joint attachment point

# Joint types
link.continuous_joint(axis)      # axis can be "x", "y", "z" or [x,y,z]
link.revolute_joint(axis, limits)
link.fixed_joint()               # Default, rarely needed explicitly

# Collision override
link.collision(geometry, origin=None)
```

### Axis Shortcuts

```python
# String shortcuts for common axes
.continuous_joint(axis="y")      # Same as axis=[0, 1, 0]
.continuous_joint(axis="x")      # Same as axis=[1, 0, 0]
.continuous_joint(axis="z")      # Same as axis=[0, 0, 1]

# Or explicit vector
.continuous_joint(axis=[0, 1, 0])
```

---

## Example: Simple Robot

See [output/py/simple_robot.py](../output/py/simple_robot.py) for complete example.

**Line count comparison:**
- Original URDF: 86 lines
- Python version: 51 lines (41% reduction)

**Key features demonstrated:**
- Inline parameters
- Python loop for wheel symmetry
- Clear hierarchy through variable assignment
- Combined methods (geometry + material)

---

## Example: Complex Robot (Floora)

4-caster mobile platform with differential drive:

**Files:**
- `pyurdf/pyurdf/floora_params.py` - 88 lines (organized parameter groups)
- `pyurdf/pyurdf/floora_build.py` - 89 lines (build script)
- **Total**: 177 lines

**Original**: 300 lines URDF/Xacro

**Reduction**: 41%

**Key patterns:**
```python
# Helper function for complex assembly
def add_caster(parent, name, angle):
    x, y = p.caster_position(angle)  # Computed position

    bearing = parent.link(f"{name}_bearing_link")
    bearing.cylinder(p.caster_bearing_radius, p.caster_bearing_width)
    bearing.continuous_joint(axis="z")
    bearing.joint_origin([x, y, -p.caster_bearing_zoff])

    wheel = bearing.link(f"{name}_wheel_link")
    wheel.cylinder(p.caster_wheel_radius, p.caster_wheel_width)
    wheel.origin([0, 0, 0], [math.pi/2, 0, 0])
    wheel.continuous_joint(axis="y")
    return bearing

# Use helper
add_caster(base_link, "caster_fl", p.caster_fl_angle)
add_caster(base_link, "caster_fr", p.caster_fr_angle)
add_caster(base_link, "caster_bl", p.caster_bl_angle)
add_caster(base_link, "caster_br", p.caster_br_angle)
```

---

## Example: Dome1 Robot

Two-tier platform with support posts:

**Files:**
- `pyurdf/pyurdf/dome1_params.py` - 80 lines
- `pyurdf/pyurdf/dome1_build.py` - 88 lines
- **Total**: 168 lines

**Original**: 326 lines URDF/Xacro

**Reduction**: 48%

**Key patterns:**
```python
# Data-driven component creation
for name, x_pos, y_pos in p.support_posts:
    post = base_link.link(name)
    post.cylinder(p.support_post_diameter/2, p.support_post_height,
                 material="mustard")
    post.joint_origin([x_pos, y_pos, z_between_plates/2])
    post.collision(
        geometry=("cylinder", p.support_post_diameter/2,
                             p.support_post_diameter)
    )

# Multiple origins (visual + joint rotation)
laser = base_link.link("laser")
laser.cylinder(p.laser_radius, p.laser_height, material="red")
laser.origin([0, 0, 0], [0, 0, math.pi/2])  # Visual rotation
laser.joint_origin([p.laser_x_offset, 0, p.laser_z_offset],
                  [0, 0, -math.pi/2])        # Joint rotation
```

---

## Advantages Over Alternatives

### vs. URDF/XML
✅ No XML boilerplate
✅ Python variables and expressions
✅ IDE autocomplete and type checking
✅ Standard debugging tools (breakpoints, print, pdb)
✅ Familiar syntax (no learning curve for Python developers)

### vs. Custom YAML DSL
✅ No new syntax to learn
✅ Full Python power (loops, functions, conditionals, classes)
✅ Better tooling support (PyCharm, VSCode, etc.)
✅ Programmatic flexibility (can generate from data, APIs, etc.)
✅ Standard testing frameworks (pytest, unittest)

### vs. Direct XML→YAML
✅ Actually reduces code (not just format change)
✅ Can abstract repeated patterns with functions
✅ Runtime flexibility (conditional logic, data-driven)

---

## Development Experience

### IDE Support
- **Autocomplete**: Methods, parameters, return types
- **Type Checking**: With type hints (future addition)
- **Refactoring**: Rename, extract function, etc.
- **Navigation**: Jump to definition, find usages
- **Documentation**: Inline docstrings in IDE

### Debugging
```python
base_link = base_footprint.link("base_link")
print(f"Created base_link: {base_link.name}")  # Debug output
base_link.cylinder(0.15, 0.05)
breakpoint()  # Set breakpoint here
base_link.mass(2.0)
```

### Testing
```python
import pytest

def test_wheel_creation():
    robot = ub.urdf("test_robot")
    base = robot.link("base")
    wheel = base.link("wheel")
    wheel.cylinder(0.05, 0.03)

    assert wheel.geometry[0] == "cylinder"
    assert wheel.geometry[1] == 0.05
```

---

## When to Use Python API

**Best for:**
- Python developers (no new syntax)
- Complex robots with many repeated patterns
- Programmatic generation (from CAD data, config files)
- Projects needing testing/validation
- Rapid prototyping and iteration

**Not ideal for:**
- Teams without Python experience
- Simple robots (DSL may be more concise)
- Situations requiring maximum conciseness (DSL wins)

---

## Implementation Status

**Completed:**
- ✅ API design and interface stubs
- ✅ Three complete example robots
- ✅ Documentation and comparisons
- ✅ Demonstrated 21-48% code reduction

**Next Steps:**
1. Implement `to_urdf()` XML generation
2. Add type hints for IDE support
3. Write unit tests
4. Add inertia auto-calculation from geometry
5. Add validation (topology, required fields)
6. Package for pip installation

---

## Project Structure

```
pyurdf/
├── lib/
│   └── urdfbuilder.py       # Core API (stubs)
├── pyurdf/
│   ├── __init__.py
│   ├── robot_params.py      # Tutorial example params
│   ├── robot_build.py       # Tutorial example build
│   ├── floora_params.py     # Floora robot params
│   ├── floora_build.py      # Floora robot build
│   ├── dome1_params.py      # Dome1 robot params
│   └── dome1_build.py       # Dome1 robot build
└── doc/                     # Archived docs (see /doc instead)
```

---

## References

- **Current Project State**: [current.md](current.md)
- **Design Philosophy**: [design_philosophy.md](design_philosophy.md)
- **API Stubs**: [pyurdf/lib/urdfbuilder.py](../pyurdf/lib/urdfbuilder.py)
- **Simple Example**: [output/py/simple_robot.py](../output/py/simple_robot.py)
- **Complex Examples**: `pyurdf/pyurdf/floora_build.py`, `dome1_build.py`
- **Comparisons**: `pyurdf/doc/floora_comparison.md`, `dome1_comparison.md`, `approach_comparison.md`
