# PyURDF - Python Builder API for URDF Robot Descriptions

A Python library providing a **builder API** for constructing URDF robot descriptions programmatically. Write clean Python code instead of verbose XML or learning a custom DSL.

## Overview

PyURDF lets you build robot descriptions using Python's natural syntax:

```python
import urdfbuilder as ub
from robot_params import RobotParams as p

robot = ub.urdf("MyRobot")
base_footprint = robot.link("base_footprint")

# Clear hierarchical structure through variables
base_link = base_footprint.link("base_link")
base_link.cylinder(p.base_radius, p.base_thickness, material="bamboo")
base_link.mass(p.base_mass)

# Wheels with symmetric positioning using Python loop
for side, reflect in [("left", 1), ("right", -1)]:
    wheel = base_link.link(f"{side}_wheel")
    wheel.cylinder(p.wheel_radius, p.wheel_width, material="gray")
    wheel.continuous_joint(axis="y")
    wheel.joint_origin([0, reflect * p.wheel_y_offset, p.wheel_z_offset])

robot.save("my_robot.urdf")
```

## Why PyURDF?

### vs. XML/URDF
- ✅ No XML boilerplate
- ✅ Python variables and expressions
- ✅ IDE autocomplete and type checking
- ✅ Standard debugging tools

### vs. Custom YAML DSL
- ✅ No new syntax to learn
- ✅ Full Python power (loops, functions, conditionals)
- ✅ Better tooling support
- ✅ Programmatic flexibility

### vs. Direct XML→YAML
- ✅ Actually reduces code (not just format change)
- ✅ 21-48% fewer lines
- ✅ Can abstract repeated patterns

## Real-World Examples

We've converted actual robot URDFs to demonstrate the approach:

### Floora Robot (4-caster mobile platform)
- **Original URDF**: 300 lines (with xacro macros)
- **PyURDF**: 177 lines (88 params + 89 build)
- **Reduction**: 21%
- **Structure**: 13 links, 11 joints, differential drive + 4 swivel casters

See: [`floora_build.py`](pyurdf/floora_build.py) | [`floora_params.py`](pyurdf/floora_params.py) | [Comparison](doc/floora_comparison.md)

### Dome1 Robot (two-tier platform)
- **Original URDF**: 326 lines (with xacro macros)
- **PyURDF**: 168 lines (80 params + 88 build)
- **Reduction**: 48%
- **Structure**: 12 links, 11 joints, circular plates connected by posts

See: [`dome1_build.py`](pyurdf/dome1_build.py) | [`dome1_params.py`](pyurdf/dome1_params.py) | [Comparison](doc/dome1_comparison.md)

## Key Features

### 1. Simple Builder Pattern
Clear, debuggable code with one operation per line:

```python
base_link = robot.link("base_footprint").link("base_link")
base_link.cylinder(0.15, 0.012, material="bamboo")
base_link.mass(2.5)
# Can set breakpoints, print state, comment out operations easily
```

### 2. Combined Methods
Frequently-used operations combined into single methods:

- **Geometry + Material**: `.cylinder(r, l, material="gray")`
- **Joint Type + Axis**: `.continuous_joint(axis="y")`
- **Origin with optional rotation**: `.origin([x,y,z], [r,p,y])`
- **Collision override**: `.collision(geometry, origin)`

### 3. Separated Parameters
Robot dimensions live in separate parameter files:

```python
# robot_params.py
class RobotParams:
    base_radius = 0.15
    wheel_diameter = 0.066
    # ... all parameters

# robot_build.py
from robot_params import RobotParams as p
base_link.cylinder(p.base_radius, p.base_thickness)
```

### 4. Python Patterns for Duplication
Use standard Python instead of macro systems:

```python
# Loop for symmetric parts
for side, reflect in [("left", 1), ("right", -1)]:
    wheel = base_link.link(f"{side}_wheel").cylinder(...)

# Data-driven component creation
support_posts = [("post_1", 0.1, 0.0), ("post_2", -0.1, 0.07)]
for name, x, y in support_posts:
    post = base_link.link(name).cylinder(...).joint_origin([x, y, z])

# Helper functions
def add_caster(parent, name, angle):
    bearing = parent.link(f"{name}_bearing").cylinder(...)
    wheel = bearing.link(f"{name}_wheel").cylinder(...)
    return bearing
```

## API Overview

### Core Classes

```python
class Robot:
    urdf(name) -> Robot
    link(name) -> Link
    save(filename)
    to_urdf() -> str

class Link:
    # Create child links
    link(name) -> Link

    # Geometry (with optional material)
    cylinder(r, l, material=None) -> Link
    box(size, material=None) -> Link
    sphere(r, material=None) -> Link
    mesh(file, scale=None, material=None) -> Link

    # Properties
    material(name) -> Link
    mass(value) -> Link
    inertia(tensor_dict) -> Link

    # Origins
    origin(xyz, rpy=None) -> Link          # Visual geometry origin
    joint_origin(xyz, rpy=None) -> Link    # Joint attachment point

    # Joint types (combined with axis)
    continuous_joint(axis) -> Link
    revolute_joint(axis, limits) -> Link
    fixed_joint() -> Link

    # Collision override
    collision(geometry, origin=None) -> Link
```

## Project Structure

```
pyurdf/
├── pyurdf/
│   ├── __init__.py
│   ├── urdfbuilder.py       # Core fluent API (stub)
│   ├── robot_params.py      # Tutorial example params
│   ├── robot_build.py       # Tutorial example build
│   ├── floora_params.py     # Floora robot params
│   ├── floora_build.py      # Floora robot build
│   ├── dome1_params.py      # Dome1 robot params
│   └── dome1_build.py       # Dome1 robot build
├── doc/
│   ├── current.md           # API design & status
│   ├── notes.md             # Design exploration
│   ├── floora_comparison.md # Floora URDF vs PyURDF
│   └── dome1_comparison.md  # Dome1 URDF vs PyURDF
└── urdf_examples/
    ├── floora.urdf          # Original Floora URDF
    └── dome1.urdf.xacro     # Original Dome1 URDF
```

## Current Status

**✅ Completed:**
- API design and interface stubs
- Three complete example robots
- Documentation and comparisons
- Demonstrated 21-48% code reduction

**🚧 Next Steps:**
1. Implement `to_urdf()` XML generation
2. Add type hints for IDE support
3. Write unit tests
4. Add inertia auto-calculation
5. Add validation (topology, required fields)

## Documentation

- [**API Design & Current Status**](doc/current.md) - Complete API specification
- [**Design Notes**](doc/notes.md) - Design decisions and alternatives explored
- [**Floora Comparison**](doc/floora_comparison.md) - Detailed URDF vs PyURDF comparison
- [**Dome1 Comparison**](doc/dome1_comparison.md) - Platform robot comparison

## Key Insights

1. **Duplication handling**: Use standard Python (functions, loops) instead of special features
2. **Parameter separation**: Keep dimensions separate from build logic
3. **Progressive disclosure**: Simple things simple, complex things possible
4. **Fluent chaining**: Shows hierarchy naturally through indentation

## License

[To be determined]

## Contributing

This is currently a design exploration and API prototype. The `urdfbuilder.py` contains stubs showing the intended interface. Implementation of actual URDF XML generation is the next major milestone.
