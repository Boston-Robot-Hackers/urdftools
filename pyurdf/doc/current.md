# PyURDF - Python Builder API for URDF Creation

## Overview

PyURDF is a Python library that provides a **builder API** for constructing URDF robot descriptions programmatically. Instead of writing XML or learning a custom DSL, users write Python code using simple method calls.

## Key Design Decisions

### 1. Builder Pattern with Method Chaining Support
- Methods return `self` to optionally enable chaining (but not required)
- Clear, readable code with one operation per line
- IDE autocomplete and type checking support
- Easy to debug - can set breakpoints on any line

### 2. Combined Methods for Common Patterns
We identified several methods that are frequently used together and combined them:

- **Geometry + Material**: `.cylinder(r, l, material="gray")`
  - Material is optional parameter on geometry methods
  - Reduces separate method calls for common case

- **Joint Type + Axis**:
  - `.continuous_joint(axis="y")` - combines `joint_type("continuous")` + `axis("y")`
  - `.revolute_joint(axis, limits)` - for revolute joints
  - `.fixed_joint()` - explicit fixed joint (though this is default)

- **Origin**: `.origin(xyz, rpy=None)`
  - Position and rotation in one call
  - Rotation optional for translation-only transforms

### 3. Separated Parameters
- Robot dimensions and constants live in separate file (`robot_params.py`)
- Build logic lives in main file (`robot_build.py`)
- Makes it easy to modify dimensions without touching build code

## API Design

### Core Classes

```python
class Robot:
    - urdf(name) -> Robot
    - link(name) -> Link
    - save(filename)
    - to_urdf() -> str

class Link:
    - link(name) -> Link          # Create child link

    # Geometry (with optional material)
    - cylinder(r, l, material=None) -> Link
    - box(size, material=None) -> Link
    - sphere(r, material=None) -> Link
    - mesh(file, scale=None, material=None) -> Link

    # Properties
    - material(name) -> Link      # Set material separately
    - mass(value) -> Link
    - origin(xyz, rpy=None) -> Link

    # Joint types (combined with axis)
    - continuous_joint(axis) -> Link
    - revolute_joint(axis, limits) -> Link
    - fixed_joint() -> Link
```

## Example Usage

**robot_params.py:**
```python
class RobotParams:
    base_radius = 0.15
    base_thickness = 0.012
    wheel_radius = 0.037
    wheel_width = 0.020
    # ... more parameters
```

**robot_build.py:**
```python
import urdfbuilder as ub
from robot_params import RobotParams as p

robot = ub.urdf("MyRobot")
base_footprint = robot.link("base_footprint")

# Hierarchy shown through variable assignment
base_link = base_footprint.link("base_link")
base_link.cylinder(p.base_radius, p.base_thickness, material="bamboo")
base_link.mass(p.base_mass)

# Each operation on separate line for clarity and debuggability
left_wheel = base_link.link("left_wheel")
left_wheel.cylinder(p.wheel_radius, p.wheel_width, material="gray")
left_wheel.continuous_joint(axis="y")
left_wheel.origin([0, p.wheel_y_offset, p.wheel_z_offset], [90, 0, 0])

robot.save("my_robot.urdf")
```

## Advantages Over Alternatives

### vs. XML/URDF
✅ No XML boilerplate
✅ Python variables and math
✅ IDE support (autocomplete, type checking)
✅ Standard debugging tools

### vs. Custom YAML DSL
✅ No new syntax to learn
✅ Full Python power (loops, functions, conditionals)
✅ Better tooling support
✅ Programmatic flexibility

### vs. Direct XML→YAML Conversion
✅ Actually reduces code (not just format change)
✅ Can abstract repeated patterns
✅ Runtime flexibility

## Implementation Status

**Current**: API design and stubs created
- ✅ Package structure (`pyurdf/`)
- ✅ API stubs in `urdfbuilder.py`
- ✅ Example robot builds:
  - `robot_build.py` / `robot_params.py` - Simple tutorial robot
  - `floora_build.py` / `floora_params.py` - 4-caster mobile robot (177 lines vs 300 URDF)
  - `dome1_build.py` / `dome1_params.py` - Two-tier platform robot (168 lines vs 326 URDF)
- ✅ Design documentation
- ✅ Comparison documents showing 21-48% reduction

**Example Robot Stats:**
- **Floora**: 13 links, 11 joints, complex casters → 21% reduction
- **Dome1**: 12 links, 11 joints, multi-tier platform → 48% reduction

**Next Steps**:
1. Implement `to_urdf()` XML generation
2. Add type hints throughout for IDE support
3. Add helper methods for common patterns (mirroring, etc.)
4. Write unit tests
5. Add inertia auto-calculation from geometry
6. Add validation (tree topology, required fields)

## Key Insights

1. **Duplication is fine in user code** - Users can write Python functions/loops to handle repetition
2. **Library can provide helpers** - Mirror methods, templates for common patterns
3. **Separation of concerns** - Parameters separate from build logic
4. **Progressive disclosure** - Simple things simple, complex things possible

## Open Questions

1. Should we provide high-level helpers for mirroring?
2. How to handle collision geometry that differs from visual?
3. Auto-calculate inertia from geometry and mass?
4. Validate robot structure (e.g., tree topology)?
