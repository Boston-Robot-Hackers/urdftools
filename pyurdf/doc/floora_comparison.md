# Floora Robot - PyURDF vs URDF Comparison

## Overview

The Floora robot is a 4-caster mobile robot with differential drive:
- 2 drive wheels (continuous joints)
- 4 swivel casters positioned at 45° intervals
- Each caster has a bearing (vertical rotation) and wheel (horizontal rotation)
- Total: 13 links, 11 joints

## File Comparison

### Original URDF/Xacro Approach

**Files:**
- `floora.urdf` - 300 lines (includes 77 lines of comments)

**Structure:**
```xml
<xacro:property name="base_radius" value="0.1495" />
<xacro:property name="wheel_radius" value="0.037" />
... (50+ property declarations)

<link name="base_link">
  <visual>
    <geometry>
      <cylinder radius="${base_radius}" length="${base_thickness}"/>
    </geometry>
    <origin xyz="0 0 ${base_visual_zoff}"/>
    <material name="Bamboo">
      <color rgba="${bamboo_r} ${bamboo_g} ${bamboo_b} ${bamboo_a}" />
    </material>
  </visual>
  <collision>...</collision>
  <inertial>...</inertial>
</link>

<xacro:macro name="caster" params="prefix angle front_or_back">
  ... (60 lines of macro definition)
</xacro:macro>
```

### PyURDF Approach

**Files:**
1. `floora_params.py` - 88 lines (all parameters)
2. `floora_build.py` - 89 lines (build script)
3. **Total: 177 lines**

**Structure:**
```python
# floora_params.py
class FlooraParams:
    base_radius = 0.1495
    wheel_radius = 0.037
    # ... organized parameter groups

# floora_build.py
import urdfbuilder as ub
from floora_params import FlooraParams as p

robot = ub.urdf("floora")
base_footprint = robot.link("base_footprint")

base_link = (base_footprint.link("base_link")
             .cylinder(p.base_radius, p.base_thickness, material="bamboo")
             .origin([0, 0, p.base_visual_zoff])
             .mass(p.base_mass)
             .inertia(p.base_inertia)
             .collision(geometry=("cylinder", p.base_radius, p.base_collision_height),
                       origin=[0, 0, -p.base_collision_height/2])
             .joint_origin([0, 0, p.wheel_zoff + p.wheel_radius]))

# Wheels - simple loop
for side, reflect in [("wheel_l", 1), ("wheel_r", -1)]:
    y_pos = reflect * (p.base_radius + p.wheel_ygap)
    wheel = (base_link.link(side)
             .cylinder(p.wheel_radius, p.wheel_width, material="gray")
             .origin([0, 0, 0], [math.pi/2, 0, 0])
             .continuous_joint(axis="y")
             .joint_origin([0, y_pos, -p.wheel_zoff])
             .mass(p.wheel_mass)
             .inertia(p.wheel_inertia))

# Casters - helper function
def add_caster(parent, name, angle):
    x, y = p.caster_position(angle)
    bearing = (parent.link(f"{name}_bearing_link")
               .cylinder(p.caster_bearing_radius, p.caster_bearing_width,
                        material="cyan")
               .continuous_joint(axis="z")
               .joint_origin([x, y, -p.caster_bearing_zoff])
               # ... more chained calls)

    wheel = (bearing.link(f"{name}_wheel_link")
             .cylinder(p.caster_wheel_radius, p.caster_wheel_width,
                      material="gray")
             # ... more chained calls)

# Add casters
add_caster(base_link, "caster_fl", p.caster_fl_angle)
# ... 3 more
```

## Key Differences

### 1. Line Count
- **URDF/Xacro**: 300 lines (223 without comments)
- **PyURDF**: 177 lines (with comments and docstrings)
- **Reduction**: ~21% fewer lines, ~26% without comments

### 2. Readability
- **URDF/Xacro**: XML verbosity, `${}` syntax, macro parameters
- **PyURDF**: Clean Python, fluent chaining, standard functions

### 3. Duplication Handling
- **URDF/Xacro**: Macros with parameter passing
- **PyURDF**: Standard Python functions and loops

### 4. Parameter Organization
- **URDF/Xacro**: All properties mixed in main file
- **PyURDF**: Separated into dedicated params file with grouping

### 5. IDE Support
- **URDF/Xacro**: Limited XML tooling, no type checking
- **PyURDF**: Full Python IDE support (autocomplete, refactoring, type hints)

### 6. Debugging
- **URDF/Xacro**: Must process through xacro, limited error messages
- **PyURDF**: Standard Python debugging, stack traces, print statements

## Code Patterns Demonstrated

### Pattern 1: Fluent Chaining
```python
base_link = (base_footprint.link("base_link")
             .cylinder(p.base_radius, p.base_thickness, material="bamboo")
             .mass(p.base_mass)
             .inertia(p.base_inertia))
```

### Pattern 2: Python Loops for Symmetry
```python
for side, reflect in [("wheel_l", 1), ("wheel_r", -1)]:
    y_pos = reflect * (p.base_radius + p.wheel_ygap)
    wheel = base_link.link(side).cylinder(...).joint_origin([0, y_pos, -p.wheel_zoff])
```

### Pattern 3: Helper Functions
```python
def add_caster(parent, name, angle):
    x, y = p.caster_position(angle)  # Computed position
    bearing = parent.link(f"{name}_bearing_link").cylinder(...)
    wheel = bearing.link(f"{name}_wheel_link").cylinder(...)
    return bearing
```

### Pattern 4: Collision Override
```python
.collision(geometry=("cylinder", radius, length), origin=[x, y, z])
```

### Pattern 5: Separate Visual and Joint Origins
```python
.origin([0, 0, visual_offset])      # Visual geometry offset
.joint_origin([x, y, z])            # Joint attachment point
```

## API Features Used

1. **Combined methods**: `.cylinder(r, l, material="gray")`
2. **Joint shortcuts**: `.continuous_joint(axis="z")`
3. **Inertia dicts**: `.inertia(p.base_inertia)`
4. **Collision override**: `.collision(geometry, origin)`
5. **Dual origins**: `.origin()` for visual, `.joint_origin()` for joint

## Benefits Summary

✅ **Less code** - 21% reduction
✅ **Better organized** - Parameters separated from logic
✅ **More readable** - Python syntax vs XML
✅ **Standard tools** - Python IDE, debugging, testing
✅ **Type safe** - Can add type hints
✅ **Easier to maintain** - Clear structure, no macro magic
✅ **Flexible** - Full Python power (math, conditionals, imports)

## Implementation Notes

The `floora_build.py` file is executable Python that demonstrates the intended API. The actual `urdfbuilder.py` is currently a stub showing the interface design. Next steps would be implementing the XML generation in `to_urdf()` method.
