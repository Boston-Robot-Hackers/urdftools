# URDF DSL Syntax Specification

Formal grammar specification for the YAML-based URDF Domain-Specific Language.

## Notation

- `<non-terminal>` - production rule
- `::=` - defined as
- `|` - alternative
- `[element]` - optional (zero or one)
- `{element}` - repetition (zero or more)
- `element+` - one or more
- Terminal symbols are in quotes or UPPERCASE

---

## Top-Level Structure

```bnf
<robot-spec> ::= <robot-decl> <hierarchy-section> [<params-section>] [<templates-section>] <links-section>

<robot-decl> ::= "robot:" IDENTIFIER
```

---

## Hierarchy Section

```bnf
<hierarchy-section> ::= "hierarchy:" <hierarchy-tree>

<hierarchy-tree> ::= <link-ref> [<children-list>]

<children-list> ::= "-" <link-ref> ":" <nested-children>
                  | "-" <link-ref>

<nested-children> ::= {"-" <link-ref> [":" <nested-children>]}

<link-ref> ::= IDENTIFIER
```

**Example:**
```yaml
hierarchy:
  root_link:
    - child1
    - child2:
        - grandchild1
        - grandchild2
```

---

## Parameters Section

```bnf
<params-section> ::= "params:" {<param-def>}

<param-def> ::= IDENTIFIER ":" <scalar-value>
              | IDENTIFIER ":" <vector-value>

<scalar-value> ::= NUMBER

<vector-value> ::= "[" NUMBER {"," NUMBER} "]"
```

---

## Templates Section

```bnf
<templates-section> ::= "templates:" {<template-def>}

<template-def> ::= IDENTIFIER ":" <link-properties>
```

---

## Links Section

```bnf
<links-section> ::= "links:" {<link-def>}

<link-def> ::= <simple-link-def>
             | <template-instances-def>
             | <mirror-def>

<simple-link-def> ::= IDENTIFIER ":" <link-properties>

<link-properties> ::= {<geometry-spec> | <material-spec> | <origin-spec> | <joint-spec> | <mass-spec>}

<template-instances-def> ::= IDENTIFIER ":" "template:" IDENTIFIER "instances:" <instance-list>
                              | IDENTIFIER ":" "template:" IDENTIFIER "at:" <instance-map>

<instance-list> ::= "-" "{" "name:" IDENTIFIER "," [<property-overrides>] "}" +

<instance-map> ::= "{" <instance-entry> {"," <instance-entry>} "}"

<instance-entry> ::= IDENTIFIER ":" <origin-value>

<mirror-def> ::= IDENTIFIER ":" "template:" IDENTIFIER "mirror_y:" <mirror-spec>

<mirror-spec> ::= "origin:" <origin-value> "names:" "[" IDENTIFIER "," IDENTIFIER "]"
                | <origin-value>                                                         # Auto-generates left_/right_ names
```

---

## Geometry Specifications

```bnf
<geometry-spec> ::= <cylinder-spec>
                  | <box-spec>
                  | <sphere-spec>
                  | <mesh-spec>
                  | <polygon-spec>

<cylinder-spec> ::= "cylinder:" "{" "radius:" <expression> "," "length:" <expression> "}"
                  | "cylinder:" "[" <expression> "," <expression> "]"              # Array shorthand [radius, length]

<box-spec> ::= "box:" "{" "size:" <vector-expr> "}"
             | "box:" <vector-expr>                                               # Array shorthand [x, y, z]

<sphere-spec> ::= "sphere:" "{" "radius:" <expression> "}"
                | "sphere:" <expression>                                          # Scalar shorthand (radius)

<mesh-spec> ::= "mesh:" "{" "filename:" STRING ["," "scale:" <vector-expr>] "}"

<polygon-spec> ::= <hexagon-spec>
                 | <octagon-spec>
                 | <ngon-spec>

<hexagon-spec> ::= "hexagon:" "{" "radius:" <expression> "," "length:" <expression> "}"
                 | "hexagon:" "[" <expression> "," <expression> "]"               # Array shorthand [radius, length]

<octagon-spec> ::= "octagon:" "{" "radius:" <expression> "," "length:" <expression> "}"
                 | "octagon:" "[" <expression> "," <expression> "]"               # Array shorthand [radius, length]

<ngon-spec> ::= "polygon:" "{" "sides:" NUMBER "," "radius:" <expression> "," "length:" <expression> "}"
```

**Polygon Geometry Notes:**
- `hexagon`: 6-sided regular polygon (prism)
- `octagon`: 8-sided regular polygon (prism)
- `polygon`: N-sided regular polygon (prism) for arbitrary side counts
- `radius`: Distance from center to vertex (circumradius)
- `length`: Height/thickness of the prism along Z-axis
- Polygons are oriented with a flat edge parallel to Y-axis by default

---

## Material Specification

```bnf
<material-spec> ::= "material:" <material-ref>

<material-ref> ::= IDENTIFIER           # Named material from library
                 | <rgba-value>         # Custom RGBA

<rgba-value> ::= "[" NUMBER "," NUMBER "," NUMBER "," NUMBER "]"
```

**Built-in Materials:**
```
red | blue | green | yellow | orange | purple | black | white | gray |
coral | sage | gold | steel | plum | terracotta | seafoam | mustard |
dusty_rose | charcoal | slate | light_blue | darkblue | aluminum |
copper | brass | chrome | plastic | rubber | carbon_fiber
```

---

## Coordinate Specifications

```bnf
<origin-spec> ::= "origin:" <origin-value>

<origin-value> ::= "{" ["xyz:" <vector-expr>] ["," "rpy:" <vector-expr>] "}"
                 | <vector-expr>                                                  # Shorthand for xyz-only (no rotation)
                 | <rotation-shortcut>                                            # Single-axis rotation shortcut

<rotation-shortcut> ::= "{" "xyz:" <vector-expr> "," <single-axis-rotation> "}"

<single-axis-rotation> ::= "rot_x:" <angle>
                         | "rot_y:" <angle>
                         | "rot_z:" <angle>

<angle> ::= <expression> ["deg"]                                                  # Angle in radians or degrees

<vector-expr> ::= "[" <expression> "," <expression> "," <expression> "]"
                | "{" IDENTIFIER "}"                                              # Reference to vector parameter
```

---

## Joint Specifications

```bnf
<joint-spec> ::= [<joint-type-spec>] [<joint-axis-spec>] [<joint-limits-spec>]

<joint-type-spec> ::= "joint_type:" <joint-type>

<joint-type> ::= "fixed" | "revolute" | "continuous" | "prismatic" | "floating" | "planar"

<joint-axis-spec> ::= "axis:" <axis-value>

<axis-value> ::= <vector-expr>
               | <axis-shorthand>

<axis-shorthand> ::= "x" | "y" | "z" | "-x" | "-y" | "-z"
               | "+x" | "+y" | "+z"

<joint-limits-spec> ::= "limits:" "{" ["lower:" NUMBER] ["," "upper:" NUMBER] ["," "effort:" NUMBER] ["," "velocity:" NUMBER] "}"
```

**Axis Shorthand Mapping:**
- `x` or `+x` → `[1, 0, 0]`
- `y` or `+y` → `[0, 1, 0]`
- `z` or `+z` → `[0, 0, 1]`
- `-x` → `[-1, 0, 0]`
- `-y` → `[0, -1, 0]`
- `-z` → `[0, 0, -1]`

**Default:** `joint_type: fixed`

---

## Mass and Inertia Specifications

```bnf
<mass-spec> ::= "mass:" <expression>

<inertial-spec> ::= "inertial:" <inertial-value>

<inertial-value> ::= "auto"                    # Auto-calculate from geometry + mass
                   | <inertia-matrix>

<inertia-matrix> ::= "{" "ixx:" NUMBER "," "ixy:" NUMBER "," "ixz:" NUMBER ","
                          "iyy:" NUMBER "," "iyz:" NUMBER "," "izz:" NUMBER "}"
```

**Default:** Inertia is auto-calculated from geometry and mass if not specified.

---

## Expressions

```bnf
<expression> ::= <term> {<addop> <term>}

<term> ::= <factor> {<mulop> <factor>}

<factor> ::= NUMBER
           | "{" IDENTIFIER "}"                      # Parameter reference
           | "{" <expression> "}"                    # Parenthesized expression
           | "-" <factor>                            # Negation

<addop> ::= "+" | "-"

<mulop> ::= "*" | "/" | "%"

<vector-expr> ::= "[" <expression> "," <expression> "," <expression> "]"
```

**Variable Substitution:**
- `{param_name}` - Simple parameter reference
- `{param1 + param2}` - Arithmetic expression
- `{wheel_base/2}` - Division
- `{(top - bottom)/2}` - Parenthesized expression

**Angular Units:**
- `90deg` - Degrees (converted to radians: π/2)
- `1.57` - Radians (used as-is)

---

## Semantic Rules

### 1. Link References
- All links referenced in `hierarchy` MUST be defined in `links` section
- Template-generated links (instances, mirrors) are implicitly defined

### 2. Parameter Scope
- Parameters defined in `params` are globally accessible
- Variable substitution uses `{param_name}` syntax
- Arithmetic expressions are evaluated at compile time

### 3. Template Expansion
- **Instances:** Generate N links from one template with property overrides
- **Mirrors:** Generate exactly 2 links (left/right) with Y-axis mirroring
- Template properties are inherited unless overridden in instances

### 4. Smart Defaults
- `origin`: `{xyz: [0, 0, 0], rpy: [0, 0, 0]}` if omitted
- `joint_type`: `fixed` if omitted
- `collision`: Equals visual geometry if omitted
- `inertial`: Auto-calculated from geometry + mass if `mass` specified but `inertial` omitted

### 5. Coordinate Frames
- `xyz`: Translation in meters `[x, y, z]`
- `rpy`: Rotation in radians or degrees `[roll, pitch, yaw]`
- Right-handed coordinate system (ROS REP 103)

### 6. Type Inference
- Geometry type inferred from key: `cylinder:`, `box:`, `sphere:`, `mesh:`, `hexagon:`, `octagon:`, `polygon:`
- Joint type inferred from `joint_type:` field, defaults to `fixed`
- Material type inferred: named string or RGBA array

---

## Complete Example

```yaml
robot: example_bot

hierarchy:
  base_footprint:
    - base_link:
        - left_wheel
        - right_wheel

params:
  wheel_radius: 0.05
  wheel_width: 0.02
  wheel_separation: 0.3

templates:
  wheel:
    cylinder: [{wheel_radius}, {wheel_width}]        # Array shorthand
    material: steel
    joint_type: continuous
    axis: y                                          # Axis shorthand
    mass: 0.5

links:
  base_footprint:
    # Virtual link, no geometry

  base_link:
    box: [0.4, 0.3, 0.1]                            # Array shorthand
    material: gray
    mass: 5.0

  wheels:
    template: wheel
    mirror_y: {xyz: [0, {wheel_separation/2}, 0], rot_x: 90deg}  # Rotation shortcut, auto-generated names
```

**Simplifications Shown:**
- `cylinder: [r, l]` - Array geometry shorthand
- `box: [x, y, z]` - Array geometry shorthand
- `axis: y` - Axis shorthand (instead of `[0, 1, 0]`)
- `rot_x: 90deg` - Single-axis rotation shortcut (instead of `rpy: [90deg, 0, 0]`)
- Auto-generated mirror names: `left_wheel`, `right_wheel` inferred from template name
---

## Validation Rules

1. **Syntax validation:** YAML must be well-formed
2. **Structural validation:** All referenced links/params must exist
3. **Type validation:** Expressions must evaluate to correct types
4. **Semantic validation:** Hierarchy must form a tree (no cycles)
5. **Physics validation:** Mass > 0 for non-virtual links
6. **REP compliance:** Optional warnings for non-standard frame names

---

**Version:** 1.0
**Last Updated:** 2025-12-20
