# Project State: URDF DSL (December 2025)

**Last Updated**: 2025-12-20
**Working Directory**: `/Users/pitosalas/mydev/dslurdf`
**Status**: Syntax finalized, ready for next phase

---

## Project Overview

**Goal**: Create a YAML-based Domain-Specific Language (DSL) to replace URDF/Xacro for robot descriptions in ROS2.

**Key Achievement**: 60% code reduction through smart defaults, flat syntax, and template system.

---

## Current Session Achievements (2025-12-20)

### 1. ✅ Syntax Consistency Review & Fixes

**Problem Found**: Three DSL example files had inconsistent syntax from brainstorming phase.

**Issues Fixed**:
- **dome1.yaml**: Removed contradictory nested visual syntax in laser link (lines 132-141)
- **Type section removed**: Decided `type:` declaration is redundant (structure already defines robot type)
- **Hierarchy moved to top**: TF tree now visible immediately after robot name
- **Flat syntax enforced**: All geometry at top level, no nested visual/collision blocks

**Files Affected**:
- `urdf-dsl/dome1.yaml` - Now canonical example with consistent syntax
- Note: `linorobot2_2wd.yaml` and `linorobot2_2wd_complete.yaml` have inconsistencies (not yet fixed)

### 2. ✅ Formal Grammar Specification Created

**New File**: `doc/dsl_syntax.md` (363 lines)
- Complete BNF grammar specification
- All production rules for robot-spec, hierarchy, params, templates, links
- Geometry specs: cylinder, box, sphere, mesh, **hexagon, octagon, polygon** (NEW)
- Expression grammar with variable substitution `{param}`
- Semantic rules and validation rules
- Complete example included

### 3. ✅ Future Extensions Documented

**New File**: `doc/dsl_syntax_futures.md` (279 lines)
- Explicit collision geometry overrides
- Gazebo simulation configuration (materials, friction, sensors, controllers)
- Conditional logic (`if/then/else`)
- Iteration and loops (`for/in`)
- File inclusion and imports
- Advanced mirroring (X, Z, arbitrary planes)
- Type annotations (moved from core - may reconsider)

### 4. ✅ Polygon Geometry Support Added

**Added to Core Syntax**:
```yaml
# Hexagonal base
base_plate:
  hexagon: {radius: 0.15, length: 0.005}

# Octagonal platform
top_plate:
  octagon: {radius: 0.12, length: 0.003}

# N-sided polygon
dodecagon:
  polygon: {sides: 12, radius: 0.20, length: 0.008}
```

---

## Design Principles (FINALIZED)

### 1. **Flat Syntax**
Geometry types as top-level fields, not nested visual/geometry/type:
```yaml
base_link:
  cylinder: {radius: 0.14, length: 0.003}
  material: yellow
  mass: 1.0
```

### 2. **Hierarchy First**
TF tree structure visible at top of file:
```yaml
robot: dome_x1

hierarchy:
  base_footprint:
    - base_link:
        - bottom_plate
        - left_wheel
        - right_wheel
```

### 3. **Smart Defaults**
- `collision` = visual geometry (unless overridden)
- `inertia` auto-calculated from geometry + mass
- `origin` = `[0, 0, 0]` when omitted
- `joint_type` = `fixed` when omitted

### 4. **Templates & Generators**
Eliminate repetition with reusable components:
```yaml
templates:
  wheel:
    cylinder: {radius: {wheel_diameter/2}, length: {wheel_width}}
    material: steel
    joint_type: continuous
    axis: [0, 1, 0]

wheels:
  template: wheel
  mirror_y:
    origin: {xyz: [{wheel_x_offset}, {wheel_base/2}, {-wheel_ground_clearance}], rpy: [90deg, 0, 0]}
    names: [left_wheel, right_wheel]
```

### 5. **Variable Substitution**
Use `{param}` syntax with arithmetic:
```yaml
params:
  wheel_base: 0.215
  wheel_diameter: 0.0625

# Later:
origin: {xyz: [0, {wheel_base/2}, {-wheel_diameter/2}]}
```

### 6. **Named Materials**
Pre-defined library: yellow, blue, red, steel, coral, plum, gold, charcoal, etc.

### 7. **No Type Declarations** (NEW DECISION)
Robot type is inferred from structure, not declared explicitly.

---

## Simplification Opportunities Identified

### **Tier 1: High Impact** (Recommended for Next Iteration)

1. **Axis Shorthand**
   ```yaml
   # Current
   axis: [0, 1, 0]
   # Proposed
   axis: y  # or x, z, -x, -y, -z
   ```

2. **Origin XYZ-Only Shorthand**
   ```yaml
   # Current
   origin: {xyz: [0.1, 0, 0.3]}
   # Proposed
   origin: [0.1, 0, 0.3]  # Implicit xyz when no rotation
   ```

3. **Geometry Array Shorthand**
   ```yaml
   # Current
   cylinder: {radius: 0.05, length: 0.02}
   box: {size: [0.1, 0.2, 0.3]}
   sphere: {radius: 0.05}

   # Proposed
   cylinder: [0.05, 0.02]        # [radius, length]
   box: [0.1, 0.2, 0.3]          # [x, y, z]
   sphere: 0.05                  # radius only
   ```

**Impact**: Would reduce dome1.yaml from ~130 lines to ~110 lines (additional 15% reduction).

### **Tier 2: Medium Impact**

4. **Rotation Shortcuts**
   ```yaml
   # Current
   rpy: [90deg, 0, 0]
   # Proposed
   rot_x: 90deg  # or [x,y,z, roll, pitch, yaw] shorthand
   ```

5. **Auto-Generated Mirror Names**
   ```yaml
   # Current
   mirror_y:
     names: [left_wheel, right_wheel]
   # Proposed
   mirror_y: ground  # Auto-generates left_/right_ prefix
   ```

6. **Compact Instance Map**
   ```yaml
   # Current
   instances:
     - name: post_1
       origin: {xyz: [0.1, 0, 0.03]}
   # Proposed
   at:
     post_1: [0.1, 0, 0.03]
     post_2: [-0.1, 0, 0.03]
   ```

### **Tier 3: Advanced - Semantic Parameter Recognition** ⭐ GAME CHANGER

**Concept**: Recognize semantic parameter names and auto-infer positioning.

**Recognized Parameters**:
- `wheel_base` → Auto-spread wheels at `±{wheel_base/2}`
- `wheel_ground_clearance` → Auto-calculate Z for ground contact
- `caster_offset` → Auto-position front/rear casters

**Example Transformation**:
```yaml
# Current (verbose)
wheels:
  template: wheel
  mirror_y:
    origin: {xyz: [{wheel_x_offset}, {wheel_base/2}, {-wheel_diameter/2 - wheel_ground_clearance}], rpy: [90deg, 0, 0]}
    names: [left_wheel, right_wheel]

# With Tier 1 + Tier 3 (semantic)
wheels:
  template: wheel
  mirror_y: ground              # Infers Y from wheel_base, Z from ground+clearance, rpy from axis
  x: {wheel_x_offset}           # Only specify unique position
```

**Inference Rules**:
1. **Rotation from axis**: `cylinder` + `axis: y` → auto-add `rpy: [90deg, 0, 0]`
2. **Ground contact**: `ground` keyword → `z: {-geometry_radius - clearance_param}`
3. **Mirror spread**: `mirror_y` + `wheel_base` param → auto-spread at `±{wheel_base/2}`

**Impact**: Transforms DSL from syntax sugar to **semantic intent declaration**.

---

## File Structure (Current)

```
/Users/pitosalas/mydev/dslurdf/
├── doc/
│   ├── current.md                  # THIS FILE - project state
│   ├── dsl_syntax.md               # ✅ NEW - Formal BNF grammar (core syntax)
│   ├── dsl_syntax_futures.md       # ✅ NEW - Future extensions
│   ├── notes.md                    # Design philosophy (277 lines)
│   ├── taxonomy.md                 # Robot archetypes (37 lines)
│   └── samples.md                  # Early syntax experiments (247 lines)
├── urdf-dsl/
│   ├── dome1.yaml                  # ✅ CANONICAL - 130 lines, consistent syntax
│   ├── linorobot2_2wd.yaml         # ⚠️ Has syntax inconsistencies
│   └── linorobot2_2wd_complete.yaml # ⚠️ Has syntax inconsistencies
└── urdf_examples/
    └── (original URDF files for comparison)
```

---

## Key Results

### dome1.yaml (Canonical Example)
- **Original URDF/Xacro**: 326 lines (14 material defs, 2 macros)
- **Current DSL**: 130 lines
- **Reduction**: 60% fewer lines, cleaner structure
- **Structure**: hierarchy → params → templates → links
- **Status**: ✅ Syntax consistent, no type section, hierarchy first

### Geometry Support
- ✅ `cylinder`, `box`, `sphere`, `mesh`
- ✅ `hexagon`, `octagon` (NEW)
- ✅ `polygon: {sides: N, ...}` (NEW - arbitrary N-gons)

---

## Design Decisions Log

### Decisions Made This Session

1. ✅ **Remove `type:` section** - Redundant, structure already defines robot type
2. ✅ **Hierarchy goes first** - TF tree visible immediately
3. ✅ **Flat syntax enforced** - No nested visual blocks
4. ✅ **Collision/Gazebo → futures** - Core stays minimal
5. ✅ **Polygon primitives added** - Hexagon, octagon, N-gon support
6. ✅ **Formal grammar documented** - BNF specification complete

### Previous Decisions (Preserved)

- Use `{param}` not `${param}` for variables
- Smart defaults over YAML anchors
- Separate hierarchy section
- Templates for reusability
- YAML compiles to URDF XML (not runtime format)
- Named material library

---

## Next Steps

### **Immediate** (Before Paper Writing)

1. **Apply Tier 1 simplifications** to dome1.yaml:
   - Axis shorthand: `axis: y`
   - Origin shorthand: `origin: [x, y, z]`
   - Geometry arrays: `cylinder: [r, l]`, `box: [x,y,z]`, `sphere: r`

2. **Fix linorobot2 files** to match dome1.yaml syntax consistency

3. **Update dsl_syntax.md** with Tier 1 shorthand syntax

### **Future Considerations**

4. **Tier 2 simplifications**: Rotation shortcuts, auto-mirror names, compact instances

5. **Tier 3 semantic recognition**: Recognize `wheel_base`, `wheel_ground_clearance`, etc.

6. **Write paper** (4 pages, journalistic style, code-heavy)

---

## Technical Reference

### Variable Substitution
```yaml
{param_name}                    # Simple substitution
{param1 + param2}              # Arithmetic
{(top - bottom)/2}             # Parentheses for grouping
```

### Angular Units
```yaml
rpy: [90deg, 0, 0]             # Degrees (converted to radians)
rpy: [1.57, 0, 0]              # Radians (used as-is)
```

### Joint Types
- `fixed` (default) - No movement
- `continuous` - Unlimited rotation (wheels)
- `revolute` - Limited rotation (joints with limits)
- `prismatic` - Linear sliding
- `floating`, `planar` - Special cases

### When Joint Specs Can Be Omitted
✅ **Omit when**: Fixed joint (90% of links - sensors, structural, virtual frames)
❌ **Required when**: Moving joint (continuous, revolute, prismatic)

---

## Important Notes

**What We're NOT Building**:
- NOT a runtime format (compiles to URDF XML)
- NOT replacing ROS2 infrastructure
- NOT a new physics engine
- NOT incompatible with existing tools

**Philosophy**:
- Reduce boilerplate, not features
- Semantic intent over verbose specification
- Compiler infers from structure and conventions
- One source of truth (no redundant declarations)

---

## Session Context for Resume

**Date**: 2025-12-20
**Working Directory**: `/Users/pitosalas/mydev/dslurdf`
**Git Repo**: No (not yet initialized)
**Platform**: macOS (Darwin 25.2.0)

**Files Created**:
- `doc/dsl_syntax.md` (formal BNF grammar)
- `doc/dsl_syntax_futures.md` (future extensions)

**Files Modified**:
- `urdf-dsl/dome1.yaml` (removed type section, hierarchy first, fixed laser link)
- `doc/current.md` (this file)

**Key Insights**:
- Semantic parameter recognition is the next big win
- Simplification can go much further than 60%
- Axis inference from joint type is powerful
- Ground contact is a common pattern worth recognizing

---

**Status**: Syntax specification complete, simplification roadmap defined
**Next Session**: Apply Tier 1 simplifications, then proceed with paper writing or implementation
