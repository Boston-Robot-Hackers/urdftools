# URDF Tools - Current Project State

**Last Updated**: 2025-12-25
**Status**: DURDF (DSL URDF) compiler implemented and tested

---

## Project Overview

This project explores **three different approaches** to making URDF (Unified Robot Description Format) creation easier, more concise, and more maintainable:

1. **DURDF (DSL URDF)** - A domain-specific language with nested hierarchy, smart defaults, and semantic shortcuts
2. **Python Builder API** - A fluent Python library leveraging standard programming constructs
3. **Direct XML→YAML Translation** - Simple format conversion (baseline for comparison)

**Key Question**: Which approach provides the best balance of conciseness, learnability, and maintainability?

---

## Recent Progress: DURDF Implementation (Dec 2025)

### ✅ DURDF Compiler Completed

The DURDF (DSL URDF) approach has been fully implemented as a working YAML→URDF compiler with the following features:

**Core Features:**
- ✅ Nested hierarchy format (tree structure through indentation)
- ✅ Three geometry primitives: box, cylinder, sphere
- ✅ Empty links (reference frames like base_footprint)
- ✅ Material/color support (RGBA color definitions)
- ✅ Joint types: fixed (default), continuous, revolute
- ✅ Joint properties: xyz, rpy, axis, type
- ✅ Link properties: geometry with rpy rotation, material
- ✅ Auto-generated collision geometry (matches visual)
- ✅ Unused clause detection and warnings

**File Structure:**
```
dslurdf/
├── src/dslurdf/
│   ├── __init__.py
│   ├── cli.py                  # Command-line interface
│   ├── durdf_loader.py         # YAML loader
│   └── urdf_generator.py       # URDF XML generator
├── tests/fixtures/
│   ├── simple.durdf            # Basic test
│   ├── hierarchy.durdf         # Hierarchy test
│   └── advanced.durdf          # Full featured test
└── README.md
```

**Example DURDF Files Created:**
- `example_urdf/simple_robot.durdf` - 6 links, 5 joints (diff-drive robot)
- `example_urdf/floora.durdf` - 12 links, 11 joints (4-caster platform)
- `example_urdf/makers_pet_loki/mp.durdf` - 11 links, 10 joints (complex hierarchy)
- `example_urdf/tb3/turtlebot3_burger.durdf` - 7 links, 6 joints (TurtleBot3)
- `dslurdf/sample_dsl/dome1.durdf` - 13 links, 12 joints (dome robot with materials)

**Generated Output:**
All DURDF files successfully convert to valid URDF XML in `example_urdf/dslurdf_test_output/urdf_output/`

### DURDF Format Example

```yaml
robot: simple_diffbot

materials:
  gray: [0.5, 0.5, 0.5, 1.0]
  black: [0, 0, 0, 1]
  red: [1, 0, 0, 1]

hierarchy:
  base_footprint:
    base_link:
      left_wheel:
      right_wheel:
      caster:
      lidar:

joints:
  base_link:
    xyz: [0, 0, 0.05]

  left_wheel:
    type: continuous
    xyz: [0, 0.12, -0.025]
    axis: [0, 1, 0]

links:
  base_footprint: {}

  base_link:
    cylinder: [0.15, 0.05]
    material: gray

  left_wheel:
    cylinder: [0.05, 0.03]
    rpy: [1.57, 0, 0]
    material: black

  caster:
    sphere: [0.025]

  lidar:
    cylinder: [0.03, 0.04]
    material: red
```

**Key Design Decisions:**
1. **Hierarchy section first** - Shows robot structure at a glance
2. **Nested indentation** - Parent-child relationships through YAML structure (no repeated parent names)
3. **Smart defaults** - Fixed joints, zero origins, auto-collision
4. **Joint properties by child name** - Keyed by child link for clarity
5. **Primitive geometries only** - No meshes (keeps it simple for educational/prototyping use)

---

## Approach Summaries

### 1. DURDF (DSL URDF) (`dslurdf/`)
- **Reduction**: ~47-60% fewer lines
- **Learning Curve**: Medium (new syntax, but intuitive)
- **Best For**: Maximum conciseness, clear hierarchy, educational use
- **Key Features**: Nested hierarchy, smart defaults, auto-collision, unused clause warnings
- **Status**: ✅ **FULLY IMPLEMENTED** - Working compiler with CLI, tests, and examples

### 2. Python Builder API (`pyurdf/`)
- **Reduction**: 21-48% fewer lines
- **Learning Curve**: Low (standard Python)
- **Best For**: Python developers, complex robots, programmatic generation
- **Key Features**: Builder pattern, full IDE support, loops/functions for duplication
- **Status**: API designed, examples created, XML generation not yet implemented

### 3. Direct XML→YAML Translation (`output/yaml/`)
- **Reduction**: Minimal (~4% or worse)
- **Learning Curve**: High (XML + YAML complexity)
- **Best For**: Nothing - included as baseline to show why simple translation fails
- **Key Features**: 1:1 XML mapping, no semantic improvements
- **Status**: Example created to demonstrate limitations

---

## Example Robot Comparisons

### Simple Robot (diff-drive with caster and lidar)

| Approach | Location | Lines | Reduction |
|----------|----------|-------|-----------|
| **Original URDF** | `example_urdf/simple_robot.urdf` | 86 | baseline |
| **DURDF** | `example_urdf/simple_robot.durdf` | 46 | 47% |
| **Python API** | `output/py/simple_robot.py` | 51 | 41% |
| **Direct YAML** | `output/yaml/simple_robot.yaml` | 104 | -21% (worse!) |

### Floora Robot (4-caster mobile platform)
- **Original**: 300 lines URDF/Xacro
- **DURDF**: 97 lines (68% reduction) - `example_urdf/floora.durdf`
- **Python API**: 177 lines (21% reduction)

### TurtleBot3 Burger
- **Original**: 199 lines URDF/Xacro
- **DURDF**: 56 lines (72% reduction) - `example_urdf/tb3/turtlebot3_burger.durdf`

### Makers Pet Loki
- **Original**: Complex URDF/Xacro with variables
- **DURDF**: 91 lines - `example_urdf/makers_pet_loki/mp.durdf`

---

## Key Findings

### Most Concise: DURDF (DSL URDF)
- Achieves 47-72% reduction through:
  - Nested hierarchy showing structure at a glance
  - Smart defaults (collision=visual, type=fixed, rpy=0)
  - No XML boilerplate
  - Concise YAML syntax
  - Joint properties keyed by child name (no duplication)

### Best Developer Experience: Python Builder API
- Moderate reduction (21-48%) but excellent tooling:
  - Full IDE support (autocomplete, refactoring, debugging)
  - No new syntax to learn
  - Standard Python patterns (loops, functions, data structures)
  - Clear separation of parameters and build logic

### Baseline (Avoid): Direct XML→YAML
- Actually increases verbosity due to:
  - YAML structure overhead for attributes (`@name`, etc.)
  - No semantic improvements
  - Still requires Xacro knowledge for variables/macros
  - Combines worst of both XML and YAML

---

## DURDF Implementation Details

### Command Line Usage

```bash
# Install dependencies
pip install pyyaml

# Convert DURDF to URDF
PYTHONPATH=dslurdf/src python3 dslurdf/src/dslurdf/cli.py input.durdf output.urdf
```

### Features Supported

| Feature | DURDF Syntax | URDF Output |
|---------|--------------|-------------|
| Empty links | `link: {}` | `<link name="..."/>` |
| Box geometry | `box: [x, y, z]` | `<box size="x y z"/>` |
| Cylinder | `cylinder: [r, l]` | `<cylinder radius="r" length="l"/>` |
| Sphere | `sphere: [r]` | `<sphere radius="r"/>` |
| Materials | `materials: {name: [r,g,b,a]}` | `<material><color rgba="..."/></material>` |
| Link material | `material: name` | `<material name="..."/>` in visual |
| Fixed joint | (default) | `<joint type="fixed">` |
| Continuous joint | `type: continuous` | `<joint type="continuous">` |
| Joint origin | `xyz: [x, y, z]` | `<origin xyz="x y z"/>` |
| Joint rotation | `rpy: [r, p, y]` | `<origin rpy="r p y"/>` |
| Joint axis | `axis: [x, y, z]` | `<axis xyz="x y z"/>` |
| Link rotation | `rpy: [r, p, y]` | `<origin rpy="r p y"/>` in visual/collision |

### CLI Output Features

The DURDF CLI provides helpful output:
- Lists all links created
- Lists all joints created
- Warns about unused top-level keys
- Warns about unused link properties
- Warns if joint properties reference non-existent children

### Example CLI Output

```
Loading example_urdf/simple_robot.durdf...
  Robot: simple_diffbot

Generating URDF...

  Links created: 6
    - base_footprint
    - base_link
    - left_wheel
    - right_wheel
    - caster
    - lidar

  Joints created: 5
    - base_footprint_to_base_link
    - base_link_to_left_wheel
    - base_link_to_right_wheel
    - base_link_to_caster
    - base_link_to_lidar

Writing output.urdf...
Done!
```

---

## File Structure

```
urdftools/
├── doc/                          # Consolidated documentation
│   ├── current.md               # This file - project overview
│   ├── design_philosophy.md     # General design principles
│   ├── dsl_approach.md          # DSL-specific documentation
│   └── py_approach.md           # Python API documentation
├── example_urdf/
│   ├── simple_robot.urdf        # Original URDF (86 lines)
│   ├── simple_robot.durdf       # DURDF version (46 lines)
│   ├── floora.durdf             # 4-caster platform (97 lines)
│   ├── makers_pet_loki/
│   │   └── mp.durdf             # Complex robot (91 lines)
│   ├── tb3/
│   │   └── turtlebot3_burger.durdf  # TurtleBot3 (56 lines)
│   └── dslurdf_test_output/     # Generated URDF files
│       └── urdf_output/
│           ├── simple_robot.urdf
│           ├── floora.urdf
│           ├── makers_pet_loki.urdf
│           └── turtlebot3_burger.urdf
├── dslurdf/                     # DURDF implementation
│   ├── src/dslurdf/
│   │   ├── cli.py               # Command-line interface
│   │   ├── durdf_loader.py      # YAML loader
│   │   └── urdf_generator.py    # URDF generator
│   └── tests/fixtures/          # Test files
│       ├── simple.durdf
│       ├── hierarchy.durdf
│       └── advanced.durdf
├── output/
│   ├── dsl/
│   │   └── simple_robot.yaml    # Historical DSL version
│   ├── py/
│   │   └── simple_robot.py      # Python version (51 lines)
│   └── yaml/
│       └── simple_robot.yaml    # Direct translation (104 lines)
├── pyurdf/                      # Python API implementation (historical)
│   ├── pyurdf/                  # Example robots + library
│   └── doc/                     # Python-specific docs (archived)
└── tex/                         # Paper drafts
    └── *.tex
```

---

## Recommendation Matrix

| User Profile | Best Approach | Reason |
|--------------|---------------|--------|
| **Want maximum conciseness** | DURDF | 47-72% reduction, clear structure |
| **Python developers** | Python API | No new syntax, full tooling |
| **Large/complex robots** | Python API or DURDF | Best abstraction capabilities |
| **Teams without coding** | DURDF | Declarative, no programming |
| **Rapid prototyping** | DURDF | Quick, readable, validates structure |
| **Educational use** | DURDF | Clear hierarchy, approachable syntax |
| **Conservative projects** | Original URDF/Xacro | Established ecosystem |

---

## Current Work: Paper Rewrite

### What We're Doing
Completely rewriting the paper in `tex/` to reflect a **comparative study** of all three approaches instead of focusing only on the DSL.

**New Paper Title**: "Beyond URDF: A Comparative Study of Three Approaches to Concise Robot Description in ROS2"

### Paper Outline Status
See [paper.md](paper.md) for complete outline.

**Completed Sections** (in paper.md):
- ✅ Abstract - Succinct, concrete comparison of three approaches
- ✅ Section 1.1: Motivation - Concrete problems with URDF/XML
- ✅ Section 1.2: Requirements - Must be compatible, simpler, leverage ROS knowledge
- ✅ Section 1.3: Our Contribution - Three approaches and key findings
- ✅ Section 2: Background - Related work discussion
- ✅ Section 3.1-3.4: URDF Challenges - Verbosity, hidden structure, repetition, Xacro limits

**Implementation Status**:
- ✅ DURDF fully implemented and tested with real robot examples
- ✅ CLI tool with comprehensive output and warnings
- ✅ Four complete robot examples converted and validated
- ⏳ Python API - designed but not yet implemented
- ✅ Direct YAML baseline - example created

### Next Steps for Paper

**Immediate** (Section 4 onward in paper.md):
1. Update Section 5: DURDF approach with implementation results
2. Write Section 6: Python Builder API approach (detailed)
3. Write Section 7: Direct XML→YAML baseline (why it fails)
4. Write Section 8: Comparative evaluation with concrete metrics
5. Write Section 9: Discussion and recommendations
6. Write Section 10: Conclusion

**Then Convert to LaTeX**:
1. Update `tex/abstract.tex` from paper.md abstract
2. Update `tex/introduction.tex` from paper.md sections 1.x
3. Update `tex/background.tex` from paper.md section 2
4. Create new sections for comparative study
5. Add code examples (DURDF samples)
6. Create comparison tables and figures

### Key Message

"Format is not the problem; semantics are the solution."

Demonstrated by:
- DURDF succeeds (47-72% reduction) - semantic features + nested hierarchy
- Python API succeeds (41% reduction) - semantic features + programmatic control
- Direct YAML fails (-21% worse) - no semantic features

The DURDF implementation proves that a well-designed DSL can achieve dramatic conciseness while maintaining readability and learnability.

---

## Implementation Status

### DURDF Implementation ✅ COMPLETE
1. ✅ YAML→URDF compiler fully working
2. ✅ CLI tool with comprehensive output
3. ✅ Four complete robot examples
4. ✅ Validation and error checking
5. ✅ Unused clause detection
6. ✅ Auto-generated collision geometry
7. ✅ Material/color support (RGBA definitions)

### Python API (TODO)
1. ⏳ Implement `to_urdf()` XML generation
2. ⏳ Create more example conversions
3. ⏳ Add validation and error checking

### Documentation
1. ✅ Consolidate documentation into `doc/`
2. ✅ Create paper outline in `doc/paper.md`
3. ✅ Document DURDF format and usage
4. ⏳ Add contribution guidelines

---

## Proposed: Bidirectional Conversion (URDF ↔ DURDF)

### Overview

Currently DURDF only supports one direction: DURDF → URDF. A reverse converter (URDF → DURDF) would make DURDF more practical for adoption by allowing users to convert existing robots.

### Architecture Design

**New Modules:**
1. `urdf_parser.py` - Parse URDF XML and extract structured data
2. `durdf_writer.py` - Write DURDF YAML from parsed data
3. Extended `cli.py` - Add `--reverse` flag for bidirectional operation

**Data Flow:**
```
URDF XML → urdf_parser.py → intermediate dict → durdf_writer.py → DURDF YAML
```

### CLI Usage
```bash
# Forward (implemented)
durdf robot.durdf robot.urdf

# Reverse (proposed)
durdf --reverse robot.urdf robot.durdf
durdf --reverse robot.urdf.xacro robot.durdf  # Auto-expands xacro
```

### Key Features

**Automatic Xacro Expansion:**
- Auto-detect `.xacro` files by extension
- Call `xacro` command to expand macros
- Parse the resulting URDF XML
- Provide helpful error if xacro not installed

**Hierarchy Reconstruction:**
- Walk through all joints to build parent→children map
- Detect root link(s) - typically base_footprint
- Recursively build nested dictionary structure
- Handle multiple disconnected components

**Smart Simplification:**
- Omit default values: `rpy: [0, 0, 0]`, `type: fixed`
- Merge collision with visual (DURDF auto-generates collision)
- Detect when visual and collision geometries match
- Use primitive geometries only

**Lossy Conversion (Documented):**

| URDF Feature | DURDF Support | Handling |
|--------------|---------------|----------|
| Box/Cylinder/Sphere | ✅ Full | Direct conversion |
| Empty links | ✅ Full | `link: {}` |
| Materials/colors | ✅ Full | RGBA color definitions |
| Mesh geometries | ❌ Not supported | Error or warn + skip |
| Inertial properties | ❌ Not supported | Warn + drop |
| Joint limits | ❌ Not supported | Warn + drop |
| Joint dynamics | ❌ Not supported | Warn + drop |
| Transmission | ❌ Not supported | Warn + drop |

### Implementation Approach

**urdf_parser.py:**
```python
def parse_urdf(xml_string: str) -> dict:
    """Parse URDF XML to intermediate dict format."""
    # Extract robot name
    # Parse all links with geometries
    # Parse all joints with properties
    # Build hierarchy from parent-child relationships
    # Return dict compatible with UrdfGenerator
```

**durdf_writer.py:**
```python
def write_durdf(data: dict, filepath: Path):
    """Write DURDF YAML from intermediate dict."""
    # Write hierarchy section (nested structure)
    # Write joints section (properties by child name)
    # Write links section (geometries)
    # Apply smart defaults (omit zeros, fixed, etc.)
```

**Xacro Processing:**
```python
def load_urdf_xml(filepath: Path) -> str:
    """Load URDF, auto-expanding xacro if needed."""
    if filepath.suffix == '.xacro':
        # Check xacro available
        # Call: xacro filepath
        # Return expanded XML
    else:
        # Return file contents
```

### Benefits

1. **Migration Path** - Users can convert existing robots to DURDF
2. **Learning Tool** - See how URDF maps to DURDF
3. **Round-trip Validation** - Test URDF → DURDF → URDF equivalence
4. **Bidirectional Tool** - One CLI for both directions
5. **Educational** - Helps users understand both formats

### Limitations

**Not a Perfect Converter:**
- Only handles primitive geometries (box, cylinder, sphere)
- Loses materials, colors, and visual styling
- Loses physics properties (mass, inertia)
- Cannot preserve mesh-based robots
- Custom joint names become auto-generated (`parent_to_child`)

**Target Use Case:**
- Simple robots using primitive shapes
- Educational/prototyping robots
- Robots created with basic geometric parts
- NOT for production robots with meshes and detailed physics

### Round-Trip Testing

**Validation Strategy:**
```bash
# Original → DURDF → URDF should be equivalent
durdf --reverse original.urdf converted.durdf
durdf converted.durdf roundtrip.urdf

# Compare original.urdf with roundtrip.urdf
# Should have same structure, may differ in:
# - Joint names (auto-generated)
# - Missing materials/inertia (documented loss)
# - Collision geometry (auto-generated)
```

### File Structure
```
dslurdf/src/dslurdf/
├── cli.py              # Extended with --reverse
├── durdf_loader.py     # Existing YAML loader
├── durdf_writer.py     # NEW - YAML writer
├── urdf_parser.py      # NEW - XML parser
└── urdf_generator.py   # Existing XML generator
```

### Status

**Not Implemented** - Documented for future consideration

This design provides a complete bidirectional tool while clearly documenting what features are preserved and what is lost in conversion. The automatic xacro expansion makes it user-friendly, and the round-trip validation ensures correctness for supported features.

---

## References

- **Paper Outline**: See [paper.md](paper.md) - Complete outline with sections in progress
- **Design Philosophy**: See [design_philosophy.md](design_philosophy.md)
- **DSL Details**: See [dsl_approach.md](dsl_approach.md)
- **Python API Details**: See [py_approach.md](py_approach.md)
- **Direct YAML**: See [yaml_approach.md](yaml_approach.md)
- **ROS Standards**: REP-105 (Coordinate Frames), REP-120 (Humanoids), REP-199 (Inertial)

---

## How to Resume This Work

### To Continue Development
1. **Test DURDF compiler**: Convert more URDFs to DURDF format
2. **Implement Python API**: Add XML generation to `pyurdf/`
3. **Add features**: Mesh support (if needed), joint limits, inertial properties
4. **Create validation**: Schema validation for DURDF files
5. **Implement bidirectional conversion**: URDF → DURDF reverse converter

### To Continue Paper
1. **Update paper.md**: Incorporate DURDF implementation results
2. **Add metrics**: Concrete line counts, complexity measures
3. **Create figures**: Comparison tables, code snippets
4. **Convert to LaTeX**: Transfer completed sections to `tex/*.tex`
5. **Add references**: Proper citations in `tex/references.tex`

**Current Position**:
- DURDF implementation: ✅ Complete
- Paper: Completed through Section 3.4 in paper.md outline
- Next: Update Section 5 with DURDF implementation details and results
