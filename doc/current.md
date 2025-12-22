# URDF Tools - Current Project State

**Last Updated**: 2025-12-21
**Status**: Comparative analysis of three approaches to simplifying URDF creation

---

## Project Overview

This project explores **three different approaches** to making URDF (Unified Robot Description Format) creation easier, more concise, and more maintainable:

1. **Semantic YAML DSL** - A domain-specific language with templates, smart defaults, and semantic shortcuts
2. **Python Builder API** - A fluent Python library leveraging standard programming constructs
3. **Direct XML→YAML Translation** - Simple format conversion (baseline for comparison)

**Key Question**: Which approach provides the best balance of conciseness, learnability, and maintainability?

---

## Approach Summaries

### 1. Semantic YAML DSL (`dslurdf/`)
- **Reduction**: ~60% fewer lines
- **Learning Curve**: Medium (new syntax)
- **Best For**: Maximum conciseness, declarative specifications
- **Key Features**: Templates, mirroring, smart defaults, explicit hierarchy
- **Status**: Syntax finalized, grammar documented, examples created

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
| **Semantic DSL** | `output/dsl/simple_robot.yaml` | 46 | 47% |
| **Python API** | `output/py/simple_robot.py` | 51 | 41% |
| **Direct YAML** | `output/yaml/simple_robot.yaml` | 104 | -21% (worse!) |

### Floora Robot (4-caster mobile platform)
- **Original**: 300 lines URDF/Xacro
- **Python API**: 177 lines (21% reduction)
- See: `pyurdf/pyurdf/floora_build.py` + `floora_params.py`

### Dome1 Robot (two-tier platform)
- **Original**: 326 lines URDF/Xacro
- **Semantic DSL**: ~130 lines (60% reduction)
- **Python API**: 168 lines (48% reduction)
- See: `pyurdf/pyurdf/dome1_build.py`, `dslurdf/urdf-dsl/dome1.yaml`

---

## Key Findings

### Most Concise: Semantic YAML DSL
- Achieves 47-60% reduction through:
  - Templates and mirroring for symmetric components
  - Smart defaults (collision=visual, joints=fixed, etc.)
  - Variable substitution with expressions
  - Explicit hierarchy section

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

## File Structure

```
urdftools/
├── doc/                          # Consolidated documentation
│   ├── current.md               # This file - project overview
│   ├── design_philosophy.md     # General design principles
│   ├── dsl_approach.md          # DSL-specific documentation
│   └── py_approach.md           # Python API documentation
├── example_urdf/
│   └── simple_robot.urdf        # Baseline example (86 lines)
├── output/
│   ├── dsl/
│   │   └── simple_robot.yaml    # DSL version (46 lines)
│   ├── py/
│   │   └── simple_robot.py      # Python version (51 lines)
│   └── yaml/
│       └── simple_robot.yaml    # Direct translation (104 lines)
├── dslurdf/                     # DSL implementation (historical)
│   ├── urdf-dsl/                # Example robots in DSL
│   └── doc/                     # DSL-specific docs (archived)
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
| **Want maximum conciseness** | Semantic DSL | 60% reduction, clear structure |
| **Python developers** | Python API | No new syntax, full tooling |
| **Large/complex robots** | Python API or DSL | Best abstraction capabilities |
| **Teams without coding** | Semantic DSL | Declarative, no programming |
| **Rapid prototyping** | Python API | Full programmatic control |
| **Educational use** | Semantic DSL | Clear hierarchy, approachable syntax |
| **Conservative projects** | Original URDF/Xacro | Established ecosystem |

---

## Next Steps

### For Paper/Publication
1. ✅ Create simple example robot in all three approaches
2. Finalize comparison metrics (lines, readability, learnability)
3. Update paper sections to reflect three-approach comparison
4. Add recommendation matrix to discussion section

### For Implementation
1. **DSL**: Implement YAML→URDF compiler
2. **Python API**: Implement `to_urdf()` XML generation
3. **Both**: Create more example conversions
4. **Both**: Add validation and error checking

### For Documentation
1. ✅ Consolidate documentation into `doc/`
2. Create tutorial for each approach
3. Document installation and usage
4. Add contribution guidelines

---

## References

- **Design Philosophy**: See [design_philosophy.md](design_philosophy.md)
- **DSL Details**: See [dsl_approach.md](dsl_approach.md)
- **Python API Details**: See [py_approach.md](py_approach.md)
- **ROS Standards**: REP-105 (Coordinate Frames), REP-120 (Humanoids), REP-199 (Inertial)

---

**Status**: Three approaches defined, simple example completed, ready for paper writing
