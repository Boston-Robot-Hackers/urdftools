# URDF Tools - Current Project State

**Last Updated**: 2025-12-22
**Status**: Rewriting paper as comparative study of three approaches

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

**Style Guide**:
- Be succinct
- No long abstract statements that mean nothing
- Concrete examples and numbers
- Paragraph format, not bullets (except for lists)

### Next Steps for Paper

**Immediate** (Section 4 onward in paper.md):
1. Finalize Section 4: Design Philosophy (applicable to all approaches)
2. Write Section 5: Semantic YAML DSL approach (detailed)
3. Write Section 6: Python Builder API approach (detailed)
4. Write Section 7: Direct XML→YAML baseline (why it fails)
5. Write Section 8: Comparative evaluation (head-to-head)
6. Write Section 9: Discussion and recommendations
7. Write Section 10: Conclusion

**Then Convert to LaTeX**:
1. Update `tex/abstract.tex` from paper.md abstract
2. Update `tex/introduction.tex` from paper.md sections 1.x
3. Update `tex/background.tex` from paper.md section 2
4. Create new sections for comparative study
5. Add code examples and tables
6. Create comparison figures

### What Changed from Original Plan

**Old Paper**: Single-approach DSL paper
- Focused only on YAML DSL
- Presented DSL as "the solution"
- 60% reduction as main result

**New Paper**: Comparative study
- Three approaches: DSL, Python API, Direct YAML
- No single "best" - recommendations by user profile
- Key finding: **semantic improvements matter more than format choice**
- Direct YAML baseline proves format conversion alone fails

### Key Message

"Format is not the problem; semantics are the solution."

Demonstrated by:
- DSL succeeds (60% reduction) - semantic features
- Python API succeeds (41% reduction) - semantic features
- Direct YAML fails (-21% worse) - no semantic features

---

## Implementation Status

### For Implementation (unchanged)
1. **DSL**: Implement YAML→URDF compiler
2. **Python API**: Implement `to_urdf()` XML generation
3. **Both**: Create more example conversions
4. **Both**: Add validation and error checking

### For Documentation (mostly done)
1. ✅ Consolidate documentation into `doc/`
2. ✅ Create paper outline in `doc/paper.md`
3. Document installation and usage
4. Add contribution guidelines

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

1. **Continue paper.md**: Work through Section 4 onward following the style guide
2. **Review examples**: Ensure code examples in `output/` match paper claims
3. **Convert to LaTeX**: Once paper.md sections are complete, transfer to `tex/*.tex`
4. **Add figures**: Create comparison tables and code snippets for LaTeX
5. **References**: Add proper citations to `tex/references.tex`

**Current Position**: Completed through Section 3.4 in paper.md outline
