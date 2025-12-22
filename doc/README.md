# URDF Tools Documentation

This directory contains consolidated documentation for the URDF Tools project, which explores three approaches to simplifying robot description in ROS2.

---

## Documentation Structure

### Core Documents

1. **[current.md](current.md)** - Project status and overview
   - Three-approach comparison summary
   - Current examples and results
   - Recommendation matrix
   - Next steps

2. **[design_philosophy.md](design_philosophy.md)** - General principles
   - Motivation and problems with URDF/Xacro
   - Core abstractions for conciseness
   - Robot archetype taxonomy
   - ROS REP standards compliance
   - Trade-offs across approaches

### Approach-Specific Documents

3. **[dsl_approach.md](dsl_approach.md)** - Semantic YAML DSL
   - Syntax specification and examples
   - Templates and mirroring
   - 60% code reduction results
   - BNF grammar reference

4. **[py_approach.md](py_approach.md)** - Python Builder API
   - API reference and patterns
   - Builder pattern explanation
   - 21-48% code reduction results
   - IDE support and debugging

5. **[yaml_approach.md](yaml_approach.md)** - Direct XML→YAML (baseline)
   - Why simple translation fails
   - Comparison showing worse results
   - Lessons learned about semantic improvement

---

## Quick Start

### Understand the Project
Start with [current.md](current.md) for an overview of all three approaches and current status.

### Learn the Philosophy
Read [design_philosophy.md](design_philosophy.md) to understand the design principles and goals.

### Explore Specific Approaches
- Want maximum conciseness? → [dsl_approach.md](dsl_approach.md)
- Python developer? → [py_approach.md](py_approach.md)
- Curious why direct translation fails? → [yaml_approach.md](yaml_approach.md)

### See Examples
Look at the working examples in `output/`:
- `output/dsl/simple_robot.yaml` - DSL version (46 lines)
- `output/py/simple_robot.py` - Python version (51 lines)
- `output/yaml/simple_robot.yaml` - Direct YAML (104 lines - worse!)
- Compare to `example_urdf/simple_robot.urdf` (86 lines - original)

---

## Archived Documentation

Historical documentation from individual approaches:
- `dslurdf/doc/` - Original DSL-specific docs
- `pyurdf/doc/` - Original Python API-specific docs

These are superseded by the consolidated docs in this directory but preserved for reference.

---

## Summary Table

| Approach | Lines | Reduction | Best For | Documentation |
|----------|-------|-----------|----------|---------------|
| **Semantic DSL** | 46 | 47% | Max conciseness | [dsl_approach.md](dsl_approach.md) |
| **Python API** | 51 | 41% | Python devs | [py_approach.md](py_approach.md) |
| **Direct YAML** | 104 | -21% | Nothing (baseline) | [yaml_approach.md](yaml_approach.md) |
| **Original URDF** | 86 | — | Compatibility | N/A |

---

## Contributing

When updating documentation:
1. Update the relevant approach-specific file (dsl_approach.md, py_approach.md)
2. Update [current.md](current.md) with new results or status
3. Update [design_philosophy.md](design_philosophy.md) if adding general principles
4. Keep examples in `output/` in sync with documentation

---

## Paper/Publication

LaTeX paper drafts are in `../tex/` directory. The paper should reference these consolidated docs as the source of truth for the three-approach comparison.
