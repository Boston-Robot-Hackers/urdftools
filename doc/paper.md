# Style guide

* be succinct
* no long abstract statements that mean nothing


# Paper Outline: Comparative Approaches to Simplifying Robot Description in ROS2

**Working Title**: "Beyond URDF: A Comparative Study of Three Approaches to Concise Robot Description in ROS2"

---

## Abstract (Draft)

We compared several different ways to simplify the creation, modifying, maintaining of URDF while maintaining full compatibility with all of ROS2. We looked at three approaches. A simple translation of xml to yaml; a domain specific language with semantic understanding of robotics and robots; a python library to express a robot's structure programatically which generates valid URDF.

The DSL achieves 47-60% code reduction through domain-specific features, while the Python API provides 21-48% reduction with full IDE support and zero learning curve for Python developers. Direct translation shows no improvement or increases verbosity by 21%.

**Keywords**: Robot Description, URDF, Domain-Specific Languages, ROS2, YAML, Python, Code Reduction

---

## 1. Introduction

### 1.1 Motivation

URDF is the standard robot description format in ROS2 for simulation, visualization, and control. A simple differential-drive robot requires 86 lines of nested XML. Visual, collision, and inertial properties must be specified separately for each link. Symmetric components like left and right wheels are manually duplicated. The robot's hierarchical structure is hidden in joint definitions. This verbosity slows development and creates barriers for newcomers. 

### 1.2 Requirements

1. Must be 100% compatible with all tools or applications which use URDF
2. Must be subtantially simpler than URDF (as measured in developer effort, or lines of code)
3. Must leverage and not undermine ROS2 and URDF knowledge, concepts and learnings of the user

### 1.3 Our Contribution

We compare three approaches to simplifying URDF creation. The Semantic YAML DSL uses templates, mirroring, and smart defaults to achieve 47-60% code reduction. The Python Builder API leverages standard Python patterns for 21-48% reduction with full IDE support. Direct XML→YAML approach shows that format conversion alone provides no benefit and can increase verbosity by 21%. 

### 2. Background and Related Work

Complexity of dealing with URDF's XML syntax is often discussed. Newcomers may notice that there are more efficient and clear ways to define a robot's structure. Experts may notice that XML is well understood and supported, and the URDF for a robot does not change a lot.

### 2.1 Alternative Robot Description Formats
- SMURF (YAML augmentation of URDF)
- SDF (Simulation Description Format)
- AMBF, PyDSLRep (simulation-specific)
- MJCF (MuJoCo)
- None provide ROS2-compatible pure replacement with semantic improvements

---

## 3. URDF Challenges

Below is a brief rehearsal of possible objections or challenges for developers writing and maintaining urdf descriptions.

### 3.1 XML Verbosity

A simple cylinder link requires 15+ lines of nested XML. Visual, collision, and inertial properties are specified in separate blocks. The same geometry is often repeated for visual and collision. Inertia tensors must be calculated manually. A differential-drive robot with two wheels, a caster, and a lidar sensor requires 86 lines of URDF.

### 3.2 Hidden Structure

Parent-child relationships are buried within joint definitions rather than explicit. The robot's TF tree is not visible at a glance. Understanding the overall topology requires reading through all joint definitions and mentally reconstructing the hierarchy.

### 3.3 Repetition

Symmetric components like left and right wheels must be manually duplicated with only position coordinates changed. Material colors are specified as RGBA values repeatedly throughout the file. Common patterns like differential drive bases have no reusable abstractions.

### 3.4 Xacro Limitations

Xacro adds macros but remains XML-based with nested structure. Expression syntax uses cryptic `${}` notation. Macro debugging is difficult as errors appear in expanded XML. Developers must learn XML, Xacro macro syntax, and ROS concepts simultaneously.

---

## 4. Design Philosophy

*This section presents principles applicable across all approaches*

### 4.1 Core Abstractions for Conciseness
1. **Link/Joint Templates**: Reusable component definitions
2. **Hierarchical Grouping**: Explicit structure declarations
3. **Duplication Handling**: Symmetric parts, mirroring, iteration
4. **Transform Abstractions**: Simplified origin specifications
5. **Geometry References**: Built-in primitives, mesh libraries
6. **Smart Defaults**: Collision=visual, inertia auto-calculation, fixed joints
7. **Material Library**: Named materials over RGBA values
8. **Semantic Shortcuts**: Common robot archetypes

### 4.2 Inference Strategies
- Structural: topology → positions
- Geometric: shapes → inertia tensors
- Semantic: wheel → continuous joint
- Contextual: material inheritance

### 4.3 ROS REP Standards Compliance
- REP-105: Coordinate frames
- REP-120: Humanoid conventions
- REP-199: Inertial parameters
- REP-103: Units and conventions

### 4.4 Design Principles Summary
1. Reduce boilerplate, not features
2. Expose structure clearly
3. Infer from context
4. Leverage existing knowledge
5. Provide escape hatches
6. Follow ROS standards
7. Enable good tooling
8. Optimize for teaching

---

## 5. Semantic YAML DSL Approach

### 5.1 Overview
Domain-specific language achieving maximum conciseness through semantic features

**Goal**: 60% code reduction while improving readability

### 5.2 Key Design Decisions

#### 5.2.1 Flat Syntax
Geometry as top-level fields, not nested blocks:
```yaml
base_link:
  cylinder: {radius: 0.14, length: 0.003}
  material: yellow
  mass: 1.0
```

#### 5.2.2 Hierarchy First
TF tree visible at file top:
```yaml
hierarchy:
  base_footprint:
    - base_link:
        - left_wheel
        - right_wheel
        - lidar
```

#### 5.2.3 Smart Defaults
- Collision = visual (unless overridden)
- Inertia auto-calculated from geometry + mass
- Origin defaults to [0,0,0]
- Joints default to fixed
- Rotation defaults to [0,0,0]

#### 5.2.4 Templates and Mirroring
```yaml
templates:
  wheel:
    cylinder: [{wheel_radius}, {wheel_width}]
    material: black
    joint_type: continuous
    axis: [0, 1, 0]

wheels:
  template: wheel
  mirror_y:
    names: [left_wheel, right_wheel]
    origin: {xyz: [0, {wheel_base/2}, {-wheel_radius}]}
```

#### 5.2.5 Variable Substitution
Parameters with arithmetic expressions:
```yaml
params:
  wheel_base: 0.24
  wheel_radius: 0.05

# Later
origin: {xyz: [0, {wheel_base/2}, {-wheel_radius}]}
```

### 5.3 Syntax Specification
- File structure (robot, hierarchy, params, templates, links)
- Geometry specifications (cylinder, box, sphere, mesh, polygon)
- Joint types (fixed, continuous, revolute)
- Origins and transforms
- Mirroring operations

### 5.4 Example: Simple Robot
**Original URDF**: 86 lines
**DSL version**: 46 lines
**Reduction**: 47%

*Include side-by-side code comparison*

### 5.5 Implementation
- YAML→URDF compiler architecture
- Template expansion
- Expression evaluator
- Validation and error checking
- BNF grammar (appendix reference)

### 5.6 Advantages
- Maximum conciseness (60% reduction)
- Immediate visibility of structure
- Declarative approach
- Consistency through templates

### 5.7 Limitations
- New syntax to learn
- Limited to DSL-supported features
- Requires compiler tool
- Less flexible than programming language

---

## 6. Python Builder API Approach

### 6.1 Overview
Fluent Python library achieving moderate reduction with zero learning curve

**Goal**: Leverage Python's familiarity and tooling ecosystem

### 6.2 Key Design Decisions

#### 6.2.1 Builder Pattern (Not Fluent Chaining)
One operation per line encouraged for clarity:
```python
base_link = base_footprint.link("base_link")
base_link.cylinder(0.15, 0.05)
base_link.mass(2.0)
```

#### 6.2.2 Combined Methods
Frequently-used operations in single calls:
```python
# Geometry + material
.cylinder(radius, length, material="gray")

# Joint type + axis
.continuous_joint(axis="y")
```

#### 6.2.3 Separate Parameters (Optional)
```python
# robot_params.py
class RobotParams:
    base_radius = 0.15
    wheel_radius = 0.05

# robot_build.py
from robot_params import RobotParams as p
base_link.cylinder(p.base_radius, p.base_height)
```

#### 6.2.4 Python Patterns for Duplication
```python
# Loop for symmetric parts
for side, reflect in [("left", 1), ("right", -1)]:
    y_pos = reflect * wheel_separation / 2
    wheel = base_link.link(f"{side}_wheel")
    wheel.cylinder(wheel_radius, wheel_width)
    wheel.continuous_joint(axis="y")
    wheel.joint_origin([0, y_pos, -0.025])
```

### 6.3 API Reference
- Core classes (Robot, Link)
- Geometry methods
- Joint methods
- Origin and transform methods
- Axis shortcuts

### 6.4 Example: Simple Robot
**Original URDF**: 86 lines
**Python version**: 51 lines
**Reduction**: 41%

*Include side-by-side code comparison*

### 6.5 Complex Robot Example: Dome1
**Original**: 326 lines URDF/Xacro
**Python**: 168 lines (80 params + 88 build)
**Reduction**: 48%

### 6.6 Development Experience
- Full IDE support (autocomplete, type checking, refactoring)
- Standard debugging (breakpoints, print, pdb)
- Testing with pytest/unittest
- Version control friendly

### 6.7 Advantages
- No new syntax (Python developers)
- Full programming language power
- Excellent tooling support
- Programmatic flexibility
- Standard testing frameworks

### 6.8 Limitations
- Less concise than DSL
- Requires Python knowledge
- Imperative rather than declarative
- More verbose for simple robots

---

## 7. Direct XML→YAML Translation (Baseline)

### 7.1 Overview
Simple format conversion with 1:1 structural mapping

**Goal**: Establish baseline - does format change alone help?

### 7.2 Translation Approach
XML elements → YAML keys
XML attributes → YAML keys with `@` prefix

```yaml
link:
  '@name': base_link
  visual:
    geometry:
      cylinder:
        '@radius': '0.15'
        '@length': '0.05'
```

### 7.3 Results
**Original URDF**: 86 lines
**Direct YAML**: 104 lines
**Change**: +21% (WORSE!)

### 7.4 Why It Fails
- YAML structure requires more lines than XML
- Awkward `@` syntax
- String values for attributes
- No semantic improvements
- Combines worst of both formats

### 7.5 What's Missing
- No hierarchy section
- No smart defaults
- No templates
- No parameter reuse
- No abstraction capabilities

### 7.6 Lessons Learned
- **Format ≠ Improvement**: Structure matters more than syntax
- **Semantics Matter**: Real gains require domain abstractions
- **Tools Must Add Value**: Beyond simple translation

### 7.7 When It Might Be Useful
- Initial conversion step (then manual improvement)
- Teams with strong YAML preference
- Tooling compatibility requirements

**Verdict**: Not recommended - use DSL or Python API instead

---

## 8. Comparative Evaluation

### 8.1 Methodology
- Three representative robots (simple, medium, complex)
- Metrics: lines of code, readability, maintainability, learnability
- Implementation in all three approaches
- Comparison to original URDF/Xacro

### 8.2 Test Cases

#### 8.2.1 Simple Robot (Diff-Drive with Lidar)
| Approach | Lines | Reduction | Notes |
|----------|-------|-----------|-------|
| Original URDF | 86 | baseline | XML verbosity |
| Direct YAML | 104 | -21% | Worse than original! |
| Semantic DSL | 46 | 47% | Templates, mirroring |
| Python API | 51 | 41% | Loops, functions |

#### 8.2.2 Floora Robot (4-Caster Platform)
| Approach | Lines | Reduction | Notes |
|----------|-------|-----------|-------|
| Original Xacro | 300 | baseline | Complex macros |
| Python API | 177 | 41% | Helper functions |

#### 8.2.3 Dome1 Robot (Two-Tier Platform)
| Approach | Lines | Reduction | Notes |
|----------|-------|-----------|-------|
| Original Xacro | 326 | baseline | Repetitive structure |
| Semantic DSL | ~130 | 60% | Maximum conciseness |
| Python API | 168 | 48% | Data-driven components |

### 8.3 Quantitative Results Summary

**Code Reduction:**
- Semantic DSL: 47-60% reduction
- Python API: 21-48% reduction
- Direct YAML: 0% to -21% (worse)

**Pattern**: Both DSL and Python API provide significant benefits; direct translation does not

### 8.4 Qualitative Comparison

#### Readability
- **DSL**: Excellent (flat syntax, visible hierarchy)
- **Python**: Good (familiar syntax, clear structure)
- **Direct YAML**: Poor (nested structure, `@` attributes)

#### Learnability
- **DSL**: Medium (new syntax, but approachable)
- **Python**: Low (no new syntax for Python devs)
- **Direct YAML**: High (XML concepts + YAML + Xacro)

#### Maintainability
- **DSL**: Excellent (templates enforce consistency)
- **Python**: Excellent (functions, testing, version control)
- **Direct YAML**: Poor (duplication, no abstractions)

#### Tooling
- **DSL**: Medium (YAML schema validation)
- **Python**: Excellent (full IDE support)
- **Direct YAML**: Low (basic YAML tools)

#### Flexibility
- **DSL**: Limited (constrained to DSL features)
- **Python**: Excellent (full programming language)
- **Direct YAML**: Limited (static definitions)

### 8.5 Trade-off Analysis

| Criterion | DSL Winner | Python Winner | Tie |
|-----------|------------|---------------|-----|
| Conciseness | ✓ | | |
| Learning Curve (Python devs) | | ✓ | |
| Learning Curve (Non-programmers) | ✓ | | |
| Tooling Support | | ✓ | |
| Flexibility | | ✓ | |
| Declarative Clarity | ✓ | | |
| Debugging | | ✓ | |
| ROS2 Compatibility | | | ✓ |

### 8.6 User Study (Future Work)
Preliminary informal feedback:
- Students: Prefer DSL (approachable, clear structure)
- Python developers: Prefer Python API (familiar, powerful)
- Educators: Prefer DSL (easier to teach)
- Industry: Mixed (depends on team skills)

---

## 9. Discussion and Recommendations

### 9.1 Recommendation Matrix

| User Profile | Best Approach | Rationale |
|--------------|---------------|-----------|
| **Want maximum conciseness** | Semantic DSL | 60% reduction, clear structure |
| **Python developers** | Python API | No new syntax, full tooling |
| **Large/complex robots** | Python API or DSL | Best abstraction capabilities |
| **Teams without coding background** | Semantic DSL | Declarative, no programming |
| **Rapid prototyping** | Python API | Full programmatic control |
| **Educational use** | Semantic DSL | Clear hierarchy, approachable |
| **Conservative projects** | Original URDF/Xacro | Established ecosystem |

### 9.2 Why Semantic Improvements Matter

**Key Finding**: Format conversion alone (Direct YAML) provides no benefit or makes things worse. Real improvements require:
1. Domain-specific abstractions (templates, mirroring)
2. Smart defaults (infer collision, inertia, origins)
3. Structure exposure (hierarchy sections)
4. Duplication elimination (loops, templates, expressions)

### 9.3 Educational Impact

#### DSL Benefits for Teaching
- Lower barrier to entry (YAML vs XML)
- Visible structure (hierarchy section)
- Less overwhelming (46 vs 86 lines)
- Clearer semantics (named materials, templates)

#### Python API Benefits for Teaching
- Builds on existing Python knowledge
- Natural progression from parameters to building
- Debugging skills transferable
- Testing reinforces best practices

### 9.4 Community Adoption Pathways

#### Incremental Adoption
Both approaches compile to standard URDF:
- No changes to ROS2 tools required
- Can coexist with existing URDF files
- Teams can migrate gradually

#### Tooling Integration
- DSL: Editor plugins, syntax highlighting, validation
- Python API: Already has IDE support
- Both: Integration with `colcon build`, `ros2 launch`

#### Ecosystem Impact
- Reduces barrier for newcomers
- Enables more complex robots through abstraction
- Improves maintainability of existing robots
- Potential influence on future ROS standards

### 9.5 Limitations and Trade-offs

#### DSL Limitations
- New syntax learning curve
- Constrained to DSL-supported features
- Requires compiler in toolchain
- Less flexible than programming language

#### Python API Limitations
- Requires Python knowledge
- Less concise than DSL
- Imperative rather than declarative
- May be overkill for very simple robots

#### Both Approaches
- Not officially part of ROS2 (community tools)
- Require compilation step (not native URDF)
- Learning investment required
- Tool maturity compared to URDF/Xacro

### 9.6 Future Extensions

#### DSL Future Work
- Conditional logic (if/then)
- Advanced mirroring (X-axis, Z-axis, arbitrary planes)
- Explicit collision overrides
- Gazebo simulation configuration
- Plugin system for custom generators

#### Python API Future Work
- Type hints for full IDE support
- Inertia auto-calculation
- Validation and topology checking
- Unit testing framework
- Package for pip installation

#### Common Future Work
- User studies (onboarding time, error rates)
- Integration with CAD tools
- Visual editors
- ROS2 package integration
- Community-driven robot type library

---

## 10. Conclusion

### 10.1 Summary of Contributions

We presented a comparative study of three approaches to simplifying robot description in ROS2:

1. **Semantic YAML DSL**: Achieves 47-60% code reduction through domain-specific features (templates, mirroring, smart defaults, hierarchy sections)

2. **Python Builder API**: Achieves 21-48% code reduction while leveraging standard Python patterns, full IDE support, and zero learning curve for Python developers

3. **Direct XML→YAML Translation**: Demonstrates that format conversion alone provides no benefit, validating the need for semantic improvements

### 10.2 Key Findings

**Format is not the problem; semantics are the solution.**
- Direct translation fails (0% or negative improvement)
- Both DSL and Python API succeed through semantic abstractions
- Choice depends on user profile and project needs

**Trade-offs matter:**
- DSL: Maximum conciseness, approachable syntax (new learning)
- Python: Moderate conciseness, familiar syntax (requires programming)
- Direct YAML: Avoid entirely

### 10.3 Recommendations

**For educators**: Start with Semantic DSL (lower barrier, visible structure)

**For Python developers**: Use Python Builder API (familiar, powerful, great tooling)

**For complex robots**: Either DSL or Python API (both support abstractions)

**For simple format conversion**: Don't bother—no benefit without semantic improvements

### 10.4 Broader Impact

This work demonstrates that:
- Domain-specific abstractions can dramatically reduce robot description complexity
- Multiple valid approaches exist—choice depends on user needs
- Semantic improvements matter more than syntax choices
- Educational accessibility and technical rigor can coexist

### 10.5 Future Work

1. **Implementation**: Complete compilers for both DSL and Python API
2. **Validation**: Formal user studies measuring onboarding time and error rates
3. **Extensions**: Advanced features (conditionals, validation, plugins)
4. **Integration**: ROS2 package ecosystem integration
5. **Standardization**: Community engagement toward potential ROS standards

### 10.6 Final Thoughts

The URDF verbosity problem has persisted for over a decade. Our comparative study shows that solutions exist, but there is no single "best" approach. Rather, the ROS2 community would benefit from multiple tools targeting different user profiles. By providing both a concise DSL and a flexible Python API, we enable newcomers and experts alike to work more efficiently, ultimately accelerating robot development and education.

---

## References

### ROS and URDF
- ROS2 Documentation
- URDF Specification (ROS Wiki)
- Xacro Documentation
- REP-105, REP-120, REP-199, REP-103

### Related Work
- SMURF
- SDF (Simulation Description Format)
- MJCF (MuJoCo)
- Community "URDF 2.0" discussions

### DSL and Language Design
- Domain-Specific Languages (Fowler)
- YAML Specification
- Builder Pattern (Gang of Four)

### Example Robots
- Linorobot 2WD
- TurtleBot variants
- Educational robot platforms

---

## Appendices

### Appendix A: DSL BNF Grammar
Complete formal grammar specification

### Appendix B: Python API Complete Reference
Full API documentation with all methods

### Appendix C: Complete Code Examples
Side-by-side comparisons for all three test robots

### Appendix D: Conversion Tools
Instructions for URDF→DSL and URDF→Python conversion

### Appendix E: Installation and Usage
Getting started guide for both approaches
