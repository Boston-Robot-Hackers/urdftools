# Direct XML→YAML Translation Approach

The direct XML→YAML translation approach converts URDF XML to YAML format with minimal semantic improvements. This approach is included primarily as a **baseline for comparison**, demonstrating why simple format conversion is insufficient.

---

## Overview

**Goal**: Translate URDF XML directly to YAML format with 1:1 structural mapping.

**Result**: Minimal or negative benefit - often **increases** verbosity.

**Verdict**: ❌ **Not Recommended** - included only to show why semantic improvements are necessary.

---

## Translation Approach

### Basic Mapping

XML elements become YAML keys, XML attributes become YAML keys with `@` prefix:

```xml
<!-- URDF XML -->
<link name="base_link">
  <visual>
    <geometry>
      <cylinder radius="0.15" length="0.05"/>
    </geometry>
  </visual>
</link>
```

```yaml
# Direct YAML translation
link:
  '@name': base_link
  visual:
    geometry:
      cylinder:
        '@radius': '0.15'
        '@length': '0.05'
```

### Problems Immediately Apparent

1. **More verbose**: YAML structure requires more lines than XML
2. **Awkward `@` syntax**: Non-idiomatic YAML
3. **String values**: Attributes become strings (`'0.15'` not `0.15`)
4. **No semantic improvement**: Same structure, different format

---

## Comparison Results

### Simple Robot Example

See [output/yaml/simple_robot.yaml](../output/yaml/simple_robot.yaml)

**Line counts:**
- Original URDF: 86 lines
- Direct YAML: 104 lines
- **Change**: +21% (WORSE!)

### Why It's Worse

```yaml
# 8 lines in YAML
joint:
  '@name': left_wheel_joint
  '@type': continuous
  parent:
    '@link': base_link
  child:
    '@link': left_wheel
  origin:
    '@xyz': '0 0.12 -0.025'
    '@rpy': '0 0 0'
  axis:
    '@xyz': '0 1 0'

# vs 7 lines in XML
<joint name="left_wheel_joint" type="continuous">
  <parent link="base_link"/>
  <child link="left_wheel"/>
  <origin xyz="0 0.12 -0.025" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
</joint>
```

YAML's structure (key-value pairs) creates more indentation levels than XML's compact attributes.

---

## What's Missing

This approach lacks all the improvements that make DSL and Python API effective:

### No Hierarchy Section
```yaml
# Structure buried in joint definitions
joint:
  '@name': base_footprint_to_base_link
  parent: {'@link': base_footprint}
  child: {'@link': base_link}

# vs DSL - structure visible at top
hierarchy:
  base_footprint:
    - base_link
```

### No Smart Defaults
```yaml
# Must specify everything explicitly
link:
  '@name': sensor
  visual:
    geometry:
      cylinder: {'@radius': '0.03', '@length': '0.04'}
  collision:
    geometry:
      cylinder: {'@radius': '0.03', '@length': '0.04'}

# vs DSL - collision=visual by default
sensor:
  cylinder: [0.03, 0.04]
```

### No Templates
```yaml
# Must repeat wheel definition
- '@name': left_wheel
  visual:
    origin: {'@xyz': '0 0 0', '@rpy': '1.57 0 0'}
    geometry:
      cylinder: {'@radius': '0.05', '@length': '0.03'}

- '@name': right_wheel
  visual:
    origin: {'@xyz': '0 0 0', '@rpy': '1.57 0 0'}
    geometry:
      cylinder: {'@radius': '0.05', '@length': '0.03'}

# vs DSL - template with mirroring
templates:
  wheel:
    cylinder: [0.05, 0.03]
    origin: {rpy: [90deg, 0, 0]}

wheels:
  template: wheel
  mirror_y: {names: [left_wheel, right_wheel]}
```

### No Parameter Reuse
```yaml
# Hard-coded values repeated
origin: {'@xyz': '0 0.12 -0.025'}
# later
origin: {'@xyz': '0 -0.12 -0.025'}

# vs DSL - parameters with expressions
params:
  wheel_separation: 0.24

# later
origin: {xyz: [0, {wheel_separation/2}, -0.025]}
origin: {xyz: [0, {-wheel_separation/2}, -0.025]}
```

---

## When This Approach Might Be Used

Despite its limitations, direct translation could be useful for:

1. **Initial conversion tool**: Start with YAML, then manually improve
2. **YAML preference**: Teams that strongly prefer YAML over XML
3. **Tooling compatibility**: If tools require YAML input

However, in practice:
- **Just use URDF/XML**: If no semantic improvements, XML is fine
- **Or use a proper DSL**: If converting anyway, get the benefits

---

## Lessons Learned

This approach teaches important lessons about DSL design:

### Format ≠ Improvement
Simply changing format (XML → YAML) provides no benefit. The structure and verbosity remain the same.

### Semantics Matter
Real improvements require:
- Understanding robot structure (hierarchy, symmetry)
- Eliminating redundancy (smart defaults, templates)
- Better abstractions (templates, mirroring, expressions)

### Tools Must Add Value
A conversion tool must provide value beyond format translation:
- ✅ **DSL**: Adds templates, defaults, expressions → 60% reduction
- ✅ **Python API**: Adds loops, functions, IDE support → 41% reduction
- ❌ **Direct YAML**: Adds... nothing → 0% reduction (or worse)

---

## Comparison Summary

| Approach | Lines | Change | Value Added |
|----------|-------|--------|-------------|
| **Original URDF** | 86 | baseline | Standard format |
| **Direct YAML** | 104 | +21% | None |
| **Semantic DSL** | 46 | -47% | Templates, defaults, hierarchy |
| **Python API** | 51 | -41% | Loops, IDE support, debugging |

---

## Conclusion

Direct XML→YAML translation is included in this comparison to demonstrate that **format conversion alone is insufficient**. Effective improvements require semantic enhancements:

- **DSL approach**: Built-in semantic features (templates, mirroring, defaults)
- **Python approach**: Leverages programming language features (loops, functions)
- **Direct YAML**: No semantic improvements → no benefit

**Recommendation**: Avoid direct translation. Use Semantic DSL for maximum conciseness or Python API for maximum flexibility.

---

## References

- **Current Project State**: [current.md](current.md)
- **Design Philosophy**: [design_philosophy.md](design_philosophy.md)
- **DSL Approach**: [dsl_approach.md](dsl_approach.md)
- **Python Approach**: [py_approach.md](py_approach.md)
- **Example**: [output/yaml/simple_robot.yaml](../output/yaml/simple_robot.yaml)
