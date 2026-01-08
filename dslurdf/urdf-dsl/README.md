# URDF DSL Examples

This directory contains robot descriptions written in the proposed YAML-based URDF DSL.

## Files

- **turtlebot.yaml** - TurtleBot-style differential drive robot with sensors
  - Converted from `urdf_examples/urdf/robots/turtlebot.urdf.xacro`
  - Demonstrates: differential drive, casters, IMU, lidar, depth camera
  - Shows inertial properties and Gazebo controller configuration

## Key Features Demonstrated

### 1. Robot Type Declaration
```yaml
type: mobile_platform
  drive_type: differential
  wheels: 2
  casters: 2
  sensors:
    imu: 1
    lidar: 1
    depth_camera: 1
```

### 2. Parameterization
All dimensions, masses, and positions defined once in `params:` section and referenced throughout.

### 3. Templates
Reusable `wheel` and `caster` templates with inertial properties.

### 4. Smart Defaults
- `inertia: auto` - automatically calculates inertia tensor from geometry and mass
- Collision geometry defaults to visual geometry
- Origin defaults to `[0, 0, 0]`
- Joint type defaults to `fixed`

### 5. Mirror Patterns
```yaml
wheels:
  template: wheel
  mirror_y:
    origin: {xyz: [0, 0.175, 0]}
    names: [left_wheel_link, right_wheel_link]
```

### 6. Gazebo Integration
Simulation-specific configuration in dedicated `gazebo:` section.

## Comparison to Original

**Original Xacro**: ~98 lines across 7 files
- turtlebot.urdf.xacro (98 lines)
- Plus 6 included files (base, wheel, caster, imu, laser, depth_sensor)

**DSL Version**: ~120 lines in 1 self-contained file
- All templates inline
- No external dependencies
- More readable structure
- Explicit inertial properties
