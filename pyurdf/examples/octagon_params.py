"""
Parameters for simple octagon robot
- Two wheels (7cm diameter, 2cm width)
- Octagonal base (30cm diameter, octagon primitive)
- Base 5cm off ground
"""

class OctagonParams:
    # Robot identity
    robot_name = "octagon_bot"

    # Base dimensions
    base_radius = 0.15          # 30cm diameter octagon = 15cm radius
    base_thickness = 0.003      # 3mm thick plate
    base_height = 0.05          # 5cm above ground
    base_mass = 2.0             # kg

    # Wheel dimensions
    wheel_diameter = 0.07       # 7cm diameter
    wheel_radius = wheel_diameter / 2  # 3.5cm radius
    wheel_width = 0.02          # 2cm width
    wheel_separation = 0.20     # 20cm between wheels
    wheel_mass = 0.3            # kg per wheel

    # Calculated positions
    # Wheels touch ground, so wheel center is at wheel_radius from ground
    # wheel_z = wheel_radius - base_height
    wheel_z_offset = wheel_radius - base_height  # -0.015m
    wheel_y_offset = wheel_separation / 2        # 0.10m

    # Visual rotation for wheels (90 degrees around X to stand upright)
    wheel_rotation = [1.5708, 0, 0]  # [90°, 0, 0] in radians
