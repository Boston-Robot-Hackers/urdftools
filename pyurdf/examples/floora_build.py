"""
Floora Robot - URDF build script using pyurdf API

This demonstrates building a 4-caster mobile robot with:
- Differential drive (2 wheels)
- 4 swivel casters positioned at 45° intervals
- Each caster has bearing (vertical rotation) and wheel (horizontal rotation)
"""

from pyurdf import urdfbuilder as ub
from floora_params import FlooraParams as p
import math

# Create robot
robot = ub.urdf("floora")

# Base footprint (ground projection)
base_footprint = robot.link("base_footprint")

# Base link with collision override
base_link = base_footprint.link("base_link")
base_link.cylinder(p.base_radius, p.base_thickness, material="bamboo")
base_link.origin([0, 0, p.base_visual_zoff])  # Visual offset
base_link.mass(p.base_mass)
base_link.inertia(p.base_inertia)
base_link.collision(
    geometry=("cylinder", p.base_radius, p.base_collision_height),
    origin=[0, 0, -p.base_collision_height/2]
)
base_link.joint_origin([0, 0, p.wheel_zoff + p.wheel_radius])  # Above ground


# Drive wheels - using loop for symmetry
for side, reflect in [("wheel_l", 1), ("wheel_r", -1)]:
    y_pos = reflect * (p.base_radius + p.wheel_ygap)

    wheel = base_link.link(side)
    wheel.cylinder(p.wheel_radius, p.wheel_width, material="gray")
    wheel.origin([0, 0, 0], [math.pi/2, 0, 0])  # Rotate to horizontal
    wheel.continuous_joint(axis="y")
    wheel.joint_origin([0, y_pos, -p.wheel_zoff])  # Wheels below base
    wheel.mass(p.wheel_mass)
    wheel.inertia(p.wheel_inertia)


# Helper function for creating casters
def add_caster(parent, name, angle):
    """Add a swivel caster (bearing + wheel) at given angle"""

    # Calculate position using polar coordinates
    x, y = p.caster_position(angle)

    # Caster bearing (vertical swivel)
    bearing = parent.link(f"{name}_bearing_link")
    bearing.cylinder(p.caster_bearing_radius, p.caster_bearing_width, material="cyan")
    bearing.continuous_joint(axis="z")  # Vertical rotation
    bearing.joint_origin([x, y, -p.caster_bearing_zoff])  # Below base at angle
    bearing.mass(p.caster_bearing_mass)
    bearing.inertia(p.caster_bearing_inertia)
    bearing.collision(
        geometry=("cylinder",
                  p.caster_bearing_radius + p.collision_margin,
                  p.caster_bearing_width)
    )

    # Caster wheel (child of bearing)
    wheel = bearing.link(f"{name}_wheel_link")
    wheel.cylinder(p.caster_wheel_radius, p.caster_wheel_width, material="gray")
    wheel.origin([0, 0, 0], [math.pi/2, 0, 0])  # Rotate to horizontal
    wheel.continuous_joint(axis="y")  # Horizontal rotation
    wheel.joint_origin([-p.caster_wheel_xoff, 0, -p.caster_wheel_zoff])  # Offset from bearing
    wheel.mass(p.caster_wheel_mass)
    wheel.inertia(p.caster_wheel_inertia)

    return bearing


# Add four casters at their respective angles
add_caster(base_link, "caster_fl", p.caster_fl_angle)   # Front-left: 45°
add_caster(base_link, "caster_bl", p.caster_bl_angle)   # Back-left: 135°
add_caster(base_link, "caster_br", p.caster_br_angle)   # Back-right: 225°
add_caster(base_link, "caster_fr", p.caster_fr_angle)   # Front-right: 315°


# Generate URDF
robot.save("floora.urdf")

print("Floora robot URDF generated successfully!")
print(f"Total links: 13 (1 footprint + 1 base + 2 wheels + 8 caster parts)")
print(f"Total mass: ~{p.base_mass + 2*p.wheel_mass + 4*(p.caster_bearing_mass + p.caster_wheel_mass):.2f}kg")
