"""
Simple differential drive robot - Python Builder API
"""

from pyurdf import urdfbuilder as ub
import math

# Parameters
base_radius = 0.15
base_height = 0.05
wheel_radius = 0.05
wheel_width = 0.03
wheel_separation = 0.24
caster_radius = 0.025
lidar_radius = 0.03
lidar_height = 0.04

# Create robot
robot = ub.urdf("simple_diffbot")

# Base footprint (virtual link at ground level)
base_footprint = robot.link("base_footprint")

# Base link (circular platform)
base_link = base_footprint.link("base_link")
base_link.cylinder(base_radius, base_height)
base_link.joint_origin([0, 0, base_height])

# Drive wheels - using loop for symmetry
for side, reflect in [("left", 1), ("right", -1)]:
    y_pos = reflect * wheel_separation / 2

    wheel = base_link.link(f"{side}_wheel")
    wheel.cylinder(wheel_radius, wheel_width)
    wheel.origin([0, 0, 0], [math.pi/2, 0, 0])  # Rotate to horizontal
    wheel.continuous_joint(axis="y")
    wheel.joint_origin([0, y_pos, -0.025])

# Caster (simple sphere at back)
caster = base_link.link("caster")
caster.sphere(caster_radius)
caster.joint_origin([-0.12, 0, -0.05])

# Lidar sensor
lidar = base_link.link("lidar")
lidar.cylinder(lidar_radius, lidar_height)
lidar.joint_origin([0, 0, 0.045])

# Generate URDF
robot.save("simple_robot_generated.urdf")
