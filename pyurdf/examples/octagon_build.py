"""
Build script for simple octagon robot using urdfbuilder API
"""

from pyurdf import urdfbuilder as ub
from octagon_params import OctagonParams as p

# Create robot with base hierarchy
robot = ub.urdf(p.robot_name)
base_footprint = robot.link("base_footprint")

base_link = base_footprint.link("base_link")
base_link.octagon(p.base_radius, p.base_thickness, material="clear")
base_link.joint_origin([0, 0, p.base_height])

# Wheels - loop for symmetric parts
for side, reflect in [("left", 1), ("right", -1)]:
    y_pos = reflect * p.wheel_y_offset
    wheel = base_link.link(f"{side}_wheel")
    wheel.cylinder(p.wheel_radius, p.wheel_width, material="black")
    wheel.continuous_joint(axis="y")
    wheel.origin([0, 0, 0], p.wheel_rotation)
    wheel.joint_origin([0, y_pos, p.wheel_z_offset])

# Generate URDF (when implementation is complete)
robot.save("octagon_bot.urdf")
