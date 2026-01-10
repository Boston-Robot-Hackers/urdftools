"""
Example robot definition using urdfbuilder API
"""

from pyurdf import urdfbuilder as ub
from robot_params import RobotParams as p

# Create robot with base hierarchy
robot = ub.urdf("MyRobot")
base_footprint = robot.link("base_footprint")

# Base link
base_link = base_footprint.link("base_link")
base_link.cylinder(p.base_radius, p.base_thickness, material="bamboo")
base_link.mass(p.base_mass)

# Left wheel
left_wheel = base_link.link("left_wheel")
left_wheel.cylinder(p.wheel_radius, p.wheel_width, material="gray")
left_wheel.continuous_joint(axis="y")
left_wheel.origin([0, p.wheel_y_offset, p.wheel_z_offset], [90, 0, 0])

# Right wheel - mirrored y position
right_wheel = base_link.link("right_wheel")
right_wheel.cylinder(p.wheel_radius, p.wheel_width, material="gray")
right_wheel.continuous_joint(axis="y")
right_wheel.origin([0, -p.wheel_y_offset, p.wheel_z_offset], [90, 0, 0])

# Lidar sensor
lidar = base_link.link("base_scan")
lidar.mesh("package://my_robot/meshes/lidar.stl")
lidar.origin([0, 0, p.lidar_z_offset])

# Generate URDF
robot.save("my_robot.urdf")
# or
print(robot.to_urdf())
