"""
Dome1 Robot - URDF build script using pyurdf API

This demonstrates building a two-tier platform robot with:
- Two circular plates (bottom and top) connected by support posts
- Differential drive (2 wheels)
- Laser sensor with support tower
- IMU sensor
- Front directional marker
"""

from pyurdf import urdfbuilder as ub
from dome1_params import Dome1Params as p
import math

# Create robot
robot = ub.urdf("dome_x1")

# Base footprint (ground projection) - visible plate on ground
base_footprint = robot.link("base_footprint")
base_footprint.cylinder(p.plate_diameter/2, p.plate_thickness, material="plum")

# Base link - small sphere marker showing robot center
base_link = base_footprint.link("base_link")
base_link.sphere(p.base_link_marker_radius, material="gold")
base_link.joint_origin([-p.base_link_x_offset, 0, p.bottom_plate_height])

# Bottom plate - coral colored platform
bottom_plate = base_link.link("bottom_plate")
bottom_plate.cylinder(p.plate_diameter/2, p.plate_thickness, material="coral")
bottom_plate.joint_origin([-p.base_link_x_offset, 0, 0])

# Top plate - charcoal colored platform
z_between_plates = p.top_plate_height - p.bottom_plate_height

top_plate = base_link.link("top_plate")
top_plate.cylinder(p.plate_diameter/2, p.plate_thickness, material="charcoal")
top_plate.joint_origin([-p.base_link_x_offset, 0, z_between_plates])

# Support posts connecting bottom and top plates
for name, x_pos, y_pos in p.support_posts:
    post = base_link.link(name)
    post.cylinder(p.support_post_diameter/2, p.support_post_height, material="mustard")
    post.joint_origin([x_pos, y_pos, z_between_plates/2])
    post.collision(
        geometry=("cylinder", p.support_post_diameter/2, p.support_post_diameter)
    )

# Drive wheels - using loop for symmetry
for side, reflect in [("left", 1), ("right", -1)]:
    y_pos = reflect * p.wheel_base/2
    z_pos = -p.wheel_diameter/2 - p.wheel_ground_clearance

    wheel = base_link.link(f"{side}_wheel")
    wheel.cylinder(p.wheel_diameter/2, p.wheel_width, material="steel")
    wheel.origin([0, 0, 0], [math.pi/2, 0, 0])  # Rotate to horizontal
    wheel.continuous_joint(axis="y")
    wheel.joint_origin([p.wheel_x_offset, y_pos, z_pos])

# Laser tower (support post attached to top plate)
laser_tower = top_plate.link("laser_tower")
laser_tower.cylinder(p.laser_tower_radius, p.laser_tower_height, material="light_blue")
laser_tower.joint_origin([p.laser_tower_x_offset, p.laser_tower_y_offset, p.laser_tower_z_offset])

# Laser sensor (attached to base_link, rotated)
laser = base_link.link("laser")
laser.cylinder(p.laser_radius, p.laser_height, material="red")
laser.origin([0, 0, 0], [0, 0, math.pi/2])  # Rotate 90° around Z
laser.joint_origin([p.laser_x_offset, 0, p.laser_z_offset], [0, 0, -math.pi/2])

# IMU sensor
imu_link = base_link.link("imu_link")
imu_link.box([p.imu_length, p.imu_width, p.imu_height], material="orange")
imu_link.joint_origin([p.imu_x_offset, 0, p.imu_z_offset])

# Front marker (visual indicator of robot front)
front_marker = base_link.link("front_marker")
front_marker.box([p.front_marker_length, p.front_marker_width, p.front_marker_thickness], material="darkblue")
front_marker.joint_origin([p.front_marker_x_offset, 0, p.front_marker_z_offset])

# Generate URDF
robot.save("dome1.urdf")

print("Dome1 robot URDF generated successfully!")
print(f"Total links: 12 (footprint + base + 2 plates + 3 posts + 2 wheels + laser + tower + IMU + marker)")
print(f"Structure: Two-tier platform with {len(p.support_posts)} support posts")
