"""
Dome1 Robot - Physical parameters and dimensions
A two-tier platform robot with laser and IMU sensors
"""

class Dome1Params:
    """Physical parameters for Dome1 robot"""

    # Robot structure dimensions
    plate_diameter = 0.280
    plate_thickness = 0.003
    bottom_plate_height = 0.065
    top_plate_height = 0.125
    base_link_x_offset = 0.0

    # Base link visualization marker (sphere)
    base_link_marker_radius = 0.01
    base_link_marker_length = 0.1

    # Wheel dimensions and positioning
    wheel_base = 0.215  # Distance between left and right wheels
    wheel_diameter = 0.0625
    wheel_width = 0.024
    wheel_x_offset = 0.08  # Forward offset from base_link
    wheel_ground_clearance = 0.005

    # Support post dimensions and positions
    support_post_diameter = 0.010
    support_post_height = 0.070

    # Support post positions (x, y)
    support_posts = [
        ("support_post_1", 0.1, 0.0),      # Front center
        ("support_post_2", -0.1, 0.070),   # Back left
        ("support_post_3", -0.1, -0.070),  # Back right
    ]

    # Laser sensor dimensions and positioning
    laser_x_offset = -0.040
    laser_z_offset = 0.130
    laser_radius = 0.03
    laser_height = 0.02

    # Laser tower (support post for laser)
    laser_tower_radius = 0.005
    laser_tower_height = 0.05
    laser_tower_x_offset = -0.05
    laser_tower_y_offset = 0.0
    laser_tower_z_offset = 0.03

    # IMU sensor dimensions and positioning
    imu_x_offset = -0.040
    imu_z_offset = 0.020
    imu_length = 0.020
    imu_width = 0.020
    imu_height = 0.010

    # Front marker dimensions and positioning
    front_marker_x_offset = 0.15
    front_marker_z_offset = 0.065
    front_marker_length = 0.04
    front_marker_width = 0.02
    front_marker_thickness = 0.001

    # Materials (RGBA colors)
    materials = {
        'slate': [0.35, 0.40, 0.45, 1.0],
        'red': [1.0, 0.0, 0.0, 1.0],
        'orange': [1.0, 0.5, 0.0, 1.0],
        'darkblue': [0.0, 0.0, 0.5, 1.0],
        'coral': [0.91, 0.45, 0.38, 1.0],
        'sage': [0.56, 0.68, 0.53, 1.0],
        'gold': [0.85, 0.68, 0.32, 1.0],
        'steel': [0.55, 0.62, 0.70, 1.0],
        'plum': [0.56, 0.35, 0.48, 0.3],
        'terracotta': [0.78, 0.45, 0.35, 1.0],
        'seafoam': [0.42, 0.70, 0.65, 1.0],
        'mustard': [0.75, 0.60, 0.22, 1.0],
        'dusty_rose': [0.72, 0.52, 0.55, 1.0],
        'charcoal': [0.25, 0.27, 0.30, 1.0],
        'light_blue': [0.5, 0.8, 1.0, 1.0],
    }
