"""
Floora Robot - Physical parameters and dimensions
"""
import math

class FlooraParams:
    """Physical parameters for Floora robot"""

    # Base dimensions
    base_radius = 0.1495
    base_thickness = 0.012
    base_height = 0.089
    base_visual_zoff = -0.006
    base_collision_height = 0.030

    # Drive wheel dimensions
    wheel_radius = 0.037
    wheel_width = 0.020
    wheel_ygap = 0.001  # Gap between wheel and base
    wheel_zoff = 0.052

    # Caster bearing dimensions
    caster_bearing_radius = 0.015
    caster_bearing_width = 0.010
    caster_bearing_off = base_radius  # Distance from center
    caster_bearing_zoff = 0.041

    # Caster wheel dimensions
    caster_wheel_radius = 0.019
    caster_wheel_width = 0.008625
    caster_wheel_xoff = 0.021  # Offset from bearing center
    caster_wheel_zoff = 0.029

    # Collision margin
    collision_margin = 0.002

    # Mass properties
    base_mass = 2.5
    wheel_mass = 0.2
    caster_bearing_mass = 0.05
    caster_wheel_mass = 0.05

    # Inertia tensors
    # Base (cylinder: r=0.1495m, h=0.012m, m=2.5kg)
    base_inertia = {
        'ixx': 0.0141, 'ixy': 0.0, 'ixz': 0.0,
        'iyy': 0.0141, 'iyz': 0.0, 'izz': 0.0279
    }

    # Wheel (cylinder: r=0.037m, h=0.020m, m=0.2kg)
    wheel_inertia = {
        'ixx': 0.000015, 'ixy': 0.0, 'ixz': 0.0,
        'iyy': 0.000015, 'iyz': 0.0, 'izz': 0.000027
    }

    # Caster bearing (small cylinder: m=0.05kg)
    caster_bearing_inertia = {
        'ixx': 0.000001, 'ixy': 0.0, 'ixz': 0.0,
        'iyy': 0.000001, 'iyz': 0.0, 'izz': 0.000001
    }

    # Caster wheel (small cylinder: r=0.019m, h=0.008625m, m=0.05kg)
    caster_wheel_inertia = {
        'ixx': 0.000003, 'ixy': 0.0, 'ixz': 0.0,
        'iyy': 0.000003, 'iyz': 0.0, 'izz': 0.000002
    }

    # Caster positioning angles (degrees)
    caster_fl_angle = 45.0   # Front-left
    caster_bl_angle = 135.0  # Back-left
    caster_br_angle = 225.0  # Back-right
    caster_fr_angle = 315.0  # Front-right

    # Materials (RGBA colors)
    bamboo_color = [0.89, 0.87, 0.77, 1.0]
    gray_color = [0.5, 0.5, 0.5, 1.0]
    cyan_color = [0.0, 1.0, 1.0, 1.0]

    @classmethod
    def caster_position(cls, angle_deg):
        """Calculate XY position for caster at given angle"""
        angle_rad = math.radians(angle_deg)
        x = cls.caster_bearing_off * math.cos(angle_rad)
        y = cls.caster_bearing_off * math.sin(angle_rad)
        return [x, y]
