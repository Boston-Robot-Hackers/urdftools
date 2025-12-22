"""
URDFBuilder - Fluent API for building URDF robot descriptions

This is a STUB showing the intended API structure.
Actual implementation to be completed.
"""

class Link:
    """Represents a URDF link with fluent API for building"""

    def __init__(self, name, parent=None):
        self.name = name
        self.parent = parent
        self.children = []
        self.geometry = None
        self.material_name = None
        self.mass_value = None
        self.inertia_tensor = None
        self.collision_geometry = None
        self.collision_origin = None
        self.joint_type = "fixed"
        self.joint_axis = None
        self.visual_origin_xyz = [0, 0, 0]
        self.visual_origin_rpy = [0, 0, 0]
        self.joint_origin_xyz = [0, 0, 0]
        self.joint_origin_rpy = [0, 0, 0]

    def link(self, name):
        """Create a child link and return it for chaining"""
        child = Link(name, parent=self)
        self.children.append(child)
        return child

    def cylinder(self, radius, length, material=None):
        """Add cylinder geometry, optionally with material"""
        self.geometry = ("cylinder", radius, length)
        if material:
            self.material_name = material
        return self  # Return self for chaining

    def box(self, size, material=None):
        """Add box geometry, optionally with material"""
        self.geometry = ("box", size)
        if material:
            self.material_name = material
        return self

    def sphere(self, radius, material=None):
        """Add sphere geometry, optionally with material"""
        self.geometry = ("sphere", radius)
        if material:
            self.material_name = material
        return self

    def mesh(self, filename, scale=None, material=None):
        """Add mesh geometry, optionally with scale and material"""
        self.geometry = ("mesh", filename, scale)
        if material:
            self.material_name = material
        return self

    def material(self, name):
        """Set material (separate from geometry if needed)"""
        self.material_name = name
        return self

    def mass(self, value):
        """Set mass value"""
        self.mass_value = value
        return self

    def inertia(self, tensor):
        """Set inertia tensor (dict with ixx, ixy, ixz, iyy, iyz, izz)"""
        self.inertia_tensor = tensor
        return self

    def collision(self, geometry, origin=None):
        """Set collision geometry (different from visual if needed)

        Args:
            geometry: Tuple like ("cylinder", radius, length) or ("box", size)
            origin: Optional [x, y, z] position offset for collision
        """
        self.collision_geometry = geometry
        self.collision_origin = origin
        return self

    def continuous_joint(self, axis):
        """Set joint as continuous with given axis"""
        self.joint_type = "continuous"
        self.joint_axis = axis
        return self

    def revolute_joint(self, axis, limits):
        """Set joint as revolute with axis and limits"""
        self.joint_type = "revolute"
        self.joint_axis = axis
        self.joint_limits = limits
        return self

    def fixed_joint(self):
        """Set joint as fixed (default)"""
        self.joint_type = "fixed"
        return self

    def origin(self, xyz, rpy=None):
        """Set origin position and optional rotation

        Note: Can be called multiple times - once for visual origin,
        once for joint origin. Implementation should track context.
        """
        # TODO: Implement proper context tracking (visual vs joint origin)
        # For now, this is just showing the intended API
        self.visual_origin_xyz = xyz
        if rpy:
            self.visual_origin_rpy = rpy
        return self

    def joint_origin(self, xyz, rpy=None):
        """Set joint origin (where this link attaches to parent)"""
        self.joint_origin_xyz = xyz
        if rpy:
            self.joint_origin_rpy = rpy
        return self

    def axis(self, direction):
        """Set joint axis (for use with separate joint_type call)"""
        self.joint_axis = direction
        return self

    def to_urdf(self):
        """Generate URDF XML string (to be implemented)"""
        # TODO: Implement URDF generation
        pass


class Robot:
    """Represents a URDF robot with fluent API"""

    def __init__(self, name):
        self.name = name
        self.root_links = []

    def link(self, name):
        """Create a root-level link and return it for chaining"""
        link = Link(name)
        self.root_links.append(link)
        return link

    def to_urdf(self):
        """Generate complete URDF XML string (to be implemented)"""
        # TODO: Implement URDF generation
        pass

    def save(self, filename):
        """Save URDF to file (to be implemented)"""
        # TODO: Implement file writing
        pass


def urdf(name):
    """Create a new robot with given name"""
    return Robot(name)
