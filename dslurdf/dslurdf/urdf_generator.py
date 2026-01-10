#!/usr/bin/env python3
# urdf_generator.py - Generates URDF XML from DURDF data
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import xml.etree.ElementTree as ET


class UrdfGenerator:
    def __init__(self):
        self.unused_top_level: list[str] = []
        self.unused_per_link: dict[str, list[str]] = {}
        self.joints_created: list[str] = []
        self.links_created: list[str] = []
        self.materials_defined: set[str] = set()  # Track defined materials
        self.validation_report: dict = {}  # TF validation and lint results
        self.joint_properties: dict = {}  # Store joint properties for validation

    def generate(self, data: dict) -> str:
        # Track all keys
        all_keys = set(data.keys())
        used_keys = set()

        robot_name = data["robot"]
        used_keys.add("robot")
        robot = ET.Element("robot", name=robot_name)

        # Constants are handled by the loader, mark as used
        if "constants" in data:
            used_keys.add("constants")

        # Add material definitions first
        if "materials" in data:
            used_keys.add("materials")
            self._add_materials(robot, data["materials"])

        if "links" in data:
            used_keys.add("links")
            for link_name, link_data in data["links"].items():
                self._add_link(robot, link_name, link_data)
                self.links_created.append(link_name)

        joints = data.get("joints", {})
        if joints:
            used_keys.add("joints")

        if "hierarchy" in data:
            used_keys.add("hierarchy")
            self._add_joints_from_hierarchy(robot, data["hierarchy"], joints)

        # Record unused top-level keys
        self.unused_top_level = sorted(all_keys - used_keys)

        # Validate and generate report
        self._validate_urdf(data)

        return self._to_xml_string(robot)

    def _add_link(self, robot: ET.Element, name: str, data: dict):
        link = ET.SubElement(robot, "link", name=name)

        # Track used keys for this link
        all_link_keys = set(data.keys())
        used_link_keys = set()

        # Handle empty links (no geometry)
        if not data:
            return

        # Try to get geometry (may be None for empty links)
        try:
            geom_type, dims = self._get_geometry(data)
            # Track geometry key used
            if "box" in data:
                used_link_keys.add("box")
            elif "cylinder" in data:
                used_link_keys.add("cylinder")
            elif "sphere" in data:
                used_link_keys.add("sphere")
            elif "octagon" in data:
                used_link_keys.add("octagon")
            elif "hexagon" in data:
                used_link_keys.add("hexagon")
            elif "mesh" in data:
                used_link_keys.add("mesh")

            rpy = data.get("rpy")
            if rpy is not None:
                used_link_keys.add("rpy")

            material = data.get("material")
            if material is not None:
                used_link_keys.add("material")

            self._add_visual_geom(link, geom_type, dims, rpy, material)
            self._add_collision_geom(link, geom_type, dims, rpy)
        except ValueError:
            # No geometry found - this is okay for reference frames
            pass

        # Record unused link keys
        unused = sorted(all_link_keys - used_link_keys)
        if unused:
            self.unused_per_link[name] = unused

    def _get_geometry(self, data: dict) -> tuple[str, list | dict]:
        if "box" in data:
            return "box", data["box"]
        if "cylinder" in data:
            return "cylinder", data["cylinder"]
        if "sphere" in data:
            return "sphere", data["sphere"]
        if "octagon" in data:
            return "octagon", data["octagon"]
        if "hexagon" in data:
            return "hexagon", data["hexagon"]
        if "mesh" in data:
            return "mesh", data["mesh"]
        raise ValueError("No geometry found in link data")

    def _add_visual_geom(self, link: ET.Element, geom_type: str, dims: list | dict, rpy: list, material: str = None):
        visual = ET.SubElement(link, "visual")
        if rpy:
            self._add_origin(visual, rpy)
        geometry = ET.SubElement(visual, "geometry")
        self._add_geometry_elem(geometry, geom_type, dims)

        # Add material reference if specified (but not for meshes - they use their own materials)
        if material and geom_type != "mesh":
            ET.SubElement(visual, "material", name=material)

    def _add_collision_geom(self, link: ET.Element, geom_type: str, dims: list | dict, rpy: list):
        collision = ET.SubElement(link, "collision")
        if rpy:
            self._add_origin(collision, rpy)
        geometry = ET.SubElement(collision, "geometry")
        self._add_geometry_elem(geometry, geom_type, dims)

    def _add_origin(self, parent: ET.Element, rpy: list):
        rpy_str = " ".join(str(v) for v in rpy)
        ET.SubElement(parent, "origin", rpy=rpy_str)

    def _add_geometry_elem(self, geom: ET.Element, geom_type: str, dims: list | dict):
        if geom_type == "box":
            size_str = " ".join(str(d) for d in dims)
            ET.SubElement(geom, "box", size=size_str)
        elif geom_type == "cylinder":
            ET.SubElement(geom, "cylinder", radius=str(dims[0]), length=str(dims[1]))
        elif geom_type == "sphere":
            ET.SubElement(geom, "sphere", radius=str(dims[0]))
        elif geom_type == "octagon":
            # Octagon: [radius, length] - approximated as cylinder
            # In production, would generate octagonal mesh
            geom.append(ET.Comment(" Octagon approximated as cylinder "))
            ET.SubElement(geom, "cylinder", radius=str(dims[0]), length=str(dims[1]))
        elif geom_type == "hexagon":
            # Hexagon: [radius, length] - approximated as cylinder
            # In production, would generate hexagonal mesh
            geom.append(ET.Comment(" Hexagon approximated as cylinder "))
            ET.SubElement(geom, "cylinder", radius=str(dims[0]), length=str(dims[1]))
        elif geom_type == "mesh":
            # Mesh: can be either a string (filename) or dict with filename and optional scale
            if isinstance(dims, str):
                # Simple format: mesh: "package://robot/meshes/base.stl"
                ET.SubElement(geom, "mesh", filename=dims)
            elif isinstance(dims, dict):
                # Dictionary format with filename and optional scale
                filename = dims.get("filename")
                if not filename:
                    raise ValueError("Mesh geometry requires 'filename' key")

                mesh_attrs = {"filename": filename}

                # Add scale if specified
                if "scale" in dims:
                    scale = dims["scale"]
                    mesh_attrs["scale"] = " ".join(str(s) for s in scale)

                ET.SubElement(geom, "mesh", **mesh_attrs)
            else:
                raise ValueError(f"Invalid mesh format: {dims}")

    def _add_joints_from_hierarchy(self, robot: ET.Element, hierarchy: dict, joints: dict):
        # Track which children are referenced in hierarchy
        children_in_hierarchy = set()

        def process_hierarchy(parent_name, children):
            if isinstance(children, list):
                # List of leaf children
                for child_name in children:
                    children_in_hierarchy.add(child_name)
                    props = joints.get(child_name, {})
                    self._add_joint(robot, parent_name, child_name, props)
            elif isinstance(children, dict):
                # Dict of children with their own descendants
                for child_name, grandchildren in children.items():
                    children_in_hierarchy.add(child_name)
                    props = joints.get(child_name, {})
                    self._add_joint(robot, parent_name, child_name, props)
                    # Recursively process descendants
                    if grandchildren:
                        process_hierarchy(child_name, grandchildren)

        for parent_name, children in hierarchy.items():
            process_hierarchy(parent_name, children)

        # Check for joints that don't match any child in hierarchy
        unused_joints = set(joints.keys()) - children_in_hierarchy
        if unused_joints:
            for joint_name in sorted(unused_joints):
                print(f"Warning: joints section for '{joint_name}' has no matching child in hierarchy")

    def _add_joint(self, robot: ET.Element, parent: str, child: str, properties: dict):
        joint_name = f"{parent}_to_{child}"
        joint_type = properties.get("type", "fixed")

        joint = ET.SubElement(robot, "joint", name=joint_name, type=joint_type)
        ET.SubElement(joint, "parent", link=parent)
        ET.SubElement(joint, "child", link=child)

        # Add origin if xyz or rpy specified
        xyz = properties.get("xyz")
        rpy = properties.get("rpy")
        if xyz or rpy:
            origin_attrs = {}
            if xyz:
                origin_attrs["xyz"] = " ".join(str(v) for v in xyz)
            if rpy:
                origin_attrs["rpy"] = " ".join(str(v) for v in rpy)
            ET.SubElement(joint, "origin", **origin_attrs)

        # Add axis if specified (for continuous/revolute joints)
        axis = properties.get("axis")
        if axis:
            axis_str = " ".join(str(v) for v in axis)
            ET.SubElement(joint, "axis", xyz=axis_str)

        self.joints_created.append(joint_name)

        # Store properties for validation
        self.joint_properties[joint_name] = {
            "type": joint_type,
            "parent": parent,
            "child": child,
            "axis": axis
        }

    def _add_materials(self, robot: ET.Element, materials: dict):
        """Add material definitions to the robot.

        Materials can be specified as RGBA list: [r, g, b, a] where values are 0-1
        """
        for name, rgba in materials.items():
            material = ET.SubElement(robot, "material", name=name)
            rgba_str = " ".join(str(v) for v in rgba)
            ET.SubElement(material, "color", rgba=rgba_str)
            self.materials_defined.add(name)

    def _validate_urdf(self, data: dict):
        """Validate URDF structure and check naming conventions."""

        # Standard ROS frame names
        STANDARD_FRAMES = {
            "base_footprint": "Ground contact reference frame",
            "base_link": "Main robot body frame",
            "odom": "Odometry frame (usually in TF tree, not URDF)",
            "map": "Map frame (usually in TF tree, not URDF)",
        }

        # Standard sensor frame suffixes
        SENSOR_SUFFIXES = ["_link", "_frame"]

        # Known sensor prefixes
        SENSOR_PREFIXES = ["camera", "lidar", "laser", "imu", "gps", "depth", "rgb", "sonar", "ultrasonic"]

        # Check link naming
        standard_links = []
        sensor_links = []
        non_standard_links = []

        for link_name in self.links_created:
            if link_name in STANDARD_FRAMES:
                standard_links.append({
                    "name": link_name,
                    "description": STANDARD_FRAMES[link_name]
                })
            elif any(link_name.startswith(prefix) for prefix in SENSOR_PREFIXES):
                # Check if it has proper suffix
                has_suffix = any(link_name.endswith(suffix) for suffix in SENSOR_SUFFIXES)
                sensor_links.append({
                    "name": link_name,
                    "standard": has_suffix,
                    "suggestion": f"{link_name}_link" if not has_suffix else None
                })
            elif link_name.endswith("_link") or link_name.endswith("_frame"):
                # Has standard suffix but not a known sensor
                standard_links.append({
                    "name": link_name,
                    "description": "Uses standard _link suffix"
                })
            else:
                # Check if it's a wheel or structural element
                if "wheel" in link_name or "caster" in link_name:
                    standard_links.append({
                        "name": link_name,
                        "description": "Drive/support component"
                    })
                else:
                    non_standard_links.append(link_name)

        # Lint checks
        lint_warnings = []
        lint_info = []

        # Check for links without geometry
        links_data = data.get("links", {})
        for link_name in self.links_created:
            link_data = links_data.get(link_name, {})
            if not link_data:
                lint_info.append(f"Link '{link_name}' has no geometry (reference frame)")

        # Check joints needing axis
        for joint_name, props in self.joint_properties.items():
            joint_type = props["type"]
            axis = props["axis"]

            if joint_type in ["revolute", "continuous", "prismatic"] and not axis:
                lint_warnings.append(
                    f"Joint '{joint_name}' is type '{joint_type}' but has no axis defined"
                )

        # Check for potential base_footprint issues
        if "base_footprint" in self.links_created:
            # Check if it's a child of something (it should be root)
            is_child = any(props["child"] == "base_footprint" for props in self.joint_properties.values())
            if is_child:
                lint_warnings.append(
                    "base_footprint should typically be the root frame, not a child"
                )

        # Store validation results
        self.validation_report = {
            "standard_frames": standard_links,
            "sensor_frames": sensor_links,
            "non_standard_frames": non_standard_links,
            "lint_warnings": lint_warnings,
            "lint_info": lint_info
        }

    def _to_xml_string(self, root: ET.Element) -> str:
        ET.indent(root, space="  ")
        xml_str = ET.tostring(root, encoding="unicode")
        return f'<?xml version="1.0"?>\n{xml_str}\n'
