#!/usr/bin/env python3
# urdf_generator.py - Generates URDF XML from DURDF data
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import xml.etree.ElementTree as ET


class UrdfGenerator:
    def generate(self, data: dict) -> str:
        robot_name = data["robot"]
        robot = ET.Element("robot", name=robot_name)

        for link_name, link_data in data["links"].items():
            self._add_link(robot, link_name, link_data)

        if "hierarchy" in data:
            self._add_joints_from_hierarchy(robot, data["hierarchy"])

        return self._to_xml_string(robot)

    def _add_link(self, robot: ET.Element, name: str, data: dict):
        link = ET.SubElement(robot, "link", name=name)

        geom_type, dims = self._get_geometry(data)
        rpy = data.get("rpy")

        self._add_visual_geom(link, geom_type, dims, rpy)
        self._add_collision_geom(link, geom_type, dims, rpy)

    def _get_geometry(self, data: dict) -> tuple[str, list]:
        if "box" in data:
            return "box", data["box"]
        if "cylinder" in data:
            return "cylinder", data["cylinder"]
        raise ValueError("No geometry found in link data")

    def _add_visual_geom(self, link: ET.Element, geom_type: str, dims: list, rpy: list):
        visual = ET.SubElement(link, "visual")
        if rpy:
            self._add_origin(visual, rpy)
        geometry = ET.SubElement(visual, "geometry")
        self._add_geometry_elem(geometry, geom_type, dims)

    def _add_collision_geom(self, link: ET.Element, geom_type: str, dims: list, rpy: list):
        collision = ET.SubElement(link, "collision")
        if rpy:
            self._add_origin(collision, rpy)
        geometry = ET.SubElement(collision, "geometry")
        self._add_geometry_elem(geometry, geom_type, dims)

    def _add_origin(self, parent: ET.Element, rpy: list):
        rpy_str = " ".join(str(v) for v in rpy)
        ET.SubElement(parent, "origin", rpy=rpy_str)

    def _add_geometry_elem(self, geom: ET.Element, geom_type: str, dims: list):
        if geom_type == "box":
            size_str = " ".join(str(d) for d in dims)
            ET.SubElement(geom, "box", size=size_str)
        elif geom_type == "cylinder":
            ET.SubElement(geom, "cylinder", radius=str(dims[0]), length=str(dims[1]))

    def _add_joints_from_hierarchy(self, robot: ET.Element, hierarchy: dict):
        for parent_name, children in hierarchy.items():
            if isinstance(children, list):
                for child_name in children:
                    self._add_joint(robot, parent_name, child_name)

    def _add_joint(self, robot: ET.Element, parent: str, child: str):
        joint_name = f"{parent}_to_{child}"
        joint = ET.SubElement(robot, "joint", name=joint_name, type="fixed")
        ET.SubElement(joint, "parent", link=parent)
        ET.SubElement(joint, "child", link=child)

    def _to_xml_string(self, root: ET.Element) -> str:
        ET.indent(root, space="  ")
        xml_str = ET.tostring(root, encoding="unicode")
        return f'<?xml version="1.0"?>\n{xml_str}\n'
