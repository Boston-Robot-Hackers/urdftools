#!/usr/bin/env python3
# test_hierarchy.py - Tests for hierarchy and cylinder geometry
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from dslurdf.durdf_loader import DurdfLoader
from dslurdf.urdf_generator import UrdfGenerator


class TestStage2Hierarchy:
    """Test hierarchy.durdf with parent-child links and cylinder geometry"""

    @pytest.fixture
    def fixtures_dir(self) -> Path:
        return Path(__file__).parent / "fixtures"

    def test_load_hierarchy(self, fixtures_dir: Path):
        """Test loading hierarchy section"""
        loader = DurdfLoader()
        data = loader.load(fixtures_dir / "hierarchy.durdf")

        assert data["robot"] == "hierarchy_bot"
        assert "hierarchy" in data
        assert "base_link" in data["hierarchy"]
        assert "top_link" in data["hierarchy"]["base_link"]

    def test_cylinder_geometry(self, fixtures_dir: Path):
        """Test cylinder geometry with radius and length"""
        loader = DurdfLoader()
        data = loader.load(fixtures_dir / "hierarchy.durdf")

        generator = UrdfGenerator()
        urdf_xml = generator.generate(data)

        root = ET.fromstring(urdf_xml)
        top_link = root.find("link[@name='top_link']")
        cylinder = top_link.find(".//visual/geometry/cylinder")

        assert cylinder is not None
        assert cylinder.get("radius") == "0.05"
        assert cylinder.get("length") == "0.2"

    def test_rotation_in_origin(self, fixtures_dir: Path):
        """Test rpy rotation appears in origin element"""
        loader = DurdfLoader()
        data = loader.load(fixtures_dir / "hierarchy.durdf")

        generator = UrdfGenerator()
        urdf_xml = generator.generate(data)

        root = ET.fromstring(urdf_xml)
        top_link = root.find("link[@name='top_link']")
        visual_origin = top_link.find(".//visual/origin")

        assert visual_origin is not None
        assert visual_origin.get("rpy") == "1.57 0 0"

    def test_joint_creation(self, fixtures_dir: Path):
        """Test joint created from hierarchy"""
        loader = DurdfLoader()
        data = loader.load(fixtures_dir / "hierarchy.durdf")

        generator = UrdfGenerator()
        urdf_xml = generator.generate(data)

        root = ET.fromstring(urdf_xml)
        joint = root.find("joint")

        assert joint is not None
        assert joint.get("type") == "fixed"

        parent = joint.find("parent")
        child = joint.find("child")

        assert parent.get("link") == "base_link"
        assert child.get("link") == "top_link"

    def test_collision_includes_rotation(self, fixtures_dir: Path):
        """Test collision geometry also has rotation"""
        loader = DurdfLoader()
        data = loader.load(fixtures_dir / "hierarchy.durdf")

        generator = UrdfGenerator()
        urdf_xml = generator.generate(data)

        root = ET.fromstring(urdf_xml)
        top_link = root.find("link[@name='top_link']")

        visual_origin = top_link.find(".//visual/origin")
        collision_origin = top_link.find(".//collision/origin")

        assert visual_origin.get("rpy") == collision_origin.get("rpy")
