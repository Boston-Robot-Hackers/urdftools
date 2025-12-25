#!/usr/bin/env python3
# test_compiler.py - Unit tests for DSL URDF compiler
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from dslurdf.durdf_loader import DurdfLoader
from dslurdf.urdf_generator import UrdfGenerator


class TestStage1Simple:
    """Test simple.durdf -> simple.urdf conversion"""

    @pytest.fixture
    def fixtures_dir(self) -> Path:
        return Path(__file__).parent / "fixtures"

    @pytest.fixture
    def expected_urdf(self, fixtures_dir: Path) -> str:
        return (fixtures_dir / "simple.urdf").read_text()

    def test_load_durdf(self, fixtures_dir: Path):
        """Test loading simple.durdf file"""
        loader = DurdfLoader()
        data = loader.load(fixtures_dir / "simple.durdf")

        assert data["robot"] == "simple_bot"
        assert "links" in data
        assert "base_link" in data["links"]
        assert data["links"]["base_link"]["box"] == [0.5, 0.3, 0.1]

    def test_generate_urdf(self, fixtures_dir: Path):
        """Test generating URDF from loaded data"""
        loader = DurdfLoader()
        data = loader.load(fixtures_dir / "simple.durdf")

        generator = UrdfGenerator()
        urdf_xml = generator.generate(data)

        assert '<?xml version="1.0"?>' in urdf_xml
        assert '<robot name="simple_bot">' in urdf_xml
        assert '<link name="base_link">' in urdf_xml

    def test_collision_equals_visual(self, fixtures_dir: Path):
        """Test that collision geometry defaults to visual geometry"""
        loader = DurdfLoader()
        data = loader.load(fixtures_dir / "simple.durdf")

        generator = UrdfGenerator()
        urdf_xml = generator.generate(data)

        root = ET.fromstring(urdf_xml)
        link = root.find("link[@name='base_link']")

        visual_box = link.find(".//visual/geometry/box")
        collision_box = link.find(".//collision/geometry/box")

        assert visual_box is not None
        assert collision_box is not None
        assert visual_box.get("size") == collision_box.get("size")
        assert visual_box.get("size") == "0.5 0.3 0.1"

    def test_full_pipeline(self, fixtures_dir: Path, expected_urdf: str):
        """Test complete pipeline matches expected output"""
        loader = DurdfLoader()
        data = loader.load(fixtures_dir / "simple.durdf")

        generator = UrdfGenerator()
        generated_urdf = generator.generate(data)

        expected_root = ET.fromstring(expected_urdf)
        generated_root = ET.fromstring(generated_urdf)

        assert expected_root.tag == generated_root.tag
        assert expected_root.get("name") == generated_root.get("name")

        expected_links = expected_root.findall("link")
        generated_links = generated_root.findall("link")
        assert len(expected_links) == len(generated_links)
