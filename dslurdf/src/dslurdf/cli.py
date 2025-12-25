#!/usr/bin/env python3
# cli.py - Command-line interface for DURDF compiler
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import sys
from pathlib import Path

from dslurdf.durdf_loader import DurdfLoader
from dslurdf.urdf_generator import UrdfGenerator


def main():
    if len(sys.argv) != 3:
        print("Usage: durdf <input.durdf> <output.urdf>")
        sys.exit(1)

    input_file = Path(sys.argv[1])
    output_file = Path(sys.argv[2])

    if not input_file.exists():
        print(f"Error: Input file '{input_file}' not found")
        sys.exit(1)

    print(f"Loading {input_file}...")
    loader = DurdfLoader()
    data = loader.load(input_file)

    robot_name = data.get("robot", "unknown")
    link_count = len(data.get("links", {}))
    has_hierarchy = "hierarchy" in data

    print(f"  Robot: {robot_name}")
    print(f"  Links: {link_count}")
    print(f"  Hierarchy: {'yes' if has_hierarchy else 'no'}")

    print("Generating URDF...")
    generator = UrdfGenerator()
    urdf_xml = generator.generate(data)

    print(f"Writing {output_file}...")
    output_file.write_text(urdf_xml)

    print("Done!")


if __name__ == "__main__":
    main()
