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
    print(f"  Robot: {robot_name}")

    print("\nGenerating URDF...")
    generator = UrdfGenerator()
    urdf_xml = generator.generate(data)

    # Report what was created
    print(f"\n  Links created: {len(generator.links_created)}")
    for link in generator.links_created:
        print(f"    - {link}")

    print(f"\n  Joints created: {len(generator.joints_created)}")
    for joint in generator.joints_created:
        print(f"    - {joint}")

    # Report unused clauses
    if generator.unused_top_level:
        print(f"\nWarning: Unused top-level keys: {', '.join(generator.unused_top_level)}")

    if generator.unused_per_link:
        print("\nWarning: Unused link properties:")
        for link_name, unused_keys in generator.unused_per_link.items():
            print(f"  {link_name}: {', '.join(unused_keys)}")

    print(f"\nWriting {output_file}...")
    output_file.write_text(urdf_xml)

    print("Done!")


if __name__ == "__main__":
    main()
