#!/usr/bin/env python3
# cli.py - Command-line interface for DURDF compiler
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import sys
from pathlib import Path

from dslurdf.durdf_loader import DurdfLoader
from dslurdf.urdf_generator import UrdfGenerator


def _display_validation_report(report: dict):
    """Display TF validation and lint report."""
    if not report:
        return

    print("\n" + "=" * 60)
    print("TF VALIDATION & LINT REPORT")
    print("=" * 60)

    # Standard frames
    if report["standard_frames"]:
        print("\n✓ Standard Frames:")
        for frame in report["standard_frames"]:
            desc = frame.get("description", "")
            if desc:
                print(f"  • {frame['name']}: {desc}")
            else:
                print(f"  • {frame['name']}")

    # Sensor frames
    if report["sensor_frames"]:
        print("\n⚙ Sensor Frames:")
        for frame in report["sensor_frames"]:
            if frame["standard"]:
                print(f"  ✓ {frame['name']}")
            else:
                print(f"  ⚠ {frame['name']} (suggest: {frame['suggestion']})")

    # Non-standard frames
    if report["non_standard_frames"]:
        print("\n⚠ Non-Standard Frame Names:")
        for name in report["non_standard_frames"]:
            print(f"  • {name}")
        print("  Note: Consider using standard suffixes like '_link' for consistency")

    # Lint warnings
    if report["lint_warnings"]:
        print("\n⚠ Lint Warnings:")
        for warning in report["lint_warnings"]:
            print(f"  • {warning}")

    # Lint info
    if report["lint_info"]:
        print("\nℹ Info:")
        for info in report["lint_info"]:
            print(f"  • {info}")

    print("=" * 60)


def main():
    if len(sys.argv) != 3:
        print("Usage: durdf <input.durdf> <output.urdf>")
        sys.exit(1)

    input_file = Path(sys.argv[1])
    output_file = Path(sys.argv[2])

    if not input_file.exists():
        print(f"Error: Input file '{input_file}' not found")
        sys.exit(1)

    if input_file.suffix != '.durdf':
        print(f"Error: Expected a .durdf file, got '{input_file.name}'")
        print("\nDURDF files are YAML-based robot descriptions.")
        print("To convert URDF to DURDF, use the --reverse flag (not yet implemented).")
        sys.exit(1)

    print(f"Loading {input_file}...")
    loader = DurdfLoader()
    data = loader.load(input_file)

    if not isinstance(data, dict):
        print(f"Error: Invalid DURDF format - expected YAML dictionary, got {type(data).__name__}")
        sys.exit(1)

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

    # Display validation report
    _display_validation_report(generator.validation_report)

    print(f"\nWriting {output_file}...")
    output_file.write_text(urdf_xml)

    print("Done!")


if __name__ == "__main__":
    main()
