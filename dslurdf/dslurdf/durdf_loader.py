#!/usr/bin/env python3
# durdf_loader.py - Loads DURDF YAML files
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import math
import re
from pathlib import Path

import yaml


class DurdfLoader:
    def load(self, filepath: Path) -> dict:
        with open(filepath) as f:
            data = yaml.safe_load(f)

        # Process constants if present
        if "constants" in data:
            data = self._substitute_constants(data)

        # Process degree notation
        data = self._convert_degrees(data)

        return data

    def _substitute_constants(self, data: dict) -> dict:
        """Replace $variable_name with values from constants section."""
        constants = data.get("constants", {})

        # Create a copy without modifying the original
        result = data.copy()

        # Recursively substitute constants in all sections
        for key, value in result.items():
            if key != "constants":  # Don't process constants section itself
                result[key] = self._substitute_in_value(value, constants)

        return result

    def _substitute_in_value(self, value, constants: dict):
        """Recursively substitute constants in a value."""
        if isinstance(value, str):
            # Replace $var_name with constant values
            def replace_var(match):
                var_name = match.group(1)
                if var_name in constants:
                    return str(constants[var_name])
                else:
                    raise ValueError(f"Unknown constant: ${var_name}")

            return re.sub(r'\$(\w+)', replace_var, value)

        elif isinstance(value, list):
            return [self._substitute_in_value(item, constants) for item in value]

        elif isinstance(value, dict):
            return {k: self._substitute_in_value(v, constants) for k, v in value.items()}

        else:
            # Numbers, booleans, None - return as-is
            return value

    def _convert_degrees(self, data: dict) -> dict:
        """Convert degree notation (e.g., '90deg') to radians."""
        result = data.copy()

        # Process all sections except constants (already processed)
        for key, value in result.items():
            if key != "constants":
                result[key] = self._convert_degrees_in_value(value)

        return result

    def _convert_degrees_in_value(self, value):
        """Recursively convert degree notation in a value."""
        if isinstance(value, str):
            # Match patterns like "90deg", "45.5deg", "-90deg"
            deg_pattern = r'^(-?\d+(?:\.\d+)?)deg$'
            match = re.match(deg_pattern, value)
            if match:
                degrees = float(match.group(1))
                radians = math.radians(degrees)
                return radians
            return value

        elif isinstance(value, list):
            return [self._convert_degrees_in_value(item) for item in value]

        elif isinstance(value, dict):
            return {k: self._convert_degrees_in_value(v) for k, v in value.items()}

        else:
            # Numbers, booleans, None - return as-is
            return value
