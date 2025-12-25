#!/usr/bin/env python3
# durdf_loader.py - Loads DURDF YAML files
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

from pathlib import Path

import yaml


class DurdfLoader:
    def load(self, filepath: Path) -> dict:
        with open(filepath) as f:
            return yaml.safe_load(f)
