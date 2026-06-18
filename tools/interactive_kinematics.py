"""
Interactive kinematics tool for skelarm.

Load a robot arm from a TOML config given on the command line and pose it
interactively: drag the joint sliders to set joint angles (forward kinematics),
or click (and drag) in the canvas to move the tip to that point (inverse
kinematics).

Usage::

    uv run python tools/interactive_kinematics.py path/to/robot.toml
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from PyQt6.QtWidgets import QApplication

from skelarm import SkelarmViewer, Skeleton


def main() -> None:
    """Launch the interactive viewer for a robot config given on the command line."""
    parser = argparse.ArgumentParser(description="Interactive forward/inverse kinematics viewer for a skelarm robot.")
    parser.add_argument("config", type=Path, help="path to the robot TOML config to load")
    args = parser.parse_args()

    if not args.config.exists():
        parser.error(f"config file not found: {args.config}")

    skeleton = Skeleton.from_toml(args.config)

    app = QApplication(sys.argv)
    viewer = SkelarmViewer(skeleton)
    viewer.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
