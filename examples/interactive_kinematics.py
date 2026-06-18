"""
Interactive kinematics example for skelarm.

This script launches a PyQt6 application where you can pose a planar robot arm two
ways: drag the joint sliders to set joint angles (forward kinematics), or click
(and drag) in the canvas to move the tip to that point (inverse kinematics).
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
from PyQt6.QtWidgets import QApplication

from skelarm import SkelarmViewer, Skeleton


def main() -> None:
    """Run the interactive GUI."""
    # 1. Load the 4-DOF robot arm from its TOML configuration.
    config_path = Path(__file__).parent / "four_dof_robot.toml"
    skeleton = Skeleton.from_toml(config_path)

    # 2. Set an initial pose within the robot's joint limits (~[20, 45, 60, 30] deg).
    skeleton.q = np.array([np.pi / 9, np.pi / 4, np.pi / 3, np.pi / 6])

    # 3. Create the Qt Application
    app = QApplication(sys.argv)

    # 4. Create and show the viewer
    viewer = SkelarmViewer(skeleton)
    viewer.show()

    # 5. Execute the application
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
