"""
Interactive GUI example for skelarm.

This script launches a PyQt6 application that allows users to interactively
control the joint angles of a planar robot arm using sliders.
"""

from __future__ import annotations

import sys

import numpy as np
from PyQt6.QtWidgets import QApplication

from skelarm import LinkProp, SkelarmViewer, Skeleton


def main() -> None:
    """Run the interactive GUI."""
    # 1. Define the robot arm (3-link example)
    link_props = [
        LinkProp(length=1.5, m=2.0, i=0.5, rgx=0.75, rgy=0.0, qmin=-np.pi, qmax=np.pi),
        LinkProp(length=1.0, m=1.5, i=0.3, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi),
        LinkProp(length=0.8, m=1.0, i=0.1, rgx=0.4, rgy=0.0, qmin=-np.pi, qmax=np.pi),
    ]
    skeleton = Skeleton(link_props)

    # 2. Set initial joint angles
    skeleton.q = np.array([np.pi / 4, -np.pi / 4, np.pi / 4])

    # 3. Create the Qt Application
    app = QApplication(sys.argv)

    # 4. Create and show the viewer
    viewer = SkelarmViewer(skeleton)
    viewer.show()

    # 5. Execute the application
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
