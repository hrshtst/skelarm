"""Real-time GUI simulation of the 4-DOF arm with interactive tip forces.

The arm runs under zero-torque control (like ``simulate_four_dof.py``). Press and
drag the left mouse button in the canvas to apply an external force at the tip:
the force is spring-like — proportional to the vector from the tip to the cursor —
and drawn as a red arrow. The current time is shown, and the joint sliders are
read-only, reflecting the simulated angles. A checkbox toggles the link centers
of mass.

The simulator itself lives in :mod:`skelarm.simulator` (:class:`SkelarmSimulator`)
so it can be reused with any robot; this example just loads the 4-DOF arm.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
from PyQt6.QtWidgets import QApplication

from skelarm import SkelarmSimulator, Skeleton

_INITIAL_Q_DEG = (0.0, 30.0, 30.0, 60.0)


def main() -> None:
    """Run the 4-DOF simulator GUI."""
    app = QApplication(sys.argv)
    config_path = Path(__file__).parent / "four_dof_robot.toml"
    skeleton = Skeleton.from_toml(config_path)
    skeleton.q = np.deg2rad(_INITIAL_Q_DEG)
    skeleton.dq = np.zeros(skeleton.num_joints)

    window = SkelarmSimulator(skeleton)
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
