"""Simulation example for a 4-DOF robot arm loaded from TOML."""

from __future__ import annotations

from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from skelarm import Skeleton, simulate_robot


def main() -> None:
    """Run the 4-DOF robot simulation."""
    # 1. Load the robot configuration
    config_path = Path(__file__).parent / "four_dof_robot.toml"
    skeleton = Skeleton.from_toml(config_path)
    print(f"Loaded {skeleton.num_links}-link robot from {config_path.name}")

    # 2. Set initial state (arbitrary pose, e.g., [0, pi/6, -pi/6, pi/3])
    skeleton.q = np.array([0.0, np.pi / 6, -np.pi / 6, np.pi / 3])
    skeleton.dq = np.zeros(skeleton.num_links)

    print(f"Initial q: {skeleton.q}")
    print(f"Initial dq: {skeleton.dq}")

    # 3. Define control function (zero torque)
    def zero_torque_control(_t: float, skel: Skeleton) -> NDArray[np.float64]:
        return np.zeros(skel.num_links)

    # 4. Run simulation
    # Simulate for 1.0 second
    time_span = (0.0, 1.0)
    dt = 0.01

    times, q_traj, dq_traj = simulate_robot(skeleton, time_span, zero_torque_control, dt=dt)

    print(f"Simulation complete. Steps: {len(times)}")
    print(f"Final q: {q_traj[-1]}")
    print(f"Final dq: {dq_traj[-1]}")

    # Basic validation: since there is no gravity and no torque,
    # the robot should remain static (velocities near zero)
    # assuming initial velocities were zero.
    assert np.allclose(q_traj[-1], skeleton.q, atol=1e-4)
    assert np.allclose(dq_traj[-1], np.zeros(skeleton.num_links), atol=1e-4)
    print("Static test passed: Robot remained stationary under zero torque/gravity.")


if __name__ == "__main__":
    main()
