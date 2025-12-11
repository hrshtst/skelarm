"""Tests for 4-DOF robot configuration."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
from numpy.typing import NDArray

from skelarm import Skeleton, compute_forward_kinematics, simulate_robot


@pytest.fixture
def four_dof_skeleton() -> Skeleton:
    """Fixture to load the 4-DOF robot."""
    config_path = Path("examples/four_dof_robot.toml")
    return Skeleton.from_toml(config_path)


def test_four_dof_loading(four_dof_skeleton: Skeleton) -> None:
    """Test that the 4-DOF robot loads correctly."""
    expected_links = 4
    expected_last_length = 0.4

    assert four_dof_skeleton.num_links == expected_links
    assert four_dof_skeleton.links[0].prop.length == 1.0
    assert four_dof_skeleton.links[3].prop.length == expected_last_length


def test_four_dof_fk(four_dof_skeleton: Skeleton) -> None:
    """Test Forward Kinematics for the 4-DOF robot."""
    # Fully extended configuration
    four_dof_skeleton.q = np.zeros(4)
    compute_forward_kinematics(four_dof_skeleton)

    # Tip position should be sum of lengths
    expected_x = 1.0 + 0.8 + 0.6 + 0.4
    assert four_dof_skeleton.links[3].xe == pytest.approx(expected_x)
    assert four_dof_skeleton.links[3].ye == pytest.approx(0.0)

    # 90 degrees at first joint
    four_dof_skeleton.q = np.array([np.pi / 2, 0.0, 0.0, 0.0])
    compute_forward_kinematics(four_dof_skeleton)

    # Tip should be at (0, total_length)
    assert four_dof_skeleton.links[3].xe == pytest.approx(0.0, abs=1e-9)
    assert four_dof_skeleton.links[3].ye == pytest.approx(expected_x)


def test_four_dof_dynamics_static(four_dof_skeleton: Skeleton) -> None:
    """Test that the 4-DOF robot remains static with zero torque/gravity."""
    four_dof_skeleton.q = np.array([0.1, 0.2, 0.3, 0.4])
    four_dof_skeleton.dq = np.zeros(4)

    def control_torques(_t: float, _skel: Skeleton) -> NDArray[np.float64]:
        return np.zeros(4)

    # Simulate for a short time
    _times, q_traj, dq_traj = simulate_robot(four_dof_skeleton, (0.0, 0.1), control_torques, dt=0.01)

    # Should not move
    assert np.allclose(q_traj[-1], four_dof_skeleton.q, atol=1e-4)
    assert np.allclose(dq_traj[-1], np.zeros(4), atol=1e-4)
