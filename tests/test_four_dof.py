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
    expected_joints = 4
    expected_total_links = 5  # base + 4 movable

    assert four_dof_skeleton.num_joints == expected_joints
    assert four_dof_skeleton.num_links == expected_total_links
    assert four_dof_skeleton.base_length == pytest.approx(0.05)
    assert four_dof_skeleton.links[1].prop.length == pytest.approx(0.1)
    assert four_dof_skeleton.links[4].prop.length == pytest.approx(0.15)


def test_four_dof_fk(four_dof_skeleton: Skeleton) -> None:
    """Test Forward Kinematics for the 4-DOF robot."""
    base_length = 0.05
    reach = 0.1 + 0.25 + 0.25 + 0.15  # sum of the movable link lengths

    # Fully extended along +x: tip is base offset plus the link lengths.
    four_dof_skeleton.q = np.zeros(4)
    compute_forward_kinematics(four_dof_skeleton)
    assert four_dof_skeleton.links[-1].xe == pytest.approx(base_length + reach)
    assert four_dof_skeleton.links[-1].ye == pytest.approx(0.0)

    # 90 degrees at the second joint (within its [0, 120] limit): the links beyond
    # the first point straight up, while the base and first link stay along +x.
    four_dof_skeleton.q = np.array([0.0, np.pi / 2, 0.0, 0.0])
    compute_forward_kinematics(four_dof_skeleton)
    assert four_dof_skeleton.links[-1].xe == pytest.approx(base_length + 0.1)
    assert four_dof_skeleton.links[-1].ye == pytest.approx(0.25 + 0.25 + 0.15)


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
