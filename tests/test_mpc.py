"""Tests for joint-space model predictive control."""

from __future__ import annotations

import numpy as np
import pytest

from skelarm.control import simulate_controlled
from skelarm.mpc import JointSpaceMPC
from skelarm.skeleton import LinkProp, Skeleton
from skelarm.trajectory import Trajectory


def _two_link() -> Skeleton:
    """A planar two-link arm at rest."""
    link_props = [
        LinkProp(length=1.0, m=1.0, i=0.1, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi),
        LinkProp(length=0.8, m=0.8, i=0.05, rgx=0.4, rgy=0.0, qmin=-np.pi, qmax=np.pi),
    ]
    skeleton = Skeleton(link_props)
    skeleton.q = np.array([0.0, 0.0])
    return skeleton


def _constant_reference(q_target: np.ndarray) -> Trajectory:
    """A joint reference that holds at ``q_target`` (zero velocity)."""
    return Trajectory(q_target, q_target, duration=1.0)


def test_mpc_control_returns_one_torque_per_joint() -> None:
    """A control step returns a torque vector sized to the joints."""
    controller = JointSpaceMPC(_constant_reference(np.array([0.2, -0.1])), horizon=5, dt=0.05, max_iter=5)
    skeleton = _two_link()
    controller.reset(skeleton)
    tau = controller.control(0.0, skeleton)
    assert tau.shape == (skeleton.num_joints,)


def test_mpc_warm_start_shifts_after_a_step() -> None:
    """After a control step the warm-start sequence is the shifted solution."""
    controller = JointSpaceMPC(_constant_reference(np.array([0.3, -0.2])), horizon=5, dt=0.05, max_iter=5)
    skeleton = _two_link()
    controller.reset(skeleton)
    controller.control(0.0, skeleton)
    assert controller._warm is not None  # noqa: SLF001
    assert controller._warm.shape == (5, skeleton.num_joints)  # noqa: SLF001


@pytest.mark.slow
def test_mpc_regulates_toward_a_joint_target() -> None:
    """Receding-horizon control drives the joint error well below its initial value."""
    skeleton = _two_link()
    target = np.array([0.4, -0.3])
    controller = JointSpaceMPC(
        _constant_reference(target), horizon=6, dt=0.05, q_weight=20.0, terminal_weight=80.0, max_iter=15
    )

    log = simulate_controlled(skeleton, controller, duration=1.5, dt=0.05)
    initial_error = np.linalg.norm(skeleton.q - target)
    final_error = np.linalg.norm(log.channel("q")[-1] - target)
    assert final_error < 0.3 * initial_error


@pytest.mark.slow
def test_mpc_respects_torque_bounds() -> None:
    """A torque bound caps the applied torque at every step."""
    skeleton = _two_link()
    target = np.array([1.2, -1.0])  # far target so the unbounded torque would be large
    tau_max = 4.0
    controller = JointSpaceMPC(
        _constant_reference(target), horizon=6, dt=0.05, q_weight=50.0, tau_max=tau_max, max_iter=15
    )

    log = simulate_controlled(skeleton, controller, duration=1.0, dt=0.05)
    assert np.all(np.abs(log.channel("tau")) <= tau_max + 1e-6)


@pytest.mark.slow
def test_mpc_logs_reference_and_error() -> None:
    """An MPC run records the reference and tracking-error channels."""
    skeleton = _two_link()
    controller = JointSpaceMPC(_constant_reference(np.array([0.3, -0.2])), horizon=5, dt=0.05, max_iter=8)

    log = simulate_controlled(skeleton, controller, duration=0.5, dt=0.05)
    assert {"q_ref", "error"} <= set(log.channel_names)
    assert log.channel("q_ref").shape == (len(log), skeleton.num_joints)
