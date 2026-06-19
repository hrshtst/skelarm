"""Tests for trajectory-tracking controllers and the fixed-step control loop."""

from __future__ import annotations

import numpy as np
import pytest

from skelarm.control import (
    ComputedTorque,
    Controller,
    InverseDynamicsFeedforwardPD,
    JointPD,
    ik_joint_reference,
    resolved_rate_joint_reference,
    simulate_controlled,
)
from skelarm.recording import StateLog
from skelarm.skeleton import LinkProp, Skeleton
from skelarm.trajectory import Trajectory


def _two_link() -> Skeleton:
    """A planar two-link arm at a bent, non-singular pose."""
    link_props = [
        LinkProp(length=1.0, m=1.0, i=0.1, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi),
        LinkProp(length=0.8, m=0.8, i=0.05, rgx=0.4, rgy=0.0, qmin=-np.pi, qmax=np.pi),
    ]
    skeleton = Skeleton(link_props)
    skeleton.q = np.array([0.3, 0.3])
    return skeleton


def _tip(skeleton: Skeleton) -> np.ndarray:
    """Current endpoint position of the arm."""
    tip = skeleton.links[-1]
    return np.array([tip.xe, tip.ye])


def _tip_at(skeleton: Skeleton, q: np.ndarray) -> np.ndarray:
    """Endpoint position the arm would have at joint angles ``q`` (no mutation)."""
    model = skeleton.clone()
    model.q = q
    return _tip(model)


def test_computed_torque_regulates_to_a_joint_target() -> None:
    """With the exact model, computed torque drives the arm to the reference target."""
    skeleton = _two_link()
    skeleton.q = np.array([0.0, 0.0])
    target = np.array([0.6, -0.4])
    reference = Trajectory([0.0, 0.0], target, duration=1.0, schedule="quintic")
    controller = ComputedTorque(reference, kp=100.0, kd=20.0)

    log = simulate_controlled(skeleton, controller, duration=2.0, dt=0.002)
    assert log.channel("q")[-1] == pytest.approx(target, abs=1e-2)


def test_joint_pd_converges_to_the_target_without_gravity() -> None:
    """A PD regulator settles on the target (zero steady-state error without gravity)."""
    skeleton = _two_link()
    skeleton.q = np.array([0.0, 0.0])
    target = np.array([0.5, -0.3])
    reference = Trajectory([0.0, 0.0], target, duration=1.0)
    controller = JointPD(reference, kp=300.0, kd=40.0)

    log = simulate_controlled(skeleton, controller, duration=3.0, dt=0.002)
    assert log.channel("q")[-1] == pytest.approx(target, abs=3e-2)


def test_inverse_dynamics_feedforward_tracks_the_target() -> None:
    """Inverse-dynamics feedforward plus PD reaches the reference target."""
    skeleton = _two_link()
    skeleton.q = np.array([0.0, 0.0])
    target = np.array([0.4, 0.5])
    reference = Trajectory([0.0, 0.0], target, duration=1.0)
    controller = InverseDynamicsFeedforwardPD(reference, kp=100.0, kd=20.0)

    log = simulate_controlled(skeleton, controller, duration=2.0, dt=0.002)
    assert log.channel("q")[-1] == pytest.approx(target, abs=1e-2)


def test_ik_joint_reference_follows_the_task_path() -> None:
    """Samplewise IK produces joint angles whose forward kinematics match the task path."""
    skeleton = _two_link()
    skeleton.q = np.array([0.6, 1.0])  # folded pose, well inside the workspace
    p0 = _tip(skeleton)
    p1 = p0 + np.array([-0.2, -0.15])
    task = Trajectory(p0, p1, duration=1.0)
    q_before = skeleton.q.copy()

    reference = ik_joint_reference(skeleton, task, dt=0.05)
    for t in (0.0, 0.5, 1.0):
        q_r, _, _ = reference.sample(t)
        assert _tip_at(skeleton, q_r) == pytest.approx(task.sample(t)[0], abs=1e-3)
    assert skeleton.q == pytest.approx(q_before)  # conversion does not mutate the input


def test_resolved_rate_reaches_the_task_target() -> None:
    """Resolved-rate conversion with task feedback ends at the task target."""
    skeleton = _two_link()
    p0 = _tip(skeleton)
    p1 = p0 + np.array([-0.15, 0.2])
    task = Trajectory(p0, p1, duration=1.0)

    reference = resolved_rate_joint_reference(skeleton, task, dt=0.01, k_task=10.0)
    q_final, _, _ = reference.sample(1.0)
    assert _tip_at(skeleton, q_final) == pytest.approx(p1, abs=1e-2)


def test_simulate_controlled_records_without_mutating_input() -> None:
    """The loop returns a StateLog of the right length and leaves the input skeleton intact."""
    skeleton = _two_link()
    before = skeleton.q.copy()
    reference = Trajectory(skeleton.q, np.array([0.4, -0.2]), duration=1.0)
    controller = ComputedTorque(reference, kp=100.0, kd=20.0)

    steps = 100
    log = simulate_controlled(skeleton, controller, duration=1.0, dt=0.01)
    assert isinstance(log, StateLog)
    assert len(log) == steps + 1
    assert {"q", "dq", "tau", "q_ref", "error"} <= set(log.channel_names)
    assert skeleton.q == pytest.approx(before)


def test_planned_reach_end_to_end_reaches_target_and_replays() -> None:
    """A minimum-jerk task reach, converted by IK and tracked, ends at the target and replays."""
    skeleton = _two_link()
    skeleton.q = np.array([0.6, 1.0])  # folded pose, well inside the workspace
    p0 = _tip(skeleton)
    target = p0 + np.array([-0.2, -0.15])
    task = Trajectory(p0, target, duration=1.0, schedule="minimum_jerk")
    reference = ik_joint_reference(skeleton, task, dt=0.02)
    controller = ComputedTorque(reference, kp=200.0, kd=30.0)

    log = simulate_controlled(skeleton, controller, duration=2.0, dt=0.002)
    assert _tip_at(skeleton, log.channel("q")[-1]) == pytest.approx(target, abs=2e-2)
    assert log.build_skeleton().num_joints == skeleton.num_joints


def test_controller_is_callable_as_a_torque_function() -> None:
    """A stateless controller can be used as a simulate_robot torque callback."""
    controller = ComputedTorque(Trajectory([0.0, 0.0], [0.1, 0.1], duration=1.0), kp=10.0, kd=2.0)
    assert isinstance(controller, Controller)
    tau = controller(0.0, _two_link())
    assert tau.shape == (2,)
