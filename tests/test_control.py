# Copyright (C) 2025-2026 Hiroshi Atsuta <atsuta@ieee.org>
# SPDX-License-Identifier: GPL-3.0-only

"""Tests for trajectory-tracking controllers and the fixed-step control loop."""

from __future__ import annotations

import warnings
from typing import TYPE_CHECKING

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
from skelarm.dynamics import integrate_with_limits
from skelarm.kinematics import compute_jacobian
from skelarm.recording import StateLog
from skelarm.skeleton import LinkProp, Skeleton
from skelarm.trajectory import Trajectory

if TYPE_CHECKING:
    from collections.abc import Callable


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


def test_simulate_controlled_enforce_limits_toggle() -> None:
    """``enforce_limits=False`` lets a joint pass the bound the default hard stop pins it at."""
    limit = 0.2

    def _final_q(*, enforce_limits: bool) -> np.ndarray:
        link_props = [
            LinkProp(length=1.0, m=1.0, i=0.1, rgx=0.5, rgy=0.0, qmin=-limit, qmax=limit),
            LinkProp(length=0.8, m=0.8, i=0.05, rgx=0.4, rgy=0.0, qmin=-np.pi, qmax=np.pi),
        ]
        skeleton = Skeleton(link_props)
        skeleton.q = np.array([0.0, 0.0])
        reference = Trajectory([0.0, 0.0], np.array([0.5, 0.0]), duration=1.0)
        controller = JointPD(reference, kp=300.0, kd=40.0)
        log = simulate_controlled(skeleton, controller, duration=2.0, dt=0.002, enforce_limits=enforce_limits)
        return log.channel("q")[-1]

    clamped = _final_q(enforce_limits=True)
    free = _final_q(enforce_limits=False)
    assert clamped[0] == pytest.approx(limit, abs=1e-2)  # pinned at the bound
    assert free[0] == pytest.approx(0.5, abs=3e-2)  # sails past it to the reference


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


def test_ik_joint_reference_warns_when_samples_do_not_converge() -> None:
    """A task path leaving the workspace triggers one aggregated warning about the failed samples."""
    skeleton = _two_link()  # total reach 1.8 m
    task = Trajectory(_tip(skeleton), np.array([3.0, 0.0]), duration=1.0)

    with pytest.warns(UserWarning, match="reference samples"):
        ik_joint_reference(skeleton, task, dt=0.1)


def test_ik_joint_reference_reachable_path_does_not_warn() -> None:
    """A path inside the workspace converts without emitting any warning."""
    skeleton = _two_link()
    skeleton.q = np.array([0.6, 1.0])  # folded pose, well inside the workspace
    p0 = _tip(skeleton)
    task = Trajectory(p0, p0 + np.array([-0.2, -0.15]), duration=1.0)

    with warnings.catch_warnings():
        warnings.simplefilter("error")
        ik_joint_reference(skeleton, task, dt=0.05)


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


class _ConstantTorque(Controller):
    """A controller that always applies the same joint torque."""

    def __init__(self, tau: np.ndarray) -> None:
        self.tau = tau

    def control(self, t: float, skeleton: Skeleton) -> np.ndarray:  # noqa: ARG002
        return self.tau


def test_simulate_controlled_applies_the_tip_force_through_the_jacobian() -> None:
    """A scripted tip force acts as ``J^T F`` on top of the controller's torque at every step."""
    skeleton = _two_link()
    tau = np.array([0.5, -0.2])
    force = np.array([3.0, -2.0])
    dt = 0.01

    log = simulate_controlled(
        skeleton, _ConstantTorque(tau), duration=0.2, dt=dt, external_force=lambda _t, _skeleton: force
    )

    model = skeleton.clone()
    lower = np.array([link.prop.qmin for link in model.links[1:]])
    upper = np.array([link.prop.qmax for link in model.links[1:]])
    expected = [model.q.copy()]
    for _ in range(20):
        integrate_with_limits(model, tau + compute_jacobian(model).T @ force, dt, lower, upper)
        expected.append(model.q.copy())
    assert log.channel("q") == pytest.approx(np.array(expected), abs=1e-12)
    assert log.channel("tau") == pytest.approx(np.tile(tau, (21, 1)))  # the controller's torque alone


def test_a_tip_force_alone_pushes_the_tip_its_way() -> None:
    """Starting at rest without control, the tip first moves partly along the force.

    The tip accelerates by ``J H^-1 J^T F``, and ``J H^-1 J^T`` is positive definite,
    so the motion has a positive component along the force (not necessarily all of it).
    """
    skeleton = _two_link()
    force = np.array([0.0, 5.0])

    log = simulate_controlled(
        skeleton, _ConstantTorque(np.zeros(2)), duration=0.05, dt=0.002, external_force=lambda _t, _skeleton: force
    )

    displacement = _tip_at(skeleton, log.channel("q")[-1]) - _tip(skeleton)
    assert displacement @ force > 0.0


def test_simulate_controlled_records_the_tip_force_for_replay() -> None:
    """The force of every frame, the final one included, is recorded as the ``ext_force`` channel."""
    skeleton = _two_link()
    seen: list[float] = []

    onset, end = 0.025, 0.065

    def pulse(t: float, _skeleton: Skeleton) -> np.ndarray:
        seen.append(t)
        return np.array([4.0, 1.0]) if onset < t < end else np.zeros(2)

    log = simulate_controlled(skeleton, _ConstantTorque(np.zeros(2)), duration=0.1, dt=0.01, external_force=pulse)

    times = np.asarray(log.times)
    expected = np.where(((times > onset) & (times < end))[:, np.newaxis], [4.0, 1.0], 0.0)
    assert log.channel("ext_force") == pytest.approx(expected)
    assert seen == pytest.approx(times.tolist())
    assert log.channel_meta["ext_force"]["columns"] == ["fx", "fy"]
    assert log.channel_meta["ext_force"]["unit"] == "N"


def test_simulate_controlled_without_a_tip_force_records_none() -> None:
    """Without a tip force, the log has no ``ext_force`` channel, as before."""
    log = simulate_controlled(_two_link(), _ConstantTorque(np.zeros(2)), duration=0.05, dt=0.01)

    assert "ext_force" not in log.channel_names


def test_a_state_dependent_tip_force_can_hold_the_tip() -> None:
    """The force sees the current state, so a stiff spring-damper can hold the tip against the controller."""
    skeleton = _two_link()
    skeleton.q = np.array([0.3, 1.5])  # bent, so the tip spring constrains both joints firmly
    held = _tip(skeleton)
    reference = Trajectory(skeleton.q, skeleton.q + np.array([0.4, -0.3]), duration=0.5)

    def hold(_t: float, state: Skeleton) -> np.ndarray:
        velocity = compute_jacobian(state) @ state.dq
        return -5000.0 * (_tip(state) - held) - 100.0 * velocity

    def tip_travel(external_force: Callable[[float, Skeleton], np.ndarray] | None) -> tuple[float, StateLog]:
        controller = JointPD(reference, kp=20.0, kd=5.0)
        log = simulate_controlled(skeleton, controller, duration=1.0, dt=0.002, external_force=external_force)
        tips = np.array([_tip_at(skeleton, q) for q in log.channel("q")])
        return float(np.max(np.linalg.norm(tips - held, axis=1))), log

    free, _ = tip_travel(None)
    blocked, log = tip_travel(hold)
    assert blocked < 0.02 * free
    assert np.abs(log.channel("error")[-1]).max() > 0.3  # noqa: PLR2004  # the controller kept pulling


def test_a_tip_force_must_be_two_dimensional() -> None:
    """A force that is not ``(fx, fy)`` is rejected."""
    with pytest.raises(ValueError, match="fx, fy"):
        simulate_controlled(
            _two_link(),
            _ConstantTorque(np.zeros(2)),
            duration=0.05,
            dt=0.01,
            external_force=lambda _t, _skeleton: np.zeros(3),
        )
