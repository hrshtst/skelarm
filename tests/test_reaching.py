"""Tests for endpoint-feedback reaching controllers (docs/reference/08)."""

from __future__ import annotations

import numpy as np
import pytest

from skelarm.control import simulate_controlled
from skelarm.dynamics import compute_forward_dynamics
from skelarm.kinematics import compute_endpoint_velocity
from skelarm.reaching import (
    OnlineReferenceShaping,
    PositionDependentShaping,
    TimeVaryingStiffness,
    VirtualSpringDamper,
    shaping_ratio,
)
from skelarm.skeleton import LinkProp, Skeleton


def _two_link() -> Skeleton:
    """A planar two-link arm at a folded, well-inside-the-workspace pose."""
    link_props = [
        LinkProp(length=1.0, m=1.0, i=0.1, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi),
        LinkProp(length=0.8, m=0.8, i=0.05, rgx=0.4, rgy=0.0, qmin=-np.pi, qmax=np.pi),
    ]
    skeleton = Skeleton(link_props)
    skeleton.q = np.array([0.6, 1.0])
    return skeleton


def _tip(skeleton: Skeleton) -> np.ndarray:
    """Current endpoint position."""
    tip = skeleton.links[-1]
    return np.array([tip.xe, tip.ye])


def _tip_at(skeleton: Skeleton, q: np.ndarray) -> np.ndarray:
    """Endpoint position at joint angles ``q`` (no mutation of ``skeleton``)."""
    model = skeleton.clone()
    model.q = q
    return _tip(model)


def _endpoint_speeds(log) -> np.ndarray:  # noqa: ANN001
    """Endpoint speed at every recorded frame, reconstructed from q and dq."""
    model = log.build_skeleton()
    q = log.channel("q")
    dq = log.channel("dq")
    speeds = []
    for i in range(len(log)):
        model.set_state(q=q[i], dq=dq[i])
        v = compute_endpoint_velocity(model)
        speeds.append(float(np.hypot(v[0], v[1])))
    return np.array(speeds)


def test_virtual_spring_damper_reaches_the_target() -> None:
    """A constant virtual spring-damper settles the endpoint on the target."""
    skeleton = _two_link()
    target = _tip(skeleton) + np.array([-0.25, -0.1])
    controller = VirtualSpringDamper(target, k_task=80.0, d_task=20.0, c_joint=0.5)

    log = simulate_controlled(skeleton, controller, duration=5.0, dt=0.002)
    assert _tip_at(skeleton, log.channel("q")[-1]) == pytest.approx(target, abs=2e-2)


def test_time_varying_stiffness_starts_at_zero_and_saturates() -> None:
    """The gamma-shaped schedule starts at k(0)=0 and approaches k0."""
    k0 = 100.0
    controller = TimeVaryingStiffness(np.array([0.5, 0.5]), k0=k0, alpha=5.0, zeta1=0.1)
    assert controller.stiffness(0.0) == pytest.approx(0.0)
    assert controller.stiffness(50.0) == pytest.approx(k0, rel=1e-3)
    assert 0.0 < controller.stiffness(0.4) < k0


def test_time_varying_stiffness_has_near_zero_initial_torque() -> None:
    """Because k(0)=0 and the arm starts at rest, the initial spring torque vanishes."""
    skeleton = _two_link()
    target = _tip(skeleton) + np.array([-0.25, -0.1])
    controller = TimeVaryingStiffness(target, k0=100.0, alpha=5.0, zeta1=0.1)
    controller.reset(skeleton)
    assert np.linalg.norm(controller.control(0.0, skeleton)) == pytest.approx(0.0, abs=1e-9)


def test_shaped_reaching_has_smaller_initial_acceleration_than_direct_spring() -> None:
    """Online shaping starts gently while a direct target spring jerks immediately."""
    skeleton = _two_link()
    target = _tip(skeleton) + np.array([-0.3, -0.15])

    direct = VirtualSpringDamper(target, k_task=120.0, d_task=20.0)
    direct.reset(skeleton)
    ddq_direct = compute_forward_dynamics(skeleton, direct.control(0.0, skeleton))

    shaped = OnlineReferenceShaping(target, k_task=120.0, d_task=20.0, r=0.5, t1=0.2, t2=0.2)
    shaped.reset(skeleton)
    ddq_shaped = compute_forward_dynamics(skeleton, shaped.control(0.0, skeleton))

    assert np.linalg.norm(ddq_shaped) < np.linalg.norm(ddq_direct)
    assert np.linalg.norm(ddq_shaped) == pytest.approx(0.0, abs=1e-9)  # equilibrium starts at the endpoint


def test_online_shaping_reaches_target_with_bell_shaped_speed() -> None:
    """Fixed-r shaping converges to the target with an interior speed peak."""
    skeleton = _two_link()
    target = _tip(skeleton) + np.array([-0.25, -0.15])
    controller = OnlineReferenceShaping(target, k_task=150.0, d_task=25.0, r=0.6, t1=0.2, t2=0.2)

    log = simulate_controlled(skeleton, controller, duration=4.0, dt=0.002)
    assert _tip_at(skeleton, log.channel("q")[-1]) == pytest.approx(target, abs=3e-2)

    speeds = _endpoint_speeds(log)
    peak = int(np.argmax(speeds))
    assert 0 < peak < len(speeds) - 1  # the peak is in the interior, not at an endpoint
    assert speeds[0] < 0.2 * speeds[peak]  # starts slow
    assert speeds[-1] < 0.2 * speeds[peak]  # ends slow


def test_position_dependent_shaping_converges_faster_than_conservative_fixed_r() -> None:
    """Position-dependent r reaches closer to the target than a fixed small r in the same time."""
    skeleton = _two_link()
    target = _tip(skeleton) + np.array([-0.25, -0.15])
    duration = 2.0

    fixed = OnlineReferenceShaping(target, k_task=150.0, d_task=25.0, r=0.1, t1=0.15, t2=0.15)
    position = PositionDependentShaping(target, k_task=150.0, d_task=25.0, a=0.01, t1=0.15, t2=0.15)

    log_fixed = simulate_controlled(skeleton, fixed, duration=duration, dt=0.002)
    log_position = simulate_controlled(skeleton, position, duration=duration, dt=0.002)

    err_fixed = np.linalg.norm(_tip_at(skeleton, log_fixed.channel("q")[-1]) - target)
    err_position = np.linalg.norm(_tip_at(skeleton, log_position.channel("q")[-1]) - target)
    assert err_position < err_fixed


def test_shaping_ratio_endpoints_match_the_paper() -> None:
    """r(d): ~sqrt(a) at the start (d=1), ~1 near the target (d=0), and smaller when pushed away."""
    a = 0.01
    assert shaping_ratio(1.0, a) == pytest.approx(np.sqrt(a))
    assert shaping_ratio(0.0, a) == pytest.approx(1.0)  # formula slightly exceeds 1 and is clamped
    assert shaping_ratio(2.0, a) < shaping_ratio(1.0, a)  # d > 1 (pushed away) reduces stiffness shift


def test_reaching_controllers_log_endpoint_and_equilibrium() -> None:
    """A reaching run records its endpoint and shaped-equilibrium channels."""
    skeleton = _two_link()
    target = _tip(skeleton) + np.array([-0.2, -0.1])
    controller = OnlineReferenceShaping(target, k_task=120.0, d_task=20.0, r=0.5, t1=0.2, t2=0.2)

    log = simulate_controlled(skeleton, controller, duration=1.0, dt=0.01)
    assert {"endpoint", "equilibrium"} <= set(log.channel_names)
    assert log.channel("equilibrium").shape == (len(log), 2)
