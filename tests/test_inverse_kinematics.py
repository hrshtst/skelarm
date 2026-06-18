"""Tests for numerical inverse kinematics."""

from __future__ import annotations

import numpy as np
import pytest
from hypothesis import given, settings
from hypothesis import strategies as st

from skelarm.kinematics import compute_forward_kinematics, compute_inverse_kinematics
from skelarm.skeleton import LinkProp, Skeleton


def _arm(num_links: int, length: float = 1.0, qmin: float = -np.pi, qmax: float = np.pi) -> Skeleton:
    """Build a uniform arm with ``num_links`` movable links of the given length."""
    props = [
        LinkProp(length=length, m=1.0, i=0.1, rgx=length / 2, rgy=0.0, qmin=qmin, qmax=qmax) for _ in range(num_links)
    ]
    return Skeleton(props)


def test_two_joint_reaches_reachable_target() -> None:
    """The default (Sugihara LM) solver reaches a reachable two-joint target."""
    skeleton = _arm(2)
    target = np.array([0.5, 1.2])

    result = compute_inverse_kinematics(skeleton, target, q0=np.array([0.5, 0.5]))

    assert result.success
    assert result.status == "converged"
    assert result.residual_norm == pytest.approx(0.0, abs=1e-6)
    tip = skeleton.links[-1]
    assert np.array([tip.xe, tip.ye]) == pytest.approx(target, abs=1e-6)


def test_lm_method_reaches_target() -> None:
    """The plain LM method also reaches a reachable target."""
    skeleton = _arm(2)
    target = np.array([0.5, 1.2])

    result = compute_inverse_kinematics(skeleton, target, method="lm", q0=np.array([0.5, 0.5]))

    assert result.success
    assert result.residual_norm == pytest.approx(0.0, abs=1e-6)


def test_redundant_arm_reaches_target_from_multiple_seeds() -> None:
    """A redundant (3-joint) arm reaches the same endpoint from different seeds."""
    target = np.array([1.5, 0.8])
    for seed in (np.array([0.2, 0.2, 0.2]), np.array([-0.5, 0.6, 0.3])):
        skeleton = _arm(3)
        result = compute_inverse_kinematics(skeleton, target, q0=seed)
        assert result.success
        tip = skeleton.links[-1]
        assert np.array([tip.xe, tip.ye]) == pytest.approx(target, abs=1e-5)


@pytest.mark.parametrize("num_links", [4, 6])
def test_higher_redundancy_arm_reaches_target_from_multiple_seeds(num_links: int) -> None:
    """Highly redundant arms (4 and 6 DOF) reach the same endpoint from different seeds."""
    target = np.array([1.5, 1.0])
    for seed_value in (0.2, -0.3):
        skeleton = _arm(num_links)
        result = compute_inverse_kinematics(skeleton, target, q0=np.full(num_links, seed_value))
        assert result.success
        tip = skeleton.links[-1]
        assert np.array([tip.xe, tip.ye]) == pytest.approx(target, abs=1e-5)


def test_unreachable_target_stalls_with_nonzero_residual() -> None:
    """An out-of-reach target terminates with a nonzero residual, not an exception."""
    skeleton = _arm(2)
    target = np.array([5.0, 0.0])  # beyond the reach of 2.0

    result = compute_inverse_kinematics(skeleton, target, q0=np.zeros(2))

    assert not result.success
    assert result.status == "stalled"
    assert result.residual_norm == pytest.approx(3.0, abs=1e-6)  # 5.0 - 2.0 stretched along +x


def test_converges_from_singular_seed() -> None:
    """A fully-stretched (singular-Jacobian) seed still converges without blowing up."""
    skeleton = _arm(2)
    target = np.array([1.8, 0.3])  # reachable, near the stretched pose

    result = compute_inverse_kinematics(skeleton, target, q0=np.zeros(2))

    assert result.success
    assert np.all(np.isfinite(result.q))


def test_joint_limits_respected() -> None:
    """Joint limits are enforced: the result stays in range and reports a limit hit."""
    lower = np.array([0.0, np.deg2rad(-150.0)])
    upper = np.array([np.deg2rad(30.0), np.deg2rad(150.0)])
    skeleton = Skeleton(
        [
            LinkProp(length=1.0, m=1.0, i=0.1, rgx=0.5, rgy=0.0, qmin=lower[0], qmax=upper[0]),
            LinkProp(length=1.0, m=1.0, i=0.1, rgx=0.5, rgy=0.0, qmin=lower[1], qmax=upper[1]),
        ]
    )
    target = np.array([0.0, 1.9])  # needs joint 1 ~90 deg, past its 30 deg limit

    result = compute_inverse_kinematics(skeleton, target, q0=np.array([0.1, 0.1]))

    assert result.joint_limits_hit
    assert not result.success
    assert np.all(result.q >= lower - 1e-9)
    assert np.all(result.q <= upper + 1e-9)


@settings(deadline=None, max_examples=50)
@given(
    q1=st.floats(min_value=-2.0, max_value=2.0),
    q2=st.floats(min_value=0.4, max_value=2.6),
)
def test_fk_ik_endpoint_roundtrip(q1: float, q2: float) -> None:
    """For an endpoint generated from a valid pose, IK recovers that endpoint."""
    skeleton = _arm(2)
    true_q = np.array([q1, q2])
    skeleton.q = true_q
    compute_forward_kinematics(skeleton)
    tip = skeleton.links[-1]
    target = np.array([tip.xe, tip.ye])

    seed = np.clip(true_q + 0.2, -np.pi, np.pi)
    result = compute_inverse_kinematics(skeleton, target, q0=seed)

    assert result.residual_norm == pytest.approx(0.0, abs=1e-4)
