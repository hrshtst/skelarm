"""Static-equilibrium checks for the inverse dynamics.

These mirror the "typical configurations and external forces" suggested in the
inverse-dynamics lesson: with the arm at rest (``dq = ddq = 0``) and no gravity,
the only loads are an external force applied at the endpoint. The actuator torque
at each joint must then balance the moment of that force about the joint, a value
that can be worked out by hand independently of the recursive Newton-Euler code.
"""

from __future__ import annotations

import numpy as np
import pytest

from skelarm.dynamics import compute_inverse_dynamics
from skelarm.kinematics import compute_forward_kinematics
from skelarm.skeleton import LinkProp, Skeleton


def _uniform_arm(num_joints: int, length: float = 1.0) -> Skeleton:
    """Build a rest arm of identical unit-ish links (mass is irrelevant when static)."""
    props = [
        LinkProp(length=length, m=1.0, i=0.1, rgx=length / 2, rgy=0.0, qmin=-np.pi, qmax=np.pi)
        for _ in range(num_joints)
    ]
    return Skeleton(props)


def _apply_static_tip_force(skeleton: Skeleton, force: tuple[float, float]) -> np.ndarray:
    """Hold the arm static, push the endpoint with a world-frame force, and run ID."""
    skeleton.dq = np.zeros(skeleton.num_joints)
    skeleton.ddq = np.zeros(skeleton.num_joints)

    tip = skeleton.links[-1]
    tip.fex, tip.fey = force
    # Point of application = the link tip, expressed in the link-local frame.
    tip.rex, tip.rey = tip.prop.length, 0.0

    compute_inverse_dynamics(skeleton)
    return skeleton.tau


def _expected_static_tau(skeleton: Skeleton, force: tuple[float, float]) -> np.ndarray:
    """Torque that balances the moment of ``force`` (applied at the tip) about each joint."""
    compute_forward_kinematics(skeleton)
    tip = skeleton.links[-1]
    fx, fy = force
    taus = [-((tip.xe - link.x) * fy - (tip.ye - link.y) * fx) for link in skeleton.links[1:]]
    return np.array(taus)


def test_straight_horizontal_arm_axial_force_needs_no_torque() -> None:
    """A force directed along a straight horizontal arm produces no joint torque."""
    skeleton = _uniform_arm(3)
    skeleton.q = np.zeros(3)  # fully extended along +x

    tau = _apply_static_tip_force(skeleton, (5.0, 0.0))

    assert tau == pytest.approx(np.zeros(3))


def test_straight_horizontal_arm_transverse_force() -> None:
    """A transverse tip force loads each joint by force times its lever arm."""
    skeleton = _uniform_arm(3)
    skeleton.q = np.zeros(3)  # joints at x = 0, 1, 2; tip at x = 3

    tau = _apply_static_tip_force(skeleton, (0.0, 3.0))

    # Each torque is minus the transverse force times the joint-to-tip distance.
    assert tau == pytest.approx(np.array([-9.0, -6.0, -3.0]))


def test_l_shaped_arm_horizontal_force() -> None:
    """An L-shaped pose with a horizontal tip force loads both joints equally."""
    skeleton = _uniform_arm(2)
    skeleton.q = np.array([0.0, np.pi / 2])  # link1 along +x, link2 straight up

    tau = _apply_static_tip_force(skeleton, (2.0, 0.0))

    # Tip at (1, 1); both joints sit a unit below the tip, lever arm 1 against F_x.
    assert tau == pytest.approx(np.array([2.0, 2.0]))


@pytest.mark.parametrize(
    ("angles", "force"),
    [
        (np.array([np.pi / 2, 0.0]), (0.0, -4.0)),  # arm straight up, downward pull
        (np.array([np.pi / 4, np.pi / 4]), (1.5, -2.0)),  # 45 deg then 90 deg, oblique force
        (np.array([np.pi / 4, np.pi / 2, -np.pi / 4]), (-3.0, 2.0)),  # three-link zig-zag
        (np.array([0.3, -0.6, 0.9, -0.2]), (2.5, 1.0)),  # arbitrary four-link pose
    ],
)
def test_static_torque_matches_moment_balance(angles: np.ndarray, force: tuple[float, float]) -> None:
    """RNE joint torques equal the external-force moment about each joint, at rest."""
    skeleton = _uniform_arm(len(angles))
    skeleton.q = angles

    expected = _expected_static_tau(skeleton, force)
    tau = _apply_static_tip_force(skeleton, force)

    assert tau == pytest.approx(expected)


def test_static_constraint_force_transmits_external_force() -> None:
    """At rest, every movable link carries the same joint constraint force, -F."""
    skeleton = _uniform_arm(3)
    skeleton.q = np.array([0.2, 0.5, -0.3])

    force = (1.5, -2.5)
    _apply_static_tip_force(skeleton, force)

    for link in skeleton.links[1:]:
        assert link.f == pytest.approx(np.array([-force[0], -force[1]]))
