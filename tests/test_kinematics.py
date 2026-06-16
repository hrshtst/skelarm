"""Tests for the kinematics module."""

from __future__ import annotations

import numpy as np
import pytest
from hypothesis import given, settings
from hypothesis import strategies as st
from hypothesis.extra.numpy import arrays

from skelarm.kinematics import (
    compute_coriolis_basis,
    compute_endpoint_acceleration,
    compute_endpoint_velocity,
    compute_forward_kinematics,
    compute_jacobian,
)
from skelarm.skeleton import LinkProp, Skeleton

# The arm is built around a fixed base link at ``links[0]``; the movable links
# therefore start at ``links[1]``.


def test_forward_kinematics_single_link_horizontal() -> None:
    """Test FK for a single link arm extended horizontally."""
    link_prop = LinkProp(length=1.0, m=1.0, i=1.0, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi)
    skeleton = Skeleton(link_props=[link_prop])
    skeleton.q = np.array([0.0])  # Horizontal

    compute_forward_kinematics(skeleton)

    # Base of the link
    assert skeleton.links[1].x == pytest.approx(0.0)
    assert skeleton.links[1].y == pytest.approx(0.0)
    # Tip of the link
    assert skeleton.links[1].xe == pytest.approx(1.0)
    assert skeleton.links[1].ye == pytest.approx(0.0)


def test_forward_kinematics_single_link_90_degrees() -> None:
    """Test FK for a single link arm rotated 90 degrees from the x-axis (pointing along +y)."""
    link_prop = LinkProp(length=1.0, m=1.0, i=1.0, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi)
    skeleton = Skeleton(link_props=[link_prop])
    skeleton.q = np.array([np.pi / 2])  # Rotated 90 degrees (points along +y)

    compute_forward_kinematics(skeleton)

    # Base of the link
    assert skeleton.links[1].x == pytest.approx(0.0)
    assert skeleton.links[1].y == pytest.approx(0.0)
    # Tip of the link
    assert skeleton.links[1].xe == pytest.approx(0.0)
    assert skeleton.links[1].ye == pytest.approx(1.0)


def test_forward_kinematics_two_links_horizontal() -> None:
    """Test FK for a two-link arm extended horizontally."""
    link_prop1 = LinkProp(length=1.0, m=1.0, i=1.0, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi)
    link_prop2 = LinkProp(length=1.0, m=1.0, i=1.0, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi)
    skeleton = Skeleton(link_props=[link_prop1, link_prop2])
    skeleton.q = np.array([0.0, 0.0])  # Both links horizontal

    compute_forward_kinematics(skeleton)

    # Link 1
    assert skeleton.links[1].x == pytest.approx(0.0)
    assert skeleton.links[1].y == pytest.approx(0.0)
    assert skeleton.links[1].xe == pytest.approx(1.0)
    assert skeleton.links[1].ye == pytest.approx(0.0)

    # Link 2 (starts where link 1 ends)
    assert skeleton.links[2].x == pytest.approx(1.0)
    assert skeleton.links[2].y == pytest.approx(0.0)
    assert skeleton.links[2].xe == pytest.approx(2.0)
    assert skeleton.links[2].ye == pytest.approx(0.0)


def test_forward_kinematics_two_links_90_degrees() -> None:
    """Test FK for a two-link arm, first link horizontal, second link perpendicular to it."""
    link_prop1 = LinkProp(length=1.0, m=1.0, i=1.0, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi)
    link_prop2 = LinkProp(length=1.0, m=1.0, i=1.0, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi)
    skeleton = Skeleton(link_props=[link_prop1, link_prop2])
    skeleton.q = np.array([0.0, np.pi / 2])  # Link1 horizontal, Link2 perpendicular to Link1

    compute_forward_kinematics(skeleton)

    # Link 1
    assert skeleton.links[1].x == pytest.approx(0.0)
    assert skeleton.links[1].y == pytest.approx(0.0)
    assert skeleton.links[1].xe == pytest.approx(1.0)
    assert skeleton.links[1].ye == pytest.approx(0.0)

    # Link 2 (starts where link 1 ends)
    assert skeleton.links[2].x == pytest.approx(1.0)
    assert skeleton.links[2].y == pytest.approx(0.0)
    assert skeleton.links[2].xe == pytest.approx(1.0)
    assert skeleton.links[2].ye == pytest.approx(1.0)


def test_forward_kinematics_two_links_45_degrees() -> None:
    """Test FK for a two-link arm, both links at 45 degrees."""
    link_prop1 = LinkProp(length=1.0, m=1.0, i=1.0, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi)
    link_prop2 = LinkProp(length=1.0, m=1.0, i=1.0, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi)
    skeleton = Skeleton(link_props=[link_prop1, link_prop2])
    # Link1 at 45 deg, Link2 at 45 deg relative to Link1 (total 90 deg from base)
    skeleton.q = np.array([np.pi / 4, np.pi / 4])

    compute_forward_kinematics(skeleton)

    # Link 1
    expected_x1 = 1.0 * np.cos(np.pi / 4)
    expected_y1 = 1.0 * np.sin(np.pi / 4)
    assert skeleton.links[1].x == pytest.approx(0.0)
    assert skeleton.links[1].y == pytest.approx(0.0)
    assert skeleton.links[1].xe == pytest.approx(expected_x1)
    assert skeleton.links[1].ye == pytest.approx(expected_y1)

    # Link 2
    # Absolute angle for link 2 is pi/4 + pi/4 = pi/2
    expected_x2 = expected_x1 + 1.0 * np.cos(np.pi / 2)
    expected_y2 = expected_y1 + 1.0 * np.sin(np.pi / 2)
    assert skeleton.links[2].x == pytest.approx(expected_x1)
    assert skeleton.links[2].y == pytest.approx(expected_y1)
    assert skeleton.links[2].xe == pytest.approx(expected_x2)
    assert skeleton.links[2].ye == pytest.approx(expected_y2)


def test_base_link_offsets_first_joint() -> None:
    """A non-zero base length shifts the first joint to (base_length, 0)."""
    link_prop = LinkProp(length=1.0, m=1.0, i=1.0, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi)
    skeleton = Skeleton(link_props=[link_prop], base_length=0.5)

    expected_total_links = 2  # base + one movable
    assert skeleton.num_links == expected_total_links
    assert skeleton.num_joints == 1
    assert skeleton.base_length == pytest.approx(0.5)

    skeleton.q = np.array([0.0])
    compute_forward_kinematics(skeleton)

    # Base link spans the origin to (0.5, 0); the movable joint starts there.
    assert skeleton.links[0].x == pytest.approx(0.0)
    assert skeleton.links[0].xe == pytest.approx(0.5)
    assert skeleton.links[1].x == pytest.approx(0.5)
    assert skeleton.links[1].xe == pytest.approx(1.5)


def test_forward_kinematics_populates_velocities_from_differential_motion() -> None:
    """Joint, tip, and COM velocities should match finite differences."""
    link_props = [
        LinkProp(length=1.0, m=1.0, i=0.1, rgx=0.4, rgy=0.1, qmin=-np.pi, qmax=np.pi),
        LinkProp(length=1.2, m=1.0, i=0.1, rgx=0.5, rgy=-0.1, qmin=-np.pi, qmax=np.pi),
    ]
    skeleton = Skeleton(link_props)
    skeleton.q = np.array([0.4, -0.7])
    skeleton.dq = np.array([0.8, 0.3])
    skeleton.ddq = np.array([0.0, 0.0])

    next_skeleton = Skeleton(link_props)
    epsilon = 1e-7
    next_skeleton.q = skeleton.q + epsilon * skeleton.dq
    next_skeleton.dq = skeleton.dq
    next_skeleton.ddq = skeleton.ddq

    compute_forward_kinematics(skeleton)
    compute_forward_kinematics(next_skeleton)

    for link, next_link in zip(skeleton.links, next_skeleton.links, strict=True):
        joint_velocity = (np.array([next_link.x, next_link.y]) - np.array([link.x, link.y])) / epsilon
        tip_velocity = (np.array([next_link.xe, next_link.ye]) - np.array([link.xe, link.ye])) / epsilon
        com_velocity = (np.array([next_link.xg, next_link.yg]) - np.array([link.xg, link.yg])) / epsilon

        assert link.v == pytest.approx(joint_velocity, abs=1e-6)
        assert np.array([link.vx, link.vy]) == pytest.approx(tip_velocity, abs=1e-6)
        assert link.vc == pytest.approx(com_velocity, abs=1e-6)


def test_forward_kinematics_populates_accelerations_analytically() -> None:
    """Tip and COM accelerations should follow planar rigid-body formulas."""
    link_props = [
        LinkProp(length=1.0, m=1.0, i=0.1, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi),
        LinkProp(length=1.2, m=1.0, i=0.1, rgx=0.6, rgy=0.2, qmin=-np.pi, qmax=np.pi),
    ]
    skeleton = Skeleton(link_props)
    skeleton.q = np.array([0.3, -0.5])
    skeleton.dq = np.array([0.7, -0.2])
    skeleton.ddq = np.array([1.1, -0.4])

    compute_forward_kinematics(skeleton)

    first_tip_acceleration = np.array(
        [
            -(0.7**2) * np.cos(0.3) - 1.1 * np.sin(0.3),
            -(0.7**2) * np.sin(0.3) + 1.1 * np.cos(0.3),
        ]
    )
    assert np.array([skeleton.links[1].ax, skeleton.links[1].ay]) == pytest.approx(first_tip_acceleration)
    assert skeleton.links[2].dv == pytest.approx(first_tip_acceleration)
    assert skeleton.links[2].dvc == pytest.approx(np.array([skeleton.links[2].agx, skeleton.links[2].agy]))


# === Jacobian and Coriolis-basis differential kinematics ===


def test_jacobian_shape_and_single_link_columns() -> None:
    """The Jacobian is 2 x num_joints and matches the closed-form for one link."""
    link_prop = LinkProp(length=1.0, m=1.0, i=0.1, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi)
    skeleton = Skeleton(link_props=[link_prop])
    angle = 0.6
    skeleton.q = np.array([angle])

    compute_forward_kinematics(skeleton)
    jacobian = compute_jacobian(skeleton)

    assert jacobian.shape == (2, 1)
    # j_x1 = -l sin(theta), j_y1 = l cos(theta)
    assert jacobian[0, 0] == pytest.approx(-np.sin(angle))
    assert jacobian[1, 0] == pytest.approx(np.cos(angle))


def test_endpoint_velocity_matches_recursive_forward_kinematics() -> None:
    """Jacobian-based endpoint velocity equals the value propagated by FK."""
    link_props = [
        LinkProp(length=1.0, m=1.0, i=0.1, rgx=0.4, rgy=0.1, qmin=-np.pi, qmax=np.pi),
        LinkProp(length=1.2, m=1.0, i=0.1, rgx=0.5, rgy=-0.1, qmin=-np.pi, qmax=np.pi),
        LinkProp(length=0.8, m=1.0, i=0.1, rgx=0.3, rgy=0.0, qmin=-np.pi, qmax=np.pi),
    ]
    skeleton = Skeleton(link_props, base_length=0.3)
    skeleton.q = np.array([0.4, -0.7, 0.2])
    skeleton.dq = np.array([0.8, 0.3, -0.5])

    compute_forward_kinematics(skeleton)
    tip = skeleton.links[-1]

    velocity = compute_endpoint_velocity(skeleton)
    assert velocity == pytest.approx(np.array([tip.vx, tip.vy]))


def test_endpoint_acceleration_matches_recursive_forward_kinematics() -> None:
    """Jacobian + Coriolis basis reproduce the FK-propagated endpoint acceleration."""
    link_props = [
        LinkProp(length=1.0, m=1.0, i=0.1, rgx=0.4, rgy=0.1, qmin=-np.pi, qmax=np.pi),
        LinkProp(length=1.2, m=1.0, i=0.1, rgx=0.5, rgy=-0.1, qmin=-np.pi, qmax=np.pi),
        LinkProp(length=0.8, m=1.0, i=0.1, rgx=0.3, rgy=0.0, qmin=-np.pi, qmax=np.pi),
    ]
    skeleton = Skeleton(link_props, base_length=0.3)
    skeleton.q = np.array([0.4, -0.7, 0.2])
    skeleton.dq = np.array([0.8, 0.3, -0.5])
    skeleton.ddq = np.array([0.6, -0.4, 1.0])

    compute_forward_kinematics(skeleton)
    tip = skeleton.links[-1]

    basis = compute_coriolis_basis(skeleton)
    assert basis.shape == (2, skeleton.num_joints)

    acceleration = compute_endpoint_acceleration(skeleton)
    assert acceleration == pytest.approx(np.array([tip.ax, tip.ay]))


@st.composite
def _differential_skeleton(draw: st.DrawFn) -> Skeleton:
    """Random arm with random base length, pose, velocity, and acceleration."""
    num_joints = draw(st.integers(min_value=1, max_value=4))
    link_props = []
    for _ in range(num_joints):
        length = draw(st.floats(min_value=0.3, max_value=1.5))
        rgx = draw(st.floats(min_value=-0.4, max_value=0.4))
        rgy = draw(st.floats(min_value=-0.2, max_value=0.2))
        link_props.append(LinkProp(length=length, m=1.0, i=0.1, rgx=rgx, rgy=rgy, qmin=-np.pi, qmax=np.pi))

    base_length = draw(st.floats(min_value=0.0, max_value=1.0))
    skeleton = Skeleton(link_props, base_length=base_length)
    elements = st.floats(min_value=-2.0, max_value=2.0)
    skeleton.q = draw(arrays(np.float64, num_joints, elements=elements))
    skeleton.dq = draw(arrays(np.float64, num_joints, elements=elements))
    skeleton.ddq = draw(arrays(np.float64, num_joints, elements=elements))
    return skeleton


@settings(deadline=None)
@given(skeleton=_differential_skeleton())
def test_jacobian_endpoint_velocity_acceleration_consistency(skeleton: Skeleton) -> None:
    """Property test: the two independent endpoint formulas always agree."""
    compute_forward_kinematics(skeleton)
    tip = skeleton.links[-1]

    velocity = compute_endpoint_velocity(skeleton)
    acceleration = compute_endpoint_acceleration(skeleton)

    assert velocity == pytest.approx(np.array([tip.vx, tip.vy]), abs=1e-9)
    assert acceleration == pytest.approx(np.array([tip.ax, tip.ay]), abs=1e-9)
