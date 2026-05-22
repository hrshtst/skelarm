"""Tests for the kinematics module."""

from __future__ import annotations

import numpy as np
import pytest

from skelarm.kinematics import compute_forward_kinematics
from skelarm.skeleton import LinkProp, Skeleton


def test_forward_kinematics_single_link_horizontal() -> None:
    """Test FK for a single link arm extended horizontally."""
    link_prop = LinkProp(length=1.0, m=1.0, i=1.0, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi)
    skeleton = Skeleton(link_props=[link_prop])
    skeleton.q = np.array([0.0])  # Horizontal

    compute_forward_kinematics(skeleton)

    # Base of the link
    assert skeleton.links[0].x == pytest.approx(0.0)
    assert skeleton.links[0].y == pytest.approx(0.0)
    # Tip of the link
    assert skeleton.links[0].xe == pytest.approx(1.0)
    assert skeleton.links[0].ye == pytest.approx(0.0)


def test_forward_kinematics_single_link_vertical() -> None:
    """Test FK for a single link arm extended vertically upwards."""
    link_prop = LinkProp(length=1.0, m=1.0, i=1.0, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi)
    skeleton = Skeleton(link_props=[link_prop])
    skeleton.q = np.array([np.pi / 2])  # Vertical upwards

    compute_forward_kinematics(skeleton)

    # Base of the link
    assert skeleton.links[0].x == pytest.approx(0.0)
    assert skeleton.links[0].y == pytest.approx(0.0)
    # Tip of the link
    assert skeleton.links[0].xe == pytest.approx(0.0)
    assert skeleton.links[0].ye == pytest.approx(1.0)


def test_forward_kinematics_two_links_horizontal() -> None:
    """Test FK for a two-link arm extended horizontally."""
    link_prop1 = LinkProp(length=1.0, m=1.0, i=1.0, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi)
    link_prop2 = LinkProp(length=1.0, m=1.0, i=1.0, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi)
    skeleton = Skeleton(link_props=[link_prop1, link_prop2])
    skeleton.q = np.array([0.0, 0.0])  # Both links horizontal

    compute_forward_kinematics(skeleton)

    # Link 1
    assert skeleton.links[0].x == pytest.approx(0.0)
    assert skeleton.links[0].y == pytest.approx(0.0)
    assert skeleton.links[0].xe == pytest.approx(1.0)
    assert skeleton.links[0].ye == pytest.approx(0.0)

    # Link 2 (starts where link 1 ends)
    assert skeleton.links[1].x == pytest.approx(1.0)
    assert skeleton.links[1].y == pytest.approx(0.0)
    assert skeleton.links[1].xe == pytest.approx(2.0)
    assert skeleton.links[1].ye == pytest.approx(0.0)


def test_forward_kinematics_two_links_90_degrees() -> None:
    """Test FK for a two-link arm, first link horizontal, second link vertical."""
    link_prop1 = LinkProp(length=1.0, m=1.0, i=1.0, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi)
    link_prop2 = LinkProp(length=1.0, m=1.0, i=1.0, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi)
    skeleton = Skeleton(link_props=[link_prop1, link_prop2])
    skeleton.q = np.array([0.0, np.pi / 2])  # Link1 horizontal, Link2 vertical relative to Link1

    compute_forward_kinematics(skeleton)

    # Link 1
    assert skeleton.links[0].x == pytest.approx(0.0)
    assert skeleton.links[0].y == pytest.approx(0.0)
    assert skeleton.links[0].xe == pytest.approx(1.0)
    assert skeleton.links[0].ye == pytest.approx(0.0)

    # Link 2 (starts where link 1 ends)
    assert skeleton.links[1].x == pytest.approx(1.0)
    assert skeleton.links[1].y == pytest.approx(0.0)
    assert skeleton.links[1].xe == pytest.approx(1.0)
    assert skeleton.links[1].ye == pytest.approx(1.0)


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
    assert skeleton.links[0].x == pytest.approx(0.0)
    assert skeleton.links[0].y == pytest.approx(0.0)
    assert skeleton.links[0].xe == pytest.approx(expected_x1)
    assert skeleton.links[0].ye == pytest.approx(expected_y1)

    # Link 2
    # Absolute angle for link 2 is pi/4 + pi/4 = pi/2
    expected_x2 = expected_x1 + 1.0 * np.cos(np.pi / 2)
    expected_y2 = expected_y1 + 1.0 * np.sin(np.pi / 2)
    assert skeleton.links[1].x == pytest.approx(expected_x1)
    assert skeleton.links[1].y == pytest.approx(expected_y1)
    assert skeleton.links[1].xe == pytest.approx(expected_x2)
    assert skeleton.links[1].ye == pytest.approx(expected_y2)


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
    assert np.array([skeleton.links[0].ax, skeleton.links[0].ay]) == pytest.approx(first_tip_acceleration)
    assert skeleton.links[1].dv == pytest.approx(first_tip_acceleration)
    assert skeleton.links[1].dvc == pytest.approx(np.array([skeleton.links[1].agx, skeleton.links[1].agy]))
