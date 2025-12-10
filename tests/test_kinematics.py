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
