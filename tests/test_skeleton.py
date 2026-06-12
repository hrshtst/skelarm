"""Tests for the Link and Skeleton data structures."""

from __future__ import annotations

import numpy as np
import pytest

from skelarm.skeleton import Link, LinkProp, Skeleton


def test_new_skeleton_starts_with_consistent_forward_kinematics() -> None:
    """Construction must leave the link positions matching the initial pose, without an explicit FK call."""
    link_props = [
        LinkProp(length=1.0, m=1.0, i=0.1, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi),
        LinkProp(length=0.8, m=0.8, i=0.05, rgx=0.4, rgy=0.0, qmin=-np.pi, qmax=np.pi),
    ]
    skeleton = Skeleton(link_props, base_length=0.5)

    assert skeleton.links[0].xe == pytest.approx(0.5)
    assert skeleton.links[1].x == pytest.approx(0.5)
    assert skeleton.links[1].xe == pytest.approx(1.5)
    assert skeleton.links[2].x == pytest.approx(1.5)
    assert skeleton.links[2].xe == pytest.approx(2.3)
    assert skeleton.links[2].ye == pytest.approx(0.0)


def test_setting_joint_state_refreshes_link_states() -> None:
    """Assigning q, dq, or ddq must leave the derived link states up to date without an explicit FK call."""
    link_prop = LinkProp(length=1.0, m=1.0, i=0.1, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi)
    skeleton = Skeleton(link_props=[link_prop])
    tip = skeleton.links[1]

    skeleton.q = np.array([np.pi / 2])
    assert tip.xe == pytest.approx(0.0, abs=1e-12)
    assert tip.ye == pytest.approx(1.0)

    skeleton.q = np.array([0.0])
    skeleton.dq = np.array([1.0])
    assert np.array([tip.vx, tip.vy]) == pytest.approx(np.array([0.0, 1.0]))

    # With w = 1 and dw = 2 the tip acceleration is dw x l + w^2 (-l) = (-1, 2).
    skeleton.ddq = np.array([2.0])
    assert np.array([tip.ax, tip.ay]) == pytest.approx(np.array([-1.0, 2.0]))


def test_link_does_not_mutate_input_dict() -> None:
    """Building a Link from a dict must not mutate the caller's dictionary."""
    properties = {"l": 1.0, "m": 2.0, "i": 0.5, "rgx": 0.5, "rgy": 0.0, "qmin": -1.0, "qmax": 1.0}

    link = Link(properties)

    # The Link picks up the renamed key internally...
    assert link.prop.length == 1.0
    # ...but the caller's dict is left untouched.
    assert properties == {"l": 1.0, "m": 2.0, "i": 0.5, "rgx": 0.5, "rgy": 0.0, "qmin": -1.0, "qmax": 1.0}
