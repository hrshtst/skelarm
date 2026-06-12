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


def test_link_does_not_mutate_input_dict() -> None:
    """Building a Link from a dict must not mutate the caller's dictionary."""
    properties = {"l": 1.0, "m": 2.0, "i": 0.5, "rgx": 0.5, "rgy": 0.0, "qmin": -1.0, "qmax": 1.0}

    link = Link(properties)

    # The Link picks up the renamed key internally...
    assert link.prop.length == 1.0
    # ...but the caller's dict is left untouched.
    assert properties == {"l": 1.0, "m": 2.0, "i": 0.5, "rgx": 0.5, "rgy": 0.0, "qmin": -1.0, "qmax": 1.0}
