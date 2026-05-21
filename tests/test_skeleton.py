"""Tests for the Link and Skeleton data structures."""

from __future__ import annotations

from skelarm.skeleton import Link


def test_link_does_not_mutate_input_dict() -> None:
    """Building a Link from a dict must not mutate the caller's dictionary."""
    properties = {"l": 1.0, "m": 2.0, "i": 0.5, "rgx": 0.5, "rgy": 0.0, "qmin": -1.0, "qmax": 1.0}

    link = Link(properties)

    # The Link picks up the renamed key internally...
    assert link.prop.length == 1.0
    # ...but the caller's dict is left untouched.
    assert properties == {"l": 1.0, "m": 2.0, "i": 0.5, "rgx": 0.5, "rgy": 0.0, "qmin": -1.0, "qmax": 1.0}
