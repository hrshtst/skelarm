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


def test_set_state_sets_all_joint_state_with_single_kinematics_pass(monkeypatch: pytest.MonkeyPatch) -> None:
    """set_state writes q, dq, and ddq together and refreshes the derived states with one FK pass."""
    import skelarm.skeleton as skeleton_module

    link_prop = LinkProp(length=1.0, m=1.0, i=0.1, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi)
    skeleton = Skeleton(link_props=[link_prop])
    tip = skeleton.links[1]

    fk_calls: list[Skeleton] = []
    real_fk = skeleton_module.compute_forward_kinematics

    def counting_fk(skel: Skeleton) -> None:
        fk_calls.append(skel)
        real_fk(skel)

    monkeypatch.setattr(skeleton_module, "compute_forward_kinematics", counting_fk)

    skeleton.set_state(q=np.array([0.0]), dq=np.array([1.0]), ddq=np.array([2.0]))

    assert len(fk_calls) == 1
    assert skeleton.q == pytest.approx(np.array([0.0]))
    assert skeleton.dq == pytest.approx(np.array([1.0]))
    assert skeleton.ddq == pytest.approx(np.array([2.0]))
    assert np.array([tip.vx, tip.vy]) == pytest.approx(np.array([0.0, 1.0]))
    # With w = 1 and dw = 2 the tip acceleration is dw x l + w^2 (-l) = (-1, 2).
    assert np.array([tip.ax, tip.ay]) == pytest.approx(np.array([-1.0, 2.0]))


def test_set_state_partial_update_keeps_other_state() -> None:
    """Arguments left as None keep their current values; derived states still refresh."""
    link_prop = LinkProp(length=1.0, m=1.0, i=0.1, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi)
    skeleton = Skeleton(link_props=[link_prop])
    tip = skeleton.links[1]

    skeleton.set_state(q=np.array([np.pi / 2]))
    skeleton.set_state(dq=np.array([1.0]))

    # q kept from the first call; the link (now along +y) spins with w = 1.
    assert skeleton.q == pytest.approx(np.array([np.pi / 2]))
    assert np.array([tip.vx, tip.vy]) == pytest.approx(np.array([-1.0, 0.0]))


def test_set_state_invalid_length_leaves_skeleton_unchanged() -> None:
    """A length mismatch in any argument raises before anything is written."""
    link_prop = LinkProp(length=1.0, m=1.0, i=0.1, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi)
    skeleton = Skeleton(link_props=[link_prop])
    skeleton.q = np.array([0.4])

    with pytest.raises(ValueError, match="dq"):
        skeleton.set_state(q=np.array([0.1]), dq=np.array([1.0, 2.0]))

    # The valid q argument must not have been applied either (validate first, then write).
    assert skeleton.q == pytest.approx(np.array([0.4]))
    assert skeleton.links[1].xe == pytest.approx(np.cos(0.4))


def test_link_does_not_mutate_input_dict() -> None:
    """Building a Link from a dict must not mutate the caller's dictionary."""
    properties = {"l": 1.0, "m": 2.0, "i": 0.5, "rgx": 0.5, "rgy": 0.0, "qmin": -1.0, "qmax": 1.0}

    link = Link(properties)

    # The Link picks up the renamed key internally...
    assert link.prop.length == 1.0
    # ...but the caller's dict is left untouched.
    assert properties == {"l": 1.0, "m": 2.0, "i": 0.5, "rgx": 0.5, "rgy": 0.0, "qmin": -1.0, "qmax": 1.0}
