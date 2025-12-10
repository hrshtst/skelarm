"""Tests for the dynamics module."""

from __future__ import annotations

import numpy as np
import pytest

from skelarm.dynamics import compute_inverse_dynamics
from skelarm.skeleton import LinkProp, Skeleton


def test_inverse_dynamics_single_link_static_horizontal_gravity() -> None:
    """
    Test ID for a single link arm held static horizontally with gravity.

    Expected torque: m * g * rgx.
    """
    link_length = 1.0
    link_mass = 1.0
    link_com_x = 0.5  # COM at half length
    gravity = np.array([0.0, -9.81])

    link_prop = LinkProp(
        length=link_length,
        m=link_mass,
        i=0.1,
        rgx=link_com_x,
        rgy=0.0,
        qmin=-np.pi,
        qmax=np.pi,
    )
    skeleton = Skeleton(link_props=[link_prop])

    # Set state: static, horizontal
    skeleton.q = np.array([0.0])
    skeleton.dq = np.array([0.0])
    skeleton.ddq = np.array([0.0])

    compute_inverse_dynamics(skeleton, grav_vec=gravity)

    expected_tau = link_prop.rgx * link_prop.m * (-gravity[1])  # Torque required to counteract gravity

    assert skeleton.links[0].tau == pytest.approx(expected_tau)


def test_inverse_dynamics_single_link_static_vertical_gravity() -> None:
    """
    Test ID for a single link arm held static vertically with gravity.

    Expected torque: 0 (if joint is at base).
    """
    link_length = 1.0
    link_mass = 1.0
    link_com_x = 0.5
    gravity = np.array([0.0, -9.81])

    link_prop = LinkProp(
        length=link_length,
        m=link_mass,
        i=0.01,
        rgx=link_com_x,
        rgy=0.0,
        qmin=-np.pi,
        qmax=np.pi,
    )
    skeleton = Skeleton(link_props=[link_prop])

    # Set state: static, vertical (pi/2)
    skeleton.q = np.array([np.pi / 2])
    skeleton.dq = np.array([0.0])
    skeleton.ddq = np.array([0.0])

    compute_inverse_dynamics(skeleton, grav_vec=gravity)

    # For a vertical arm (q=pi/2), the COM is directly above the joint.
    # The moment arm for gravity about the joint is zero.
    expected_tau = 0.0

    assert skeleton.links[0].tau == pytest.approx(expected_tau)


def test_inverse_dynamics_single_link_static_no_gravity() -> None:
    """
    Test ID for a single link arm held static with no gravity.

    Expected torque: 0.
    """
    link_length = 1.0
    link_mass = 1.0
    link_com_x = 0.5
    gravity = np.array([0.0, 0.0])  # No gravity

    link_prop = LinkProp(
        length=link_length,
        m=link_mass,
        i=0.01,
        rgx=link_com_x,
        rgy=0.0,
        qmin=-np.pi,
        qmax=np.pi,
    )
    skeleton = Skeleton(link_props=[link_prop])

    # Set state: static, horizontal (or any angle)
    skeleton.q = np.array([0.0])
    skeleton.dq = np.array([0.0])
    skeleton.ddq = np.array([0.0])

    compute_inverse_dynamics(skeleton, grav_vec=gravity)

    expected_tau = 0.0

    assert skeleton.links[0].tau == pytest.approx(expected_tau)
