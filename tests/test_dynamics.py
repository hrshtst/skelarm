"""Tests for the dynamics module."""

from __future__ import annotations

import numpy as np
import pytest

from skelarm.dynamics import (
    compute_coriolis_gravity_vector,
    compute_forward_dynamics,
    compute_inverse_dynamics,
    compute_mass_matrix,
    simulate_robot,
)
from skelarm.skeleton import LinkProp, Skeleton


def test_inverse_dynamics_single_link_static_no_gravity() -> None:
    """Test ID for a single link arm held static with no gravity.

    Expected torque: 0.
    """
    link_length = 1.0
    link_mass = 1.0
    link_com_x = 0.5

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

    compute_inverse_dynamics(skeleton)  # No grav_vec passed

    expected_tau = 0.0

    assert skeleton.links[0].tau == pytest.approx(expected_tau)


def test_compute_mass_matrix_single_link() -> None:
    """Test compute_mass_matrix for a single link.

    Expected: M = I + m*r^2.
    """
    link_length = 1.0
    link_mass = 1.0
    link_inertia = 0.1
    link_com_x = 0.5

    link_prop = LinkProp(
        length=link_length,
        m=link_mass,
        i=link_inertia,
        rgx=link_com_x,
        rgy=0.0,
        qmin=-np.pi,
        qmax=np.pi,
    )
    skeleton = Skeleton(link_props=[link_prop])

    # Set q and dq (shouldn't affect mass matrix if ID is implemented correctly)
    skeleton.q = np.array([0.0])
    skeleton.dq = np.array([0.0])

    mass_matrix = compute_mass_matrix(skeleton)

    # For a single link, the mass matrix is a 1x1 matrix.
    # Here, M = I + m*r^2 where r is the distance from the joint to COM
    expected_mass = link_inertia + link_mass * (link_com_x**2 + link_prop.rgy**2)

    assert mass_matrix.shape == (1, 1)
    assert mass_matrix[0, 0] == pytest.approx(expected_mass)


def test_compute_coriolis_gravity_vector_single_link_static_no_gravity() -> None:
    """Test compute_coriolis_gravity_vector for a single link arm static with no gravity.

    Expected h_vector: 0.
    """
    link_length = 1.0
    link_mass = 1.0
    link_com_x = 0.5

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
    skeleton.ddq = np.array([0.0])  # ddq should be zero for h vector

    # Calculate h vector
    h_vector = compute_coriolis_gravity_vector(skeleton)  # No grav_vec passed

    expected_h = 0.0

    assert h_vector.shape == (1,)
    assert h_vector[0] == pytest.approx(expected_h)


def test_compute_forward_dynamics_single_link_consistency() -> None:
    """Test compute_forward_dynamics for consistency with inverse dynamics.

    M * ddq + h = tau.
    """
    link_length = 1.0
    link_mass = 1.0
    link_inertia = 0.1
    link_com_x = 0.5

    link_prop = LinkProp(
        length=link_length,
        m=link_mass,
        i=link_inertia,
        rgx=link_com_x,
        rgy=0.0,
        qmin=-np.pi,
        qmax=np.pi,
    )
    skeleton = Skeleton(link_props=[link_prop])

    # Set arbitrary state
    skeleton.q = np.array([np.pi / 4])
    skeleton.dq = np.array([0.5])

    # Set arbitrary control torque
    tau_input = np.array([1.0])

    # Compute ddq using forward dynamics
    ddq_computed = compute_forward_dynamics(skeleton, tau_input)  # No grav_vec passed

    # Now verify with inverse dynamics: M * ddq + h should equal tau_input
    # To do this, set the computed ddq on a copy of the skeleton and run ID

    # Compute M(q)
    mass_matrix = compute_mass_matrix(skeleton)  # Implicit zero grav

    # Compute h(q, dq)
    coriolis_gravity_vector = compute_coriolis_gravity_vector(skeleton)  # No grav_vec passed

    # Calculate M * ddq + h
    tau_reconstructed = mass_matrix @ ddq_computed + coriolis_gravity_vector

    assert tau_reconstructed.shape == tau_input.shape
    assert tau_reconstructed[0] == pytest.approx(tau_input[0])


def test_simulate_robot_single_link_static_no_gravity() -> None:
    """Test simulate_robot for a single link arm starting static with no gravity.

    Robot should remain static.
    """
    link_length = 1.0
    link_mass = 1.0
    link_inertia = 0.1
    link_com_x = 0.5

    link_prop = LinkProp(
        length=link_length,
        m=link_mass,
        i=link_inertia,
        rgx=link_com_x,
        rgy=0.0,
        qmin=-np.pi,
        qmax=np.pi,
    )
    initial_skeleton = Skeleton(link_props=[link_prop])

    # Start from some position, no initial velocity
    initial_skeleton.q = np.array([np.pi / 4])
    initial_skeleton.dq = np.array([0.0])

    # No control torques
    def control_torques_func(_t: float, _skeleton: Skeleton) -> np.ndarray:
        return np.array([0.0])

    time_span = (0.0, 1.0)  # Simulate for a short period
    dt = 0.01

    _times, q_traj, dq_traj = simulate_robot(
        initial_skeleton,
        time_span,
        control_torques_func,
        dt=dt,
    )

    # Robot should remain static
    assert q_traj[-1, 0] == pytest.approx(initial_skeleton.q[0])
    assert dq_traj[-1, 0] == pytest.approx(initial_skeleton.dq[0])
