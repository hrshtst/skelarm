"""Tests for the dynamics module."""

from __future__ import annotations

import numpy as np
import pytest
from hypothesis import given, settings
from hypothesis import strategies as st
from hypothesis.extra.numpy import arrays

from skelarm.dynamics import (
    compute_coriolis_gravity_vector,
    compute_forward_dynamics,
    compute_inverse_dynamics,
    compute_kinetic_energy_rate,
    compute_mass_matrix,
    simulate_robot,
)
from skelarm.kinematics import compute_forward_kinematics
from skelarm.skeleton import LinkProp, Skeleton


# Define strategies for generating LinkProp and Skeleton data
@st.composite
def link_props_strategy(draw: st.DrawFn, min_links: int = 1, max_links: int = 3) -> list[LinkProp]:
    """Strategy to generate a list of LinkProp objects."""
    num_links = draw(st.integers(min_value=min_links, max_value=max_links))
    link_props = []
    for _ in range(num_links):
        length = draw(st.floats(min_value=0.5, max_value=1.5, allow_nan=False, allow_infinity=False))
        mass = draw(st.floats(min_value=0.5, max_value=3.0, allow_nan=False, allow_infinity=False))
        inertia = draw(st.floats(min_value=0.05, max_value=0.5, allow_nan=False, allow_infinity=False))
        # Ensure COM is within the link's bounds and not too close to the joint center to avoid issues
        rgx = draw(st.floats(min_value=-length * 0.4, max_value=length * 0.4, allow_nan=False, allow_infinity=False))
        rgy = draw(st.floats(min_value=-0.05, max_value=0.05, allow_nan=False, allow_infinity=False))
        qmin = draw(st.floats(min_value=-np.pi / 2, max_value=np.pi / 2 - 0.1, allow_nan=False, allow_infinity=False))
        qmax = draw(st.floats(min_value=qmin + 0.1, max_value=np.pi / 2, allow_nan=False, allow_infinity=False))
        link_props.append(LinkProp(length=length, m=mass, i=inertia, rgx=rgx, rgy=rgy, qmin=qmin, qmax=qmax))
    return link_props


@st.composite
def skeleton_strategy(draw: st.DrawFn) -> Skeleton:
    """Strategy to generate a Skeleton object with random state."""
    link_props = draw(link_props_strategy())
    skeleton = Skeleton(link_props)
    num_links = len(link_props)

    # Generate random q, dq, ddq within reasonable ranges
    q = draw(arrays(np.float64, num_links, elements=st.floats(min_value=-np.pi / 2, max_value=np.pi / 2)))
    dq = draw(arrays(np.float64, num_links, elements=st.floats(min_value=-1.0, max_value=1.0)))
    ddq = draw(arrays(np.float64, num_links, elements=st.floats(min_value=-5.0, max_value=5.0)))

    skeleton.q = q
    skeleton.dq = dq
    skeleton.ddq = ddq
    return skeleton


# === Unit Tests for Basic Dynamics Functions (as before) ===


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


def test_forward_dynamics_singular_mass_matrix_raises_informative_error() -> None:
    """A degenerate (massless, inertia-less) link yields a singular M and a clear error."""
    degenerate_link = LinkProp(length=1.0, m=0.0, i=0.0, rgx=0.0, rgy=0.0, qmin=-np.pi, qmax=np.pi)
    skeleton = Skeleton(link_props=[degenerate_link])
    skeleton.q = np.array([0.0])
    skeleton.dq = np.array([0.0])

    with pytest.raises(ValueError, match="singular"):
        compute_forward_dynamics(skeleton, tau=np.array([0.0]))


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


def test_inverse_dynamics_single_link_external_tip_force_requires_opposing_torque() -> None:
    """An upward tip force on a horizontal link reduces the required actuator torque."""
    link_prop = LinkProp(length=1.0, m=1.0, i=0.1, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi)
    skeleton = Skeleton(link_props=[link_prop])
    skeleton.q = np.array([0.0])
    skeleton.dq = np.array([0.0])
    skeleton.ddq = np.array([0.0])
    skeleton.links[0].fey = 2.0
    skeleton.links[0].rex = 1.0

    compute_inverse_dynamics(skeleton)

    assert skeleton.tau == pytest.approx(np.array([-2.0]))


def test_inverse_dynamics_external_force_on_nonterminal_link_affects_upstream_torques() -> None:
    """External forces should apply to every link at rex/rey, not only the tip link."""
    link_props = [
        LinkProp(length=1.0, m=1.0, i=0.1, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi),
        LinkProp(length=1.0, m=1.0, i=0.1, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi),
    ]
    skeleton = Skeleton(link_props)
    skeleton.q = np.array([0.0, 0.0])
    skeleton.dq = np.array([0.0, 0.0])
    skeleton.ddq = np.array([0.0, 0.0])
    skeleton.links[0].fey = 2.0
    skeleton.links[0].rex = 0.5

    compute_inverse_dynamics(skeleton)

    assert skeleton.tau == pytest.approx(np.array([-1.0, 0.0]))


def test_forward_dynamics_round_trip_with_external_force() -> None:
    """FD should include external forces stored on links and remain consistent with ID."""
    link_prop = LinkProp(length=1.0, m=1.0, i=0.1, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi)
    skeleton = Skeleton(link_props=[link_prop])
    skeleton.q = np.array([0.0])
    skeleton.dq = np.array([0.0])
    skeleton.links[0].fey = 2.0
    skeleton.links[0].rex = 1.0

    ddq = compute_forward_dynamics(skeleton, tau=np.array([0.0]))
    skeleton.ddq = ddq
    compute_inverse_dynamics(skeleton)

    assert ddq == pytest.approx(np.array([2.0 / 0.35]))
    assert skeleton.tau == pytest.approx(np.array([0.0]))


def test_mass_matrix_ignores_external_forces() -> None:
    """Mass matrix construction should be independent of current external loads."""
    link_prop = LinkProp(length=1.0, m=1.0, i=0.1, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi)
    unloaded = Skeleton(link_props=[link_prop])
    loaded = Skeleton(link_props=[link_prop])
    loaded.links[0].fey = 2.0
    loaded.links[0].rex = 1.0

    assert compute_mass_matrix(loaded) == pytest.approx(compute_mass_matrix(unloaded))


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


# === Hypothesis Tests ===


@pytest.mark.slow
@given(skel=skeleton_strategy(), tau_elements=st.floats(min_value=-10.0, max_value=10.0))
@settings(deadline=10000)  # Increase deadline to 10 seconds
def test_dynamics_consistency_hypothesis(skel: Skeleton, tau_elements: float) -> None:
    """Property-based test for dynamics consistency: ID(FD(tau)) approx tau."""
    num_links = skel.num_links

    # Generate random control torques
    tau_input = np.full(num_links, tau_elements, dtype=np.float64)

    # Compute ddq using forward dynamics
    ddq_computed = compute_forward_dynamics(skel, tau_input)

    # Set the computed ddq on the skeleton
    skel.ddq = ddq_computed

    # Compute tau_reconstructed using inverse dynamics
    compute_inverse_dynamics(skel)
    tau_reconstructed = skel.tau

    # Assert consistency
    for i in range(num_links):
        assert tau_reconstructed[i] == pytest.approx(tau_input[i], abs=1e-6)


@pytest.mark.slow
@given(skel=skeleton_strategy())
@settings(deadline=10000)  # Increase deadline to 10 seconds
def test_local_energy_conservation_hypothesis(skel: Skeleton) -> None:
    """Property-based test for local energy conservation: dKE/dt should be 0 when tau_input = 0."""
    num_links = skel.num_links

    # Ensure current q and dq are set on the skeleton
    compute_forward_kinematics(skel)  # Computes w, v, vc

    # No external torques for energy conservation check
    tau_input = np.full(num_links, 0.0, dtype=np.float64)

    # Calculate the rate of change of kinetic energy
    dke_dt = compute_kinetic_energy_rate(skel, tau_input)

    # dKE/dt should be zero if there are no external torques and no potential energy change
    assert dke_dt == pytest.approx(0.0, abs=1e-6)


@pytest.mark.slow
@given(skel=skeleton_strategy())
@settings(deadline=10000)
def test_mass_matrix_symmetric_positive_definite_hypothesis(skel: Skeleton) -> None:
    """The mass matrix M(q) must always be symmetric and positive-definite."""
    mass_matrix = compute_mass_matrix(skel)

    # Symmetry: M == M^T.
    assert mass_matrix == pytest.approx(mass_matrix.T, abs=1e-9)

    # Positive-definiteness: all eigenvalues strictly positive. M is symmetric, so use eigvalsh.
    eigenvalues = np.linalg.eigvalsh(mass_matrix)
    assert np.all(eigenvalues > 0)


def test_two_link_dynamics_round_trip_with_gravity() -> None:
    """ID/FD round-trip with two links and non-zero gravity: FD(tau) then ID should recover tau."""
    link_props = [
        LinkProp(length=1.0, m=1.5, i=0.1, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi),
        LinkProp(length=0.8, m=1.0, i=0.05, rgx=0.4, rgy=0.0, qmin=-np.pi, qmax=np.pi),
    ]
    skeleton = Skeleton(link_props)
    skeleton.q = np.array([np.pi / 6, -np.pi / 4])
    skeleton.dq = np.array([0.3, -0.2])

    grav_vec = np.array([0.0, -9.81], dtype=np.float64)
    tau_input = np.array([0.7, -0.3])

    ddq = compute_forward_dynamics(skeleton, tau_input, grav_vec=grav_vec)

    skeleton.ddq = ddq
    compute_inverse_dynamics(skeleton, grav_vec=grav_vec)

    assert skeleton.tau == pytest.approx(tau_input, abs=1e-6)
