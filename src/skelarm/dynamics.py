"""Provides functions for robot arm dynamics."""

from __future__ import annotations

from collections.abc import Callable
from copy import deepcopy
from typing import TYPE_CHECKING

import numpy as np
from scipy.integrate import solve_ivp

from skelarm.kinematics import compute_forward_kinematics

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from skelarm.skeleton import Skeleton


def _cross_2d(a: NDArray[np.float64], b: NDArray[np.float64]) -> float:
    """Return the scalar z component of a 2D cross product."""
    return float(a[0] * b[1] - a[1] * b[0])


def _rotate_to_base(vector: NDArray[np.float64], angle: float) -> NDArray[np.float64]:
    """Rotate a 2D vector from the link frame into the base frame."""
    cos_angle = np.cos(angle)
    sin_angle = np.sin(angle)
    return np.array(
        [
            cos_angle * vector[0] - sin_angle * vector[1],
            sin_angle * vector[0] + cos_angle * vector[1],
        ],
        dtype=np.float64,
    )


def _clear_external_forces(skeleton: Skeleton) -> None:
    """Clear external loads on a temporary skeleton."""
    for link in skeleton.links:
        link.fex = 0.0
        link.fey = 0.0


def compute_inverse_dynamics(
    skeleton: Skeleton,
    grav_vec: NDArray[np.float64] | None = None,
) -> None:
    """Compute the inverse dynamics of the robot arm using the Recursive Newton-Euler algorithm.

    Updates the ``tau`` (joint torque) for each link in the skeleton.

    Parameters
    ----------
    skeleton : Skeleton
        The Skeleton object containing the robot arm's links and their states.
    grav_vec : NDArray[np.float64] | None, optional
        A 2D gravity vector. Defaults to zero (planar motion).
    """
    if grav_vec is None:
        grav_vec = np.array([0.0, 0.0], dtype=np.float64)

    compute_forward_kinematics(skeleton)

    # Backward Pass (endpoint down to the first movable joint; the base link at
    # index 0 is fixed and carries no actuation torque).
    for i in range(skeleton.num_links - 1, 0, -1):
        link = skeleton.links[i]

        # fi is the inertial force with optional gravity folded into the
        # effective COM acceleration.
        fi = link.prop.m * (link.dvc - grav_vec)
        ni = link.prop.i * link.dw

        # Forces/moments from the succeeding link
        if i == skeleton.num_links - 1:
            succ_f = np.array([0.0, 0.0], dtype=np.float64)
            succ_n = 0.0
        else:
            succ_link = skeleton.links[i + 1]
            succ_f = succ_link.f
            succ_n = succ_link.n

        # Vector from current joint to COM (in base frame). Use this link's own
        # absolute angle (stored during the forward pass), not the loop-final value.
        rc_curr_base_frame = _rotate_to_base(
            np.array([link.prop.rgx, link.prop.rgy], dtype=np.float64),
            link.q_absolute,
        )

        # Vector from current joint to the next joint (full link) in the base frame.
        # The succeeding link's force acts at the next joint, so this full-length
        # vector is its moment arm about the current joint.
        l_curr_base_frame = _rotate_to_base(
            np.array([link.prop.length, 0.0], dtype=np.float64),
            link.q_absolute,
        )
        ext_f = np.array([link.fex, link.fey], dtype=np.float64)
        ext_r = _rotate_to_base(np.array([link.rex, link.rey], dtype=np.float64), link.q_absolute)

        # Force balance: parent force supplies inertial and child loads, minus
        # force already supplied by the environment.
        link.f = fi + succ_f - ext_f

        # Moment balance: n_i = N_i + n_{i+1} + (r_{i, i+1} x f_{i+1}) + (r_{i, com} x F_i)
        # 2D cross product: x*fy - y*fx
        link.n = ni + succ_n + _cross_2d(l_curr_base_frame, succ_f) + _cross_2d(rc_curr_base_frame, fi)
        link.n -= _cross_2d(ext_r, ext_f)

        # Joint torque
        # link.n is torque ON the link.
        # link.tau is joint torque (reaction).
        link.tau = link.n


def compute_mass_matrix(
    skeleton: Skeleton,
    _grav_vec: NDArray[np.float64] | None = None,  # Renamed to _grav_vec as it's ignored
) -> NDArray[np.float64]:
    """Compute the mass matrix M(q) for the robot arm.

    Parameters
    ----------
    skeleton : Skeleton
        The Skeleton object.
    _grav_vec : NDArray[np.float64] | None, optional
        Ignored; the mass matrix is computed with zero gravity.

    Returns
    -------
    NDArray[np.float64]
        The N x N mass matrix.
    """
    # Mass matrix calculation requires zero gravity
    # We pass explicit zero vector to ensure no gravity influence
    zero_grav = np.array([0.0, 0.0], dtype=np.float64)

    num_joints = skeleton.num_joints
    mass_matrix = np.zeros((num_joints, num_joints), dtype=np.float64)

    # Write the link state directly rather than through the eager q/dq/ddq
    # setters: each setter assignment would re-run forward kinematics, and
    # inverse dynamics refreshes the kinematics itself anyway.
    temp_skeleton = deepcopy(skeleton)
    _clear_external_forces(temp_skeleton)
    for link in temp_skeleton.links[1:]:
        link.dq = 0.0

    for j in range(num_joints):
        for k, link in enumerate(temp_skeleton.links[1:]):
            link.ddq = 1.0 if k == j else 0.0

        compute_inverse_dynamics(temp_skeleton, grav_vec=zero_grav)
        mass_matrix[:, j] = temp_skeleton.tau

    return mass_matrix


def compute_coriolis_gravity_vector(
    skeleton: Skeleton,
    grav_vec: NDArray[np.float64] | None = None,
) -> NDArray[np.float64]:
    """Compute the bias vector h(q, dq).

    The returned vector includes Coriolis, optional gravity, and any external
    forces stored on the skeleton's links.

    Parameters
    ----------
    skeleton : Skeleton
        The Skeleton object.
    grav_vec : NDArray[np.float64] | None, optional
        The gravity vector. Defaults to zero (planar motion).

    Returns
    -------
    NDArray[np.float64]
        The N-dimensional bias vector h.
    """
    if grav_vec is None:
        grav_vec = np.array([0.0, 0.0], dtype=np.float64)

    # Raw link writes for the same reason as in compute_mass_matrix.
    temp_skeleton = deepcopy(skeleton)
    for link in temp_skeleton.links[1:]:
        link.ddq = 0.0

    compute_inverse_dynamics(temp_skeleton, grav_vec=grav_vec)
    return temp_skeleton.tau


def compute_forward_dynamics(
    skeleton: Skeleton,
    tau: NDArray[np.float64],
    grav_vec: NDArray[np.float64] | None = None,
) -> NDArray[np.float64]:
    """Compute joint accelerations ddq given torques.

    Parameters
    ----------
    skeleton : Skeleton
        The Skeleton object.
    tau : NDArray[np.float64]
        Joint torques.
    grav_vec : NDArray[np.float64] | None, optional
        The gravity vector. Defaults to zero (planar motion).

    Returns
    -------
    NDArray[np.float64]
        Joint accelerations ddq.
    """
    if grav_vec is None:
        grav_vec = np.array([0.0, 0.0], dtype=np.float64)

    temp_skeleton = deepcopy(skeleton)
    mass_matrix = compute_mass_matrix(temp_skeleton)
    coriolis_gravity_vector = compute_coriolis_gravity_vector(temp_skeleton, grav_vec=grav_vec)

    rhs = tau - coriolis_gravity_vector
    try:
        return np.linalg.solve(mass_matrix, rhs).astype(np.float64)
    except np.linalg.LinAlgError as exc:
        msg = (
            "Mass matrix is singular for the current configuration; forward dynamics has no unique "
            "solution. Check for links with zero mass and inertia or other degenerate properties."
        )
        raise ValueError(msg) from exc


def compute_kinetic_energy(skeleton: Skeleton) -> float:
    """Compute the total kinetic energy of the robot arm.

    Parameters
    ----------
    skeleton : Skeleton
        The Skeleton object with link velocities (w, v, vc) computed.

    Returns
    -------
    float
        The total kinetic energy.
    """
    total_ke = 0.0
    for link in skeleton.links:
        # Kinetic energy of a rigid body: 0.5 * m * vc^2 + 0.5 * I * w^2
        # vc is a 2D vector, so vc^2 = vc_x^2 + vc_y^2
        vc_squared = np.dot(link.vc, link.vc)
        ke_translational = 0.5 * link.prop.m * vc_squared
        ke_rotational = 0.5 * link.prop.i * (link.w**2)
        total_ke += ke_translational + ke_rotational
    return total_ke


def compute_kinetic_energy_rate(
    skeleton: Skeleton,
    tau: NDArray[np.float64],
    grav_vec: NDArray[np.float64] | None = None,
) -> float:
    """Compute the rate of change of kinetic energy (dKE/dt).

    dKE/dt = dq^T * tau_applied.
    In the context of the dynamics equation M*ddq + h = tau,
    dKE/dt should be dq^T * (M*ddq + h). This must equal dq^T * tau_applied.

    Parameters
    ----------
    skeleton : Skeleton
        The Skeleton object with current q and dq.
    tau : NDArray[np.float64]
        The N-dimensional vector of joint torques.
    grav_vec : NDArray[np.float64] | None, optional
        The gravity vector. Defaults to zero (planar motion).

    Returns
    -------
    float
        The rate of change of kinetic energy.
    """
    if grav_vec is None:
        grav_vec = np.array([0.0, 0.0], dtype=np.float64)

    # Need current ddq to check consistency
    ddq = compute_forward_dynamics(skeleton, tau, grav_vec)

    # Reconstruct tau from ddq, M, h
    temp_skeleton = deepcopy(skeleton)
    mass_matrix = compute_mass_matrix(temp_skeleton)
    coriolis_gravity_vector = compute_coriolis_gravity_vector(temp_skeleton, grav_vec=grav_vec)

    # The torque on the left side of the equation M*ddq + h = tau
    tau_lhs = mass_matrix @ ddq + coriolis_gravity_vector

    # dKE/dt = dq^T * tau
    # We check dq^T * tau_lhs, which should be equal to dq^T * tau (input)
    return float(np.dot(skeleton.dq, tau_lhs))


def simulate_robot(
    initial_skeleton: Skeleton,
    time_span: tuple[float, float],
    control_torques_func: Callable[[float, Skeleton], NDArray[np.float64]],
    grav_vec: NDArray[np.float64] | None = None,
    dt: float = 0.01,
    rtol: float = 1e-6,
    atol: float = 1e-8,
) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
    """Simulate robot dynamics.

    Parameters
    ----------
    initial_skeleton : Skeleton
        The initial Skeleton state (q, dq).
    time_span : tuple[float, float]
        A tuple (start_time, end_time) for the simulation.
    control_torques_func : Callable[[float, Skeleton], NDArray[np.float64]]
        A callable ``f(t, skeleton) -> tau`` returning the N-dimensional control
        torques for the current time and skeleton state.
    grav_vec : NDArray[np.float64] | None, optional
        The gravity vector. Defaults to zero (planar motion).
    dt : float, optional
        Time step for the simulation, used for output points.
    rtol : float, optional
        Relative tolerance for the ODE solver.
    atol : float, optional
        Absolute tolerance for the ODE solver.

    Returns
    -------
    tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]
        A tuple (times, q_trajectory, dq_trajectory) of NumPy arrays.
    """
    if grav_vec is None:
        grav_vec = np.array([0.0, 0.0], dtype=np.float64)

    num_joints = initial_skeleton.num_joints

    def ode_system(t: float, state: NDArray[np.float64]) -> NDArray[np.float64]:
        q = state[:num_joints]
        dq = state[num_joints:]

        # Raw link writes bypass the eager setters (which would run FK twice);
        # one explicit FK pass then gives the control callback link positions
        # and velocities consistent with the state it is handed.
        current_skeleton = deepcopy(initial_skeleton)
        for link, q_value, dq_value in zip(current_skeleton.links[1:], q, dq, strict=True):
            link.q = q_value
            link.dq = dq_value
        compute_forward_kinematics(current_skeleton)

        tau = control_torques_func(t, current_skeleton)
        ddq = compute_forward_dynamics(current_skeleton, tau, grav_vec)

        return np.concatenate((dq, ddq))

    initial_state = np.concatenate((initial_skeleton.q, initial_skeleton.dq))
    # Sample every dt and end exactly at the final time. A naive
    # ``arange(t0, t1 + dt, dt)`` can overshoot t1 through float rounding,
    # which solve_ivp rejects.
    t_eval = np.arange(time_span[0], time_span[1], dt)
    t_eval = np.append(t_eval[t_eval < time_span[1]], time_span[1])

    solution = solve_ivp(
        ode_system,
        time_span,
        initial_state,
        t_eval=t_eval,
        method="RK45",
        rtol=rtol,
        atol=atol,
    )

    if not solution.success:
        msg = f"ODE integration failed: {solution.message}"
        raise RuntimeError(msg)

    q_trajectory = solution.y[:num_joints, :].T
    dq_trajectory = solution.y[num_joints:, :].T

    return solution.t, q_trajectory, dq_trajectory
