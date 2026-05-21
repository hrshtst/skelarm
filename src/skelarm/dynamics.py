"""Provides functions for robot arm dynamics."""

from __future__ import annotations

from collections.abc import Callable
from copy import deepcopy
from typing import TYPE_CHECKING

import numpy as np
from scipy.integrate import solve_ivp

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from skelarm.skeleton import Skeleton


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

    # Initialize base angular and linear velocities/accelerations
    # Assuming base is fixed at (0,0) with no rotation
    prev_w = 0.0  # Angular velocity of the previous link frame
    prev_dw = 0.0  # Angular acceleration of the previous link frame
    prev_dv = -grav_vec  # Linear acceleration of the previous link origin (base)
    prev_link_vec = np.array([0.0, 0.0], dtype=np.float64)  # Previous link's joint-to-tip vector (base frame)

    # Forward Pass (Base to End-effector)
    for i, link in enumerate(skeleton.links):
        # Rotation matrix from previous frame to current frame (for 2D, purely angular)
        if i == 0:
            link_absolute_angle = link.q
        else:
            prev_link = skeleton.links[i - 1]
            link_absolute_angle = prev_link.q_absolute + link.q

        link.q_absolute = link_absolute_angle

        # Angular velocity and acceleration (scalar sum for 2D)
        link.w = prev_w + link.dq
        link.dw = prev_dw + link.ddq

        # Vector spanning the current link (current joint to its tip), in the base frame.
        link_vec = np.array(
            [
                link.prop.length * np.cos(link_absolute_angle),
                link.prop.length * np.sin(link_absolute_angle),
            ]
        )

        # The current joint sits at the previous link's tip, so acceleration is
        # propagated from the previous joint across the previous link's vector.
        r_prev_to_curr = prev_link_vec

        # Vector from current joint to COM (relative to link frame)
        rc_curr = np.array([link.prop.rgx, link.prop.rgy])

        # Rotate rc_curr to base frame
        r_curr_to_base = np.array(
            [
                [np.cos(link_absolute_angle), -np.sin(link_absolute_angle)],
                [np.sin(link_absolute_angle), np.cos(link_absolute_angle)],
            ]
        )
        rc_curr_base_frame = r_curr_to_base @ rc_curr

        # Linear acceleration of link origin (joint)
        # 2D cross product terms:
        # dw x r = [-dw * ry, dw * rx]
        # w x (w x r) = [-w^2 * rx, -w^2 * ry]

        cross_dw_r_prev = np.array([-prev_dw * r_prev_to_curr[1], prev_dw * r_prev_to_curr[0]])
        term3 = np.array([-(prev_w**2) * r_prev_to_curr[0], -(prev_w**2) * r_prev_to_curr[1]])

        link.dv = prev_dv + cross_dw_r_prev + term3

        # Linear acceleration of center of mass (COM)
        # dvc_i = dv_i + dw_i x rc_i + w_i x (w_i x rc_i)
        cross_dw_rc = np.array([-link.dw * rc_curr_base_frame[1], link.dw * rc_curr_base_frame[0]])
        term_com3 = np.array([-(link.w**2) * rc_curr_base_frame[0], -(link.w**2) * rc_curr_base_frame[1]])

        link.dvc = link.dv + cross_dw_rc + term_com3

        # Update for next iteration
        prev_w = link.w
        prev_dw = link.dw
        prev_dv = link.dv
        prev_link_vec = link_vec

    # Backward Pass (End-effector to Base)
    for i in range(skeleton.num_links - 1, -1, -1):
        link = skeleton.links[i]

        # fi is the inertial force. Gravity is handled via initial prev_dv.
        fi = link.prop.m * link.dvc
        ni = link.prop.i * link.dw

        # Forces/moments from the succeeding link
        if i == skeleton.num_links - 1:
            succ_f = np.array([link.fex, link.fey])
            succ_n = 0.0
        else:
            succ_link = skeleton.links[i + 1]
            succ_f = succ_link.f
            succ_n = succ_link.n

        # Vector from current joint to COM (in base frame). Use this link's own
        # absolute angle (stored during the forward pass), not the loop-final value.
        link_absolute_angle = link.q_absolute
        rc_curr = np.array([link.prop.rgx, link.prop.rgy])
        r_curr_to_base = np.array(
            [
                [np.cos(link_absolute_angle), -np.sin(link_absolute_angle)],
                [np.sin(link_absolute_angle), np.cos(link_absolute_angle)],
            ]
        )
        rc_curr_base_frame = r_curr_to_base @ rc_curr

        # Vector from current joint to the next joint (full link) in the base frame.
        # The succeeding link's force acts at the next joint, so this full-length
        # vector is its moment arm about the current joint.
        l_curr_base_frame = np.array(
            [
                link.prop.length * np.cos(link_absolute_angle),
                link.prop.length * np.sin(link_absolute_angle),
            ]
        )

        # Force balance: f_i = F_i + f_{i+1}
        link.f = fi + succ_f

        # Moment balance: n_i = N_i + n_{i+1} + (r_{i, i+1} x f_{i+1}) + (r_{i, com} x F_i)
        # 2D cross product: x*fy - y*fx
        cross_l_succ_f = l_curr_base_frame[0] * succ_f[1] - l_curr_base_frame[1] * succ_f[0]
        cross_rc_fi = rc_curr_base_frame[0] * fi[1] - rc_curr_base_frame[1] * fi[0]

        link.n = ni + succ_n + cross_l_succ_f + cross_rc_fi

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

    num_links = skeleton.num_links
    mass_matrix = np.zeros((num_links, num_links), dtype=np.float64)

    original_q = skeleton.q
    original_dq = skeleton.dq
    original_ddq = skeleton.ddq

    temp_skeleton = deepcopy(skeleton)
    temp_skeleton.dq = np.zeros(num_links)

    for j in range(num_links):
        ddq_j_one = np.zeros(num_links)
        ddq_j_one[j] = 1.0
        temp_skeleton.ddq = ddq_j_one

        compute_inverse_dynamics(temp_skeleton, grav_vec=zero_grav)
        mass_matrix[:, j] = temp_skeleton.tau

    skeleton.q = original_q
    skeleton.dq = original_dq
    skeleton.ddq = original_ddq

    return mass_matrix


def compute_coriolis_gravity_vector(
    skeleton: Skeleton,
    grav_vec: NDArray[np.float64] | None = None,
) -> NDArray[np.float64]:
    """Compute the Coriolis and gravity vector h(q, dq).

    Parameters
    ----------
    skeleton : Skeleton
        The Skeleton object.
    grav_vec : NDArray[np.float64] | None, optional
        The gravity vector. Defaults to zero (planar motion).

    Returns
    -------
    NDArray[np.float64]
        The N-dimensional vector h.
    """
    if grav_vec is None:
        grav_vec = np.array([0.0, 0.0], dtype=np.float64)

    num_links = skeleton.num_links
    original_q = skeleton.q
    original_dq = skeleton.dq
    original_ddq = skeleton.ddq

    temp_skeleton = deepcopy(skeleton)
    temp_skeleton.ddq = np.zeros(num_links)

    compute_inverse_dynamics(temp_skeleton, grav_vec=grav_vec)
    h_vector = temp_skeleton.tau

    # Restore original state
    skeleton.q = original_q
    skeleton.dq = original_dq
    skeleton.ddq = original_ddq

    return h_vector


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

    num_links = initial_skeleton.num_links

    def ode_system(t: float, state: NDArray[np.float64]) -> NDArray[np.float64]:
        q = state[:num_links]
        dq = state[num_links:]

        current_skeleton = deepcopy(initial_skeleton)
        current_skeleton.q = q
        current_skeleton.dq = dq
        # ddq is computed, not set from state

        tau = control_torques_func(t, current_skeleton)
        ddq = compute_forward_dynamics(current_skeleton, tau, grav_vec)

        return np.concatenate((dq, ddq))

    initial_state = np.concatenate((initial_skeleton.q, initial_skeleton.dq))
    t_eval = np.arange(time_span[0], time_span[1] + dt, dt)

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

    q_trajectory = solution.y[:num_links, :].T
    dq_trajectory = solution.y[num_links:, :].T

    return solution.t, q_trajectory, dq_trajectory
