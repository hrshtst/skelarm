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

    Updates the `tau` (joint torque) for each link in the skeleton.

    :param skeleton: The Skeleton object containing the robot arm's links and their states.
    :param grav_vec: A 2D NumPy array representing the gravity vector. Defaults to zero (planar motion).
    """
    if grav_vec is None:
        grav_vec = np.array([0.0, 0.0], dtype=np.float64)

    # Initialize base angular and linear velocities/accelerations
    # Assuming base is fixed at (0,0) with no rotation
    prev_w = 0.0  # Angular velocity of the previous link frame
    prev_dw = 0.0  # Angular acceleration of the previous link frame
    prev_dv = -grav_vec  # Linear acceleration of the previous link origin (base)

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

        # Vector from previous joint to current joint (in base frame)
        r_prev_to_curr = np.array(
            [
                link.prop.length * np.cos(link_absolute_angle),
                link.prop.length * np.sin(link_absolute_angle),
            ],
        )

        # Vector from current joint to COM (relative to link frame)
        rc_curr = np.array([link.prop.rgx, link.prop.rgy])

        # Rotate rc_curr to base frame
        r_curr_to_base = np.array(
            [
                [np.cos(link_absolute_angle), -np.sin(link_absolute_angle)],
                [np.sin(link_absolute_angle), np.cos(link_absolute_angle)],
            ],
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

        # Vector from current joint to COM (in base frame)
        # Reuse calculation logic
        rc_curr = np.array([link.prop.rgx, link.prop.rgy])
        r_curr_to_base = np.array(
            [
                [np.cos(link.q_absolute), -np.sin(link.q_absolute)],
                [np.sin(link.q_absolute), np.cos(link.q_absolute)],
            ],
        )
        rc_curr_base_frame = r_curr_to_base @ rc_curr

        # Vector from COM to tip (succeeding joint) in base frame
        l_curr_base_frame = np.array(
            [
                link.prop.length * np.cos(link.q_absolute),
                link.prop.length * np.sin(link.q_absolute),
            ],
        )
        lc_curr_base_frame = l_curr_base_frame - rc_curr_base_frame

        # Force balance: f_i = F_i + f_{i+1}
        link.f = fi + succ_f

        # Moment balance: n_i = N_i + n_{i+1} + (r_{i, i+1} x f_{i+1}) + (r_{i, com} x F_i)
        # 2D cross product: x*fy - y*fx
        cross_lc_succ_f = lc_curr_base_frame[0] * succ_f[1] - lc_curr_base_frame[1] * succ_f[0]
        cross_rc_fi = rc_curr_base_frame[0] * fi[1] - rc_curr_base_frame[1] * fi[0]

        link.n = ni + succ_n + cross_lc_succ_f + cross_rc_fi

        # Joint torque
        # link.n is torque ON the link.
        # link.tau is joint torque (reaction).
        link.tau = link.n


def compute_mass_matrix(
    skeleton: Skeleton,
    grav_vec: NDArray[np.float64] | None = None,
) -> NDArray[np.float64]:
    """Compute the mass matrix M(q) for the robot arm.

    :param skeleton: The Skeleton object.
    :param grav_vec: Ignored, should be zero for mass matrix.
    :return: The N x N mass matrix.
    """
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

        compute_inverse_dynamics(temp_skeleton, grav_vec=grav_vec)
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

    :param skeleton: The Skeleton object.
    :param grav_vec: The gravity vector.
    :return: The N-dimensional vector h.
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

    :param skeleton: The Skeleton object.
    :param tau: Joint torques.
    :param grav_vec: Gravity vector.
    :return: Joint accelerations ddq.
    """
    if grav_vec is None:
        grav_vec = np.array([0.0, 0.0], dtype=np.float64)

    temp_skeleton = deepcopy(skeleton)
    mass_matrix = compute_mass_matrix(temp_skeleton)
    coriolis_gravity_vector = compute_coriolis_gravity_vector(temp_skeleton, grav_vec=grav_vec)

    rhs = tau - coriolis_gravity_vector
    return np.linalg.solve(mass_matrix, rhs)


def simulate_robot(
    initial_skeleton: Skeleton,
    time_span: tuple[float, float],
    control_torques_func: Callable[[float, Skeleton], NDArray[np.float64]],
    grav_vec: NDArray[np.float64] | None = None,
    dt: float = 0.01,
) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
    """Simulate robot dynamics.

    :param initial_skeleton: The initial Skeleton state (q, dq).
    :param time_span: A tuple (start_time, end_time) for the simulation.
    :param control_torques_func: A callable function `f(t, skeleton) -> tau` that returns
                                 the N-dimensional control torques for the current time and skeleton state.
    :param grav_vec: The gravity vector.
    :param dt: Time step for the simulation, used for output points.
    :return: A tuple (times, q_trajectory, dq_trajectory) of NumPy arrays.
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
        rtol=1e-6,
        atol=1e-8,
    )

    if not solution.success:
        msg = f"ODE integration failed: {solution.message}"
        raise RuntimeError(msg)

    q_trajectory = solution.y[:num_links, :].T
    dq_trajectory = solution.y[num_links:, :].T

    return solution.t, q_trajectory, dq_trajectory
