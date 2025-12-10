"""Provides functions for robot arm dynamics."""

from __future__ import annotations

from typing import TYPE_CHECKING, cast

import numpy as np

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from skelarm.skeleton import Skeleton


def compute_inverse_dynamics(
    skeleton: Skeleton,
    grav_vec: NDArray[np.float64] = None,  # type: ignore[assignment]
) -> None:
    """
    Compute the inverse dynamics of the robot arm using the Recursive Newton-Euler algorithm.

    Updates the `tau` (joint torque) for each link in the skeleton.

    :param skeleton: The Skeleton object containing the robot arm's links and their states.
    :param grav_vec: A 2D NumPy array representing the gravity vector (e.g., [0.0, -9.81]).
    """
    if grav_vec is None:
        grav_vec = cast("NDArray[np.float64]", np.array([0.0, -9.81]))

    # Initialize base angular and linear velocities/accelerations
    # Assuming base is fixed at (0,0) with no rotation
    prev_w = 0.0  # Angular velocity of the previous link frame
    prev_dw = 0.0  # Angular acceleration of the previous link frame
    prev_dv = -grav_vec  # Linear acceleration of the previous link origin (base)

    # Forward Pass (Base to End-effector)
    for i, link in enumerate(skeleton.links):
        # Rotation matrix from previous frame to current frame
        # For 2D, this is just the total angle of the previous link
        # The angle of current link is relative to previous link
        if i == 0:
            # First link's orientation is absolute joint angle
            link_absolute_angle = link.q
        else:
            prev_link = skeleton.links[i - 1]
            # Absolute angle of current link is absolute angle of previous link + current relative joint angle
            link_absolute_angle = prev_link.q_absolute + link.q

        link.q_absolute = link_absolute_angle  # Store absolute angle in Link

        # Angular velocity and acceleration
        link.w = prev_w + link.dq  # For 2D planar, angular velocity is scalar sum
        link.dw = prev_dw + link.ddq  # For 2D planar, angular acceleration is scalar sum

        # Vector from previous joint to current joint (l_i-1 in i-1 frame)
        # In current coordinate system, length vector is [L, 0] rotated by current_angle
        r_prev_to_curr = np.array(
            [
                link.prop.length * np.cos(link_absolute_angle),
                link.prop.length * np.sin(link_absolute_angle),
            ],
        )

        # Vector from current joint to COM (rg_i in i frame)
        rc_curr = np.array([link.prop.rgx, link.prop.rgy])  # relative to current link frame

        # Rotate rc_curr to base frame
        # Only rotate by link's absolute angle, as rgx/rgy are relative to the link's own frame
        r_curr_to_base = np.array(
            [
                [np.cos(link_absolute_angle), -np.sin(link_absolute_angle)],
                [np.sin(link_absolute_angle), np.cos(link_absolute_angle)],
            ],
        )
        rc_curr_base_frame = r_curr_to_base @ rc_curr

        # Linear acceleration of link origin (joint)
        # For 2D, cross product with scalar z-component: cross(omega, [x,y]) = [-omega*y, omega*x]
        # prev_w and prev_dw are scalars (z-component of angular velocity/acceleration vectors)

        # Calculate terms for linear acceleration of joint
        term1 = prev_dv

        cross_dw_r_prev = np.array([-prev_dw * r_prev_to_curr[1], prev_dw * r_prev_to_curr[0]])

        cross_w_r_prev = np.array([-prev_w * r_prev_to_curr[1], prev_w * r_prev_to_curr[0]])
        term3 = np.array([-prev_w * cross_w_r_prev[1], prev_w * cross_w_r_prev[0]])  # w x (w x r)

        link.dv = term1 + cross_dw_r_prev + term3

        # Linear acceleration of center of mass (COM)
        cross_dw_rc = np.array([-link.dw * rc_curr_base_frame[1], link.dw * rc_curr_base_frame[0]])
        cross_w_rc = np.array([-link.w * rc_curr_base_frame[1], link.w * rc_curr_base_frame[0]])
        term_com3 = np.array([-link.w * cross_w_rc[1], link.w * cross_w_rc[0]])

        link.dvc = link.dv + cross_dw_rc + term_com3

        # Update for next iteration
        prev_w = link.w
        prev_dw = link.dw
        prev_dv = link.dv

    # Backward Pass (End-effector to Base)
    # Initialize force and moment at the end-effector (assuming no external forces at tip)
    # The last link's f and n are the external forces/moments at the tip.
    # We assume they are 0 for now.

    # Iterate backwards from the last link to the first
    for i in range(skeleton.num_links - 1, -1, -1):
        link = skeleton.links[i]

        # Calculate force and moment on COM (fi, ni)
        fi = link.prop.m * link.dvc
        ni = link.prop.i * link.dw

        # Forces and moments from the succeeding link
        if i == skeleton.num_links - 1:
            # If last link, no succeeding link, so external forces are considered
            # For now, assume no external forces/moments at the tip.
            succ_f = np.array([link.fex, link.fey])
            succ_n = 0.0  # External moment not explicitly defined in Link, assuming 0
        else:
            succ_link = skeleton.links[i + 1]
            succ_f = succ_link.f  # Force exerted by current link on succ_link (f_i+1)
            succ_n = succ_link.n  # Moment exerted by current link on succ_link (n_i+1)

        # Vector from current joint to tip of current link (l_i) in base frame
        curr_l_base_frame = np.array(
            [
                link.prop.length * np.cos(link.q_absolute),
                link.prop.length * np.sin(link.q_absolute),
            ],
        )

        # Vector from current joint to COM (rc_curr_base_frame)
        rc_curr = np.array([link.prop.rgx, link.prop.rgy])  # relative to current link frame
        r_curr_to_base = np.array(
            [
                [np.cos(link.q_absolute), -np.sin(link.q_absolute)],
                [np.sin(link.q_absolute), np.cos(link.q_absolute)],
            ],
        )
        rc_curr_base_frame = r_curr_to_base @ rc_curr

        # Vector from COM to tip of current link (lc_i) in base frame
        lc_curr_base_frame = curr_l_base_frame - rc_curr_base_frame

        # Force exerted by parent link on current link (link.f)
        link.f = fi + succ_f

        # Moment exerted by parent link on current link (link.n)
        # For 2D, cross([x,y], [fx,fy]) = x*fy - y*fx (scalar z-component)
        cross_lc_succ_f = lc_curr_base_frame[0] * succ_f[1] - lc_curr_base_frame[1] * succ_f[0]
        cross_rc_fi = rc_curr_base_frame[0] * fi[1] - rc_curr_base_frame[1] * fi[0]

        link.n = ni + succ_n + cross_lc_succ_f + cross_rc_fi

        # Calculate joint torque (tau_i)
        # For 2D revolute joint, tau is the z-component of the moment.
        # It's simply the link.n value after accounting for friction and external joint moments
        # (which we ignore for now).
        link.tau = link.n
