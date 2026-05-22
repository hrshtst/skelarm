"""Provides functions for robot arm kinematics."""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from skelarm.skeleton import Skeleton


def compute_forward_kinematics(skeleton: Skeleton) -> None:
    """Compute the forward kinematics for the given skeleton.

    Updates the (x, y) positions of each link's end-effector (tip) and joints,
    as well as angular, joint-origin, tip, and COM velocities and accelerations.

    The base of the robot arm is assumed to be at (0, 0) with zero velocity and
    acceleration.

    Parameters
    ----------
    skeleton : Skeleton
        The Skeleton object containing the robot arm's links and joint angles.
    """
    current_x = 0.0
    current_y = 0.0
    current_angle = 0.0  # Absolute angle of the current link

    prev_w = 0.0
    prev_dw = 0.0
    prev_v = np.array([0.0, 0.0], dtype=np.float64)
    prev_dv = np.array([0.0, 0.0], dtype=np.float64)

    for link in skeleton.links:
        # Store the start of the link (joint position)
        link.x = current_x
        link.y = current_y

        # Add the current joint angle to the absolute angle
        current_angle += link.q
        link.q_absolute = current_angle

        cos_angle = np.cos(current_angle)
        sin_angle = np.sin(current_angle)

        # Compute angular velocity and acceleration (scalar for 2D)
        link.w = prev_w + link.dq
        link.dw = prev_dw + link.ddq

        link.v = prev_v
        link.dv = prev_dv

        link_vec = np.array([link.prop.length * cos_angle, link.prop.length * sin_angle], dtype=np.float64)
        cross_dw_link = np.array([-link.dw * link_vec[1], link.dw * link_vec[0]], dtype=np.float64)
        centripetal_link = np.array([-(link.w**2) * link_vec[0], -(link.w**2) * link_vec[1]], dtype=np.float64)
        tip_v = link.v + np.array([-link.w * link_vec[1], link.w * link_vec[0]], dtype=np.float64)
        tip_dv = link.dv + cross_dw_link + centripetal_link

        current_x += link_vec[0]
        current_y += link_vec[1]

        # Store the end-effector position of the current link
        link.xe = current_x
        link.ye = current_y
        link.vx = tip_v[0]
        link.vy = tip_v[1]
        link.ax = tip_dv[0]
        link.ay = tip_dv[1]

        # Vector from current joint to COM (relative to link frame)
        rc_curr = np.array([link.prop.rgx, link.prop.rgy])

        # Rotate rc_curr to base frame
        r_curr_to_base = np.array(
            [
                [cos_angle, -sin_angle],
                [sin_angle, cos_angle],
            ]
        )
        rc_curr_base_frame = r_curr_to_base @ rc_curr
        link.xg = link.x + rc_curr_base_frame[0]
        link.yg = link.y + rc_curr_base_frame[1]

        # Compute linear velocity of COM
        # vc_i = v_i + w_i x rc_i
        cross_w_rc = np.array([-link.w * rc_curr_base_frame[1], link.w * rc_curr_base_frame[0]])
        link.vc = link.v + cross_w_rc
        cross_dw_rc = np.array([-link.dw * rc_curr_base_frame[1], link.dw * rc_curr_base_frame[0]])
        centripetal_rc = np.array([-(link.w**2) * rc_curr_base_frame[0], -(link.w**2) * rc_curr_base_frame[1]])
        link.dvc = link.dv + cross_dw_rc + centripetal_rc
        link.agx = link.dvc[0]
        link.agy = link.dvc[1]

        # Update for next iteration
        prev_w = link.w
        prev_dw = link.dw
        prev_v = tip_v
        prev_dv = tip_dv
