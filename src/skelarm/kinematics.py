"""Provides functions for robot arm kinematics."""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from skelarm.skeleton import Skeleton


def compute_forward_kinematics(skeleton: Skeleton) -> None:
    """Compute the forward kinematics (positions and velocities) for the given skeleton.

    Updates the (x, y) positions of each link's end-effector (tip) and joints,
    as well as the linear and angular velocities of joints and COM.

    The base of the robot arm is assumed to be at (0, 0) with zero velocity.

    :param skeleton: The Skeleton object containing the robot arm's links and joint angles.
    """
    current_x = 0.0
    current_y = 0.0
    current_angle = 0.0  # Absolute angle of the current link

    # Initialize base velocities
    prev_w = 0.0  # Angular velocity of the previous link frame
    prev_v = np.array([0.0, 0.0], dtype=np.float64)  # Linear velocity of the previous link origin (base)

    for link in skeleton.links:
        # Store the start of the link (joint position)
        link.x = current_x
        link.y = current_y

        # Add the current joint angle to the absolute angle
        current_angle += link.q
        link.q_absolute = current_angle

        # Calculate the end-effector position of the current link
        delta_x = link.prop.length * np.cos(current_angle)
        delta_y = link.prop.length * np.sin(current_angle)

        current_x += delta_x
        current_y += delta_y

        # Store the end-effector position of the current link
        link.xe = current_x
        link.ye = current_y

        # Compute angular velocity (scalar for 2D)
        link.w = prev_w + link.dq

        # Vector from previous joint to current joint (in base frame)
        r_prev_to_curr = np.array(
            [
                link.prop.length * np.cos(current_angle),
                link.prop.length * np.sin(current_angle),
            ]
        )

        # Compute linear velocity of current joint
        # v_i = v_{i-1} + w_{i-1} x r_i
        # 2D cross product: w x r = [-w*ry, w*rx]
        cross_w_r_prev = np.array([-prev_w * r_prev_to_curr[1], prev_w * r_prev_to_curr[0]])
        link.v = prev_v + cross_w_r_prev

        # Vector from current joint to COM (relative to link frame)
        rc_curr = np.array([link.prop.rgx, link.prop.rgy])

        # Rotate rc_curr to base frame
        r_curr_to_base = np.array(
            [
                [np.cos(current_angle), -np.sin(current_angle)],
                [np.sin(current_angle), np.cos(current_angle)],
            ]
        )
        rc_curr_base_frame = r_curr_to_base @ rc_curr

        # Compute linear velocity of COM
        # vc_i = v_i + w_i x rc_i
        cross_w_rc = np.array([-link.w * rc_curr_base_frame[1], link.w * rc_curr_base_frame[0]])
        link.vc = link.v + cross_w_rc

        # Update for next iteration
        prev_w = link.w
        prev_v = link.v
