"""Provides functions for robot arm kinematics."""

from __future__ import annotations

import numpy as np

from skelarm.skeleton import Skeleton


def compute_forward_kinematics(skeleton: Skeleton) -> None:
    """Compute the forward kinematics for the given skeleton.

    Updates the (x, y) positions of each link's end-effector (tip) and joints.
    The base of the robot arm is assumed to be at (0, 0).

    :param skeleton: The Skeleton object containing the robot arm's links and joint angles.
    """
    current_x = 0.0
    current_y = 0.0
    current_angle = 0.0  # Absolute angle of the current link

    for link in skeleton.links:
        # Store the start of the link (joint position)
        link.x = current_x
        link.y = current_y

        # Add the current joint angle to the absolute angle
        current_angle += link.q

        # Calculate the end-effector position of the current link
        delta_x = link.prop.length * np.cos(current_angle)
        delta_y = link.prop.length * np.sin(current_angle)

        current_x += delta_x
        current_y += delta_y

        # Store the end-effector position of the current link
        link.xe = current_x
        link.ye = current_y
