"""Provides utility functions for plotting robot arm states and trajectories."""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    import matplotlib.axes
    from numpy.typing import NDArray

from skelarm.skeleton import Skeleton


def draw_skeleton(ax: matplotlib.axes.Axes, skeleton: Skeleton, color: str = "blue", linewidth: float = 2.0) -> None:
    """
    Draw the robot arm skeleton on a given Matplotlib Axes object.

    :param ax: The Matplotlib Axes object to draw on.
    :param skeleton: The Skeleton object containing the robot arm's links.
    :param color: Color of the robot arm links.
    :param linewidth: Width of the lines representing the links.
    """
    if not skeleton.links:
        return

    # Extract joint and end-effector positions
    joint_x = [link.x for link in skeleton.links]
    joint_y = [link.y for link in skeleton.links]
    tip_x = [link.xe for link in skeleton.links]
    tip_y = [link.ye for link in skeleton.links]

    # Combine all x and y coordinates for plotting
    all_x_coords = [0.0, *joint_x]  # Start from base (0,0)
    all_y_coords = [0.0, *joint_y]  # Start from base (0,0)

    # For each link, draw a line from its start (joint) to its end (tip)
    for i in range(skeleton.num_links):
        x_coords = [skeleton.links[i].x, skeleton.links[i].xe]
        y_coords = [skeleton.links[i].y, skeleton.links[i].ye]
        ax.plot(x_coords, y_coords, color=color, linewidth=linewidth, marker="o", markersize=5)

    # Set appropriate limits
    all_coords = [abs(c) for c in all_x_coords + all_y_coords + tip_x + tip_y]
    max_range = max(all_coords) * 1.1 if all_coords else 1.0  # Ensure max_range is not 0 for empty list

    ax.set_xlim(-max_range, max_range)
    ax.set_ylim(-max_range, max_range)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("X Position")
    ax.set_ylabel("Y Position")
    ax.set_title("Robot Arm Skeleton")
    ax.grid()  # FBT003: Use ax.grid() instead of ax.grid(True)


def plot_trajectory(
    ax: matplotlib.axes.Axes,
    trajectory_x: NDArray[np.float64],
    trajectory_y: NDArray[np.float64],
    color: str = "red",
    linestyle: str = "-",
    linewidth: float = 1.0,
) -> None:
    """
    Plot a 2D trajectory on a given Matplotlib Axes object.

    :param ax: The Matplotlib Axes object to draw on.
    :param trajectory_x: NumPy array of x-coordinates for the trajectory.
    :param trajectory_y: NumPy array of y-coordinates for the trajectory.
    :param color: Color of the trajectory line.
    :param linestyle: Style of the trajectory line (e.g., '-', '--', ':').
    :param linewidth: Width of the trajectory line.
    """
    ax.plot(trajectory_x, trajectory_y, color=color, linestyle=linestyle, linewidth=linewidth, label="Tip Trajectory")
    ax.legend()
    ax.set_xlabel("X Position")
    ax.set_ylabel("Y Position")
    ax.set_title("Tip Trajectory")
    ax.grid()  # FBT003: Use ax.grid() instead of ax.grid(True)
