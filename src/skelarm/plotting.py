"""Provides utility functions for plotting robot arm states and trajectories."""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    import matplotlib.axes
    from numpy.typing import NDArray

from skelarm.skeleton import Skeleton


def draw_skeleton(ax: matplotlib.axes.Axes, skeleton: Skeleton, color: str = "blue", linewidth: float = 2.0) -> None:
    """Draw the robot arm skeleton on a given Matplotlib Axes object.

    Parameters
    ----------
    ax : matplotlib.axes.Axes
        The Matplotlib Axes object to draw on.
    skeleton : Skeleton
        The Skeleton object containing the robot arm's links.
    color : str, optional
        Color of the robot arm links.
    linewidth : float, optional
        Width of the lines representing the links.
    """
    if not skeleton.links:
        return

    # Draw each link as a line from its start joint (x, y) to its tip (xe, ye).
    for link in skeleton.links:
        ax.plot([link.x, link.xe], [link.y, link.ye], color=color, linewidth=linewidth, marker="o", markersize=5)

    # Scale the axes to fit the base (0, 0) and every joint and tip position.
    coords = [0.0]
    for link in skeleton.links:
        coords.extend((link.x, link.y, link.xe, link.ye))
    max_range = max(abs(c) for c in coords) * 1.1

    ax.set_xlim(-max_range, max_range)
    ax.set_ylim(-max_range, max_range)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("X Position")
    ax.set_ylabel("Y Position")
    ax.set_title("Robot Arm Skeleton")
    ax.grid()


def plot_trajectory(
    ax: matplotlib.axes.Axes,
    trajectory_x: NDArray[np.float64],
    trajectory_y: NDArray[np.float64],
    color: str = "red",
    linestyle: str = "-",
    linewidth: float = 1.0,
) -> None:
    """Plot a 2D trajectory on a given Matplotlib Axes object.

    Parameters
    ----------
    ax : matplotlib.axes.Axes
        The Matplotlib Axes object to draw on.
    trajectory_x : NDArray[np.float64]
        Array of x-coordinates for the trajectory.
    trajectory_y : NDArray[np.float64]
        Array of y-coordinates for the trajectory.
    color : str, optional
        Color of the trajectory line.
    linestyle : str, optional
        Style of the trajectory line (e.g., '-', '--', ':').
    linewidth : float, optional
        Width of the trajectory line.
    """
    ax.plot(trajectory_x, trajectory_y, color=color, linestyle=linestyle, linewidth=linewidth, label="Tip Trajectory")
    ax.legend()
    ax.set_xlabel("X Position")
    ax.set_ylabel("Y Position")
    ax.set_title("Tip Trajectory")
    ax.grid()
