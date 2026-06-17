"""Provides utility functions for plotting robot arm states and trajectories."""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    import matplotlib.axes
    from numpy.typing import NDArray

    from skelarm.skeleton import Skeleton


def draw_skeleton(
    ax: matplotlib.axes.Axes,
    skeleton: Skeleton,
    color: str = "blue",
    linewidth: float = 1.5,
    *,
    base_color: str = "gray",
    label: str | None = None,
    title: str | None = "Robot Arm Skeleton",
) -> None:
    """Draw the robot arm skeleton on a given Matplotlib Axes object.

    Parameters
    ----------
    ax : matplotlib.axes.Axes
        The Matplotlib Axes object to draw on.
    skeleton : Skeleton
        The Skeleton object containing the robot arm's links.
    color : str, optional
        Color of the movable robot arm links.
    linewidth : float, optional
        Width of the lines representing the links.
    base_color : str, optional
        Color of the fixed base link (``links[0]``), drawn distinctly from the
        movable links.
    label : str | None, optional
        Legend label for the arm. When given, a single legend entry is added and
        the legend is shown; when ``None`` (default) no label or legend is added.
    title : str | None, optional
        Axes title. Pass ``None`` to leave the existing title untouched so the
        skeleton can share an Axes with another plot (e.g. a trajectory).
    """
    # Draw each link from its start joint (x, y) to its tip (xe, ye) as a line only;
    # the fixed base link (links[0]) is drawn in base_color, the movable links use
    # color. The legend label (if any) goes on the first movable link so its swatch
    # matches color rather than the base color.
    label_index = 1 if len(skeleton.links) > 1 else None
    for i, link in enumerate(skeleton.links):
        ax.plot(
            [link.x, link.xe],
            [link.y, link.ye],
            color=base_color if i == 0 else color,
            linewidth=linewidth,
            label=label if i == label_index else None,
            zorder=1,
        )

    # Draw the origin and every joint as markers on top of the links, all the same
    # size, so the circles are never partially covered by a link segment.
    joint_x = [skeleton.links[0].x, *(link.xe for link in skeleton.links)]
    joint_y = [skeleton.links[0].y, *(link.ye for link in skeleton.links)]
    ax.plot(joint_x, joint_y, linestyle="none", marker="o", markersize=5, color=color, zorder=2)

    # Fit the view to the data with a margin and equal aspect, rather than forcing
    # a symmetric box centred on the origin. Autoscaling stays enabled so any other
    # artists (e.g. an overlaid trajectory) are included instead of clipped, and a
    # zero-extent arm still gets a valid, non-degenerate range.
    ax.set_aspect("equal", adjustable="box")
    ax.margins(0.1)
    ax.relim()
    ax.autoscale(enable=True)
    ax.set_xlabel("X Position")
    ax.set_ylabel("Y Position")
    if label is not None and label_index is not None:
        ax.legend()
    if title is not None:
        ax.set_title(title)
    ax.grid()


def plot_trajectory(
    ax: matplotlib.axes.Axes,
    trajectory_x: NDArray[np.float64],
    trajectory_y: NDArray[np.float64],
    color: str = "red",
    linestyle: str = "-",
    linewidth: float = 1.0,
    *,
    label: str = "Tip Trajectory",
    title: str | None = "Tip Trajectory",
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
    label : str, optional
        Legend label for the trajectory.
    title : str | None, optional
        Axes title. Pass ``None`` to leave the existing title untouched so the
        trajectory can share an Axes with another plot (e.g. a skeleton).
    """
    ax.plot(trajectory_x, trajectory_y, color=color, linestyle=linestyle, linewidth=linewidth, label=label)
    ax.legend()
    ax.set_xlabel("X Position")
    ax.set_ylabel("Y Position")
    if title is not None:
        ax.set_title(title)
    ax.grid()
