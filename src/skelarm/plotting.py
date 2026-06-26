"""Provides utility functions for plotting robot arm states and trajectories."""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from collections.abc import Sequence

    import matplotlib.axes
    from numpy.typing import NDArray

    from skelarm.skeleton import Skeleton

# Colors mirroring the GUI canvas (skelarm.canvas.SkelarmCanvas).
_LINK_COLOR = "#0064c8"  # blue, movable links
_BASE_LINK_COLOR = "#969696"  # gray, fixed base link
_JOINT_COLOR = "#00aa00"  # green, joints
_ORIGIN_COLOR = "black"  # origin
_COM_COLOR = "#c80000"  # red, centers of mass

_JOINT_MARKER_SIZE = 4
_COM_MARKER_SIZE = 6  # a bit larger than the joints

# Task target marker, mirroring the GUI/player overlay (skelarm.canvas._draw_target_marker):
# a filled dot inside a hollow ring, purple by default to match the task's default color.
_GOAL_COLOR = "purple"  # same as scenario._DEFAULT_TARGET_COLOR and QColor(task.color) in the player
_GOAL_DOT_SIZE = 8  # filled center-dot marker size (pt)
_GOAL_RING_SIZE = 18  # hollow ring marker size (pt), used when no success tolerance is given
_GOAL_RING_WIDTH = 1.3  # ring outline width


def draw_skeleton(
    ax: matplotlib.axes.Axes,
    skeleton: Skeleton,
    color: str = _LINK_COLOR,
    linewidth: float = 1.5,
    *,
    base_color: str = _BASE_LINK_COLOR,
    joint_color: str = _JOINT_COLOR,
    show_com: bool = False,
    com_color: str = _COM_COLOR,
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
        Color of the movable robot arm links (defaults to the GUI's blue).
    linewidth : float, optional
        Width of the lines representing the links.
    base_color : str, optional
        Color of the fixed base link (``links[0]``), drawn distinctly from the
        movable links (defaults to the GUI's gray).
    joint_color : str, optional
        Color of the joint markers (defaults to the GUI's green). The origin is
        always drawn black, matching the GUI.
    show_com : bool, optional
        When ``True``, draw each movable link's center of mass as a marker that is
        a bit larger than the joints.
    com_color : str, optional
        Color of the center-of-mass markers (defaults to red).
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

    # Draw the markers on top of the links (same size, never covered by a segment),
    # matching the GUI: joints in joint_color, the origin in black.
    origin_x, origin_y = skeleton.links[0].x, skeleton.links[0].y
    joint_x = [link.xe for link in skeleton.links]
    joint_y = [link.ye for link in skeleton.links]
    ax.plot(
        [origin_x],
        [origin_y],
        linestyle="none",
        marker="o",
        markersize=_JOINT_MARKER_SIZE,
        color=_ORIGIN_COLOR,
        zorder=2,
    )
    ax.plot(joint_x, joint_y, linestyle="none", marker="o", markersize=_JOINT_MARKER_SIZE, color=joint_color, zorder=2)

    # Optionally mark each movable link's center of mass, a bit larger than joints.
    if show_com:
        com_x = [link.xg for link in skeleton.links[1:]]
        com_y = [link.yg for link in skeleton.links[1:]]
        ax.plot(com_x, com_y, linestyle="none", marker="o", markersize=_COM_MARKER_SIZE, color=com_color, zorder=3)

    # Fit the view to the data with a margin and equal aspect, rather than forcing
    # a symmetric box centered on the origin. Autoscaling stays enabled so any other
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


def draw_target(
    ax: matplotlib.axes.Axes,
    position: NDArray[np.float64] | Sequence[float],
    *,
    color: str = _GOAL_COLOR,
    tolerance: float | None = None,
    active: bool = True,
    label: str | None = None,
) -> None:
    """Draw a task target marker matching the interactive player's canvas overlay.

    Mirrors :meth:`skelarm.canvas.SkelarmCanvas._draw_target_marker` so a static
    Matplotlib plot shows the goal exactly as the live GUI / replay player does. An
    *active* target is a filled dot inside a hollow ring; an *inactive* one (a
    multi-target candidate not currently sought) is a dashed hollow ring with no
    fill. The ring is drawn at the success ``tolerance`` radius in data space when
    given, otherwise as a small fixed-size marker.

    Parameters
    ----------
    ax : matplotlib.axes.Axes
        The Matplotlib Axes object to draw on.
    position : NDArray[np.float64] or sequence of float
        Target ``[x, y]`` in world (data) coordinates.
    color : str, optional
        Marker color (defaults to the GUI's purple, matching the task's default
        color and the player overlay).
    tolerance : float | None, optional
        Success radius in meters. When given, the ring is a circle of this data-space
        radius; when ``None`` (default), a small fixed-size ring marker is drawn.
    active : bool, optional
        Draw the filled-dot-plus-ring active style (default). When ``False``, draw the
        dashed hollow ring used for an inactive multi-target candidate.
    label : str | None, optional
        Legend label for the target, added to a single artist. ``None`` (default) adds
        no label.
    """
    from matplotlib.patches import Circle

    x, y = float(position[0]), float(position[1])
    if active:
        # Filled center dot, carrying the legend label.
        ax.plot(
            [x],
            [y],
            marker="o",
            markersize=_GOAL_DOT_SIZE,
            markerfacecolor=color,
            markeredgecolor=color,
            linestyle="none",
            label=label,
            zorder=4,
        )
        if tolerance is not None:
            ax.add_patch(
                Circle((x, y), float(tolerance), fill=False, edgecolor=color, linewidth=_GOAL_RING_WIDTH, zorder=4),
            )
        else:
            ax.plot(
                [x],
                [y],
                marker="o",
                markersize=_GOAL_RING_SIZE,
                markerfacecolor="none",
                markeredgecolor=color,
                markeredgewidth=_GOAL_RING_WIDTH,
                linestyle="none",
                zorder=4,
            )
    elif tolerance is not None:
        ax.add_patch(
            Circle(
                (x, y),
                float(tolerance),
                fill=False,
                edgecolor=color,
                linewidth=_GOAL_RING_WIDTH,
                linestyle="--",
                zorder=4,
                label=label,
            ),
        )
    else:
        ax.plot(
            [x],
            [y],
            marker="o",
            markersize=_GOAL_RING_SIZE,
            markerfacecolor="none",
            markeredgecolor=color,
            markeredgewidth=_GOAL_RING_WIDTH,
            linestyle="none",
            label=label,
            zorder=4,
        )


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
