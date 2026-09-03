# Copyright (C) 2025-2026 Hiroshi Atsuta <atsuta@ieee.org>
# SPDX-License-Identifier: GPL-3.0-only

"""Compare joint-PD and computed-torque reaching trajectories per joint.

The two state logs must contain ``q`` and ``q_ref`` channels sampled at the same
times, with identical ``q_ref`` values. Each panel shows the common minimum-jerk
joint reference and the trajectory tracked by each controller, in degrees.

Run from the repository root:

    uv run python examples/compare_reaching_controllers.py \
        path/to/reach_4dof_pd.sklog.npz \
        path/to/reach_4dof_ct.sklog.npz \
        --output reach_4dof_comparison.png
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import TYPE_CHECKING, cast

import numpy as np

from skelarm import StateLog

if TYPE_CHECKING:
    from matplotlib.axes import Axes
    from matplotlib.figure import Figure
    from numpy.typing import NDArray

_SERIES_DIMENSIONS = 2


def _joint_channels(log: StateLog, label: str) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Return the measured and reference joint-angle channels from one log."""
    missing = [name for name in ("q", "q_ref") if name not in log.channel_names]
    if missing:
        msg = f"{label} log is missing required channel(s): {', '.join(missing)}"
        raise ValueError(msg)
    q = log.channel("q")
    q_ref = log.channel("q_ref")
    if q.ndim != _SERIES_DIMENSIONS or q_ref.shape != q.shape:
        msg = f"{label} log requires matching 2-D q and q_ref channels, got {q.shape} and {q_ref.shape}"
        raise ValueError(msg)
    return q, q_ref


def load_comparison(
    joint_pd_path: str | Path,
    computed_torque_path: str | Path,
) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
    """Load two aligned reaching logs and extract their common joint reference.

    Parameters
    ----------
    joint_pd_path : str | Path
        State log produced with the joint-PD controller.
    computed_torque_path : str | Path
        State log produced with the computed-torque controller.

    Returns
    -------
    tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]
        Timestamps, common reference, joint-PD trajectory, and computed-torque
        trajectory. Joint angles remain in radians for further analysis.

    Raises
    ------
    ValueError
        If the logs do not contain compatible joint series on a common time grid
        or their recorded references differ.
    """
    joint_pd_log = StateLog.load(joint_pd_path)
    computed_torque_log = StateLog.load(computed_torque_path)
    joint_pd, joint_pd_ref = _joint_channels(joint_pd_log, "joint-PD")
    computed_torque, computed_torque_ref = _joint_channels(computed_torque_log, "computed-torque")
    joint_pd_times = joint_pd_log.times
    computed_torque_times = computed_torque_log.times

    if joint_pd.shape != computed_torque.shape:
        msg = f"joint trajectories have different shapes: {joint_pd.shape} and {computed_torque.shape}"
        raise ValueError(msg)
    if joint_pd_times.shape != computed_torque_times.shape or not np.allclose(
        joint_pd_times, computed_torque_times, rtol=0.0, atol=1e-12
    ):
        msg = "logs must use the same timestamps"
        raise ValueError(msg)
    if not np.allclose(joint_pd_ref, computed_torque_ref, rtol=1e-12, atol=1e-12):
        msg = "logs do not contain a common reference"
        raise ValueError(msg)
    return joint_pd_times, joint_pd_ref, joint_pd, computed_torque


def plot_comparison(
    times: NDArray[np.float64],
    reference: NDArray[np.float64],
    joint_pd: NDArray[np.float64],
    computed_torque: NDArray[np.float64],
) -> tuple[Figure, list[Axes]]:
    """Plot the common reference and both tracked trajectories for every joint.

    Parameters
    ----------
    times : NDArray[np.float64]
        Shared timestamps, shape ``(N,)``.
    reference : NDArray[np.float64]
        Common reference joint angles in radians, shape ``(N, J)``.
    joint_pd : NDArray[np.float64]
        Joint-PD trajectory in radians, shape ``(N, J)``.
    computed_torque : NDArray[np.float64]
        Computed-torque trajectory in radians, shape ``(N, J)``.

    Returns
    -------
    tuple[Figure, list[Axes]]
        The comparison figure and its axes, one per joint.

    Raises
    ------
    ValueError
        If the arrays do not have compatible time-series shapes.
    """
    if times.ndim != 1 or reference.ndim != _SERIES_DIMENSIONS or reference.shape[0] != times.size:
        msg = f"expected times (N,) and reference (N, J), got {times.shape} and {reference.shape}"
        raise ValueError(msg)
    if joint_pd.shape != reference.shape or computed_torque.shape != reference.shape:
        msg = (
            "tracked trajectories must match the reference shape; "
            f"got reference {reference.shape}, joint PD {joint_pd.shape}, computed torque {computed_torque.shape}"
        )
        raise ValueError(msg)
    if reference.shape[1] == 0:
        msg = "at least one joint is required"
        raise ValueError(msg)

    import matplotlib.pyplot as plt

    joint_count = reference.shape[1]
    figure, axes_grid = plt.subplots(joint_count, 1, figsize=(10.0, 2.2 * joint_count), sharex=True, squeeze=False)
    axes = [cast("Axes", axis) for axis in axes_grid[:, 0]]
    reference_deg = np.degrees(reference)
    joint_pd_deg = np.degrees(joint_pd)
    computed_torque_deg = np.degrees(computed_torque)

    for index, axis in enumerate(axes):
        axis.plot(times, reference_deg[:, index], "k--", linewidth=1.5, label="reference")
        axis.plot(times, joint_pd_deg[:, index], color="tab:blue", linewidth=1.2, label="joint PD")
        axis.plot(times, computed_torque_deg[:, index], color="tab:orange", linewidth=1.2, label="computed torque")
        axis.set_ylabel(f"joint {index + 1} (deg)")
        axis.grid(visible=True, alpha=0.3)

    axes[-1].set_xlabel("time (s)")
    handles, labels = axes[0].get_legend_handles_labels()
    figure.legend(handles, labels, loc="upper center", bbox_to_anchor=(0.5, 0.995), ncols=3)
    figure.suptitle("Reaching trajectory tracking: joint PD vs computed torque", y=0.955)
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.92))
    return figure, axes


def build_parser() -> argparse.ArgumentParser:
    """Build the command-line parser."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("joint_pd_log", type=Path, help="joint-PD .sklog.npz file")
    parser.add_argument("computed_torque_log", type=Path, help="computed-torque .sklog.npz file")
    parser.add_argument("-o", "--output", type=Path, help="save the figure to this path")
    parser.add_argument("--dpi", type=int, default=150, help="output resolution for --output (default: 150)")
    parser.add_argument("--no-show", action="store_true", help="do not open the interactive plot window")
    return parser


def main() -> None:
    """Load the requested logs, plot their joint trajectories, and optionally save the figure."""
    parser = build_parser()
    args = parser.parse_args()
    try:
        data = load_comparison(args.joint_pd_log, args.computed_torque_log)
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    figure, _ = plot_comparison(*data)
    if args.output is not None:
        figure.savefig(args.output, dpi=args.dpi, bbox_inches="tight")
        print(f"wrote {args.output}")
    if args.no_show:
        import matplotlib.pyplot as plt

        plt.close(figure)
    else:
        import matplotlib.pyplot as plt

        plt.show()


if __name__ == "__main__":
    main()
