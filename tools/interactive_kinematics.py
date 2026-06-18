"""
Interactive kinematics tool for skelarm.

Load a robot arm from a TOML config given on the command line and pose it
interactively: drag the joint sliders to set joint angles (forward kinematics),
or click (and drag) in the canvas to move the tip to that point (inverse
kinematics).

Usage::

    uv run python tools/interactive_kinematics.py path/to/robot.toml
    uv run python tools/interactive_kinematics.py robot.toml --method sr_inverse --show-com
    uv run python tools/interactive_kinematics.py robot.toml --pose 20,45,60,30
    uv run python tools/interactive_kinematics.py robot.toml --initial pose.toml
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
from PyQt6.QtWidgets import QApplication

from skelarm import SkelarmViewer, Skeleton


def build_parser() -> argparse.ArgumentParser:
    """Build the command-line argument parser for the tool."""
    parser = argparse.ArgumentParser(description="Interactive forward/inverse kinematics viewer for a skelarm robot.")
    parser.add_argument("config", type=Path, help="path to the robot TOML config to load")
    parser.add_argument("--method", default=None, help="IK solver method (e.g. lm_sugihara, sr_inverse)")
    parser.add_argument("--show-com", action="store_true", help="overlay each link's center of mass at startup")
    pose_group = parser.add_mutually_exclusive_group()
    pose_group.add_argument("--pose", default=None, help="initial joint angles in degrees, e.g. 20,45,60,30")
    pose_group.add_argument("--initial", type=Path, default=None, help="TOML file with an [initial] table to apply")
    return parser


def load_skeleton(args: argparse.Namespace) -> Skeleton:
    """Load the skeleton from ``args.config`` and apply any ``--pose`` / ``--initial``.

    Parameters
    ----------
    args : argparse.Namespace
        Parsed command-line arguments.

    Returns
    -------
    Skeleton
        The loaded skeleton with the requested initial pose applied.

    Raises
    ------
    FileNotFoundError
        If the config (or ``--initial`` file) does not exist.
    ValueError
        If ``--pose`` has a different number of values than the robot's joints.
    """
    config: Path = args.config
    if not config.exists():
        msg = f"config file not found: {config}"
        raise FileNotFoundError(msg)
    skeleton = Skeleton.from_toml(config)

    if args.pose is not None:
        pose = np.deg2rad([float(value) for value in args.pose.split(",")])
        if len(pose) != skeleton.num_joints:
            msg = f"--pose has {len(pose)} values but the arm has {skeleton.num_joints} joints"
            raise ValueError(msg)
        skeleton.q = pose
    elif args.initial is not None:
        initial: Path = args.initial
        if not initial.exists():
            msg = f"initial file not found: {initial}"
            raise FileNotFoundError(msg)
        skeleton.apply_initial_toml(initial)

    return skeleton


def main() -> None:
    """Parse arguments, build the viewer, and run the interactive application."""
    parser = build_parser()
    args = parser.parse_args()
    try:
        skeleton = load_skeleton(args)
    except (FileNotFoundError, ValueError) as exc:
        parser.error(str(exc))

    app = QApplication(sys.argv)
    viewer = SkelarmViewer(skeleton)

    if args.method is not None:
        available = [viewer.method_combo.itemText(i) for i in range(viewer.method_combo.count())]
        if args.method not in available:
            parser.error(f"--method {args.method!r} is unavailable for this arm; choose from {available}")
        viewer.method_combo.setCurrentText(args.method)
    if args.show_com:
        viewer.com_checkbox.setChecked(True)

    viewer.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
