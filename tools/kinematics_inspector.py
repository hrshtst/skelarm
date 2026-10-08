# Copyright (C) 2025-2026 Hiroshi Atsuta <atsuta@ieee.org>
# SPDX-License-Identifier: GPL-3.0-only

"""
Interactive kinematics tool for skelarm.

Load a robot arm from a TOML config given on the command line and pose it
interactively: drag the joint sliders to set joint angles (forward kinematics),
or click (and drag) in the canvas to move the tip to that point (inverse
kinematics). If the config (or ``--task``) has a ``[task]`` table, its targets and
reference path are drawn, and the status readout gives the tip's distance to the
target.

Usage::

    uv run python tools/kinematics_inspector.py path/to/robot.toml
    uv run python tools/kinematics_inspector.py robot.toml --method sr_inverse --show-com
    uv run python tools/kinematics_inspector.py robot.toml --pose 20,45,60,30
    uv run python tools/kinematics_inspector.py robot.toml --initial pose.toml
    uv run python tools/kinematics_inspector.py robot.toml --task task.toml
"""

from __future__ import annotations

import argparse
import sys
import tomllib
from pathlib import Path

import numpy as np
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QKeySequence
from PyQt6.QtWidgets import QApplication, QCheckBox, QComboBox, QLabel, QPushButton

from skelarm import SkelarmViewer, Skeleton, Task, apply_active_target, make_icon

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # allow `tools.` imports when run as a script
from tools._scenario_cli import task_overlays


class KinematicsInspector(SkelarmViewer):
    """A SkelarmViewer with extra inspection controls.

    Adds a center-of-mass overlay toggle, an IK-method selector, a reset-pose
    button, and a live status readout (endpoint position, the distance to the
    task's target, and the latest IK result) on top of the minimal slider/canvas
    viewer. A given task is drawn: its target markers and its reference path.
    """

    def __init__(self, skeleton: Skeleton, *, task: Task | None = None) -> None:
        """Build the inspector controls on top of the base viewer, drawing ``task`` if given."""
        super().__init__(skeleton)
        self._initial_q = skeleton.q.copy()  # pose to restore on reset
        self._task = task
        if task is not None:
            self.canvas.overlay_targets, self.canvas.overlay_path = task_overlays(task, skeleton)

        self.com_checkbox = QCheckBox("Show center of mass")
        self.com_checkbox.toggled.connect(self._on_show_com_toggled)
        self.add_control(self.com_checkbox)

        # Newton-Raphson needs a square Jacobian, so only offer it for a 2-joint arm.
        self.add_control(QLabel("IK method"))
        self.method_combo = QComboBox()
        methods = ["lm_sugihara", "lm", "sr_inverse", "pseudoinverse"]
        if skeleton.num_joints == 2:  # noqa: PLR2004
            methods.append("nr")
        self.method_combo.addItems(methods)
        # Picked with the mouse only: a focused combo box would take R (reset) for its type-ahead search.
        self.method_combo.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.method_combo.currentTextChanged.connect(self._on_method_changed)
        self.add_control(self.method_combo)

        self.reset_button = QPushButton("Reset pose")
        self.reset_button.setIcon(make_icon("mdi6.backup-restore"))
        self.reset_button.setShortcut(QKeySequence("R"))
        self.reset_button.setToolTip("Reset pose (R)")
        self.reset_button.clicked.connect(self._on_reset)
        self.add_control(self.reset_button)

        self.status_label = QLabel()
        self.status_label.setWordWrap(True)
        self.add_control(self.status_label)

        self.pose_updated.connect(self._update_status)
        self._update_status()

    def _on_show_com_toggled(self) -> None:
        """Toggle the center-of-mass overlay on the canvas and repaint."""
        self.canvas.show_com = self.com_checkbox.isChecked()
        self.canvas.update_skeleton()

    def _on_method_changed(self, method: str) -> None:
        """Route the selected IK method to the canvas solver."""
        self.canvas.ik_method = method

    def _on_reset(self) -> None:
        """Restore the initial pose and clear any IK target."""
        self.skeleton.q = self._initial_q.copy()
        self.canvas.clear_ik_target()
        self.refresh_from_skeleton()

    def _update_status(self) -> None:
        """Refresh the status label with the endpoint position and latest IK result."""
        tip = self.skeleton.links[-1]
        text = f"Tip: ({tip.xe:.3f}, {tip.ye:.3f}) m"
        if self._task is not None and self._task.target is not None:
            x, y = self._task.target
            distance = float(np.hypot(tip.xe - x, tip.ye - y))
            text += f"\nTarget: ({x:.3f}, {y:.3f}) m, {distance:.3f} m from the tip"
        result = self.canvas.last_ik_result
        if result is not None:
            text += f"\nIK: {result.status}, residual={result.residual_norm:.3g}"
        self.status_label.setText(text)


def build_parser() -> argparse.ArgumentParser:
    """Build the command-line argument parser for the tool."""
    parser = argparse.ArgumentParser(description="Interactive forward/inverse kinematics viewer for a skelarm robot.")
    parser.add_argument("config", type=Path, help="path to the robot TOML config to load")
    parser.add_argument("--method", default=None, help="IK solver method (e.g. lm_sugihara, sr_inverse)")
    parser.add_argument("--show-com", action="store_true", help="overlay each link's center of mass at startup")
    parser.add_argument("--initial", type=Path, default=None, help="TOML file with an [initial] table to apply")
    parser.add_argument(
        "--pose",
        default=None,
        help="initial joint angles in degrees, e.g. 20,45,60,30 (overrides --initial)",
    )
    parser.add_argument(
        "--task", type=Path, default=None, help="TOML file whose [task] is drawn instead of the config's"
    )
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

    # Pose precedence (each overwrites the previous): zeros -> per-link q0 ->
    # config [initial] (handled by from_toml) -> --initial file -> --pose.
    if args.initial is not None:
        initial: Path = args.initial
        if not initial.exists():
            msg = f"initial file not found: {initial}"
            raise FileNotFoundError(msg)
        skeleton.apply_initial_toml(initial)
    if args.pose is not None:
        pose = np.deg2rad([float(value) for value in args.pose.split(",")])
        if len(pose) != skeleton.num_joints:
            msg = f"--pose has {len(pose)} values but the arm has {skeleton.num_joints} joints"
            raise ValueError(msg)
        skeleton.q = pose

    return skeleton


def load_task(args: argparse.Namespace) -> Task | None:
    """Load the task to draw: the ``[task]`` of ``--task`` if given, else the config's, if any.

    A multi-target task's configured active candidate becomes its target, as when a
    scenario is loaded, so that the status readout measures the distance to it.

    Parameters
    ----------
    args : argparse.Namespace
        Parsed command-line arguments.

    Returns
    -------
    Task | None
        The task, or ``None`` when the config has no ``[task]`` table and no ``--task`` is given.

    Raises
    ------
    FileNotFoundError
        If the ``--task`` file does not exist.
    ValueError
        If the ``--task`` file has no ``[task]`` table, the table is not a valid task, or a
        multi-target task's active index is out of range.
    """
    if args.task is not None:
        path: Path = args.task
        if not path.exists():
            msg = f"task file not found: {path}"
            raise FileNotFoundError(msg)
        with path.open("rb") as f:
            data = tomllib.load(f)
        if "task" not in data:
            msg = f"no [task] section in {path}"
            raise ValueError(msg)
    else:
        with args.config.open("rb") as f:
            data = tomllib.load(f)
        if "task" not in data:
            return None
    task = Task.from_dict(data["task"])
    if task.type == "multi_target_reaching":
        apply_active_target(task)
    return task


def main() -> None:
    """Parse arguments, build the viewer, and run the interactive application."""
    parser = build_parser()
    args = parser.parse_args()
    try:
        skeleton = load_skeleton(args)
        task = load_task(args)
    except (FileNotFoundError, ValueError) as exc:
        parser.error(str(exc))

    app = QApplication(sys.argv)
    viewer = KinematicsInspector(skeleton, task=task)

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
