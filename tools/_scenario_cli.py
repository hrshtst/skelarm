"""Shared building blocks for the interactive scenario simulators.

Holds the CLI config-override machinery (``--initial`` / ``--pose`` / ``--task`` /
``--controller`` / ``--no-joint-limits``), the ``ScenarioSimulator`` base GUI (record /
export / reset, with the task config embedded in the recording so the player can draw
overlays), and ``task_overlays`` — the one place that turns a :class:`~skelarm.Task` into
drawable target markers and a reference polyline, reused by the simulators and the player.
"""

from __future__ import annotations

import tomllib
from pathlib import Path
from typing import TYPE_CHECKING, Any

import numpy as np
from PyQt6.QtGui import QColor
from PyQt6.QtWidgets import QCheckBox, QFileDialog, QPushButton

from skelarm import (
    SkelarmSimulator,
    active_target_index,
    build_curve,
    multi_target_specs,
    run_scenario,
    scenario_from_config,
)

if TYPE_CHECKING:
    import argparse

    from numpy.typing import NDArray

    from skelarm import Scenario, Skeleton, Task

_DRAG_STIFFNESS = 20.0  # N/m for the mouse drag; firm enough to perturb a controlled arm
_CURVE_SAMPLES = 256  # points used to draw a periodic curve as a polyline


# --------------------------------------------------------------------------- CLI overrides


def read_toml(path: Path) -> dict[str, Any]:
    """Parse a TOML file into a dict."""
    with path.open("rb") as f:
        return tomllib.load(f)


def read_section(path: Path, section: str) -> dict[str, Any]:
    """Read a single ``[section]`` table from an override TOML file.

    Raises
    ------
    FileNotFoundError
        If the file does not exist.
    ValueError
        If the section is missing.
    """
    if not path.exists():
        msg = f"override file not found: {path}"
        raise FileNotFoundError(msg)
    data = read_toml(path)
    if section not in data:
        msg = f"no [{section}] section in {path}"
        raise ValueError(msg)
    return dict(data[section])


def apply_overrides(
    config: dict[str, Any],
    *,
    initial: Path | None,
    pose: str | None,
    task: Path | None,
    controller: Path | None,
) -> None:
    """Overwrite the config's ``[initial]`` / ``[task]`` / ``[controller]`` tables in place.

    ``--initial`` replaces the whole ``[initial]`` table; ``--pose`` then overrides just
    its ``q`` (degrees), matching the kinematics/dynamics tools.
    """
    if task is not None:
        config["task"] = read_section(task, "task")
    if controller is not None:
        config["controller"] = read_section(controller, "controller")
    if initial is not None:
        config["initial"] = read_section(initial, "initial")
    if pose is not None:
        config.setdefault("initial", {})["q"] = [float(value) for value in pose.split(",")]


def build_scenario(
    config: str | Path,
    *,
    initial: str | Path | None = None,
    pose: str | None = None,
    task: str | Path | None = None,
    controller: str | Path | None = None,
) -> Scenario:
    """Load a scenario config and apply the command-line section overrides."""
    data = read_toml(Path(config))
    apply_overrides(
        data,
        initial=None if initial is None else Path(initial),
        pose=pose,
        task=None if task is None else Path(task),
        controller=None if controller is None else Path(controller),
    )
    return scenario_from_config(data)


def save_scenario_run(
    scenario: Scenario,
    output: str | Path,
    *,
    duration: float | None = None,
    enforce_limits: bool | None = None,
) -> Path:
    """Run a built scenario headlessly and save a replayable ``.sklog.npz``.

    The log embeds the full scenario config (via ``run_scenario``), so it replays in the
    player with task overlays. Returns the output path.
    """
    log = run_scenario(scenario, duration=duration, enforce_limits=enforce_limits)
    out_path = Path(output)
    log.save(out_path)
    print(f"{type(scenario.controller).__name__}: wrote {len(log)} frames to {out_path}")
    return out_path


def run_headless(
    config: str | Path,
    *,
    output: str | Path | None = None,
    duration: float | None = None,
    initial: str | Path | None = None,
    pose: str | None = None,
    task: str | Path | None = None,
    controller: str | Path | None = None,
    enforce_limits: bool | None = None,
) -> Path:
    """Build a scenario (with overrides), run it headlessly, and save a replayable log."""
    scenario = build_scenario(config, initial=initial, pose=pose, task=task, controller=controller)
    out_path = Path(output) if output is not None else Path(config).with_suffix(".sklog.npz")
    return save_scenario_run(scenario, out_path, duration=duration, enforce_limits=enforce_limits)


def add_override_arguments(parser: argparse.ArgumentParser) -> None:
    """Add the shared ``--initial`` / ``--pose`` / ``--task`` / ``--controller`` / ``--no-joint-limits`` flags."""
    parser.add_argument(
        "--initial", type=Path, default=None, help="TOML file whose [initial] table overrides the base initial pose"
    )
    parser.add_argument(
        "--pose", default=None, help="initial joint angles in degrees, e.g. 20,45,60,30 (overrides --initial)"
    )
    parser.add_argument("--task", type=Path, default=None, help="TOML file whose [task] table overrides the base task")
    parser.add_argument(
        "--controller", type=Path, default=None, help="TOML file whose [controller] table overrides the base controller"
    )
    parser.add_argument(
        "--no-joint-limits",
        action="store_true",
        help="override [simulator].enforce_limits off: no dynamics hard stop (limits apply to kinematics only)",
    )


def resolve_enforce_limits(args: argparse.Namespace, scenario: Scenario) -> bool:
    """``--no-joint-limits`` overrides the scenario's ``[simulator].enforce_limits`` off."""
    return False if args.no_joint_limits else scenario.simulator.enforce_limits


# --------------------------------------------------------------------------- task overlays


def task_overlays(
    task: Task,
    skeleton: Skeleton,
) -> tuple[list[tuple[NDArray[np.float64], QColor, float | None, bool]], NDArray[np.float64] | None]:
    """Turn a task into drawable ``(target markers, reference polyline)`` for the canvas.

    The markers carry the active flag (for multi-target emphasis); the polyline is the
    periodic curve, the tracked tip trajectory, or the forward kinematics of a per-joint
    reference (or ``None`` when the task has no path).
    """
    return _target_markers(task), _reference_path(task, skeleton)


def _target_markers(task: Task) -> list[tuple[NDArray[np.float64], QColor, float | None, bool]]:
    """Target markers for reaching / multi-target tasks (the active one flagged)."""
    if task.type == "multi_target_reaching":
        active = active_target_index(task)
        return [
            (pos, QColor(color), tolerance, index == active)
            for index, (pos, _, color, tolerance) in enumerate(multi_target_specs(task))
        ]
    if task.target is not None:
        return [(np.asarray(task.target, dtype=np.float64), QColor(task.color), task.tolerance, True)]
    return []


def _reference_path(task: Task, skeleton: Skeleton) -> NDArray[np.float64] | None:
    """The task's reference polyline in task space, or ``None``."""
    if task.type == "periodic_curve":
        curve = build_curve(str(task.params["curve"]), task.params)
        return np.array([curve(theta) for theta in np.linspace(0.0, 2.0 * np.pi, _CURVE_SAMPLES)], dtype=np.float64)
    samples = task.params.get("reference_samples")
    if samples is None:
        return None
    values = np.atleast_2d(np.asarray(samples["values"], dtype=np.float64))
    if task.type == "trajectory_tracking":
        return values  # already a tip (x, y) series
    if task.type == "joint_trajectory_tracking":
        return _forward_kinematics_path(skeleton, values)
    return None


def _forward_kinematics_path(skeleton: Skeleton, q_series: NDArray[np.float64]) -> NDArray[np.float64]:
    """Tip path obtained by forward kinematics of a per-joint reference series."""
    arm = skeleton.clone()
    tips = []
    for q in q_series:
        arm.q = q  # the setter refreshes forward kinematics
        tip = arm.links[-1]
        tips.append([tip.xe, tip.ye])
    return np.asarray(tips, dtype=np.float64)


# --------------------------------------------------------------------------- base GUI


class ScenarioSimulator(SkelarmSimulator):
    """Interactive scenario GUI: a controller drives the arm, the mouse adds tip forces.

    Adds Reset / Record / Export controls, embeds the scenario config in the recording
    (so the player can later draw the task overlay), and draws the task's reference path.
    Subclasses add task-specific readouts or live controls.
    """

    def __init__(self, scenario: Scenario, *, stiffness: float = _DRAG_STIFFNESS, enforce_limits: bool = True) -> None:
        """Build the scenario GUI for a loaded scenario."""
        super().__init__(
            scenario.skeleton,
            controller=scenario.controller,
            target=scenario.task.target,
            target_color=scenario.task.color,
            target_tolerance=scenario.task.tolerance,
            stiffness=stiffness,
            enforce_limits=enforce_limits,
            log_extra={"source_config": dict(scenario.source_config)} if scenario.source_config else None,
        )
        self._task = scenario.task

        self.reset_button = QPushButton("Reset")
        self.reset_button.clicked.connect(self.reset)
        self.add_control(self.reset_button)

        self.record_checkbox = QCheckBox("Record states")
        self.record_checkbox.setChecked(True)
        self.record_checkbox.toggled.connect(self._on_record_toggled)
        self.add_control(self.record_checkbox)

        self.export_button = QPushButton("Export…")
        self.export_button.clicked.connect(self._on_export)
        self.add_control(self.export_button)

        # Draw the reference curve / trajectory behind the arm (the target marker comes
        # from the base canvas's `target` / `secondary_targets`).
        _, self.canvas.overlay_path = task_overlays(scenario.task, scenario.skeleton)

        self.start_recording()

    def _on_record_toggled(self, checked: bool) -> None:  # noqa: FBT001
        """Start or stop appending frames to the state log."""
        if checked:
            self.start_recording()
        else:
            self.stop_recording()

    def _on_export(self) -> None:
        """Prompt for a path and export the recorded state log."""
        if self.state_log is None or len(self.state_log) == 0:
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "Export state log", "run.sklog.npz", "State log (*.npz);;TOML (*.toml)"
        )
        if path:
            if Path(path).suffix == ".toml":
                self.state_log.export_toml(path)
            else:
                self.state_log.save(path)
