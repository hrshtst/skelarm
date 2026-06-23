"""
Interactive reaching simulator (with a headless batch mode) for skelarm.

Loads a combined scenario config (``[skeleton]`` / ``[initial]`` / ``[task]`` /
``[controller]``; see :func:`skelarm.scenario.load_scenario`) and, by default,
opens a real-time GUI: the controller drives the arm toward the task target (shown
as a purple marker) while you press and drag the left mouse button to apply an
external force at the tip (drawn as a red arrow), like ``tools/dynamics_simulator.py``.
The run can be recorded and exported to a ``.sklog.npz`` log for replay/analysis.

With ``--save PATH`` it instead runs headlessly (no GUI), simulating the planned
reach and writing the log to PATH -- handy for batch comparisons across controllers
(``--controller``) or tasks (``--task``). The ``[initial]`` / ``[task]`` /
``[controller]`` sections can be overridden from separate files in both modes.

Usage::

    uv run python tools/reach.py examples/reach.toml                       # interactive GUI
    uv run python tools/reach.py examples/reach.toml --controller pd.toml  # GUI, other controller
    uv run python tools/reach.py examples/reach.toml --save run.sklog.npz  # headless batch run
    uv run python tools/reach.py examples/reach.toml --save far.sklog.npz --task far.toml
    uv run python tools/replay.py run.sklog.npz                            # replay a saved run
"""

from __future__ import annotations

import argparse
import sys
import tomllib
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np
from PyQt6.QtWidgets import QApplication, QCheckBox, QFileDialog, QLabel, QPushButton

from skelarm import SkelarmSimulator, run_scenario, scenario_from_config

if TYPE_CHECKING:
    from typing import Any

    from skelarm.scenario import Scenario

_DRAG_STIFFNESS = 20.0  # N/m for the mouse drag; firm enough to perturb a controlled arm


class ReachSimulator(SkelarmSimulator):
    """Interactive reaching GUI: a controller drives the arm; the mouse adds tip forces.

    Adds reset, record/export, and a tip-to-target error readout on top of the base
    simulator, and starts recording so the run can be exported to a ``.sklog.npz``.
    """

    def __init__(self, scenario: Scenario, *, stiffness: float = _DRAG_STIFFNESS) -> None:
        """Build the reach GUI for a loaded scenario."""
        super().__init__(
            scenario.skeleton,
            controller=scenario.controller,
            target=scenario.task.target,
            stiffness=stiffness,
        )
        self.setWindowTitle("Skelarm Reach")

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

        self.status_label = QLabel()
        self.status_label.setWordWrap(True)
        self.add_control(self.status_label)

        self.start_recording()
        self._update_status()

    def step(self) -> None:
        """Advance one tick, then refresh the target-error readout."""
        super().step()
        self._update_status()

    def reset(self) -> None:
        """Reset the run and refresh the readout."""
        super().reset()
        self._update_status()

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
            self, "Export state log", "reach.sklog.npz", "State log (*.npz);;TOML (*.toml)"
        )
        if path:
            if Path(path).suffix == ".toml":
                self.state_log.export_toml(path)
            else:
                self.state_log.save(path)

    def _update_status(self) -> None:
        """Show the target, the tip, and the tip-to-target error."""
        tip = self.skeleton.links[-1]
        target = self.canvas.target
        assert target is not None  # always set for a reach scenario
        error = float(np.hypot(tip.xe - target[0], tip.ye - target[1]))
        self.status_label.setText(
            f"Target: ({target[0]:.3f}, {target[1]:.3f}) m\n"
            f"Tip: ({tip.xe:.3f}, {tip.ye:.3f}) m\n"
            f"Error: {error * 1000:.1f} mm"
        )


def build_parser() -> argparse.ArgumentParser:
    """Build the command-line argument parser for the tool."""
    parser = argparse.ArgumentParser(description="Interactive (or headless) skelarm reaching simulator.")
    parser.add_argument("config", type=Path, help="path to a combined scenario TOML config")
    parser.add_argument(
        "--save",
        type=Path,
        default=None,
        metavar="PATH",
        help="run headlessly (no GUI) and write the recorded run to this .sklog.npz",
    )
    parser.add_argument(
        "--duration", type=float, default=None, help="override the task's simulated duration (s; --save only)"
    )
    parser.add_argument(
        "--stiffness", type=float, default=_DRAG_STIFFNESS, help="mouse-drag force per meter (N/m, GUI only)"
    )
    parser.add_argument(
        "--initial", type=Path, default=None, help="TOML file whose [initial] table overrides the base initial pose"
    )
    parser.add_argument(
        "--pose", default=None, help="initial joint angles in degrees, e.g. 20,45,60,30 (overrides --initial)"
    )
    parser.add_argument("--task", type=Path, default=None, help="TOML file whose [task] table overrides the base task")
    parser.add_argument(
        "--controller",
        type=Path,
        default=None,
        help="TOML file whose [controller] table overrides the base controller",
    )
    return parser


def _read_toml(path: Path) -> dict[str, Any]:
    """Parse a TOML file into a dict."""
    with path.open("rb") as f:
        return tomllib.load(f)


def _read_section(path: Path, section: str) -> dict[str, Any]:
    """Read a single ``[section]`` table from an override TOML file."""
    if not path.exists():
        msg = f"override file not found: {path}"
        raise FileNotFoundError(msg)
    data = _read_toml(path)
    if section not in data:
        msg = f"no [{section}] section in {path}"
        raise ValueError(msg)
    return dict(data[section])


def _apply_overrides(
    config: dict[str, Any],
    *,
    initial: Path | None,
    pose: str | None,
    task: Path | None,
    controller: Path | None,
) -> None:
    """Overwrite the config's ``[initial]`` / ``[task]`` / ``[controller]`` tables in place.

    ``--initial`` replaces the whole ``[initial]`` table; ``--pose`` then overrides
    just its ``q`` (degrees), matching the kinematics/dynamics tools.
    """
    if task is not None:
        config["task"] = _read_section(task, "task")
    if controller is not None:
        config["controller"] = _read_section(controller, "controller")
    if initial is not None:
        config["initial"] = _read_section(initial, "initial")
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
    data = _read_toml(Path(config))
    _apply_overrides(
        data,
        initial=None if initial is None else Path(initial),
        pose=pose,
        task=None if task is None else Path(task),
        controller=None if controller is None else Path(controller),
    )
    return scenario_from_config(data)


def run_reach(
    config: str | Path,
    *,
    output: str | Path | None = None,
    duration: float | None = None,
    initial: str | Path | None = None,
    pose: str | None = None,
    task: str | Path | None = None,
    controller: str | Path | None = None,
) -> Path:
    """Headlessly simulate the planned reach (with optional overrides) and save the log.

    Parameters
    ----------
    config : str | Path
        Path to the combined scenario TOML config.
    output : str | Path | None, optional
        Output ``.sklog.npz`` path; defaults to the config path with that suffix.
    duration : float | None, optional
        Override the task's simulated duration (seconds).
    initial, pose, task, controller
        Section overrides; see :func:`build_scenario`.

    Returns
    -------
    Path
        The path the state log was written to.
    """
    scenario = build_scenario(config, initial=initial, pose=pose, task=task, controller=controller)
    log = run_scenario(scenario, duration=duration)

    out_path = Path(output) if output is not None else Path(config).with_suffix(".sklog.npz")
    log.save(out_path)

    tip = scenario.skeleton.clone()
    tip.q = log.channel("q")[-1]
    endpoint = np.array([tip.links[-1].xe, tip.links[-1].ye])
    error = float(np.linalg.norm(endpoint - scenario.task.target))
    print(f"reach: {type(scenario.controller).__name__} -> tip ({endpoint[0]:.3f}, {endpoint[1]:.3f}) m")
    print(f"target ({scenario.task.target[0]:.3f}, {scenario.task.target[1]:.3f}) m, error {error * 1000:.2f} mm")
    print(f"wrote {len(log)} frames to {out_path}")
    return out_path


def main() -> None:
    """Parse arguments and either run headlessly (``--save``) or open the GUI."""
    parser = build_parser()
    args = parser.parse_args()
    if not args.config.exists():
        parser.error(f"config file not found: {args.config}")

    if args.save is not None:
        try:
            run_reach(
                args.config,
                output=args.save,
                duration=args.duration,
                initial=args.initial,
                pose=args.pose,
                task=args.task,
                controller=args.controller,
            )
        except (FileNotFoundError, ValueError) as exc:
            parser.error(str(exc))
        return

    try:
        scenario = build_scenario(
            args.config, initial=args.initial, pose=args.pose, task=args.task, controller=args.controller
        )
    except (FileNotFoundError, ValueError) as exc:
        parser.error(str(exc))

    app = QApplication(sys.argv)
    window = ReachSimulator(scenario, stiffness=args.stiffness)
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
