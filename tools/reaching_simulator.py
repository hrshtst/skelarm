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

    uv run python tools/reaching_simulator.py examples/reach.toml                       # interactive GUI
    uv run python tools/reaching_simulator.py examples/reach.toml --controller pd.toml  # GUI, other controller
    uv run python tools/reaching_simulator.py examples/reach.toml --save run.sklog.npz  # headless batch run
    uv run python tools/reaching_simulator.py examples/reach.toml --save far.sklog.npz --task far.toml
    uv run python tools/player.py run.sklog.npz                            # replay a saved run
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np
from PyQt6.QtWidgets import QApplication, QLabel

from skelarm import run_scenario

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # allow `tools.` imports when run as a script
from tools._scenario_cli import (
    ScenarioSimulator,
    add_override_arguments,
    build_scenario,
    resolve_enforce_limits,
)

if TYPE_CHECKING:
    from skelarm.scenario import Scenario

_DRAG_STIFFNESS = 20.0  # N/m for the mouse drag; firm enough to perturb a controlled arm


class ReachSimulator(ScenarioSimulator):
    """Interactive reaching GUI: a controller drives the arm; the mouse adds tip forces.

    Adds a tip-to-target error readout on top of the shared scenario simulator (which
    provides the reset / record / export controls and embeds the task config).
    """

    def __init__(self, scenario: Scenario, *, stiffness: float = _DRAG_STIFFNESS, enforce_limits: bool = True) -> None:
        """Build the reach GUI for a loaded scenario."""
        super().__init__(scenario, stiffness=stiffness, enforce_limits=enforce_limits)
        self.setWindowTitle("Skelarm Reach")
        self.status_label = QLabel()
        self.status_label.setWordWrap(True)
        self.add_control(self.status_label)
        self._update_status()

    def step(self) -> None:
        """Advance one tick, then refresh the target-error readout."""
        super().step()
        self._update_status()

    def reset(self) -> None:
        """Reset the run and refresh the readout."""
        super().reset()
        self._update_status()

    def _update_status(self) -> None:
        """Show the target (with its label), the tip, the error, and success."""
        tip = self.skeleton.links[-1]
        target = self.canvas.target
        assert target is not None  # always set for a reach scenario
        error = float(np.hypot(tip.xe - target[0], tip.ye - target[1]))
        name = f" “{self._task.label}”" if self._task.label else ""
        error_line = f"Error: {error * 1000:.1f} mm"
        if self._task.tolerance is not None:
            within = error <= self._task.tolerance
            error_line += "  ✓ reached" if within else f"  (tol {self._task.tolerance * 1000:.0f} mm)"
        self.status_label.setText(
            f"Target{name}: ({target[0]:.3f}, {target[1]:.3f}) m\nTip: ({tip.xe:.3f}, {tip.ye:.3f}) m\n{error_line}"
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
    add_override_arguments(parser)
    return parser


def run_reach(
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
    enforce_limits : bool | None, optional
        Override the joint-limit hard stop; ``None`` uses the scenario's
        ``[task].enforce_limits``. The resolved value is embedded for reproducible re-runs.

    Returns
    -------
    Path
        The path the state log was written to.
    """
    scenario = build_scenario(config, initial=initial, pose=pose, task=task, controller=controller)
    log = run_scenario(scenario, duration=duration, enforce_limits=enforce_limits)

    out_path = Path(output) if output is not None else Path(config).with_suffix(".sklog.npz")
    log.save(out_path)

    tip = scenario.skeleton.clone()
    tip.q = log.channel("q")[-1]
    endpoint = np.array([tip.links[-1].xe, tip.links[-1].ye])
    target = scenario.task.require_target()
    error = float(np.linalg.norm(endpoint - target))
    print(f"reach: {type(scenario.controller).__name__} -> tip ({endpoint[0]:.3f}, {endpoint[1]:.3f}) m")
    print(f"target ({target[0]:.3f}, {target[1]:.3f}) m, error {error * 1000:.2f} mm")
    print(f"wrote {len(log)} frames to {out_path}")
    return out_path


def main() -> None:
    """Parse arguments and either run headlessly (``--save``) or open the GUI."""
    parser = build_parser()
    args = parser.parse_args()
    if not args.config.exists():
        parser.error(f"config file not found: {args.config}")

    # The CLI flag overrides the config; absent, the scenario's [task].enforce_limits applies.
    enforce_override = False if args.no_joint_limits else None

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
                enforce_limits=enforce_override,
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
    window = ReachSimulator(scenario, stiffness=args.stiffness, enforce_limits=resolve_enforce_limits(args, scenario))
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
