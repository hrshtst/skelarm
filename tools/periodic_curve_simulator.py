"""
Interactive periodic-curve tracing simulator for skelarm.

Loads a scenario whose ``[task]`` is ``periodic_curve`` (a closed task-space curve traced
repeatedly) driven by a tracking controller, and opens a real-time GUI: the controller
drives the tip around the curve (drawn behind the arm) while you press and drag the left
mouse button to apply an external force at the tip as a disturbance. Record / Export the
run for replay. ``--initial`` / ``--pose`` / ``--task`` / ``--controller`` override config
sections; ``--save PATH`` runs headlessly and writes a replayable log.

Usage::

    uv run python tools/periodic_curve_simulator.py examples/periodic_curve.toml
    uv run python tools/periodic_curve_simulator.py examples/periodic_curve.toml --controller pd.toml
    uv run python tools/periodic_curve_simulator.py examples/periodic_curve.toml --save curve.sklog.npz
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import TYPE_CHECKING

from PyQt6.QtWidgets import QApplication

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # allow `tools.` imports when run as a script
from tools._scenario_cli import (
    ScenarioSimulator,
    add_override_arguments,
    build_scenario,
    resolve_enforce_limits,
    save_scenario_run,
)

if TYPE_CHECKING:
    from skelarm import Scenario

_DRAG_STIFFNESS = 20.0  # N/m for the mouse drag
_TASK_TYPE = "periodic_curve"


class CurveTraceSimulator(ScenarioSimulator):
    """Interactive periodic-curve tracing GUI (the reference curve is drawn behind the arm)."""

    def __init__(self, scenario: Scenario, *, stiffness: float = _DRAG_STIFFNESS, enforce_limits: bool = True) -> None:
        """Build the curve-tracing GUI for a loaded scenario."""
        super().__init__(scenario, stiffness=stiffness, enforce_limits=enforce_limits)
        self.setWindowTitle("Skelarm Curve Trace")


def build_parser() -> argparse.ArgumentParser:
    """Build the command-line argument parser for the tool."""
    parser = argparse.ArgumentParser(description="Interactive (or headless) periodic-curve tracing simulator.")
    parser.add_argument("config", type=Path, help="path to a periodic_curve scenario TOML config")
    parser.add_argument(
        "--save", type=Path, default=None, metavar="PATH", help="run headlessly and write the run to this .sklog.npz"
    )
    parser.add_argument("--duration", type=float, default=None, help="override the simulated duration (s; --save only)")
    parser.add_argument("--stiffness", type=float, default=_DRAG_STIFFNESS, help="mouse-drag force per meter (N/m)")
    add_override_arguments(parser)
    return parser


def main() -> None:
    """Parse arguments and either run headlessly (``--save``) or open the GUI."""
    parser = build_parser()
    args = parser.parse_args()
    if not args.config.exists():
        parser.error(f"config file not found: {args.config}")
    try:
        scenario = build_scenario(
            args.config, initial=args.initial, pose=args.pose, task=args.task, controller=args.controller
        )
    except (FileNotFoundError, ValueError) as exc:
        parser.error(str(exc))
    if scenario.task.type != _TASK_TYPE:
        parser.error(f"expected a {_TASK_TYPE!r} task, got {scenario.task.type!r}")

    enforce_limits = resolve_enforce_limits(args, scenario)
    if args.save is not None:
        save_scenario_run(scenario, args.save, duration=args.duration, enforce_limits=enforce_limits)
        return

    app = QApplication(sys.argv)
    window = CurveTraceSimulator(scenario, stiffness=args.stiffness, enforce_limits=enforce_limits)
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
