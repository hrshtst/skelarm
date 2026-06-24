"""
Interactive multiple-target reaching simulator for skelarm.

Load a scenario whose ``[task]`` is ``multi_target_reaching`` (a list of candidate
targets, one active) driven by a reaching controller. The active target is filled; the
others are drawn as dashed rings. Press a **number key** ``1``..``N`` to switch the
active target live — the controller is retargeted and re-homed on the fly. Press/drag
the left mouse button to apply an external force at the tip, as in the reaching
simulator.

Switching live retargets the spring-damper *reaching* controllers (their target is a
settable property); trajectory-tracking controllers ignore the live switch (their plan
is fixed at build time).

Usage::

    uv run python tools/multi_target_simulator.py examples/multi_target.toml
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QColor
from PyQt6.QtWidgets import QApplication, QLabel

from skelarm import EndpointController, SkelarmSimulator, load_scenario
from skelarm.scenario import active_target_index, multi_target_specs

if TYPE_CHECKING:
    from PyQt6.QtGui import QKeyEvent

    from skelarm.scenario import Scenario

_DRAG_STIFFNESS = 20.0  # N/m for the mouse drag


class MultiTargetReachSimulator(SkelarmSimulator):
    """Reaching GUI over several candidate targets, with live number-key switching."""

    def __init__(self, scenario: Scenario, *, stiffness: float = _DRAG_STIFFNESS, enforce_limits: bool = True) -> None:
        """Build the multi-target reach GUI for a loaded scenario."""
        super().__init__(
            scenario.skeleton,
            controller=scenario.controller,
            target=scenario.task.target,
            target_color=scenario.task.color,
            target_tolerance=scenario.task.tolerance,
            stiffness=stiffness,
            enforce_limits=enforce_limits,
        )
        self._task = scenario.task
        self._specs = multi_target_specs(scenario.task)
        self._active = active_target_index(scenario.task)
        self.setWindowTitle("Skelarm Multi-Target Reach")

        hint = QLabel(f"Press 1-{len(self._specs)} to switch the active target.")
        hint.setWordWrap(True)
        self.add_control(hint)
        self._refresh_markers()

    @property
    def active_index(self) -> int:
        """The index of the currently active target."""
        return self._active

    def switch_to(self, index: int) -> None:
        """Make candidate ``index`` the active target and retarget the controller."""
        if not 0 <= index < len(self._specs):
            return
        self._active = index
        pos, label, color, tolerance = self._specs[index]
        self._task.target, self._task.label, self._task.color, self._task.tolerance = pos, label, color, tolerance
        self.canvas.target = np.asarray(pos, dtype=np.float64)
        self.canvas.target_color = QColor(color)
        self.canvas.target_tolerance = tolerance
        if isinstance(self._controller, EndpointController):
            self._controller.target = pos  # live retarget (settable property)
            self._controller.reset(self.skeleton)  # re-home any reference shaping
        self._refresh_markers()

    def _refresh_markers(self) -> None:
        """Redraw the inactive candidates as dashed rings."""
        self.canvas.secondary_targets = [
            (np.asarray(pos, dtype=np.float64), QColor(color))
            for i, (pos, _, color, _) in enumerate(self._specs)
            if i != self._active
        ]
        self.canvas.update()

    def keyPressEvent(self, a0: QKeyEvent | None) -> None:  # noqa: N802
        """Switch the active target on a number key ``1``..``9``."""
        if a0 is not None and Qt.Key.Key_1.value <= a0.key() <= Qt.Key.Key_9.value:
            self.switch_to(a0.key() - Qt.Key.Key_1.value)
            return
        super().keyPressEvent(a0)


def build_parser() -> argparse.ArgumentParser:
    """Build the command-line argument parser for the tool."""
    parser = argparse.ArgumentParser(description="Interactive multiple-target reaching simulator.")
    parser.add_argument("config", type=Path, help="multi-target scenario TOML")
    parser.add_argument("--stiffness", type=float, default=_DRAG_STIFFNESS, help="mouse-drag force per meter (N/m)")
    parser.add_argument(
        "--no-joint-limits",
        action="store_true",
        help="do not enforce joint limits in the dynamics (limits apply to kinematics only)",
    )
    return parser


def main() -> None:
    """Parse arguments and open the multi-target reaching GUI."""
    parser = build_parser()
    args = parser.parse_args()
    config: Path = args.config
    if not config.exists():
        parser.error(f"config file not found: {config}")

    app = QApplication(sys.argv)
    scenario = load_scenario(config)
    window = MultiTargetReachSimulator(scenario, stiffness=args.stiffness, enforce_limits=not args.no_joint_limits)
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
