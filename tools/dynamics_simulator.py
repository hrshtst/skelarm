"""
Interactive dynamics simulator tool for skelarm.

Load a robot arm from a TOML config given on the command line and simulate it in
real time under zero-torque control. Press and drag the left mouse button in the
canvas to apply a spring-like external force at the tip (drawn as a red arrow).
On top of the base :class:`~skelarm.SkelarmSimulator` this tool adds pause/resume,
single-step, reset, a live viscous-friction spin box (joint damping that dissipates
energy), a status panel (kinetic energy and tip position/speed), and an optional
tip-trajectory plot shown when the GUI closes.

Usage::

    uv run python tools/dynamics_simulator.py path/to/robot.toml
    uv run python tools/dynamics_simulator.py robot.toml --friction 0.2 --show-com
    uv run python tools/dynamics_simulator.py robot.toml --stiffness 0.2 --pose 20,45,60,30
    uv run python tools/dynamics_simulator.py robot.toml --initial pose.toml --no-plot
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np
from PyQt6.QtWidgets import QApplication, QCheckBox, QDoubleSpinBox, QFileDialog, QLabel, QPushButton

from skelarm import (
    SkelarmSimulator,
    Skeleton,
    compute_endpoint_velocity,
    compute_kinetic_energy,
)

if TYPE_CHECKING:
    from numpy.typing import NDArray

_FRICTION_MAX = 10.0  # upper bound for the live viscous-friction spin box (N·m·s/rad)


class DynamicsSimulator(SkelarmSimulator):
    """A SkelarmSimulator with playback controls, a status panel, and trajectory recording.

    Adds pause/resume, single-step (while paused), reset, a live viscous-friction
    spin box, a readout of kinetic energy and tip position/speed, and recording of
    the tip trajectory (see :attr:`trajectory`) for plotting after the GUI closes.
    """

    def __init__(
        self, skeleton: Skeleton, *, stiffness: float | None = None, friction: float = 0.0, enforce_limits: bool = True
    ) -> None:
        """Build the simulator controls on top of the base simulator."""
        if stiffness is None:
            super().__init__(skeleton, friction=friction, enforce_limits=enforce_limits)
        else:
            super().__init__(skeleton, stiffness=stiffness, friction=friction, enforce_limits=enforce_limits)

        self._trajectory_x: list[float] = []
        self._trajectory_y: list[float] = []

        self.pause_button = QPushButton("Pause")
        self.pause_button.clicked.connect(self._on_pause_toggled)
        self.add_control(self.pause_button)

        self.step_button = QPushButton("Step")
        self.step_button.setEnabled(False)  # only meaningful while paused
        self.step_button.clicked.connect(self._on_single_step)
        self.add_control(self.step_button)

        self.reset_button = QPushButton("Reset")
        self.reset_button.clicked.connect(self.reset)
        self.add_control(self.reset_button)

        self.add_control(QLabel("Viscous friction (N·m·s/rad)"))
        self.friction_spin = QDoubleSpinBox()
        self.friction_spin.setDecimals(3)
        self.friction_spin.setRange(0.0, _FRICTION_MAX)
        self.friction_spin.setSingleStep(0.01)
        self.friction_spin.setValue(self.friction)
        self.friction_spin.valueChanged.connect(self._on_friction_changed)
        self.add_control(self.friction_spin)

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

        self.start_recording()  # record by default; the checkbox toggles it
        self._record_point()  # seed the trajectory with the starting tip position
        self._update_status()

    @property
    def trajectory(self) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """The recorded tip trajectory as ``(xs, ys)`` arrays."""
        return np.array(self._trajectory_x), np.array(self._trajectory_y)

    def step(self) -> None:
        """Advance one tick, then record the tip and refresh the status panel."""
        super().step()
        self._record_point()
        self._update_status()

    def reset(self) -> None:
        """Restore the initial state, clear the trajectory, and refresh the status panel."""
        super().reset()
        self._trajectory_x.clear()
        self._trajectory_y.clear()
        self._record_point()
        self._update_status()

    def export(self, path: str | Path) -> None:
        """Write the recorded state log to ``path`` (``.toml`` exports TOML, else ``.npz``).

        Raises
        ------
        RuntimeError
            If recording was never started, so there is nothing to export.
        """
        if self.state_log is None:
            msg = "nothing to export: recording has not been started"
            raise RuntimeError(msg)
        if Path(path).suffix == ".toml":
            self.state_log.export_toml(path)
        else:
            self.state_log.save(path)

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
            self.export(path)

    def show_trajectory_plot(self) -> None:
        """Plot the final pose and the recorded tip trajectory with Matplotlib.

        Does nothing if fewer than two points were recorded (no visible motion).
        Intended to be called after the Qt event loop exits.
        """
        if len(self._trajectory_x) < 2:  # noqa: PLR2004
            return
        import matplotlib.pyplot as plt

        from skelarm import draw_skeleton, plot_trajectory

        xs, ys = self.trajectory
        _, ax = plt.subplots()
        draw_skeleton(ax, self.skeleton, title="Final pose and tip trajectory")
        plot_trajectory(ax, xs, ys, title=None)
        plt.show()

    def _record_point(self) -> None:
        """Append the current tip position to the trajectory."""
        tip = self.skeleton.links[-1]
        self._trajectory_x.append(tip.xe)
        self._trajectory_y.append(tip.ye)

    def _on_pause_toggled(self) -> None:
        """Toggle between running and paused, updating the buttons accordingly."""
        if self.running:
            self.pause()
            self.pause_button.setText("Resume")
            self.step_button.setEnabled(True)
        else:
            self.resume()
            self.pause_button.setText("Pause")
            self.step_button.setEnabled(False)

    def _on_single_step(self) -> None:
        """Advance a single tick while paused."""
        if not self.running:
            self.step()

    def _on_friction_changed(self, value: float) -> None:
        """Apply the live viscous friction coefficient to the simulation."""
        self.friction = value

    def _update_status(self) -> None:
        """Refresh the status label with kinetic energy and tip position/speed."""
        tip = self.skeleton.links[-1]
        velocity = compute_endpoint_velocity(self.skeleton)
        speed = float(np.hypot(velocity[0], velocity[1]))
        energy = compute_kinetic_energy(self.skeleton)
        self.status_label.setText(
            f"Tip: ({tip.xe:.3f}, {tip.ye:.3f}) m\nTip speed: {speed:.3f} m/s\nKinetic energy: {energy:.4g} J"
        )


def build_parser() -> argparse.ArgumentParser:
    """Build the command-line argument parser for the tool."""
    parser = argparse.ArgumentParser(description="Interactive real-time dynamics simulator for a skelarm robot.")
    parser.add_argument("config", type=Path, help="path to the robot TOML config to load")
    parser.add_argument(
        "--stiffness", type=float, default=None, help="drag stiffness in N/m (default: library default)"
    )
    parser.add_argument(
        "--friction",
        type=float,
        default=0.0,
        help="joint viscous friction coefficient in N·m·s/rad (default: 0, frictionless)",
    )
    parser.add_argument("--show-com", action="store_true", help="overlay each link's center of mass at startup")
    parser.add_argument("--initial", type=Path, default=None, help="TOML file with an [initial] table to apply")
    parser.add_argument(
        "--pose",
        default=None,
        help="initial joint angles in degrees, e.g. 20,45,60,30 (overrides --initial)",
    )
    parser.add_argument("--no-plot", action="store_true", help="do not plot the tip trajectory when the GUI closes")
    parser.add_argument(
        "--no-joint-limits",
        action="store_true",
        help="do not enforce joint limits in the dynamics (limits then apply to kinematics only)",
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


def main() -> None:
    """Parse arguments, build the simulator, and run the interactive application."""
    parser = build_parser()
    args = parser.parse_args()
    try:
        skeleton = load_skeleton(args)
    except (FileNotFoundError, ValueError) as exc:
        parser.error(str(exc))

    app = QApplication(sys.argv)
    simulator = DynamicsSimulator(
        skeleton, stiffness=args.stiffness, friction=args.friction, enforce_limits=not args.no_joint_limits
    )
    if args.show_com:
        simulator.com_checkbox.setChecked(True)

    simulator.show()
    app.exec()

    if not args.no_plot:
        simulator.show_trajectory_plot()


if __name__ == "__main__":
    main()
