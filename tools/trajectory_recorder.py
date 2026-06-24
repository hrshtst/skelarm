"""
Interactive joint-trajectory recorder for skelarm.

Teach a motion by grabbing the robot's tip with the mouse and dragging it; the
recorded joint trajectory (and tip path) is saved as a ``.sklog.npz`` state log that
replays with ``tools/player.py``. The user teaches in task space ``(x, y)``; two
modes turn that into per-joint angles:

  - ``ik``       : solve inverse kinematics each refresh so the tip tracks the cursor.
  - ``dynamics`` : apply a spring force at the tip and integrate forward dynamics
                   (with viscous friction), like ``tools/dynamics_simulator.py``.

Recording starts when you first grab the tip (``t=0``) and stops at the max duration,
the **Finish** button, or window close. The logger samples at the configured rate,
independent of the GUI/sim update rate. An optional ``[task]`` target is drawn, and a
plot of the recorded motion is shown afterward (unless ``--no-plot``).

Usage::

    uv run python tools/trajectory_recorder.py examples/four_dof_robot.toml
    uv run python tools/trajectory_recorder.py robot.toml --mode dynamics --sample-rate 100
    uv run python tools/trajectory_recorder.py robot.toml --duration 15 --output run.sklog.npz
    uv run python tools/player.py teach.sklog.npz   # replay the recorded trajectory
"""

from __future__ import annotations

import argparse
import sys
import tomllib
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np
from PyQt6.QtCore import QTimer
from PyQt6.QtGui import QCloseEvent, QColor
from PyQt6.QtWidgets import QApplication, QHBoxLayout, QLabel, QMainWindow, QPushButton, QVBoxLayout, QWidget

from skelarm import (
    Skeleton,
    StateLog,
    Task,
    compute_inverse_kinematics,
    compute_jacobian,
    integrate_with_limits,
)
from skelarm.simulator import SimulatorCanvas

if TYPE_CHECKING:
    from numpy.typing import NDArray

_TIMER_MS = 20  # GUI refresh / update period (ms)
_SUBSTEPS = 4  # dynamics-mode physics substeps per refresh tick
_DRAG_STIFFNESS = 30.0  # N/m for the mouse drag (dynamics mode)
_FRICTION = 0.2  # joint viscous friction (dynamics mode; "some by default")
_SAMPLE_RATE = 50.0  # logger sampling rate (Hz)
_DURATION = 10.0  # max recording duration (s)


class RecorderWindow(QMainWindow):
    """Teach and record a joint trajectory by dragging the robot's tip.

    Recording auto-starts on the first grab and samples at ``sample_rate`` Hz until
    ``duration`` seconds elapse, the **Finish** button is pressed, or the window is
    closed. In ``ik`` mode the tip tracks the cursor via inverse kinematics each
    refresh; in ``dynamics`` mode the drag applies a tip force integrated under
    forward dynamics with viscous friction.
    """

    def __init__(
        self,
        skeleton: Skeleton,
        *,
        mode: str = "ik",
        sample_rate: float = _SAMPLE_RATE,
        duration: float = _DURATION,
        output: str | Path = "teach.sklog.npz",
        method: str = "lm_sugihara",
        stiffness: float = _DRAG_STIFFNESS,
        friction: float = _FRICTION,
        task: Task | None = None,
        show_com: bool = False,
        enforce_limits: bool = True,
    ) -> None:
        """Build the recorder window."""
        super().__init__()
        self.skeleton = skeleton
        self._mode = mode
        self._sample_dt = 1.0 / sample_rate
        self._duration = duration
        self._output = Path(output)
        self._method = method
        self._stiffness = stiffness
        self._friction = friction
        self._task = task
        # When limits are not enforced in the dynamics, the hard stop is disabled and
        # joint limits apply only to the kinematic (IK) path, which always clamps.
        self._lower = (
            np.array([link.prop.qmin for link in skeleton.links[1:]], dtype=np.float64) if enforce_limits else None
        )
        self._upper = (
            np.array([link.prop.qmax for link in skeleton.links[1:]], dtype=np.float64) if enforce_limits else None
        )

        self.time = 0.0
        self._recording = False
        self._finished = False
        self._saved = False
        self._next_sample = 0.0
        self.log = self._new_log()

        self.canvas = SimulatorCanvas(skeleton)
        self.canvas.show_com = show_com
        self.canvas.show_drag_arrow = mode == "dynamics"  # no force cue for the kinematic IK drag
        reach = sum(link.prop.length for link in skeleton.links)
        self.canvas.grab_radius = max(0.12 * reach, 0.05)  # grab near the tip
        if task is not None and task.target is not None:
            self.canvas.target = np.asarray(task.target, dtype=np.float64)
            self.canvas.target_color = QColor(task.color)
            self.canvas.target_tolerance = task.tolerance

        self.setWindowTitle("Skelarm Trajectory Recorder")
        self.resize(1024, 768)
        central = QWidget()
        self.setCentralWidget(central)
        layout = QHBoxLayout(central)
        layout.addWidget(self.canvas, stretch=3)

        panel = QWidget()
        controls = QVBoxLayout(panel)
        controls.addWidget(QLabel(f"<b>Trajectory recorder</b> — {mode} mode"))
        hint = QLabel("Grab the tip (left-drag) to teach a motion. Recording starts on the first grab.")
        hint.setWordWrap(True)
        controls.addWidget(hint)
        self.status_label = QLabel("waiting for the first grab…")
        self.status_label.setWordWrap(True)
        controls.addWidget(self.status_label)
        self.finish_button = QPushButton("Finish")
        self.finish_button.clicked.connect(self._finish)
        controls.addWidget(self.finish_button)
        controls.addStretch()
        layout.addWidget(panel, stretch=1)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self.tick)
        self._timer.start(_TIMER_MS)

    @property
    def saved(self) -> bool:
        """Whether a recording was saved to the output path."""
        return self._saved

    @property
    def finished(self) -> bool:
        """Whether recording has stopped (max duration, Finish, or close)."""
        return self._finished

    def _new_log(self) -> StateLog:
        """Start a fresh state log with the channels for the active mode."""
        joints = [f"j{i + 1}" for i in range(self.skeleton.num_joints)]
        channel_meta: dict[str, dict[str, object]] = {
            "q": {"unit": "rad", "label": "joint angle", "columns": joints},
            "tip": {"unit": "m", "label": "tip position", "columns": ["x", "y"]},
        }
        if self._mode == "dynamics":
            channel_meta["dq"] = {"unit": "rad/s", "label": "joint velocity", "columns": joints}
            channel_meta["ext_force"] = {"unit": "N", "label": "external tip force", "columns": ["fx", "fy"]}
        return StateLog(self.skeleton, producer="trajectory_recorder", channel_meta=channel_meta)

    def tick(self) -> None:
        """Advance one refresh tick, sampling and rendering as needed."""
        if self._finished:
            return
        if not self._recording:
            if self.canvas.drag_point is None:
                return  # idle until the first grab
            self._begin()
        if self._mode == "ik":
            self._step_ik()
        else:
            self._step_dynamics()
        self._refresh()
        if self.time >= self._duration:
            self._finish()

    def _begin(self) -> None:
        """Start recording at the current pose (t = 0)."""
        self._recording = True
        self.time = 0.0
        self.log = self._new_log()
        self._record(0.0)
        self._next_sample = self._sample_dt

    def _step_ik(self) -> None:
        """Track the cursor with IK at the refresh rate, then sample."""
        target = self.canvas.drag_point
        if target is not None:
            compute_inverse_kinematics(
                self.skeleton, np.asarray(target, dtype=np.float64), method=self._method, q0=self.skeleton.q
            )
        self.time += _TIMER_MS / 1000.0
        self._collect()

    def _step_dynamics(self) -> None:
        """Integrate forward dynamics under the tip force, sampling each substep."""
        dt = _TIMER_MS / 1000.0 / _SUBSTEPS
        for _ in range(_SUBSTEPS):
            tau = compute_jacobian(self.skeleton).T @ self.canvas.external_force(self._stiffness)
            tau = tau - self._friction * self.skeleton.dq
            integrate_with_limits(self.skeleton, tau, dt, self._lower, self._upper)
            self.time += dt
            self._collect()

    def _collect(self) -> None:
        """Record a frame for each sample boundary reached since the last one."""
        while self._next_sample <= self.time + 1e-9:
            self._record(self._next_sample)
            self._next_sample += self._sample_dt

    def _record(self, t: float) -> None:
        """Append one frame at time ``t``: joint angles, tip, and (dynamics) dq / force."""
        tip = self.skeleton.links[-1]
        channels: dict[str, NDArray[np.float64]] = {
            "q": self.skeleton.q,
            "tip": np.array([tip.xe, tip.ye], dtype=np.float64),
        }
        if self._mode == "dynamics":
            channels["dq"] = self.skeleton.dq
            channels["ext_force"] = self.canvas.external_force(self._stiffness)
        self.log.record(t, **channels)

    def _refresh(self) -> None:
        """Repaint the arm and update the status readout."""
        self.canvas.update_skeleton()
        self.status_label.setText(f"recording…  t = {self.time:.2f} / {self._duration:.1f} s,  {len(self.log)} samples")

    def _stop_and_save(self) -> None:
        """Stop recording and save the log (idempotent)."""
        if self._finished:
            return
        self._finished = True
        self._timer.stop()
        if self._recording and len(self.log) > 1:
            self.log.save(self._output)
            self._saved = True
            print(f"wrote {len(self.log)} samples to {self._output}")

    def _finish(self) -> None:
        """Finish recording (Finish button / max duration) and close the window."""
        self._stop_and_save()
        self.close()

    def closeEvent(self, a0: QCloseEvent | None) -> None:  # noqa: N802
        """Save the recording when the window is closed."""
        self._stop_and_save()
        super().closeEvent(a0)

    def show_plot(self) -> None:
        """Plot the taught tip path, final pose, and joint angles (blocking)."""
        import matplotlib.pyplot as plt

        from skelarm import draw_skeleton, plot_trajectory

        tip = self.log.channel("tip")
        q = self.log.channel("q")
        times = self.log.times
        _, (ax_path, ax_q) = plt.subplots(1, 2, figsize=(11, 5))
        draw_skeleton(ax_path, self.skeleton, title="Recorded trajectory")
        plot_trajectory(ax_path, tip[:, 0], tip[:, 1], title=None)
        if self._task is not None and self._task.target is not None:
            target = np.asarray(self._task.target, dtype=np.float64)
            ax_path.plot(
                target[0],
                target[1],
                marker="*",
                color=self._task.color,
                markersize=14,
                linestyle="none",
                label="target",
            )
            ax_path.legend()
        for j in range(q.shape[1]):
            ax_q.plot(times, np.rad2deg(q[:, j]), label=f"j{j + 1}")
        ax_q.set_xlabel("time [s]")
        ax_q.set_ylabel("joint angle [deg]")
        ax_q.grid(visible=True)
        ax_q.legend(loc="best", fontsize="small")
        plt.tight_layout()
        plt.show()


def build_parser() -> argparse.ArgumentParser:
    """Build the command-line argument parser for the tool."""
    parser = argparse.ArgumentParser(description="Interactively teach and record a joint trajectory.")
    parser.add_argument("config", type=Path, help="path to a robot TOML config (optional [initial] / [task])")
    parser.add_argument("--mode", choices=("ik", "dynamics"), default="ik", help="teaching mode (default: ik)")
    parser.add_argument(
        "--output", type=Path, default=Path("teach.sklog.npz"), help="output .sklog.npz (default: teach.sklog.npz)"
    )
    parser.add_argument(
        "--sample-rate", type=float, default=_SAMPLE_RATE, help="logger sampling rate in Hz (default: 50)"
    )
    parser.add_argument(
        "--duration", type=float, default=_DURATION, help="max recording duration in seconds (default: 10)"
    )
    parser.add_argument("--method", default="lm_sugihara", help="IK solver method (ik mode; default: lm_sugihara)")
    parser.add_argument(
        "--stiffness", type=float, default=_DRAG_STIFFNESS, help="drag force per meter in N/m (dynamics mode)"
    )
    parser.add_argument(
        "--friction", type=float, default=_FRICTION, help="joint viscous friction in N*m*s/rad (dynamics mode)"
    )
    parser.add_argument("--initial", type=Path, default=None, help="TOML file with an [initial] table to apply")
    parser.add_argument(
        "--pose", default=None, help="initial joint angles in degrees, e.g. 20,45,60,30 (overrides --initial)"
    )
    parser.add_argument("--task", type=Path, default=None, help="TOML file whose [task] target is drawn")
    parser.add_argument("--show-com", action="store_true", help="overlay each link's center of mass")
    parser.add_argument("--no-plot", action="store_true", help="do not plot the trajectory after recording")
    parser.add_argument(
        "--no-joint-limits",
        action="store_true",
        help="do not enforce joint limits in the dynamics (dynamics mode; limits still clamp the IK path)",
    )
    return parser


def load_setup(args: argparse.Namespace) -> tuple[Skeleton, Task | None]:
    """Load the robot (with pose overrides) and an optional task target.

    Raises
    ------
    FileNotFoundError
        If the config, ``--initial``, or ``--task`` file does not exist.
    ValueError
        If ``--pose`` length mismatches the DOF or a ``--task`` file lacks ``[task]``.
    """
    config: Path = args.config
    if not config.exists():
        msg = f"config file not found: {config}"
        raise FileNotFoundError(msg)
    skeleton = Skeleton.from_toml(config)

    if args.initial is not None:
        if not args.initial.exists():
            msg = f"initial file not found: {args.initial}"
            raise FileNotFoundError(msg)
        skeleton.apply_initial_toml(args.initial)
    if args.pose is not None:
        pose = np.deg2rad([float(value) for value in args.pose.split(",")])
        if len(pose) != skeleton.num_joints:
            msg = f"--pose has {len(pose)} values but the arm has {skeleton.num_joints} joints"
            raise ValueError(msg)
        skeleton.q = pose

    task = None
    if args.task is not None:
        if not args.task.exists():
            msg = f"task file not found: {args.task}"
            raise FileNotFoundError(msg)
        with args.task.open("rb") as f:
            data = tomllib.load(f)
        if "task" not in data:
            msg = f"no [task] section in {args.task}"
            raise ValueError(msg)
        task = Task.from_dict(data["task"])
    else:
        with config.open("rb") as f:
            data = tomllib.load(f)
        if "task" in data:
            task = Task.from_dict(data["task"])
    return skeleton, task


def main() -> None:
    """Parse arguments, run the recorder, and plot the result."""
    parser = build_parser()
    args = parser.parse_args()
    try:
        skeleton, task = load_setup(args)
    except (FileNotFoundError, ValueError) as exc:
        parser.error(str(exc))

    app = QApplication(sys.argv)
    window = RecorderWindow(
        skeleton,
        mode=args.mode,
        sample_rate=args.sample_rate,
        duration=args.duration,
        output=args.output,
        method=args.method,
        stiffness=args.stiffness,
        friction=args.friction,
        task=task,
        show_com=args.show_com,
        enforce_limits=not args.no_joint_limits,
    )
    window.show()
    app.exec()

    if window.saved and not args.no_plot:
        window.show_plot()


if __name__ == "__main__":
    main()
