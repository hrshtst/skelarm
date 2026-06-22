"""
Replay and analysis tool for skelarm state logs.

Load a ``.sklog.npz`` recording (see :class:`skelarm.StateLog`) and replay the
robot motion: the arm is rebuilt from the embedded geometry and driven from the
recorded joint angles, so no dynamics are simulated. A timeline slider scrubs
frames, play/pause animates them at a chosen speed, and "Plot channels…" opens
Matplotlib plots of every recorded channel versus time for analysis (e.g. a
controller's tracking error) without re-running the simulation.

Usage::

    uv run python tools/replay.py run.sklog.npz
    uv run python tools/replay.py run.sklog.npz --show-com --speed 0.5
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
from PyQt6.QtCore import QSignalBlocker, Qt, QTimer
from PyQt6.QtWidgets import (
    QApplication,
    QCheckBox,
    QDoubleSpinBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QWidget,
)

from skelarm import SkelarmCanvas, StateLog, compute_forward_kinematics

_TIMER_MS = 20  # playback/render period in milliseconds


class PlaybackWindow(QMainWindow):
    """A timeline player for a recorded :class:`~skelarm.StateLog`.

    The arm is reconstructed from the log's embedded geometry and posed from the
    recorded ``q`` channel; scrubbing and playback are purely kinematic. The same
    window can open per-channel analysis plots via :meth:`build_channel_figure`.
    """

    def __init__(self, log: StateLog, *, show_com: bool = False, speed: float = 1.0) -> None:
        """Build the player for ``log``.

        Parameters
        ----------
        log : StateLog
            The recording to replay. It must contain a ``q`` channel.
        show_com : bool, optional
            Overlay each link's center of mass at startup.
        speed : float, optional
            Initial playback speed multiplier.

        Raises
        ------
        ValueError
            If the log has no ``q`` channel to drive the arm.
        """
        super().__init__()
        if "q" not in log.channel_names:
            msg = "log has no 'q' channel; cannot replay the arm"
            raise ValueError(msg)

        self.log = log
        self.skeleton = log.build_skeleton()
        self._q = log.channel("q")
        self._times = log.times
        self._n = len(log)
        self._frame = 0
        self._play_time = float(self._times[0]) if self._n else 0.0
        self._speed = speed

        self.canvas = SkelarmCanvas(self.skeleton)
        self.canvas.show_com = show_com
        self.setWindowTitle("Skelarm Replay")
        self.resize(1024, 768)

        central = QWidget()
        self.setCentralWidget(central)
        layout = QHBoxLayout(central)
        layout.addWidget(self.canvas, stretch=3)

        panel = QWidget()
        controls = QVBoxLayout(panel)
        controls.addWidget(QLabel(f"<b>Replay</b> — {log.producer or 'state log'}"))

        self.time_label = QLabel()
        time_font = self.time_label.font()
        time_font.setPointSize(time_font.pointSize() + 4)
        time_font.setBold(True)
        self.time_label.setFont(time_font)
        controls.addWidget(self.time_label)

        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(0, max(self._n - 1, 0))
        self.slider.valueChanged.connect(self.set_frame)
        controls.addWidget(self.slider)

        self.play_button = QPushButton("Play")
        self.play_button.clicked.connect(self._on_play_clicked)
        controls.addWidget(self.play_button)

        controls.addWidget(QLabel("Playback speed"))
        self.speed_spin = QDoubleSpinBox()
        self.speed_spin.setDecimals(2)
        self.speed_spin.setRange(0.1, 10.0)
        self.speed_spin.setSingleStep(0.1)
        self.speed_spin.setValue(speed)
        self.speed_spin.valueChanged.connect(self._on_speed_changed)
        controls.addWidget(self.speed_spin)

        self.com_checkbox = QCheckBox("Show center of mass")
        self.com_checkbox.setChecked(show_com)
        self.com_checkbox.toggled.connect(self._on_show_com_toggled)
        controls.addWidget(self.com_checkbox)

        self.plot_button = QPushButton("Plot channels…")
        self.plot_button.clicked.connect(self._on_plot_channels)
        controls.addWidget(self.plot_button)

        controls.addStretch()
        layout.addWidget(panel, stretch=1)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._on_timeout)
        self._show_frame(0)

    @property
    def frame(self) -> int:
        """The index of the frame currently shown."""
        return self._frame

    @property
    def is_playing(self) -> bool:
        """Whether the timeline is currently advancing."""
        return self._timer.isActive()

    @property
    def speed(self) -> float:
        """Playback speed multiplier (log seconds per real second)."""
        return self._speed

    @speed.setter
    def speed(self, value: float) -> None:
        self._speed = float(value)

    def set_frame(self, index: int) -> None:
        """Jump to ``index`` and sync the playback clock to that frame's time."""
        index = int(np.clip(index, 0, self._n - 1))
        self._play_time = float(self._times[index])
        self._show_frame(index)

    def advance(self, seconds: float) -> None:
        """Advance the playback clock by ``seconds`` (scaled by :attr:`speed`)."""
        if self._n <= 1:
            return
        self._play_time += seconds * self._speed
        if self._play_time >= self._times[-1]:
            self._play_time = float(self._times[-1])
            self._show_frame(self._n - 1)
            self.pause()
            return
        index = int(np.searchsorted(self._times, self._play_time, side="right") - 1)
        self._show_frame(max(index, 0))

    def play(self) -> None:
        """Start (or restart from the beginning) playback."""
        if self._frame >= self._n - 1:
            self.set_frame(0)
        self._timer.start(_TIMER_MS)
        self.play_button.setText("Pause")

    def pause(self) -> None:
        """Pause playback."""
        self._timer.stop()
        self.play_button.setText("Play")

    def build_channel_figure(self):  # noqa: ANN201  # matplotlib Figure (lazy import)
        """Build a Matplotlib figure with one time-series subplot per channel."""
        import matplotlib.pyplot as plt

        names = self.log.channel_names
        figure, axes = plt.subplots(len(names), 1, sharex=True, squeeze=False)
        times = self.log.times
        for axis, name in zip(axes[:, 0], names, strict=True):
            data = self.log.channel(name)
            meta = self.log.channel_meta.get(name, {})
            columns = meta.get("columns")
            if data.ndim == 1:
                axis.plot(times, data, label=name)
            else:
                for j in range(data.shape[1]):
                    label = columns[j] if columns and j < len(columns) else f"{name}[{j}]"
                    axis.plot(times, data[:, j], label=label)
            ylabel = meta.get("label", name)
            if meta.get("unit"):
                ylabel = f"{ylabel} [{meta['unit']}]"
            axis.set_ylabel(ylabel)
            axis.grid(visible=True)
            axis.legend(loc="best", fontsize="small")
        axes[-1, 0].set_xlabel("time [s]")
        figure.tight_layout()
        return figure

    def _show_frame(self, index: int) -> None:
        """Pose the arm to frame ``index`` and refresh the slider, label, and canvas."""
        index = int(np.clip(index, 0, self._n - 1))
        self._frame = index
        for link, value in zip(self.skeleton.links[1:], self._q[index], strict=True):
            link.q = float(value)
        compute_forward_kinematics(self.skeleton)
        self.canvas.update_skeleton()
        with QSignalBlocker(self.slider):
            self.slider.setValue(index)
        self._update_time_label()

    def _update_time_label(self) -> None:
        """Show the current time and frame index."""
        current = self._times[self._frame] if self._n else 0.0
        self.time_label.setText(f"t = {current:.2f} s   (frame {self._frame + 1}/{self._n})")

    def _on_timeout(self) -> None:
        """Advance one render tick of playback."""
        self.advance(_TIMER_MS / 1000.0)

    def _on_play_clicked(self) -> None:
        """Toggle play/pause."""
        if self.is_playing:
            self.pause()
        else:
            self.play()

    def _on_speed_changed(self, value: float) -> None:
        """Apply the speed spin box to playback."""
        self._speed = value

    def _on_show_com_toggled(self) -> None:
        """Toggle the center-of-mass overlay."""
        self.canvas.show_com = self.com_checkbox.isChecked()
        self.canvas.update_skeleton()

    def _on_plot_channels(self) -> None:
        """Open the per-channel analysis plots without blocking the player."""
        import matplotlib.pyplot as plt

        if not self.log.channel_names:
            return
        self.build_channel_figure()
        plt.show(block=False)


def build_parser() -> argparse.ArgumentParser:
    """Build the command-line argument parser for the tool."""
    parser = argparse.ArgumentParser(description="Replay and analyze a recorded skelarm state log.")
    parser.add_argument("logfile", type=Path, help="path to a .sklog.npz state log")
    parser.add_argument("--show-com", action="store_true", help="overlay each link's center of mass")
    parser.add_argument("--speed", type=float, default=1.0, help="initial playback speed multiplier (default: 1.0)")
    return parser


def main() -> None:
    """Parse arguments, load the log, and run the player."""
    parser = build_parser()
    args = parser.parse_args()
    if not args.logfile.exists():
        parser.error(f"log file not found: {args.logfile}")
    try:
        log = StateLog.load(args.logfile)
    except (OSError, ValueError, KeyError) as exc:
        parser.error(f"could not load {args.logfile}: {exc}")

    app = QApplication(sys.argv)
    try:
        window = PlaybackWindow(log, show_com=args.show_com, speed=args.speed)
    except ValueError as exc:
        parser.error(str(exc))
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
