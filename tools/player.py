"""
Replay and analysis tool for skelarm state logs.

Load a ``.sklog.npz`` recording (see :class:`skelarm.StateLog`) and replay the
robot motion: the arm is rebuilt from the embedded geometry and driven from the
recorded joint angles, so no dynamics are simulated. A timeline slider scrubs
frames, play/pause animates them at a chosen speed, and "Plot channels…" opens
Matplotlib plots of every recorded channel versus time for analysis (e.g. a
controller's tracking error) without re-running the simulation. When the log
recorded an external tip force (``ext_force`` channel, as the dynamics simulator
does), it is drawn as a red arrow at the tip and toggled by "Show external force".

The replay can also be exported headlessly (no GUI window) to an ``.mp4`` video or an
animated ``.gif`` with ``--export``: each frame is rendered from the same canvas the
interactive player uses — task overlay, centers of mass, and external-force arrow
included — and encoded with ``imageio``.

Usage::

    uv run python tools/player.py run.sklog.npz
    uv run python tools/player.py run.sklog.npz --show-com --speed 0.5
    uv run python tools/player.py run.sklog.npz --export run.mp4          # headless mp4
    uv run python tools/player.py run.sklog.npz --export run.gif --fps 24  # headless gif
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path
from typing import TYPE_CHECKING, cast

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

from skelarm import SkelarmCanvas, StateLog, Task, compute_forward_kinematics

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # allow `tools.` imports when run as a script
from tools._scenario_cli import task_overlays

if TYPE_CHECKING:
    from collections.abc import Buffer

    from numpy.typing import NDArray

_TIMER_MS = 20  # playback/render period in milliseconds
_PANEL_WIDTH_PX = 300  # fixed side-panel width so the varying time/frame readout can't resize it
_EXPORT_FPS = 30.0  # default output frame rate for --export
_EXPORT_SIZE_PX = 800  # square frame size (px) for --export; a multiple of 16 keeps mp4 codecs happy
_EXPORT_SUFFIXES = (".mp4", ".gif")  # --export formats, selected from the output path extension


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
        # Recorded external tip force (N), shown as an arrow when present.
        self._force = log.channel("ext_force") if "ext_force" in log.channel_names else None
        self._show_force = self._force is not None

        self.canvas = SkelarmCanvas(self.skeleton)
        self.canvas.show_com = show_com
        if self._force is not None:
            self.canvas.force_scale = self._force_arrow_scale()

        # Reconstruct the task from the embedded config to draw its overlays (target /
        # active-target emphasis / periodic curve / reference trajectory).
        self._has_targets, self._has_reference = self._build_task_overlays()

        self.setWindowTitle("Skelarm Replay")
        self.resize(1024, 768)

        central = QWidget()
        self.setCentralWidget(central)
        layout = QHBoxLayout(central)
        layout.addWidget(self.canvas, stretch=3)

        panel = QWidget()
        panel.setFixedWidth(_PANEL_WIDTH_PX)  # keep a constant width; the time/frame readout won't resize it
        controls = QVBoxLayout(panel)
        self.controls_panel = panel  # exposed for sizing (fixed width) and tests
        controls.addWidget(QLabel(f"<b>Replay</b> — {log.producer or 'state log'}"))

        self.time_label = QLabel()
        self.time_label.setWordWrap(True)  # wrap rather than clip if the readout outgrows the fixed width
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

        # External-force arrow controls, shown only when the log recorded a force.
        self.force_checkbox = QCheckBox("Show external force")
        self.force_checkbox.setChecked(True)
        self.force_checkbox.toggled.connect(self._on_show_force_toggled)
        self.force_label = QLabel()
        if self._force is not None:
            controls.addWidget(self.force_checkbox)
            controls.addWidget(self.force_label)

        # Task-overlay toggles, shown only when the log embeds the matching data.
        self.target_checkbox = QCheckBox("Show target(s)")
        self.target_checkbox.setChecked(True)
        self.target_checkbox.toggled.connect(self._on_show_targets_toggled)
        if self._has_targets:
            controls.addWidget(self.target_checkbox)
        self.reference_checkbox = QCheckBox("Show reference")
        self.reference_checkbox.setChecked(True)
        self.reference_checkbox.toggled.connect(self._on_show_reference_toggled)
        if self._has_reference:
            controls.addWidget(self.reference_checkbox)

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

    def export(self, path: str | Path, *, fps: float = _EXPORT_FPS, size: int = _EXPORT_SIZE_PX) -> int:
        """Render the whole replay to a video / animated GIF on disk, no window shown.

        The motion is reconstructed from the log and drawn by the same canvas the
        interactive player uses, so the exported frames include the task overlay,
        the centers of mass, and any external-force arrow exactly as on screen. The
        output format is taken from ``path``'s extension (``.mp4`` or ``.gif``).
        Frames are resampled at ``fps`` over the recording's timeline (scaled by
        :attr:`speed`), so the file plays back at the chosen speed. Encoding streams
        frame by frame through ``imageio``; no per-frame images are left on disk.

        Parameters
        ----------
        path : str or pathlib.Path
            Output file. Its extension selects the format and must be ``.mp4`` or ``.gif``.
        fps : float, optional
            Output frame rate in frames per second (default: 30).
        size : int, optional
            Side length in pixels of the (square) rendered frame (default: 800).

        Returns
        -------
        int
            The number of frames written.

        Raises
        ------
        ValueError
            If ``path``'s extension is unsupported, ``fps`` is not positive, or the
            log has no frames.
        """
        import imageio.v2 as imageio

        path = Path(path)
        if path.suffix.lower() not in _EXPORT_SUFFIXES:
            msg = f"unsupported export format {path.suffix!r}; use one of {', '.join(_EXPORT_SUFFIXES)}"
            raise ValueError(msg)
        if fps <= 0:
            msg = f"fps must be positive, got {fps}"
            raise ValueError(msg)
        if self._n == 0:
            msg = "log has no frames to export"
            raise ValueError(msg)

        # Pin the canvas to an exact square: a plain resize() is overridden by the window
        # layout, leaving non-multiple-of-16 dimensions that ffmpeg would silently rescale.
        self.canvas.setFixedSize(size, size)
        times = self._times
        t0, t_end = float(times[0]), float(times[-1])
        span = t_end - t0
        speed = max(self._speed, 1e-9)  # guard against a zero/negative --speed
        # One output frame per 1/fps of real time; the log clock advances by `speed` per real second.
        n_frames = 1 if span <= 0 else int(np.floor(span / speed * fps)) + 1
        # The ffmpeg (mp4) backend takes a frame rate; the pillow (gif) backend takes a per-frame
        # duration in milliseconds and an infinite loop count.
        if path.suffix.lower() == ".mp4":
            writer = imageio.get_writer(path, fps=fps)
        else:
            writer = imageio.get_writer(path, duration=1000.0 / fps, loop=0)
        with writer:
            for k in range(n_frames):
                log_t = t0 + (k / fps) * speed
                index = int(np.clip(np.searchsorted(times, log_t, side="right") - 1, 0, self._n - 1))
                self._show_frame(index)
                writer.append_data(self._grab_frame_rgb())
        return n_frames

    def _grab_frame_rgb(self) -> NDArray[np.uint8]:
        """Grab the current canvas as an ``(H, W, 3)`` uint8 RGB array (offscreen-safe)."""
        from PyQt6.QtGui import QImage

        image = self.canvas.grab().toImage().convertToFormat(QImage.Format.Format_RGBA8888)
        height, width = image.height(), image.width()
        bits = image.constBits()
        assert bits is not None  # populated for a non-null grabbed image
        bits.setsize(height * image.bytesPerLine())
        # `bits` is a sip.voidptr exposing the buffer protocol at runtime; cast for the type checker.
        buffer = np.frombuffer(cast("Buffer", bits), dtype=np.uint8).reshape((height, image.bytesPerLine() // 4, 4))
        return np.ascontiguousarray(buffer[:, :width, :3])

    def _show_frame(self, index: int) -> None:
        """Pose the arm to frame ``index`` and refresh the slider, label, and canvas."""
        index = int(np.clip(index, 0, self._n - 1))
        self._frame = index
        for link, value in zip(self.skeleton.links[1:], self._q[index], strict=True):
            link.q = float(value)
        compute_forward_kinematics(self.skeleton)
        if self._force is not None:
            force = self._force[index]
            self.canvas.tip_force = force if self._show_force else None
            self.force_label.setText(f"Ext. force: {float(np.hypot(force[0], force[1])):.3g} N")
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

    def _on_show_force_toggled(self) -> None:
        """Toggle the external-force arrow overlay and redraw the current frame."""
        self._show_force = self.force_checkbox.isChecked()
        self._show_frame(self._frame)

    def _on_show_targets_toggled(self) -> None:
        """Toggle the task target markers and redraw."""
        self.canvas.show_overlay_targets = self.target_checkbox.isChecked()
        self.canvas.update_skeleton()

    def _on_show_reference_toggled(self) -> None:
        """Toggle the reference curve / trajectory overlay and redraw."""
        self.canvas.show_overlay_path = self.reference_checkbox.isChecked()
        self.canvas.update_skeleton()

    def _force_arrow_scale(self) -> float:
        """Meters per Newton so the largest recorded force spans ~40% of the arm's reach."""
        assert self._force is not None  # only called when a force channel is present
        magnitudes = np.hypot(self._force[:, 0], self._force[:, 1])
        peak = float(magnitudes.max()) if magnitudes.size else 0.0
        if peak <= 0.0:
            return 0.0
        reach = sum(link.prop.length for link in self.skeleton.links)
        return 0.4 * reach / peak

    def _build_task_overlays(self) -> tuple[bool, bool]:
        """Reconstruct the task from the embedded config and set the canvas overlays.

        Returns ``(has_targets, has_reference)`` so the GUI only adds the toggles whose
        data is present. Logs without an embedded task draw no overlays.
        """
        task_cfg = self.log.extra.get("source_config", {}).get("task")
        if not task_cfg:
            return False, False
        try:
            task = Task.from_dict(task_cfg)
        except (ValueError, KeyError):
            return False, False
        targets, path = task_overlays(task, self.skeleton)
        self.canvas.overlay_targets = targets
        self.canvas.overlay_path = path
        return bool(targets), path is not None

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
    parser.add_argument(
        "--export",
        type=Path,
        metavar="PATH",
        help="render the replay to PATH (.mp4 or .gif) headlessly instead of opening the GUI",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=_EXPORT_FPS,
        help=f"output frame rate for --export (default: {_EXPORT_FPS:g})",
    )
    return parser


def main() -> None:
    """Parse arguments, load the log, and run the player (or export it headlessly)."""
    parser = build_parser()
    args = parser.parse_args()
    if not args.logfile.exists():
        parser.error(f"log file not found: {args.logfile}")
    if args.export is not None and args.export.suffix.lower() not in _EXPORT_SUFFIXES:
        parser.error(f"unsupported export format {args.export.suffix!r}; use one of {', '.join(_EXPORT_SUFFIXES)}")
    try:
        log = StateLog.load(args.logfile)
    except (OSError, ValueError, KeyError) as exc:
        parser.error(f"could not load {args.logfile}: {exc}")

    # Export mode renders offscreen so no GUI window is ever shown (headless).
    if args.export is not None:
        os.environ["QT_QPA_PLATFORM"] = "offscreen"

    app = QApplication(sys.argv)
    try:
        window = PlaybackWindow(log, show_com=args.show_com, speed=args.speed)
    except ValueError as exc:
        parser.error(str(exc))

    if args.export is not None:
        frames = window.export(args.export, fps=args.fps)
        print(f"wrote {frames} frames to {args.export}")
        return

    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
