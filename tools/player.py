# Copyright (C) 2025-2026 Hiroshi Atsuta <atsuta@ieee.org>
# SPDX-License-Identifier: GPL-3.0-only

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

Task overlays come from the embedded ``extra.source_config.task`` (a full,
rerunnable scenario) or, when a producer has no scenario to embed, from the
playback-only ``extra.playback.task`` table with the same ``[task]`` schema
(e.g. ``{type = "reaching", target = {pos = [x, y], tolerance = r}}``). A
playback-only log can be drawn but not re-run; a malformed playback table is
rejected on load.

Given several log files, the player opens a playlist window beside it: double-click a
file (or press Enter) to load and play it, ``N`` / ``P`` in the player load the next /
previous one, and each finished log moves on to the next file that loads (the end of
the list stops). Files that fail to load are greyed out and skipped; the side panel and
the window title name the playing file, and "Plot channels…" plots it.

The replay can also be exported headlessly (no GUI window) to an ``.mp4`` video or an
animated ``.gif`` with ``--export``: each frame is rendered from the same canvas the
interactive player uses — task overlay, centers of mass, and external-force arrow
included — and encoded with ``imageio``. Export renders a single log.

Usage::

    uv run python tools/player.py run.sklog.npz
    uv run python tools/player.py reach_*.sklog.npz                     # several logs: a playlist
    uv run python tools/player.py run.sklog.npz --show-com --speed 0.5
    uv run python tools/player.py run.sklog.npz --export run.mp4          # headless mp4
    uv run python tools/player.py run.sklog.npz --export run.gif --fps 24  # headless gif
"""

from __future__ import annotations

import argparse
import os
import sys
import zipfile
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, NoReturn, cast

import numpy as np
from PyQt6.QtCore import QEvent, QObject, QPoint, QSignalBlocker, Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QKeySequence, QShortcut
from PyQt6.QtWidgets import (
    QApplication,
    QCheckBox,
    QDoubleSpinBox,
    QHBoxLayout,
    QLabel,
    QListWidget,
    QListWidgetItem,
    QMainWindow,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QWidget,
)

from skelarm import (
    SkelarmCanvas,
    StateLog,
    Task,
    TransportBar,
    bind_quit_key,
    compute_forward_kinematics,
    compute_jacobian,
    make_icon,
)

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # allow `tools.` imports when run as a script
from tools._scenario_cli import task_overlays

if TYPE_CHECKING:
    from collections.abc import Buffer

    from numpy.typing import NDArray
    from PyQt6.QtGui import QColor

    from skelarm import Skeleton

_TIMER_MS = 20  # playback/render period in milliseconds
_PANEL_WIDTH_PX = 300  # fixed side-panel width so the varying time/frame readout can't resize it
_EXPORT_FPS = 30.0  # default output frame rate for --export
_EXPORT_SIZE_PX = 800  # square frame size (px) for --export; a multiple of 16 keeps mp4 codecs happy
_EXPORT_SUFFIXES = (".mp4", ".gif")  # --export formats, selected from the output path extension
_EXPORT_PANEL_WIDTH_PX = 304  # --panel side-panel width; a multiple of 16 keeps mp4 codecs happy
_PLAYING_MARK = "▶ "  # prefixes the playlist entry of the loaded log
# What loading and checking a log file can raise: unreadable, not an archive, missing members, not replayable.
_LOAD_ERRORS = (OSError, ValueError, KeyError, zipfile.BadZipFile)


class _ExportPanel(QWidget):
    """Simulator-style side panel rendered into ``--panel`` export frames (never shown as a window).

    Mirrors the live simulator's control panel: the bold time readout, one
    read-only degree-labeled slider per joint, the tip position, and — when the
    log recorded them — the tip speed, external-force magnitude, viscous
    friction, and active-target index.
    """

    def __init__(self, log: StateLog, skeleton: Skeleton) -> None:
        """Build the panel widgets for the log's joints and recorded channels."""
        super().__init__()
        self._skeleton = skeleton  # posed by the export loop before every frame grab
        self._times = log.times
        names = log.channel_names
        self._dq = log.channel("dq") if "dq" in names else None
        self._force = log.channel("ext_force") if "ext_force" in names else None
        self._friction = log.channel("friction") if "friction" in names else None
        self._active = log.channel("active_target") if "active_target" in names else None

        layout = QVBoxLayout(self)
        layout.addWidget(QLabel("<b>Simulation</b>"))
        self._time_label = QLabel()
        font = self._time_label.font()
        font.setPointSize(font.pointSize() + 6)
        font.setBold(True)
        self._time_label.setFont(font)
        layout.addWidget(self._time_label)

        self._joint_labels: list[QLabel] = []
        self._sliders: list[QSlider] = []
        for link in skeleton.links[1:]:
            label = QLabel()
            layout.addWidget(label)
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(round(float(np.degrees(link.prop.qmin))), round(float(np.degrees(link.prop.qmax))))
            slider.setEnabled(False)  # read-only: shows the replayed angle
            layout.addWidget(slider)
            self._joint_labels.append(label)
            self._sliders.append(slider)

        def _optional_label(present: bool) -> QLabel | None:  # noqa: FBT001
            if not present:
                return None
            label = QLabel()
            layout.addWidget(label)
            return label

        self._tip_label = QLabel()
        layout.addWidget(self._tip_label)
        self._speed_label = _optional_label(self._dq is not None)
        self._force_label = _optional_label(self._force is not None)
        self._friction_label = _optional_label(self._friction is not None)
        self._active_label = _optional_label(self._active is not None)
        layout.addStretch(1)

    def show_frame(self, index: int) -> None:
        """Refresh every readout for frame ``index`` (the shared skeleton is already posed)."""
        self._time_label.setText(f"t = {float(self._times[index]):.2f} s")
        sliders = zip(self._sliders, self._joint_labels, self._skeleton.q, strict=True)
        for i, (slider, label, angle) in enumerate(sliders):
            deg = round(float(np.degrees(angle)))
            slider.setValue(deg)
            label.setText(f"Joint {i + 1}: {deg}°")
        tip = self._skeleton.links[-1]
        self._tip_label.setText(f"Tip: ({tip.xe:.3f}, {tip.ye:.3f}) m")
        if self._speed_label is not None and self._dq is not None:
            speed = float(np.linalg.norm(compute_jacobian(self._skeleton) @ self._dq[index]))
            self._speed_label.setText(f"Tip speed: {speed:.3f} m/s")
        if self._force_label is not None and self._force is not None:
            force = self._force[index]
            self._force_label.setText(f"Ext. force: {float(np.hypot(force[0], force[1])):.3g} N")
        if self._friction_label is not None and self._friction is not None:
            self._friction_label.setText(f"Friction: {float(self._friction[index]):.3f} N·m·s/rad")
        if self._active_label is not None and self._active is not None:
            self._active_label.setText(f"Active target: {round(float(self._active[index])) + 1}")


@dataclass(frozen=True, eq=False)
class _Replay:
    """Everything the player derives from one log, checked before the window switches to it."""

    log: StateLog
    skeleton: Skeleton
    q: NDArray[np.float64]
    times: NDArray[np.float64]
    force: NDArray[np.float64] | None
    active_target: NDArray[np.float64] | None
    targets: list[tuple[NDArray[np.float64], QColor, float | None, bool]]
    path: NDArray[np.float64] | None


def _prepare_replay(log: StateLog) -> _Replay:
    """Check that ``log`` can be replayed and derive its arm, channels, and task overlays.

    Every channel the window reads per frame is checked for its shape here, so a log
    that does not fit is rejected before the window changes, never on a later frame.

    Raises
    ------
    ValueError
        If the log has no ``q`` channel, a replayed channel of the wrong shape, or
        malformed playback metadata.
    """
    if "q" not in log.channel_names:
        msg = "log has no 'q' channel; cannot replay the arm"
        raise ValueError(msg)
    skeleton = log.build_skeleton()
    frames = len(log)
    names = log.channel_names
    q = _checked_channel(log, "q", (frames, skeleton.num_joints), "one angle per joint per frame")
    force = (
        _checked_channel(log, "ext_force", (frames, 2), "a tip force (fx, fy) per frame")
        if "ext_force" in names
        else None
    )
    active_target = (
        _checked_channel(log, "active_target", (frames,), "one target index per frame")
        if "active_target" in names
        else None
    )
    targets, path = _task_overlays_of(log, skeleton)
    return _Replay(
        log=log,
        skeleton=skeleton,
        q=q,
        times=log.times,
        force=force,
        active_target=active_target,
        targets=targets,
        path=path,
    )


def _checked_channel(log: StateLog, name: str, shape: tuple[int, ...], meaning: str) -> NDArray[np.float64]:
    """Return the channel ``name``, raising ``ValueError`` unless it has the replayed ``shape``."""
    data = log.channel(name)
    if data.shape != shape:
        msg = f"log channel {name!r} has shape {data.shape}, expected {shape} ({meaning})"
        raise ValueError(msg)
    return data


def _task_overlays_of(
    log: StateLog, skeleton: Skeleton
) -> tuple[list[tuple[NDArray[np.float64], QColor, float | None, bool]], NDArray[np.float64] | None]:
    """Reconstruct the task from the embedded metadata and return its ``(targets, path)`` overlays.

    A full ``extra.source_config.task`` (as embedded by the simulators) always wins;
    ``extra.playback.task`` is a playback-only fallback for producers that can describe
    the task (target, tolerance) without a rerunnable scenario. Logs without either
    draw no overlays.

    Raises
    ------
    ValueError
        If the playback-only metadata is malformed.
    """
    task_cfg = log.extra.get("source_config", {}).get("task")
    if task_cfg:
        try:
            task = Task.from_dict(task_cfg)
        except (ValueError, KeyError):
            return [], None
    else:
        playback_cfg = _playback_task(log.extra)
        if playback_cfg is None:
            return [], None
        try:
            task = Task.from_dict(playback_cfg)
        except (ValueError, KeyError) as exc:
            # Playback metadata exists only to be drawn; drawing nothing would hide the
            # producer's mistake, so a malformed table is an explicit error.
            msg = f"log carries malformed extra.playback.task metadata: {exc}"
            raise ValueError(msg) from exc
    return task_overlays(task, skeleton)


class PlaybackWindow(QMainWindow):
    """A timeline player for a recorded :class:`~skelarm.StateLog`.

    The arm is reconstructed from the log's embedded geometry and posed from the
    recorded ``q`` channel; scrubbing and playback are purely kinematic. The same
    window can open per-channel analysis plots via :meth:`build_channel_figure`, and
    :meth:`load_log` switches it to another log in place (the playlist uses this).
    """

    # Emitted when playback runs into the last frame (not on a jump, scrub, or step to it).
    playback_finished = pyqtSignal()

    # The replayed log and what is derived from it, all (re)set by _switch_to.
    log: StateLog
    skeleton: Skeleton
    _q: NDArray[np.float64]
    _times: NDArray[np.float64]
    _n: int
    _force: NDArray[np.float64] | None
    _active_target: NDArray[np.float64] | None
    _shown_active: int | None
    _has_targets: bool
    _has_reference: bool
    _frame: int  # the frame shown
    _play_time: float  # the playback clock (log seconds)

    def __init__(self, log: StateLog, *, show_com: bool = False, speed: float = 1.0, name: str | None = None) -> None:
        """Build the player for ``log``.

        Parameters
        ----------
        log : StateLog
            The recording to replay. It must contain a ``q`` channel.
        show_com : bool, optional
            Overlay each link's center of mass at startup.
        speed : float, optional
            Initial playback speed multiplier.
        name : str or None, optional
            The file name shown in the side panel and the title (hidden when ``None``).

        Raises
        ------
        ValueError
            If the log has no ``q`` channel to drive the arm, or carries malformed
            playback metadata.
        """
        super().__init__()
        replay = _prepare_replay(log)
        self._speed = speed
        self._show_force = True  # mirrors the "Show external force" checkbox
        self.canvas = SkelarmCanvas(replay.skeleton)
        self.canvas.show_com = show_com
        self.resize(1024, 768)

        central = QWidget()
        self.setCentralWidget(central)
        layout = QHBoxLayout(central)
        layout.addWidget(self.canvas, stretch=3)

        panel = QWidget()
        panel.setFixedWidth(_PANEL_WIDTH_PX)  # keep a constant width; the time/frame readout won't resize it
        controls = QVBoxLayout(panel)
        self.controls_panel = panel  # exposed for sizing (fixed width) and tests
        self.header_label = QLabel()
        controls.addWidget(self.header_label)
        self.file_label = QLabel()  # the playing file's name
        self.file_label.setWordWrap(True)
        controls.addWidget(self.file_label)

        self.time_label = QLabel()
        self.time_label.setWordWrap(True)  # wrap rather than clip if the readout outgrows the fixed width
        time_font = self.time_label.font()
        time_font.setPointSize(time_font.pointSize() + 4)
        time_font.setBold(True)
        self.time_label.setFont(time_font)
        controls.addWidget(self.time_label)

        self.slider = QSlider(Qt.Orientation.Horizontal)  # its range follows the loaded log
        self.slider.valueChanged.connect(self.set_frame)
        controls.addWidget(self.slider)

        self.transport_bar = TransportBar(
            step_label="Next frame", reset_label="Back to start", back_label="Previous frame"
        )
        self.transport_bar.play_button.toggled.connect(self._on_play_toggled)
        self.transport_bar.step_button.clicked.connect(self._on_step_clicked)
        self.transport_bar.reset_button.clicked.connect(self._on_reset_clicked)
        assert self.transport_bar.back_button is not None  # back_label was given
        self.transport_bar.back_button.clicked.connect(self._on_back_clicked)
        controls.addWidget(self.transport_bar)
        # Historical aliases so tests and scripts keep addressing the buttons directly.
        self.play_button = self.transport_bar.play_button
        self.step_button = self.transport_bar.step_button
        self.reset_button = self.transport_bar.reset_button
        self.back_button = self.transport_bar.back_button

        # Window-level navigation keys with no buttons of their own.
        self.home_shortcut = QShortcut(QKeySequence("Home"), self)
        self.home_shortcut.activated.connect(self.reset_button.click)
        self.end_shortcut = QShortcut(QKeySequence("End"), self)
        self.end_shortcut.activated.connect(self._on_end_shortcut)
        self.quit_shortcut = bind_quit_key(self)
        # Next / previous log of a playlist: disabled until a playlist connects them.
        self.next_shortcut = QShortcut(QKeySequence("N"), self)
        self.previous_shortcut = QShortcut(QKeySequence("P"), self)
        for shortcut in (self.next_shortcut, self.previous_shortcut):
            shortcut.setAutoRepeat(False)  # a held key must not race through the list
            shortcut.setEnabled(False)

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

        # External-force arrow controls, shown only while the log recorded a force.
        self.force_checkbox = QCheckBox("Show external force")
        self.force_checkbox.setChecked(True)
        self.force_checkbox.toggled.connect(self._on_show_force_toggled)
        controls.addWidget(self.force_checkbox)
        self.force_label = QLabel()
        controls.addWidget(self.force_label)

        # Task-overlay toggles, shown only while the log embeds the matching data.
        self.target_checkbox = QCheckBox("Show target(s)")
        self.target_checkbox.setChecked(True)
        self.target_checkbox.toggled.connect(self._on_show_targets_toggled)
        controls.addWidget(self.target_checkbox)
        self.reference_checkbox = QCheckBox("Show reference")
        self.reference_checkbox.setChecked(True)
        self.reference_checkbox.toggled.connect(self._on_show_reference_toggled)
        controls.addWidget(self.reference_checkbox)

        self.plot_button = QPushButton("Plot channels…")
        self.plot_button.setIcon(make_icon("mdi6.chart-line"))
        self.plot_button.clicked.connect(self._on_plot_channels)
        controls.addWidget(self.plot_button)

        # Reopens the playlist window; shown only when a playlist drives this player.
        self.playlist_button = QPushButton("Playlist")
        self.playlist_button.setIcon(make_icon("mdi6.playlist-play"))
        self.playlist_button.setVisible(False)
        controls.addWidget(self.playlist_button)

        controls.addStretch()
        layout.addWidget(panel, stretch=1)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._on_timeout)
        self._switch_to(replay, name)

    def load_log(self, log: StateLog, *, name: str | None = None) -> None:
        """Replay ``log`` in this window instead, starting paused at its first frame.

        The log is checked before anything changes, so a log that cannot be replayed
        leaves the current one on screen. Viewer settings (speed, centers of mass, and
        the overlay toggles) carry over.

        Parameters
        ----------
        log : StateLog
            The recording to replay next.
        name : str or None, optional
            The file name shown in the side panel and the title (hidden when ``None``).

        Raises
        ------
        ValueError
            If the log has no ``q`` channel or carries malformed playback metadata.
        """
        replay = _prepare_replay(log)
        self.pause()
        self._switch_to(replay, name)

    def _switch_to(self, replay: _Replay, name: str | None) -> None:
        """Make ``replay`` the shown log: arm, channels, overlays, toggles, labels, and frame 0."""
        self.log = replay.log
        self.skeleton = replay.skeleton
        self.canvas.skeleton = replay.skeleton
        self._q = replay.q
        self._times = replay.times
        self._n = len(replay.log)
        self._force = replay.force  # recorded external tip force (N), shown as an arrow
        if self._force is None:
            self.canvas.tip_force = None
        else:
            self.canvas.force_scale = self._force_arrow_scale()
        # Task overlays (target / active-target emphasis / periodic curve / reference trajectory).
        self.canvas.overlay_targets = replay.targets
        self.canvas.overlay_path = replay.path
        self._has_targets, self._has_reference = bool(replay.targets), replay.path is not None
        # Recorded active-target index (multi-target runs); replays live switches.
        self._active_target = replay.active_target
        self._shown_active = None

        self.force_checkbox.setVisible(self._force is not None)
        self.force_label.setVisible(self._force is not None)
        self.target_checkbox.setVisible(self._has_targets)
        self.reference_checkbox.setVisible(self._has_reference)
        self.header_label.setText(f"<b>Replay</b> — {replay.log.producer or 'state log'}")
        self.file_label.setText(name or "")
        self.file_label.setVisible(name is not None)
        self.setWindowTitle(f"Skelarm Replay — {name}" if name else "Skelarm Replay")
        with QSignalBlocker(self.slider):
            self.slider.setRange(0, max(self._n - 1, 0))
        self._frame = 0
        self._play_time = float(self._times[0]) if self._n else 0.0
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
        """Advance the playback clock by ``seconds`` (scaled by :attr:`speed`).

        Reaching the last frame pauses; if the timeline was playing, that is the natural
        end and :attr:`playback_finished` fires (a one-frame log ends on its first tick).
        """
        self._play_time += seconds * self._speed
        if self._play_time >= self._times[-1]:
            self._play_time = float(self._times[-1])
            self._show_frame(self._n - 1)
            was_playing = self.is_playing
            self.pause()
            if was_playing:
                self.playback_finished.emit()
            return
        index = int(np.searchsorted(self._times, self._play_time, side="right") - 1)
        self._show_frame(max(index, 0))

    def play(self) -> None:
        """Start (or restart from the beginning) playback."""
        if self._frame >= self._n - 1:
            self.set_frame(0)
        self._timer.start(_TIMER_MS)
        self.transport_bar.set_playing(True)

    def pause(self) -> None:
        """Pause playback."""
        self._timer.stop()
        self.transport_bar.set_playing(False)

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

    def export(
        self, path: str | Path, *, fps: float = _EXPORT_FPS, size: int = _EXPORT_SIZE_PX, panel: bool = False
    ) -> int:
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
            Side length in pixels of the (square) rendered canvas (default: 800).
        panel : bool, optional
            Composite a simulator-style side panel next to the canvas — the time
            readout, per-joint sliders, and the recorded parameter readouts
            (external force, viscous friction, active target) when those
            channels exist. Widens each frame by 304 px; for ``.mp4`` output the
            composited width is additionally padded up to the next multiple of
            16 so the codec won't rescale it (default: canvas only).

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
        export_panel: _ExportPanel | None = None
        if panel:
            export_panel = _ExportPanel(self.log, self.skeleton)
            export_panel.setFixedSize(_EXPORT_PANEL_WIDTH_PX, size)
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
                frame = self._grab_widget_rgb(self.canvas)
                if export_panel is not None:
                    export_panel.show_frame(index)
                    frame = np.hstack((frame, self._grab_widget_rgb(export_panel)))
                    if path.suffix.lower() == ".mp4":
                        # Guard: keep the composited width a multiple of 16 so ffmpeg
                        # won't silently rescale; GIFs have no such constraint.
                        pad = (-frame.shape[1]) % 16
                        if pad:
                            frame = np.pad(frame, ((0, 0), (0, pad), (0, 0)), mode="edge")
                writer.append_data(frame)
        return n_frames

    def _grab_widget_rgb(self, widget: QWidget) -> NDArray[np.uint8]:
        """Grab a widget as an ``(H, W, 3)`` uint8 RGB array (offscreen-safe)."""
        from PyQt6.QtGui import QImage

        image = widget.grab().toImage().convertToFormat(QImage.Format.Format_RGBA8888)
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
        self._apply_active_target(index)
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

    def _on_play_toggled(self, playing: bool) -> None:  # noqa: FBT001
        """Start or pause playback when the transport toggle changes."""
        if playing:
            self.play()
        else:
            self.pause()

    def _on_step_clicked(self) -> None:
        """Advance a single frame while paused."""
        if not self.is_playing:
            self.set_frame(self._frame + 1)

    def _on_back_clicked(self) -> None:
        """Step a single frame backward while paused."""
        if not self.is_playing:
            self.set_frame(self._frame - 1)

    def _on_reset_clicked(self) -> None:
        """Pause playback and jump back to the first frame."""
        self.pause()
        self.set_frame(0)

    def _on_end_shortcut(self) -> None:
        """Pause playback and jump to the last frame."""
        self.pause()
        self.set_frame(self._n - 1)

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

    def _apply_active_target(self, index: int) -> None:
        """Re-flag the overlay markers to the frame's recorded active target, if recorded."""
        if self._active_target is None or not self.canvas.overlay_targets:
            return
        active = round(float(self._active_target[index]))
        if active == self._shown_active:
            return
        self._shown_active = active
        self.canvas.overlay_targets = [
            (pos, color, tolerance, i == active)
            for i, (pos, color, tolerance, _) in enumerate(self.canvas.overlay_targets)
        ]

    def _on_plot_channels(self) -> None:
        """Open the per-channel analysis plots without blocking the player."""
        import matplotlib.pyplot as plt

        if not self.log.channel_names:
            return
        self.build_channel_figure()
        plt.show(block=False)


class PlaylistWindow(QWidget):
    """A playlist of log files that a :class:`PlaybackWindow` replays one after another.

    Double-click a file (or press Enter on it) to load and play it; when a log
    finishes, the next file that loads is played, and the end of the list stops.
    A file that fails to load is greyed out with the reason and skipped. Closing
    this window only hides it (the player's Playlist button reopens it); closing
    the player closes it too.
    """

    def __init__(self, paths: Sequence[Path], player: PlaybackWindow, *, current: int = 0) -> None:
        """List ``paths`` and drive ``player``, which already shows ``paths[current]``."""
        super().__init__()
        self.paths = list(paths)
        self.player = player
        self._current = current
        self._failed: set[int] = set()
        self.setWindowTitle("Skelarm Playlist")
        self.resize(360, 480)

        layout = QVBoxLayout(self)
        layout.addWidget(QLabel("Double-click (or Enter) to play a log"))
        self.list_widget = QListWidget()
        for path in self.paths:
            item = QListWidgetItem(path.name)
            item.setToolTip(str(path))
            self.list_widget.addItem(item)
        # Activation is the platform's double-click (or Enter on the current row).
        self.list_widget.itemActivated.connect(self._on_item_activated)
        layout.addWidget(self.list_widget)

        self.play_shortcut = QShortcut(QKeySequence("Space"), self)  # play/pause without leaving the list
        self.play_shortcut.activated.connect(self.player.play_button.click)
        player.playback_finished.connect(self._on_player_finished)
        player.playlist_button.setVisible(True)
        player.playlist_button.setToolTip("Show the playlist (N / P: next / previous log)")
        player.playlist_button.clicked.connect(self.show_and_raise)
        # N / P live on the player only: in this window they would hijack the list's type-to-search.
        player.next_shortcut.activated.connect(self.play_next)
        player.previous_shortcut.activated.connect(self.play_previous)
        player.next_shortcut.setEnabled(True)
        player.previous_shortcut.setEnabled(True)
        player.installEventFilter(self)  # closing the player closes the playlist too
        self._mark_current()

    @property
    def current(self) -> int:
        """The index of the log loaded in the player."""
        return self._current

    def play_index(self, index: int, *, play: bool = True) -> bool:
        """Load the file at ``index`` into the player and play it (unless ``play`` is false).

        Returns ``False``, leaving the current log in the player, when the file fails to
        load; the entry is then greyed out with the reason.
        """
        path = self.paths[index]
        try:
            self.player.load_log(StateLog.load(path), name=path.name)
        except _LOAD_ERRORS as exc:
            self.mark_failed(index, exc)
            return False
        self._current = index
        self._mark_current()
        if play:
            self.player.play()
        return True

    def play_next(self) -> bool:
        """Load the next file that loads, keeping a playing player playing; ``False`` at the end."""
        return self._move(+1, play=self.player.is_playing)

    def play_previous(self) -> bool:
        """Load the previous file that loads, keeping a playing player playing; ``False`` at the start."""
        return self._move(-1, play=self.player.is_playing)

    def mark_failed(self, index: int, error: Exception) -> None:
        """Grey out the entry at ``index`` with the reason it could not be loaded."""
        self._failed.add(index)
        item = self.list_widget.item(index)
        assert item is not None  # every path has an entry
        item.setFlags(item.flags() & ~Qt.ItemFlag.ItemIsEnabled)
        item.setToolTip(f"{self.paths[index]}\ncould not load: {error}")
        print(f"could not load {self.paths[index]}: {error}", file=sys.stderr)

    def show_and_raise(self) -> None:
        """Show the playlist window in front."""
        self.show()
        self.raise_()
        self.activateWindow()

    def eventFilter(self, a0: QObject | None, a1: QEvent | None) -> bool:  # noqa: N802
        """Close together with the player window."""
        if a0 is self.player and a1 is not None and a1.type() == QEvent.Type.Close:
            self.close()
        return False

    def _mark_current(self) -> None:
        """Prefix the loaded log's entry with the now-playing mark and select it."""
        for row, path in enumerate(self.paths):
            item = self.list_widget.item(row)
            assert item is not None  # every path has an entry
            item.setText(f"{_PLAYING_MARK}{path.name}" if row == self._current else path.name)
        self.list_widget.setCurrentRow(self._current)

    def _on_item_activated(self, item: QListWidgetItem) -> None:
        """Load and play the activated entry."""
        self.play_index(self.list_widget.row(item))

    def _on_player_finished(self) -> None:
        """Play the next file that loads; the end of the list just stops."""
        self._move(+1, play=True)

    def _move(self, step: int, *, play: bool) -> bool:
        """Load the nearest file in direction ``step`` that loads, skipping failed ones."""
        index = self._current + step
        while 0 <= index < len(self.paths):
            if index not in self._failed and self.play_index(index, play=play):
                return True
            index += step
        return False


def open_playlist(
    paths: Sequence[Path], *, show_com: bool = False, speed: float = 1.0
) -> tuple[PlaybackWindow, PlaylistWindow]:
    """Open a player on the first of ``paths`` that loads, with a playlist window for all of them.

    The player starts paused; files that fail to load are marked in the playlist.

    Raises
    ------
    ValueError
        If none of the logs can be replayed.
    """
    failures: dict[int, Exception] = {}
    for index, path in enumerate(paths):
        try:
            player = PlaybackWindow(StateLog.load(path), show_com=show_com, speed=speed, name=path.name)
        except _LOAD_ERRORS as exc:
            failures[index] = exc
            continue
        playlist = PlaylistWindow(paths, player, current=index)
        for failed, error in failures.items():
            playlist.mark_failed(failed, error)
        return player, playlist
    reasons = "; ".join(f"{paths[index].name}: {error}" for index, error in failures.items())
    msg = f"none of the logs can be replayed ({reasons})"
    raise ValueError(msg)


def _reject_playback_shape(key: str, value: object) -> NoReturn:
    """Refuse playback metadata of the wrong shape with the error the schema promises (a ``ValueError``)."""
    msg = f"log carries malformed {key} metadata: expected a table, got {type(value).__name__}"
    raise ValueError(msg)


def _playback_task(extra: Mapping[str, object]) -> Mapping[str, object] | None:
    """The ``extra.playback.task`` table, or ``None`` when absent; a non-table shape is a clear error."""
    playback = extra.get("playback")
    if playback is None:
        return None
    if not isinstance(playback, Mapping):
        _reject_playback_shape("extra.playback", playback)
    task = playback.get("task")
    if task is None:
        return None
    if not isinstance(task, Mapping):
        _reject_playback_shape("extra.playback.task", task)
    return task


def build_parser() -> argparse.ArgumentParser:
    """Build the command-line argument parser for the tool."""
    parser = argparse.ArgumentParser(description="Replay and analyze a recorded skelarm state log.")
    parser.add_argument(
        "logfile", type=Path, nargs="+", help="path to a .sklog.npz state log; several open a playlist window"
    )
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
    parser.add_argument(
        "--panel",
        action="store_true",
        help="include a simulator-style side panel (time, sliders, parameter readouts) in the --export frames",
    )
    return parser


def _checked_paths(parser: argparse.ArgumentParser, args: argparse.Namespace) -> list[Path]:
    """Return the log paths after checking they exist and fit the export options (exits otherwise)."""
    paths: list[Path] = args.logfile
    missing = [str(path) for path in paths if not path.exists()]
    if missing:
        parser.error(f"log file not found: {', '.join(missing)}")
    if args.export is not None:
        if len(paths) > 1:
            parser.error("--export takes a single log; export each file separately")
        if args.export.suffix.lower() not in _EXPORT_SUFFIXES:
            parser.error(f"unsupported export format {args.export.suffix!r}; use one of {', '.join(_EXPORT_SUFFIXES)}")
    return paths


def _run_playlist(parser: argparse.ArgumentParser, args: argparse.Namespace, paths: list[Path]) -> NoReturn:
    """Run the player with a playlist window over ``paths`` until the player is closed."""
    app = QApplication(sys.argv)
    try:
        player, playlist = open_playlist(paths, show_com=args.show_com, speed=args.speed)
    except ValueError as exc:
        parser.error(str(exc))
    player.show()
    playlist.move(player.frameGeometry().topRight() + QPoint(8, 0))  # beside the player
    playlist.show()
    sys.exit(app.exec())


def main() -> None:
    """Parse arguments, load the log(s), and run the player (or export one log headlessly)."""
    parser = build_parser()
    args = parser.parse_args()
    paths = _checked_paths(parser, args)
    if len(paths) > 1:
        _run_playlist(parser, args, paths)

    logfile = paths[0]
    try:
        log = StateLog.load(logfile)
    except _LOAD_ERRORS as exc:
        parser.error(f"could not load {logfile}: {exc}")

    # Export mode renders offscreen so no GUI window is ever shown (headless).
    if args.export is not None:
        os.environ["QT_QPA_PLATFORM"] = "offscreen"

    app = QApplication(sys.argv)
    try:
        window = PlaybackWindow(log, show_com=args.show_com, speed=args.speed, name=logfile.name)
    except ValueError as exc:
        parser.error(str(exc))

    if args.export is not None:
        frames = window.export(args.export, fps=args.fps, panel=args.panel)
        print(f"wrote {frames} frames to {args.export}")
        return

    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
