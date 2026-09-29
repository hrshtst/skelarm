# Copyright (C) 2025-2026 Hiroshi Atsuta <atsuta@ieee.org>
# SPDX-License-Identifier: GPL-3.0-only

"""
Interactive joint-trajectory recorder for skelarm.

Teach a motion by grabbing the robot's tip with the mouse and dragging it; each take is
saved as a ``.sklog.npz`` state log that replays with ``tools/player.py``. The user teaches
in task space ``(x, y)``; two modes turn that into per-joint angles:

  - ``ik``       : solve inverse kinematics each tick so the tip tracks the cursor.
  - ``dynamics`` : apply a spring force at the tip and integrate forward dynamics
                   (with viscous friction), like ``tools/dynamics_simulator.py``.

Controls are window-wide keyboard shortcuts mirrored by buttons:

  - **Space** starts a take from the ready (reset) posture: the reset state is logged at
    ``t = 0`` and the stationary pre-roll is recorded until you move the tip. Pressing
    Space again (or holding it) during a take changes nothing. ``--start-on-grab``
    restores the legacy behavior of starting on the first grab instead.
  - **S** saves the take and keeps it visible: recording stops, nothing is reset.
  - **Shift+S** saves and prepares the next take: after a successful save the posture,
    velocity, drag state, log, and clock are reset and recording waits for Space. An
    already-saved take is not written again.
  - **R** resets: an *unsaved* take is discarded (its samples are dropped, no file is
    written, no take number is consumed); a saved take keeps its file. Either way the
    arm returns to the reset posture at rest with a fresh log, waiting for Space.
    **S then R** is the same as **Shift+S**: both go through the one save and the one
    reset operation.
  - **Q** (and the window's close button) closes; when unsaved samples exist a modal
    warning offers *Save and close*, *Discard and close*, and *Cancel* (the default;
    dismissing the dialog also cancels). Sampling pauses while the warning is open and
    Cancel resumes the take exactly where it was. An empty or already-saved take closes
    without a warning and without writing another file.

Acquisition clock: the recorder runs one timer tick per sample period, so
``--sample-rate`` must give a whole number of milliseconds (100, 50, 25, 20, 10 Hz, ...).
Every tick performs exactly one pose update (one IK solve toward the current cursor, or
the dynamics substeps) and records exactly one sample. The sample's ``time`` is the
actual elapsed time since the take started, read from the wall clock when the tick's
pose update begins, so a late timer tick shows up as a longer interval instead of being
hidden; the nominal tick clock ``k * period`` is kept beside it as the ``nominal_time``
channel. No sample is ever duplicated or invented, and the time spent in the
unsaved-take warning is excluded from both clocks. The realized tick spacing (mean,
maximum, late-tick count) is also summarized in the log's ``[extra.acquisition]``
table. The display repaints at most every 20 ms, independently of sampling.

Trails: ``--show-tip-trail`` draws the current take's tip path from the logged (FK) tip
samples, never the cursor path; ``--show-past-trails`` keeps the tip paths of the takes
saved in this session as faint transparent lines behind the current one. Both are
checkboxes too and can be hidden independently. ``--past-trail-history last`` limits
the faint overlay to the most recently saved take (``all``, the default, draws every
saved take); the history, the saved files, and the samples are the same either way.
Only saved takes enter the history (once each: S then R and Shift+S agree), R drops an
unsaved trail with its take, and a new session always starts with an empty history,
whatever files exist on disk. The overlay settings, the display-history mode, the color
policy, the takes in the history, and the saved takes actually drawn while a take was
recorded are stored under ``[extra.display]`` of its log. The overlays never move the
robot or change the logged samples.

Outputs: ``--output`` names one exact file (the second take of a session is then refused
rather than overwriting it); a name without the ``.npz`` suffix gets it appended, as
NumPy would, so the file that is checked is the file that is written. With
``--multi-take`` the name is the base of numbered files, ``reach.sklog.npz`` ->
``reach_001.sklog.npz``, ``reach_002.sklog.npz``, ...; numbering continues after any file
of that base already present, and an existing file is never overwritten: the save is
refused, reported, and the take stays for a retry. Each take is written to a temporary
file beside its target and published only once complete, by a hard link that refuses an
existing file; a failed write leaves nothing behind, and a file system without hard
links refuses the save and keeps the take. A take with no samples beyond ``t = 0`` is never written and consumes
no number. Saving opens no dialog and no plot; ``--plot`` plots the last visible take
after the window closes.

Usage::

    uv run python tools/trajectory_recorder.py examples/four_dof_robot.toml
    uv run python tools/trajectory_recorder.py robot.toml --mode dynamics --sample-rate 100
    uv run python tools/trajectory_recorder.py robot.toml --output reach.sklog.npz --multi-take
    uv run python tools/trajectory_recorder.py robot.toml --multi-take --show-tip-trail --show-past-trails
    uv run python tools/trajectory_recorder.py robot.toml --multi-take --show-past-trails --past-trail-history last
    uv run python tools/trajectory_recorder.py robot.toml --duration 0 --plot   # no cap; plot afterward
    uv run python tools/player.py reach_001.sklog.npz   # replay a recorded take
"""

from __future__ import annotations

import argparse
import os
import re
import sys
import time as wall_clock
import tomllib
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import TYPE_CHECKING, Literal

import numpy as np
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QCloseEvent, QColor, QKeySequence, QShortcut
from PyQt6.QtWidgets import (
    QApplication,
    QCheckBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from skelarm import (
    Skeleton,
    StateLog,
    Task,
    bind_quit_key,
    compute_inverse_kinematics,
    compute_jacobian,
    integrate_with_limits,
    make_icon,
)
from skelarm.canvas import TrailOverlay
from skelarm.simulator import SimulatorCanvas

if TYPE_CHECKING:
    from collections.abc import Callable

    from numpy.typing import NDArray

_DISPLAY_MS = 20  # repaint period (ms); sampling follows the tick, not the display
_PANEL_WIDTH_PX = 300  # fixed side-panel width so its content can't resize it
_SUBSTEPS = 4  # dynamics-mode physics substeps per tick
_DRAG_STIFFNESS = 30.0  # N/m for the mouse drag (dynamics mode)
_FRICTION = 0.2  # joint viscous friction (dynamics mode; "some by default")
_SAMPLE_RATE = 50.0  # logger sampling rate (Hz); one timer tick per sample
_DURATION = 10.0  # max recording duration (s); zero or negative means no cap
_LATE_TICK_FACTOR = 1.5  # a wall-clock tick slower than this multiple of the period counts as late
_LOG_SUFFIX = ".sklog.npz"
_MIN_SAMPLES = 2  # a take holding only the t = 0 frame is empty
_TICK_TOLERANCE_MS = 1e-9
_CURRENT_TRAIL_COLOR = QColor(200, 30, 120, 230)  # magenta, clearly visible over the blue arm
_CURRENT_TRAIL_WIDTH_PX = 2.0
_PAST_TRAIL_COLOR = QColor(70, 90, 160, 55)  # faint, transparent slate blue behind the current take
_PAST_TRAIL_WIDTH_PX = 2.0
_PAST_TRAIL_HISTORIES = ("all", "last")  # draw every saved take of the session, or only the most recent one


class CloseChoice(Enum):
    """The answer to the unsaved-take warning shown when closing."""

    CANCEL = "cancel"
    SAVE = "save"
    DISCARD = "discard"


@dataclass(frozen=True, eq=False)
class SavedTrail:
    """The tip path of a take saved in this session, kept for the faint past-trail overlay."""

    take: int
    path: Path
    points: NDArray[np.float64]


def _split_log_name(output: Path) -> tuple[str, str]:
    """Split ``reach.sklog.npz`` into ``("reach", ".sklog.npz")`` (other suffixes keep theirs)."""
    name = output.name
    if name.endswith(_LOG_SUFFIX):
        return name[: -len(_LOG_SUFFIX)], _LOG_SUFFIX
    return output.stem, output.suffix


def normalized_output(output: Path) -> Path:
    """Return the file NumPy will actually write for ``output``: ``.npz`` is appended unless already present."""
    return output if output.suffix == ".npz" else output.with_name(output.name + ".npz")


def numbered_output(output: Path, number: int) -> Path:
    """Return the numbered take file for ``output`` used as a base, e.g. ``reach_001.sklog.npz``."""
    base, suffix = _split_log_name(output)
    return output.with_name(f"{base}_{number:03d}{suffix}")


def _highest_existing_take(output: Path) -> int:
    """Return the highest take number already present beside ``output`` (0 when there is none)."""
    base, suffix = _split_log_name(output)
    pattern = re.compile(rf"^{re.escape(base)}_(\d+){re.escape(suffix)}$")
    highest = 0
    for candidate in output.parent.glob(f"{base}_*{suffix}"):
        match = pattern.match(candidate.name)
        if match is not None:
            highest = max(highest, int(match.group(1)))
    return highest


def tick_period_ms(sample_rate: float) -> int:
    """Return the timer period for ``sample_rate`` in whole milliseconds.

    Raises
    ------
    ValueError
        If the rate is not positive or its period is not a whole number of milliseconds.
    """
    if sample_rate <= 0:
        msg = f"sample rate must be positive, got {sample_rate:g}"
        raise ValueError(msg)
    period_ms = 1000.0 / sample_rate
    rounded = round(period_ms)
    if rounded < 1 or abs(period_ms - rounded) > _TICK_TOLERANCE_MS:
        msg = (
            f"sample rate {sample_rate:g} Hz gives a {period_ms:.4g} ms period; the timer tick must be a "
            "whole number of milliseconds (for example 100, 50, 25, 20, or 10 Hz)"
        )
        raise ValueError(msg)
    return int(rounded)


class _TickTiming:
    """Realized wall-clock spacing of the recorded ticks of one take."""

    def __init__(self, period_s: float) -> None:
        self._period_s = period_s
        self.count = 0
        self.total_s = 0.0
        self.max_s = 0.0
        self.late = 0
        self._last: float | None = None

    def start(self, now: float) -> None:
        """Begin a take at wall-clock reading ``now``: the next tick is measured against it."""
        self.count = 0
        self.total_s = 0.0
        self.max_s = 0.0
        self.late = 0
        self._last = now

    def resume(self, now: float) -> None:
        """Continue after a pause: the next tick is measured against ``now``, not across the pause."""
        self._last = now

    def mark(self, now: float) -> None:
        """Account one recorded tick at wall-clock reading ``now``."""
        if self._last is not None:
            elapsed = now - self._last
            self.count += 1
            self.total_s += elapsed
            self.max_s = max(self.max_s, elapsed)
            if elapsed > _LATE_TICK_FACTOR * self._period_s:
                self.late += 1
        self._last = now

    def as_meta(self) -> dict[str, float | int]:
        """Return the statistics as plain TOML-friendly values."""
        mean = self.total_s / self.count if self.count else 0.0
        return {
            "ticks": self.count,
            "wall_mean_tick_s": mean,
            "wall_max_tick_s": self.max_s,
            "late_ticks": self.late,
            "late_tick_factor": _LATE_TICK_FACTOR,
        }


class RecorderWindow(QMainWindow):
    """Teach and record joint-trajectory takes by dragging the robot's tip.

    The window is a small state machine: ``ready`` (at the reset posture, waiting for
    Space), ``recording`` (one sample per timer tick), and ``stopped`` (a take is
    visible, saved or not). :meth:`save_take` and :meth:`reset_take` are the only save
    and reset operations; Shift+S, the duration cap, and *Save and close* reuse them.

    Parameters
    ----------
    skeleton : Skeleton
        The arm to teach; its posture at construction is the reset posture.
    mode : {"ik", "dynamics"}, optional
        How the cursor drives the joints.
    sample_rate : float, optional
        Samples per second; its period must be a whole number of milliseconds.
    duration : float, optional
        Recording cap in seconds; zero or negative removes the cap.
    output : str or Path, optional
        Exact output file, or the base of numbered files with ``multi_take``.
    multi_take : bool, optional
        Number the outputs (``base_001.sklog.npz``, ...) instead of using ``output`` as is.
    start_on_grab : bool, optional
        Start recording on the first grab instead of on Space / the Start button.
    show_tip_trail : bool, optional
        Draw the current take's logged tip path while recording.
    show_past_trails : bool, optional
        Draw the tip paths of the takes saved in this session as faint lines.
    past_trail_history : {"all", "last"}, optional
        Which saved takes the faint overlay draws: every take saved in this session
        (``"all"``) or only the most recently saved one (``"last"``). The session
        history, the saved files, and the logged samples are the same either way.
    unsaved_prompt : callable, optional
        Replaces the modal unsaved-take warning (tests inject a stub); it must return
        a :class:`CloseChoice`.
    run_timer : bool, optional
        Drive :meth:`tick` from a live timer (tests drive it by hand).
    clock : callable, optional
        Monotonic wall-clock reading in seconds (``time.perf_counter`` by default;
        tests inject a fake one).
    """

    def __init__(
        self,
        skeleton: Skeleton,
        *,
        mode: str = "ik",
        sample_rate: float = _SAMPLE_RATE,
        duration: float = _DURATION,
        output: str | Path = "teach.sklog.npz",
        multi_take: bool = False,
        start_on_grab: bool = False,
        show_tip_trail: bool = False,
        show_past_trails: bool = False,
        past_trail_history: Literal["all", "last"] = "all",
        method: str = "lm_sugihara",
        stiffness: float = _DRAG_STIFFNESS,
        friction: float = _FRICTION,
        task: Task | None = None,
        show_com: bool = False,
        enforce_limits: bool = True,
        unsaved_prompt: Callable[[], CloseChoice] | None = None,
        run_timer: bool = True,
        clock: Callable[[], float] = wall_clock.perf_counter,
    ) -> None:
        """Build the recorder window."""
        super().__init__()
        self.skeleton = skeleton
        self._mode = mode
        self.tick_ms = tick_period_ms(sample_rate)
        self._tick_dt = self.tick_ms / 1000.0
        self._display_every = max(1, round(_DISPLAY_MS / self.tick_ms))
        if past_trail_history not in _PAST_TRAIL_HISTORIES:
            msg = f"past_trail_history must be one of {', '.join(_PAST_TRAIL_HISTORIES)}, got {past_trail_history!r}"
            raise ValueError(msg)
        self._past_trail_history: Literal["all", "last"] = past_trail_history
        self._duration = duration if duration > 0 else None  # None: record until saved
        self._output = normalized_output(Path(output))
        if self._output != Path(output):
            print(f"output resolved to {self._output} (NumPy appends .npz)")
        self._multi_take = multi_take
        self._start_on_grab = start_on_grab
        self._method = method
        self._stiffness = stiffness
        self._friction = friction
        self._task = task
        self._unsaved_prompt: Callable[[], CloseChoice] = unsaved_prompt or self._ask_unsaved
        self._run_timer = run_timer
        self._clock = clock
        self._t0 = 0.0  # wall-clock reading at the start of the take (shifted past dialog pauses)
        # When limits are not enforced in the dynamics, the hard stop is disabled and
        # joint limits apply only to the kinematic (IK) path, which always clamps.
        self._lower = (
            np.array([link.prop.qmin for link in skeleton.links[1:]], dtype=np.float64) if enforce_limits else None
        )
        self._upper = (
            np.array([link.prop.qmax for link in skeleton.links[1:]], dtype=np.float64) if enforce_limits else None
        )
        self._reset_q = skeleton.q.copy()

        self._state = "ready"
        self._saved = False
        self._closed = False
        self._takes_saved = 0
        self._take_number = _highest_existing_take(self._output) + 1 if multi_take else 1
        self._last_saved_path: Path | None = None
        self.time = 0.0
        self._ticks = 0
        self.log = self._new_log()
        self._timing = _TickTiming(self._tick_dt)
        self._history: list[SavedTrail] = []  # saved takes of this session only, in save order
        self._history_at_start: tuple[int, ...] = ()
        self._drawn_at_start: tuple[SavedTrail, ...] = ()  # the saved takes the overlay draws for this take
        self._past_shown_during_take = False

        self.canvas = SimulatorCanvas(skeleton)
        self.canvas.show_com = show_com
        self.canvas.show_drag_arrow = mode == "dynamics"  # no force cue for the kinematic IK drag
        self.canvas.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        reach = sum(link.prop.length for link in skeleton.links)
        self.canvas.grab_radius = max(0.12 * reach, 0.05)  # grab near the tip
        if task is not None and task.target is not None:
            self.canvas.target = np.asarray(task.target, dtype=np.float64)
            self.canvas.target_color = QColor(task.color)
            self.canvas.target_tolerance = task.tolerance

        self.setWindowTitle("Skelarm Trajectory Recorder")
        self.resize(1024, 768)
        self.quit_shortcut = bind_quit_key(self)
        self.quit_shortcut.setAutoRepeat(False)
        central = QWidget()
        self.setCentralWidget(central)
        layout = QHBoxLayout(central)
        layout.addWidget(self.canvas, stretch=3)

        panel = QWidget()
        panel.setFixedWidth(_PANEL_WIDTH_PX)  # keep a constant width regardless of the status text
        controls = QVBoxLayout(panel)
        self.controls_panel = panel  # exposed for sizing (fixed width) and tests
        controls.addWidget(QLabel(f"<b>Trajectory recorder</b> — {mode} mode, {sample_rate:g} Hz"))
        hint = QLabel(
            "Space starts a take from the reset posture (the still pre-roll is recorded). "
            "Drag the tip (left-drag) to teach. S saves and keeps the take visible; "
            "Shift+S saves and prepares the next take; R resets, discarding only an unsaved take; "
            "Q closes."
            if not start_on_grab
            else "Recording starts on the first grab (--start-on-grab). Drag the tip (left-drag) to teach. "
            "S saves; Shift+S saves and prepares the next take; R resets, discarding only an unsaved take; "
            "Q closes."
        )
        hint.setWordWrap(True)
        controls.addWidget(hint)
        self.status_label = QLabel()
        self.status_label.setWordWrap(True)
        controls.addWidget(self.status_label)

        self.buttons: dict[str, QPushButton] = {}
        self.shortcuts: dict[str, QShortcut] = {}
        actions: tuple[tuple[str, str, str, str, Callable[[], object]], ...] = (
            ("start", "Start", "Space", "mdi6.record-circle-outline", self.start),
            ("save", "Save", "S", "mdi6.content-save", self.save_take),
            ("save_next", "Save and next take", "Shift+S", "mdi6.content-save-move", self.save_and_next),
            ("reset", "Reset", "R", "mdi6.restart", self.reset_take),
        )
        for name, label, key, icon, slot in actions:
            button = QPushButton(f"{label} ({key})")
            button.setIcon(make_icon(icon))
            button.setToolTip(f"{label} ({key})")
            # Buttons never take focus, so Space cannot be swallowed as a button click and
            # every shortcut keeps working with the focus on the drawing canvas.
            button.setFocusPolicy(Qt.FocusPolicy.NoFocus)
            button.clicked.connect(slot)
            controls.addWidget(button)
            self.buttons[name] = button
            shortcut = QShortcut(QKeySequence(key), self)
            shortcut.setContext(Qt.ShortcutContext.WindowShortcut)
            shortcut.setAutoRepeat(False)  # holding a key must not repeat the action
            shortcut.activated.connect(slot)
            self.shortcuts[name] = shortcut
        self.checkboxes: dict[str, QCheckBox] = {}
        for name, label, checked in (
            ("tip_trail", "Current tip trail", show_tip_trail),
            ("past_trails", "Faint saved trails", show_past_trails),
        ):
            box = QCheckBox(label)
            box.setChecked(checked)
            box.setFocusPolicy(Qt.FocusPolicy.NoFocus)  # Space must never toggle a box
            box.toggled.connect(self._on_overlay_toggled)
            controls.addWidget(box)
            self.checkboxes[name] = box
        controls.addStretch()
        layout.addWidget(panel, stretch=1)

        self._timer = QTimer(self)
        self._timer.setTimerType(Qt.TimerType.PreciseTimer)
        self._timer.timeout.connect(self.tick)
        if run_timer:
            self._timer.start(self.tick_ms)
        if multi_take and self._take_number > 1:
            print(f"continuing after existing take {self._take_number - 1:03d}: next output is {self.output_path}")
        self._refresh()

    # ------------------------------------------------------------------ state ---

    @property
    def state(self) -> str:
        """``"ready"``, ``"recording"``, or ``"stopped"`` (a take is visible, saved or not)."""
        return self._state

    @property
    def saved(self) -> bool:
        """Whether the visible take has been saved."""
        return self._saved

    @property
    def closed(self) -> bool:
        """Whether the window accepted a close."""
        return self._closed

    @property
    def takes_saved(self) -> int:
        """How many takes this session has written."""
        return self._takes_saved

    @property
    def take_number(self) -> int:
        """The number of the take being recorded, or of the next one once it is saved."""
        return self._take_number

    @property
    def last_saved_path(self) -> Path | None:
        """Where the most recent take was written, if any."""
        return self._last_saved_path

    @property
    def show_tip_trail(self) -> bool:
        """Whether the current take's tip path is drawn."""
        return self.checkboxes["tip_trail"].isChecked()

    @property
    def show_past_trails(self) -> bool:
        """Whether the saved takes' tip paths are drawn faintly."""
        return self.checkboxes["past_trails"].isChecked()

    @property
    def past_trail_history(self) -> Literal["all", "last"]:
        """Which saved takes the faint overlay draws: ``"all"`` of this session, or only the ``"last"`` one."""
        return self._past_trail_history

    def current_trail(self) -> NDArray[np.float64]:
        """Return the logged tip positions of the visible take, shape ``(n, 2)`` (the drawn trail's geometry)."""
        if len(self.log) == 0:
            return np.zeros((0, 2), dtype=np.float64)
        return self.log.channel("tip")

    def saved_trails(self) -> tuple[SavedTrail, ...]:
        """Return the takes saved in this session, in save order (the past-trail history)."""
        return tuple(self._history)

    def _drawn_history(self) -> tuple[SavedTrail, ...]:
        """Return the saved takes the faint overlay draws: the whole history, or only its most recent take."""
        if self._past_trail_history == "last":
            return tuple(self._history[-1:])
        return tuple(self._history)

    @property
    def output_path(self) -> Path:
        """The file the next save would write."""
        return numbered_output(self._output, self._take_number) if self._multi_take else self._output

    def _take_is_empty(self) -> bool:
        return self._state == "ready" or len(self.log) < _MIN_SAMPLES

    def _has_unsaved_take(self) -> bool:
        return not self._take_is_empty() and not self._saved

    # -------------------------------------------------------------- recording ---

    def _new_log(self) -> StateLog:
        """Start a fresh state log with the channels for the active mode."""
        joints = [f"j{i + 1}" for i in range(self.skeleton.num_joints)]
        channel_meta: dict[str, dict[str, object]] = {
            "q": {"unit": "rad", "label": "joint angle", "columns": joints},
            "tip": {"unit": "m", "label": "tip position", "columns": ["x", "y"]},
            "nominal_time": {"unit": "s", "label": "nominal tick clock (tick index x period)"},
        }
        if self._mode == "dynamics":
            channel_meta["dq"] = {"unit": "rad/s", "label": "joint velocity", "columns": joints}
            channel_meta["ext_force"] = {"unit": "N", "label": "external tip force", "columns": ["fx", "fy"]}
            channel_meta["friction"] = {"unit": "N*m*s/rad", "label": "viscous friction"}
        return StateLog(self.skeleton, producer="trajectory_recorder", channel_meta=channel_meta)

    def start(self) -> None:
        """Start a take from the ready state: log the reset state at ``t = 0`` and keep sampling.

        Ignored unless the recorder is ready, so a repeated or held Space never restarts
        the clock or adds a take.
        """
        if self._state != "ready":
            return
        now = self._clock()
        self._state = "recording"
        self._saved = False
        self.time = 0.0
        self._ticks = 0
        self._t0 = now
        self.log = self._new_log()
        self._record(0.0, 0.0)
        self._timing.start(now)
        self._history_at_start = tuple(trail.take for trail in self._history)
        self._drawn_at_start = self._drawn_history()
        self._past_shown_during_take = self.show_past_trails
        print(f"take {self._take_number:03d}: recording started")
        self._refresh()

    def tick(self) -> None:
        """Advance one tick: one pose update, one sample, and a throttled repaint."""
        if self._state != "recording":
            if self._start_on_grab and self._state == "ready" and self.canvas.drag_point is not None:
                self.start()
            else:
                return
        now = self._clock()  # the sample's acquisition time: when this tick's pose update begins
        self._timing.mark(now)
        if self._mode == "ik":
            self._step_ik()
        else:
            self._step_dynamics()
        self._ticks += 1
        self.time = now - self._t0
        self._record(self.time, self._ticks * self._tick_dt)
        if self._ticks % self._display_every == 0:
            self._refresh()
        if self._duration is not None and self.time >= self._duration - _TICK_TOLERANCE_MS:
            self._stop()
            self.save_take()

    def _step_ik(self) -> None:
        """One IK solve toward the current cursor (the pose update of this tick)."""
        target = self.canvas.drag_point
        if target is not None:
            compute_inverse_kinematics(
                self.skeleton, np.asarray(target, dtype=np.float64), method=self._method, q0=self.skeleton.q
            )

    def _step_dynamics(self) -> None:
        """Integrate forward dynamics under the tip force over this tick's substeps."""
        dt = self._tick_dt / _SUBSTEPS
        for _ in range(_SUBSTEPS):
            tau = compute_jacobian(self.skeleton).T @ self.canvas.external_force(self._stiffness)
            tau = tau - self._friction * self.skeleton.dq
            integrate_with_limits(self.skeleton, tau, dt, self._lower, self._upper)

    def _record(self, t: float, nominal: float) -> None:
        """Append one frame at elapsed time ``t`` (nominal tick time ``nominal``): joint angles, tip, and more."""
        tip = self.skeleton.links[-1]
        channels: dict[str, NDArray[np.float64]] = {
            "q": self.skeleton.q,
            "tip": np.array([tip.xe, tip.ye], dtype=np.float64),
            "nominal_time": np.asarray(nominal, dtype=np.float64),
        }
        if self._mode == "dynamics":
            channels["dq"] = self.skeleton.dq
            channels["ext_force"] = self.canvas.external_force(self._stiffness)
            channels["friction"] = np.asarray(self._friction, dtype=np.float64)
        self.log.record(t, **channels)

    def _stop(self) -> None:
        """Stop sampling; the take stays visible."""
        if self._state == "recording":
            self._state = "stopped"
            self._refresh()

    def _acquisition_meta(self) -> dict[str, object]:
        """Describe the acquisition clock and the realized tick timing of this take."""
        return {
            "clock": "wall-clock",
            "time_channel": "seconds since the take started, read when each tick's pose update begins; "
            "dialog pauses excluded",
            "nominal_time_channel": "tick index x tick_period_s",
            "mode": self._mode,
            "tick_period_s": self._tick_dt,
            "sample_period_s": self._tick_dt,
            "pose_updates_per_sample": 1,
            "display_period_s": self._display_every * self._tick_dt,
            **self._timing.as_meta(),
        }

    def _display_meta(self) -> dict[str, object]:
        """Describe the overlays of this take: settings, display history, color policy, and the saved takes drawn."""
        visible = self._drawn_at_start if self._past_shown_during_take else ()
        return {
            "show_tip_trail": self.show_tip_trail,
            "show_past_trails": self.show_past_trails,
            "past_trail_history": self._past_trail_history,
            "past_trails_shown_during_take": self._past_shown_during_take,
            "history_takes": list(self._history_at_start),
            "visible_source_takes": [trail.take for trail in visible],
            "visible_source_files": [trail.path.name for trail in visible],
            "policy": {
                "current_color": _CURRENT_TRAIL_COLOR.name(),
                "current_alpha": _CURRENT_TRAIL_COLOR.alpha(),
                "current_width_px": _CURRENT_TRAIL_WIDTH_PX,
                "past_color": _PAST_TRAIL_COLOR.name(),
                "past_alpha": _PAST_TRAIL_COLOR.alpha(),
                "past_width_px": _PAST_TRAIL_WIDTH_PX,
                "order": "saved trails behind the current trail",
                "source": "logged tip samples (forward kinematics), not the cursor",
            },
        }

    # ------------------------------------------------------- save and reset ---

    def save_take(self) -> bool:
        """Stop and save the visible take, keeping it visible; the one save operation.

        Returns ``True`` when the take is saved afterwards (also when it already was),
        ``False`` when there is nothing to save or the write failed. A failed write, a
        file already present under the output name included, leaves the take unsaved
        and intact for a retry and never touches the existing bytes.
        """
        if self._take_is_empty():
            self._set_status("nothing to save: no take recorded (press Space to start one)")
            return False
        self._stop()
        if self._saved:
            self._set_status(f"take {self._take_number - 1:03d} is already saved to {self._last_saved_path}")
            return True
        path = self.output_path
        if path.exists():
            self._report_save_failure(f"not saved: {path} already exists (the take is kept; move that file, then save)")
            return False
        self.log.extra["acquisition"] = self._acquisition_meta()
        self.log.extra["display"] = self._display_meta()
        try:
            self._write_take(path)
        except OSError as exc:
            self._report_save_failure(f"not saved: {exc} (the take is kept for a retry)")
            return False
        self._saved = True
        self._last_saved_path = path
        self._takes_saved += 1
        number = self._take_number
        self._history.append(SavedTrail(number, path, self.log.channel("tip").copy()))
        if self._multi_take:
            self._take_number += 1
        print(f"saved take {number:03d} ({len(self.log)} samples) to {path}")
        self._refresh()
        return True

    def _write_take(self, path: Path) -> None:
        """Write the log to ``path`` through a temporary file, publishing it only once complete.

        The temporary lives beside the target and is hard-linked into place, which fails
        atomically if ``path`` appeared meanwhile. A file system without hard links gets
        no rename fallback (a rename could replace a file published in between): the
        save is refused and the take kept. Any failure leaves neither ``path`` nor the
        temporary behind.

        Raises
        ------
        OSError
            If writing or publishing fails (``FileExistsError`` when ``path`` exists).
        """
        temporary = path.with_name(f".{path.name}.tmp-{os.getpid()}.npz")  # keep .npz so NumPy adds nothing
        try:
            self.log.save(temporary)
            try:
                os.link(temporary, path)
            except FileExistsError:
                raise
            except OSError as exc:
                msg = (
                    f"cannot publish {path} atomically on this file system ({exc.strerror or exc}); "
                    "save to a location that supports hard links"
                )
                raise OSError(msg) from exc
        finally:
            temporary.unlink(missing_ok=True)

    def save_and_next(self) -> bool:
        """Save the take, then reset for the next one; the reset happens only after a save.

        An empty take is not written and just resets; an already-saved take advances
        without being written again. Returns whether the take was saved.
        """
        if self._take_is_empty():
            self.reset_take()
            return False
        if not self.save_take():
            return False
        self.reset_take()
        return True

    def reset_take(self) -> None:
        """Return to the reset posture at rest with a fresh log and clock; the one reset operation.

        An unsaved take is discarded (no file, no take number consumed); a saved take keeps
        its file. Pressing R while already ready changes nothing but the readout.
        """
        if self._has_unsaved_take():
            print(f"discarded unsaved take {self._take_number:03d} ({len(self.log)} samples)")
        self.skeleton.q = self._reset_q.copy()
        self.skeleton.dq = np.zeros(self.skeleton.num_joints, dtype=np.float64)
        self.canvas.drag_point = None
        self.log = self._new_log()
        self.time = 0.0
        self._ticks = 0
        self._state = "ready"
        self._saved = False
        self._refresh()

    def _report_save_failure(self, message: str) -> None:
        print(message, file=sys.stderr)
        self._set_status(message)

    # ----------------------------------------------------------------- close ---

    def _ask_unsaved(self) -> CloseChoice:
        """Show the modal unsaved-take warning; Cancel is the default and the escape answer."""
        box = QMessageBox(self)
        box.setIcon(QMessageBox.Icon.Warning)
        box.setWindowTitle("Unsaved take")
        box.setText(f"Take {self._take_number:03d} has {len(self.log)} unsaved samples.")
        save = box.addButton("Save and close", QMessageBox.ButtonRole.AcceptRole)
        discard = box.addButton("Discard and close", QMessageBox.ButtonRole.DestructiveRole)
        cancel = box.addButton("Cancel", QMessageBox.ButtonRole.RejectRole)
        box.setDefaultButton(cancel)
        box.setEscapeButton(cancel)
        box.exec()
        clicked = box.clickedButton()
        if clicked is save:
            return CloseChoice.SAVE
        if clicked is discard:
            return CloseChoice.DISCARD
        return CloseChoice.CANCEL

    def closeEvent(self, a0: QCloseEvent | None) -> None:  # noqa: N802
        """Close, warning first when unsaved samples exist; never write a saved take again."""
        if a0 is None:
            return
        if self._has_unsaved_take():
            self._timer.stop()  # no sampling while the warning is open
            pause_start = self._clock()
            choice = self._unsaved_prompt()
            if choice is CloseChoice.DISCARD:
                print(f"discarded unsaved take {self._take_number:03d} ({len(self.log)} samples)")
            elif choice is not CloseChoice.SAVE or not self.save_take():
                self._resume_after_pause(pause_start)  # cancel: continue exactly where the take was
                a0.ignore()
                return
        self._timer.stop()
        self._closed = True
        a0.accept()
        super().closeEvent(a0)

    def _resume_after_pause(self, pause_start: float) -> None:
        """Continue a take after a dialog: the pause enters neither the timestamps nor the tick statistics."""
        now = self._clock()
        if self._state == "recording":
            self._t0 += now - pause_start
            self._timing.resume(now)
        if self._run_timer:
            self._timer.start(self.tick_ms)

    # --------------------------------------------------------------- display ---

    def _on_overlay_toggled(self, _checked: bool) -> None:  # noqa: FBT001
        """Redraw after a checkbox change; the overlays never touch the log or the robot."""
        if self.show_past_trails and self._state == "recording":
            self._past_shown_during_take = True
        self._refresh_trails()
        self.canvas.update_skeleton()

    def _refresh_trails(self) -> None:
        """Rebuild the canvas overlays: faint saved trails (by display history) behind, the current trail in front."""
        trails: list[TrailOverlay] = []
        if self.show_past_trails:
            past = self._drawn_history()
            trails.extend(TrailOverlay(t.points, _PAST_TRAIL_COLOR, _PAST_TRAIL_WIDTH_PX) for t in past)
        if self.show_tip_trail and len(self.log) >= _MIN_SAMPLES:
            trails.append(TrailOverlay(self.current_trail(), _CURRENT_TRAIL_COLOR, _CURRENT_TRAIL_WIDTH_PX))
        self.canvas.trails = trails

    def _refresh(self) -> None:
        """Repaint the arm with its overlays and update the status readout and button states."""
        self._refresh_trails()
        self.canvas.update_skeleton()
        self._set_status(self._status_text())

    def _set_status(self, text: str) -> None:
        self.status_label.setText(text)
        empty = self._take_is_empty()
        self.buttons["start"].setEnabled(self._state == "ready")
        self.buttons["save"].setEnabled(not empty and not self._saved)
        self.buttons["save_next"].setEnabled(not empty)

    def _status_text(self) -> str:
        if self._state == "ready":
            how = "grab the tip" if self._start_on_grab else "press Space"
            return f"READY — take {self._take_number:03d} → {self.output_path.name}; {how} to start"
        if self._state == "recording":
            cap = "" if self._duration is None else f" / {self._duration:.1f}"
            return f"RECORDING take {self._take_number:03d}  t = {self.time:.2f}{cap} s,  {len(self.log)} samples"
        if self._saved:
            number = self._take_number - 1 if self._multi_take else self._take_number
            return f"SAVED take {number:03d} → {self._last_saved_path}"
        return f"STOPPED (unsaved) take {self._take_number:03d}, {len(self.log)} samples — S saves, R discards"

    def show_plot(self) -> None:
        """Plot the taught tip path, final pose, and joint angles (blocking)."""
        import matplotlib.pyplot as plt

        from skelarm import draw_skeleton, draw_target, plot_trajectory

        tip = self.log.channel("tip")
        q = self.log.channel("q")
        times = self.log.times
        _, (ax_path, ax_q) = plt.subplots(1, 2, figsize=(11, 5))
        draw_skeleton(ax_path, self.skeleton, title="Recorded trajectory")
        plot_trajectory(ax_path, tip[:, 0], tip[:, 1], title=None)
        if self._task is not None and self._task.target is not None:
            target = np.asarray(self._task.target, dtype=np.float64)
            draw_target(ax_path, target, color=self._task.color, tolerance=self._task.tolerance, label="target")
            ax_path.legend()
        for j in range(q.shape[1]):
            ax_q.plot(times, np.rad2deg(q[:, j]), label=f"j{j + 1}")
        ax_q.set_xlabel("time [s]")
        ax_q.set_ylabel("joint angle [deg]")
        ax_q.grid(visible=True)
        ax_q.legend(loc="best", fontsize="small")
        plt.tight_layout()
        plt.show()


def _sample_rate_argument(text: str) -> float:
    """Parse ``--sample-rate``, rejecting rates whose period is not a whole number of milliseconds."""
    rate = float(text)
    try:
        tick_period_ms(rate)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(str(exc)) from exc
    return rate


def build_parser() -> argparse.ArgumentParser:
    """Build the command-line argument parser for the tool."""
    parser = argparse.ArgumentParser(description="Interactively teach and record joint-trajectory takes.")
    parser.add_argument("config", type=Path, help="path to a robot TOML config (optional [initial] / [task])")
    parser.add_argument("--mode", choices=("ik", "dynamics"), default="ik", help="teaching mode (default: ik)")
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("teach.sklog.npz"),
        help="output .sklog.npz, or the base of numbered files with --multi-take (default: teach.sklog.npz)",
    )
    parser.add_argument(
        "--multi-take",
        action="store_true",
        help="number the outputs (reach.sklog.npz -> reach_001.sklog.npz, ...) instead of using --output as is",
    )
    parser.add_argument(
        "--start-on-grab",
        action="store_true",
        help="start recording on the first grab instead of on Space / Start",
    )
    parser.add_argument(
        "--show-tip-trail", action="store_true", help="draw the current take's logged tip path while recording"
    )
    parser.add_argument(
        "--show-past-trails",
        action="store_true",
        help="draw the tip paths of the takes saved in this session as faint lines behind the current take",
    )
    parser.add_argument(
        "--past-trail-history",
        choices=_PAST_TRAIL_HISTORIES,
        default="all",
        help="which saved takes --show-past-trails draws: every take saved in this session, or only the last "
        "saved one; the saved files and the history are the same either way (default: all)",
    )
    parser.add_argument(
        "--sample-rate",
        type=_sample_rate_argument,
        default=_SAMPLE_RATE,
        help="logger sampling rate in Hz; one timer tick per sample, so the period must be whole ms (default: 50)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=_DURATION,
        help="max recording duration in seconds, reached takes stop and save but stay open; "
        "zero or negative records until saved (default: 10)",
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
    parser.add_argument("--plot", action="store_true", help="plot the last visible take after the window closes")
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
    """Parse arguments, run the recorder, and plot the last visible take when asked."""
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
        multi_take=args.multi_take,
        start_on_grab=args.start_on_grab,
        show_tip_trail=args.show_tip_trail,
        show_past_trails=args.show_past_trails,
        past_trail_history=args.past_trail_history,
        method=args.method,
        stiffness=args.stiffness,
        friction=args.friction,
        task=task,
        show_com=args.show_com,
        enforce_limits=not args.no_joint_limits,
    )
    window.show()
    window.canvas.setFocus()
    app.exec()

    if window.saved and args.plot:
        window.show_plot()


if __name__ == "__main__":
    main()
