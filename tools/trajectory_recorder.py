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

Grabbing: a left press grabs the tip only within ``--grab-radius`` of it (5 cm by
default), the dashed circle drawn around the tip; a press farther away grabs nothing.
In ``ik`` mode the tip then moves as the cursor moves from where it was pressed, so a
grab off the tip's center never makes the tip jump to the cursor. In ``dynamics`` mode
the spring pulls the tip toward the cursor itself.

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
  - **E** (with ``--prepare``) enters or leaves the preparation phase, described below.
  - **1** to **9** (with ``--prepare`` and a ``multi_target_reaching`` task) choose the
    active target, only while preparing; the target buttons in the side panel do the same.
  - **Q** (and the window's close button) closes; when unsaved samples exist a modal
    warning offers *Save and close*, *Discard and close*, and *Cancel* (the default;
    dismissing the dialog also cancels). Sampling pauses while the warning is open and
    Cancel resumes the take exactly where it was. An empty or already-saved take closes
    without a warning and without writing another file.

Preparation (``--prepare``): E switches to a phase that sets the next take's start
posture and target. It is entered from the ready state, or after the visible take is
saved, in which case the arm stays where that take ended. While preparing, a grab within
the grab radius poses the arm by inverse kinematics, in both modes and at rest; nothing
is logged, the clock does not run, and Space, S, and Shift+S do nothing. With a
``multi_target_reaching`` task the keys 1 to 9 choose the active target, which no other
state can change. E again makes the posed arm the ready posture of the following takes,
until the next preparation; R instead cancels, back to the previous ready posture and
target. Leaving brings each joint by whole turns into (-180, 180] deg where its limits
hold that angle, so a joint dragged once around, which looks the same, does not start
the takes a turn away; the hand does not move. The readout gives the ready posture, and
while preparing the one being posed, in degrees. During a take, or while a take is not
saved, E is refused until it is saved (S) or discarded (R).

Acquisition clock: ``--sample-rate`` is a best-effort request. The recorder runs one
timer tick per sample period (rounded to whole milliseconds), and every tick performs
one pose update (one IK solve toward the current cursor, or the dynamics substeps) and
records one sample. The sample's ``time`` is the real elapsed time since the take
started, so a late tick shows up as a longer interval and a replay follows the motion as
it was performed, even when the requested rate is more than the machine can keep up
with. Dynamics mode simulates that same elapsed time, in fixed substeps capped at a few
periods after a stall. Starting a take restarts the timer, so the first sample follows
the ``t = 0`` frame by one full period (with ``--start-on-grab`` the tick that sees the
grab only logs ``t = 0``); the time spent in the unsaved-take warning is excluded. Each
saved log records the requested and achieved rates under ``[extra.acquisition]``, and
the save message prints them. The display repaints about every 20 ms, independently of
sampling.

Trails: ``--show-tip-trail`` draws the current take's tip path from the logged (FK) tip
samples, never the cursor path; ``--show-past-trails`` keeps the tip paths of the takes
saved in this session as faint transparent lines behind the current one. Both are
checkboxes too and can be hidden independently. ``--past-trail-history last`` limits
the faint overlay to the most recently saved take (``all``, the default, draws every
saved take); the history, the saved files, and the samples are the same either way.
Only saved takes enter the history (once each: S then R and Shift+S agree), R drops an
unsaved trail with its take, and a new session always starts with an empty history,
whatever files exist on disk. The overlays are a drawing aid only: they never move the
robot, change the logged samples, or enter the saved log.

Task guide: a ``[task]`` in the config (or ``--task``) is drawn as a guide. A reaching
task's target is drawn with its tolerance ring, and a ``periodic_curve`` task's closed
curve as a gray line, so a periodic motion is taught by moving the tip around it.
The guide stays in place through every take, and ``--plot`` draws it too.

Outputs: ``--output`` names one exact file (the second take of a session is then refused
rather than overwriting it); a name without the ``.npz`` suffix gets it appended, as
NumPy would, so the file that is checked is the file that is written. With
``--multi-take`` the name is the base of numbered files, ``reach.sklog.npz`` ->
``reach_001.sklog.npz``, ``reach_002.sklog.npz``, ...; numbering continues after any file
of that base already present, and a number taken meanwhile (e.g. by another session) is
skipped, never overwritten. Each take is written straight to its
name, and a failed write removes the partial file. A take with no samples beyond
``t = 0`` is never written and consumes no number. Saving opens no dialog and no plot;
``--plot`` plots the last visible take after the window closes. When the config (or
``--task``) has a ``[task]``, each take stores it under ``[extra.playback.task]``, so
the player draws the target or the curve when replaying or exporting the take. A
``multi_target_reaching`` task draws every target, the active one emphasized, and each
take stores the task with ``active`` set to the target that was active during it.

Usage::

    uv run python tools/trajectory_recorder.py examples/four_dof_robot.toml
    uv run python tools/trajectory_recorder.py robot.toml --mode dynamics --sample-rate 100
    uv run python tools/trajectory_recorder.py robot.toml --grab-radius 0.03   # grab only within 3 cm of the tip
    uv run python tools/trajectory_recorder.py robot.toml --output reach.sklog.npz --multi-take
    uv run python tools/trajectory_recorder.py robot.toml --multi-take --show-tip-trail --show-past-trails
    uv run python tools/trajectory_recorder.py robot.toml --multi-take --show-past-trails --past-trail-history last
    uv run python tools/trajectory_recorder.py robot.toml --duration 0 --plot   # no cap; plot afterward
    uv run python tools/trajectory_recorder.py examples/multi_target.toml --prepare --multi-take
    uv run python tools/trajectory_recorder.py examples/periodic_curve.toml   # follow the gray curve
    uv run python tools/player.py reach_001.sklog.npz   # replay a recorded take
"""

from __future__ import annotations

import argparse
import functools
import glob
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
    QButtonGroup,
    QCheckBox,
    QGridLayout,
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
    active_target_index,
    apply_active_target,
    bind_quit_key,
    compute_inverse_kinematics,
    compute_jacobian,
    integrate_with_limits,
    make_icon,
    multi_target_specs,
)
from skelarm.canvas import TrailOverlay
from skelarm.simulator import SimulatorCanvas

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # allow `tools.` imports when run as a script
from tools._scenario_cli import task_overlays

if TYPE_CHECKING:
    from collections.abc import Callable

    from numpy.typing import NDArray

_DISPLAY_MS = 20  # target repaint period (ms), rounded to whole ticks; sampling follows the tick, not the display
_PANEL_WIDTH_PX = 300  # fixed side-panel width so its content can't resize it
_SUBSTEPS = 4  # dynamics-mode physics substeps per sample period
_MAX_CATCH_UP_PERIODS = 3  # after a stall, simulate at most this many periods in one tick
_GRAB_RADIUS = 0.05  # m: a press grabs the tip only this close to it
_DRAG_STIFFNESS = 30.0  # N/m for the mouse drag (dynamics mode)
_FRICTION = 0.2  # joint viscous friction (dynamics mode; "some by default")
_SAMPLE_RATE = 50.0  # requested logger sampling rate (Hz), best effort; one timer tick per sample
_DURATION = 10.0  # max recording duration (s); zero or negative means no cap
_LOG_SUFFIX = ".sklog.npz"
_MIN_SAMPLES = 2  # a take holding only the t = 0 frame is empty
_TIME_TOLERANCE_S = 1e-9
_CURRENT_TRAIL_COLOR = QColor(200, 30, 120, 230)  # magenta, clearly visible over the blue arm
_CURRENT_TRAIL_WIDTH_PX = 2.0
_PAST_TRAIL_COLOR = QColor(70, 90, 160, 55)  # faint, transparent slate blue behind the current take
_PAST_TRAIL_WIDTH_PX = 2.0
_PAST_TRAIL_HISTORIES = ("all", "last")  # draw every saved take of the session, or only the most recent one
_MULTI_TARGET = "multi_target_reaching"  # the task type whose targets the preparation chooses from
_NUMBER_KEYS = 9  # the keys 1 to 9 choose the first nine targets
_TARGET_COLUMNS = 3  # target buttons per row of the side panel


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
    for candidate in output.parent.glob(f"{glob.escape(base)}_*{glob.escape(suffix)}"):  # match the name literally
        match = pattern.match(candidate.name)
        if match is not None:
            highest = max(highest, int(match.group(1)))
    return highest


def _targets_of(
    task: Task | None,
) -> tuple[list[tuple[NDArray[np.float64], str | None, str, float | None]], int | None]:
    """Return a multi-target task's targets and active index, after applying that target to the task.

    Another task, or none, has no targets and no active index.

    Raises
    ------
    ValueError
        If the task has no targets or its active index is out of range.
    """
    if task is None or task.type != _MULTI_TARGET:
        return [], None
    apply_active_target(task)  # as the scenario tools take it: the task's target is the active one
    return multi_target_specs(task), active_target_index(task)


def _turn_nearest_zero(skeleton: Skeleton) -> NDArray[np.float64]:
    """Return the arm's posture with each joint moved by whole turns into (-180, 180] deg.

    A joint whose limits do not hold that angle keeps its own. Whole turns leave the tip
    where it is, and a joint already in that range keeps its angle exactly.
    """
    q = skeleton.q
    wrapped = q - 2.0 * np.pi * np.ceil((q - np.pi) / (2.0 * np.pi))
    lower = np.array([link.prop.qmin for link in skeleton.links[1:]], dtype=np.float64)
    upper = np.array([link.prop.qmax for link in skeleton.links[1:]], dtype=np.float64)
    return np.where((lower <= wrapped) & (wrapped <= upper), wrapped, q)


def _posture_text(q: NDArray[np.float64]) -> str:
    """Return the joint angles for the readout in whole degrees, ``q = (30, 396) deg``."""
    return f"q = ({', '.join(str(round(float(angle))) for angle in np.rad2deg(q))}) deg"


def tick_period_ms(sample_rate: float) -> int:
    """Return the timer period for ``sample_rate`` rounded to whole milliseconds (at least 1 ms).

    The rate is a best-effort request: the achieved rate is whatever the machine sustains.

    Raises
    ------
    ValueError
        If the rate is not positive.
    """
    if sample_rate <= 0:
        msg = f"sample rate must be positive, got {sample_rate:g}"
        raise ValueError(msg)
    return max(1, round(1000.0 / sample_rate))


class RecorderWindow(QMainWindow):
    """Teach and record joint-trajectory takes by dragging the robot's tip.

    The window is a small state machine: ``ready`` (at the reset posture, waiting for
    Space), ``recording`` (one sample per timer tick), and ``stopped`` (a take is
    visible, saved or not). :meth:`save_take` and :meth:`reset_take` are the only save
    and reset operations; Shift+S, the duration cap, and *Save and close* reuse them.
    With ``prepare`` a fourth state, ``preparing``, poses the next take's start and
    chooses its target; :meth:`toggle_prepare` enters and leaves it.

    Parameters
    ----------
    skeleton : Skeleton
        The arm to teach; its posture at construction is the reset posture.
    mode : {"ik", "dynamics"}, optional
        How the cursor drives the joints.
    sample_rate : float, optional
        Requested samples per second, a best effort; the timer period is rounded to whole milliseconds.
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
    prepare : bool, optional
        Add the preparation phase (E and a Prepare button): the tip is dragged to pose
        the next take's start and, with a ``multi_target_reaching`` task, the keys 1 to 9
        and the target buttons choose its target. Leaving it makes the posed arm the
        ready posture, each joint at the turn nearest zero that its limits hold. The
        readout gives the ready posture and the one being posed.
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
        grab_radius: float = _GRAB_RADIUS,
        stiffness: float = _DRAG_STIFFNESS,
        friction: float = _FRICTION,
        task: Task | None = None,
        show_com: bool = False,
        enforce_limits: bool = True,
        prepare: bool = False,
        unsaved_prompt: Callable[[], CloseChoice] | None = None,
        run_timer: bool = True,
        clock: Callable[[], float] = wall_clock.perf_counter,
    ) -> None:
        """Build the recorder window."""
        super().__init__()
        self.skeleton = skeleton
        self._mode = mode
        self.tick_ms = tick_period_ms(sample_rate)
        self._sample_rate = sample_rate
        self._tick_dt = self.tick_ms / 1000.0
        self._display_every = max(1, round(_DISPLAY_MS / self.tick_ms))
        if past_trail_history not in _PAST_TRAIL_HISTORIES:
            msg = f"past_trail_history must be one of {', '.join(_PAST_TRAIL_HISTORIES)}, got {past_trail_history!r}"
            raise ValueError(msg)
        self._past_trail_history: Literal["all", "last"] = past_trail_history
        if grab_radius <= 0:
            msg = f"grab_radius must be positive, got {grab_radius}"
            raise ValueError(msg)
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
        self._targets, self._active = _targets_of(task)  # a multi-target task's targets, the active one applied
        self._ready_target = self._active  # the ready state's target, to which R returns from the preparation
        self._prepare = prepare
        self._notice = ""  # a refused E, shown with the take's readout until the next take starts
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
        self._history: list[SavedTrail] = []  # saved takes of this session only, in save order
        self._grab_offset: NDArray[np.float64] | None = None  # tip minus the point pressed, for the current IK drag
        self._grab_gesture: int | None = None  # the canvas's drag gesture that offset belongs to

        self.canvas = SimulatorCanvas(skeleton)
        self.canvas.show_com = show_com
        self.canvas.show_drag_arrow = mode == "dynamics"  # no force cue for the kinematic IK drag
        self.canvas.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self.canvas.grab_radius = grab_radius  # a press grabs the tip only this close to it
        self.canvas.show_grab_range = True
        if task is not None and task.target is not None:
            self.canvas.target = np.asarray(task.target, dtype=np.float64)
            self.canvas.target_color = QColor(task.color)
            self.canvas.target_tolerance = task.tolerance
        self._draw_targets()  # a multi-target task draws every target, the active one emphasized
        # A periodic task's curve (or a tracked path) is drawn in gray, as a guide to follow.
        self.canvas.overlay_path = None if task is None else task_overlays(task, skeleton)[1]

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
            "Drag the tip (left-drag, starting within the dashed circle) to teach. S saves and keeps the take visible; "
            "Shift+S saves and prepares the next take; R resets, discarding only an unsaved take; "
            "Q closes."
            if not start_on_grab
            else "Recording starts on the first grab (--start-on-grab). "
            "Drag the tip (left-drag, starting within the dashed circle) to teach. "
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
        if prepare:  # an option: without it there is no such button and no E key
            actions += (("prepare", "Prepare", "E", "mdi6.cursor-move", self.toggle_prepare),)
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
        self.target_buttons: list[QPushButton] = []
        self._add_preparation_controls(controls, hint)
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
        """``"ready"``, ``"recording"``, ``"stopped"`` (a take is visible, saved or not), or ``"preparing"``."""
        return self._state

    @property
    def active_target(self) -> int | None:
        """The index of the active target of a multi-target task, ``None`` with another task or none."""
        return self._active

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

    def _saved_take_number(self) -> int:
        """Return the number of the visible saved take: multi-take numbering has already moved past it."""
        return self._take_number - 1 if self._multi_take else self._take_number

    # -------------------------------------------------------------- recording ---

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
        self._notice = ""
        self.time = 0.0
        self._ticks = 0
        self._t0 = now
        self.log = self._new_log()
        self._record(0.0)
        if self._run_timer:
            self._timer.start(self.tick_ms)  # restart the period: the first sample follows t = 0 by a full period
        print(f"take {self._take_number:03d}: recording started")
        self._refresh()

    def tick(self) -> None:
        """Advance one tick: one pose update, one sample, and a throttled repaint."""
        if self._state == "preparing":
            self._pose()
            return
        if self._state != "recording":
            if self._start_on_grab and self._state == "ready" and self.canvas.drag_point is not None:
                self.start()  # this tick only logs t = 0; the first pose update and sample are the next tick's
            return
        t = self._clock() - self._t0  # the sample's time: real elapsed time when this tick's pose update begins
        if self._mode == "ik":
            self._step_ik()
        else:
            self._step_dynamics(t - self.time)
        self._ticks += 1
        self.time = t
        self._record(t)
        if self._ticks % self._display_every == 0:
            self._refresh()
        if self._duration is not None and self.time >= self._duration - _TIME_TOLERANCE_S:
            self._stop()
            self.save_take()

    def _step_ik(self) -> None:
        """One IK solve that moves the tip as the cursor has moved since the grab (the pose update of this tick).

        The first tick of every new drag gesture takes the offset between the tip and
        the point pressed (the tip has not moved since), so a grab off the tip's center
        never makes the tip jump to the cursor, not even when the previous drag ended and
        this one began between two ticks.
        """
        cursor = self.canvas.drag_point
        if cursor is None:
            self._grab_offset = None
            return
        if self._grab_offset is None or self._grab_gesture != self.canvas.drag_gesture:
            tip = self.skeleton.links[-1]
            origin = self.canvas.drag_origin or cursor
            self._grab_offset = np.array([tip.xe - origin[0], tip.ye - origin[1]], dtype=np.float64)
            self._grab_gesture = self.canvas.drag_gesture
        target = np.asarray(cursor, dtype=np.float64) + self._grab_offset
        compute_inverse_kinematics(self.skeleton, target, method=self._method, q0=self.skeleton.q)

    def _step_dynamics(self, elapsed: float) -> None:
        """Integrate forward dynamics under the tip force over the ``elapsed`` real time since the last sample.

        The substeps keep a fixed size (a fraction of the sample period); their count follows
        ``elapsed`` but is capped after a stall, so catching up can never snowball.
        """
        dt = self._tick_dt / _SUBSTEPS
        steps = min(max(1, round(elapsed / dt)), _MAX_CATCH_UP_PERIODS * _SUBSTEPS)
        for _ in range(steps):
            tau = compute_jacobian(self.skeleton).T @ self.canvas.external_force(self._stiffness)
            tau = tau - self._friction * self.skeleton.dq
            integrate_with_limits(self.skeleton, tau, dt, self._lower, self._upper)

    def _record(self, t: float) -> None:
        """Append one frame at elapsed time ``t``: joint angles, tip, and (dynamics) dq, force, friction."""
        tip = self.skeleton.links[-1]
        channels: dict[str, NDArray[np.float64]] = {
            "q": self.skeleton.q,
            "tip": np.array([tip.xe, tip.ye], dtype=np.float64),
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

    def _achieved_rate(self) -> float:
        """Return the samples per second this take actually achieved (0 for a take without duration)."""
        duration = float(self.log.times[-1]) if len(self.log) else 0.0
        return (len(self.log) - 1) / duration if duration > 0 else 0.0

    def _acquisition_meta(self) -> dict[str, object]:
        """Record the requested and the achieved sampling rate of this take."""
        return {"requested_rate_hz": self._sample_rate, "achieved_rate_hz": self._achieved_rate()}

    # ------------------------------------------------------- save and reset ---

    def save_take(self) -> bool:
        """Stop and save the visible take, keeping it visible; the one save operation.

        Returns ``True`` when the take is saved afterwards (also when it already was),
        ``False`` when there is nothing to save or the write failed. A failed write, a
        file already present under the output name included, leaves the take unsaved
        and intact for a retry and never touches the existing bytes.
        """
        if self._state == "preparing":  # nothing to save while posing
            return False
        if self._take_is_empty():
            self._set_status("nothing to save: no take recorded (press Space to start one)")
            return False
        self._stop()
        if self._saved:
            self._set_status(f"take {self._saved_take_number():03d} is already saved to {self._last_saved_path}")
            return True
        path = self.output_path
        if self._multi_take:
            while path.exists():  # another session took this number meanwhile: never overwrite, move on
                print(f"{path.name} exists; moving on to take {self._take_number + 1:03d}")
                self._take_number += 1
                path = self.output_path
        elif path.exists():
            self._report_save_failure(f"not saved: {path} already exists (the take is kept; move that file, then save)")
            return False
        self.log.extra["acquisition"] = self._acquisition_meta()
        if self._task is not None:  # so the player draws the target when replaying the take
            self.log.extra["playback"] = {"task": self._task.to_dict()}
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
        self._refresh_saved_trails()
        if self._multi_take:
            self._take_number += 1
        print(
            f"saved take {number:03d} ({len(self.log)} samples, "
            f"{self._achieved_rate():.1f} Hz of {self._sample_rate:g} Hz requested) to {path}"
        )
        self._refresh()
        return True

    def _write_take(self, path: Path) -> None:
        """Write the log straight to ``path``, removing the partial file if the write fails.

        Raises
        ------
        OSError
            If writing fails.
        """
        try:
            self.log.save(path)
        except OSError:
            path.unlink(missing_ok=True)
            raise

    def save_and_next(self) -> bool:
        """Save the take, then reset for the next one; the reset happens only after a save.

        An empty take is not written and just resets; an already-saved take advances
        without being written again. Returns whether the take was saved.
        """
        if self._state == "preparing":  # nothing to save while posing, and no reset either
            return False
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
        its file. Pressing R while already ready changes nothing but the readout. While
        preparing, R cancels the preparation: the target returns to the ready state's too.
        """
        if self._has_unsaved_take():
            print(f"discarded unsaved take {self._take_number:03d} ({len(self.log)} samples)")
        if self._state == "preparing" and self._ready_target is not None:
            self._activate(self._ready_target)
        self.skeleton.q = self._reset_q.copy()
        self.skeleton.dq = np.zeros(self.skeleton.num_joints, dtype=np.float64)
        self.canvas.drag_point = None
        self._grab_offset = None
        self._grab_gesture = None
        self.log = self._new_log()
        self.time = 0.0
        self._ticks = 0
        self._state = "ready"
        self._saved = False
        self._refresh()

    def _report_save_failure(self, message: str) -> None:
        print(message, file=sys.stderr)
        self._set_status(message)

    # ----------------------------------------------------------- preparation ---

    def toggle_prepare(self) -> None:
        """Enter the preparation phase, or leave it with the posed arm as the ready posture of the following takes.

        Entering is allowed when ready, or when the visible take is saved: that take's log
        is dropped as a reset drops it, but the arm stays where it is, at rest. During a
        take or with a take not saved, E is refused and nothing changes. Leaving keeps the
        posture and the chosen target for every take until the next preparation, each
        joint brought by whole turns into (-180, 180] deg where its limits hold that angle,
        so the hand does not move.
        """
        if self._state == "preparing":
            self._reset_q = _turn_nearest_zero(self.skeleton)
            self._ready_target = self._active
            target = "" if self._active is None else f", target {self._target_name(self._active)}"
            print(f"take {self._take_number:03d}: prepared at q = {np.rad2deg(self._reset_q).round(1)} deg{target}")
            self.reset_take()
            return
        if not self._can_prepare():
            self._notice = " — to prepare, save (S) or discard (R) first"
            self._set_status(self._status_text())
            return
        posture = self.skeleton.q.copy()
        self.reset_take()  # a fresh log and clock at rest, the drag let go ...
        self.skeleton.q = posture  # ... with the arm where it is
        self._state = "preparing"
        self._refresh()

    def select_target(self, index: int) -> None:
        """Make target ``index`` (from 0) the active one; only while preparing, so a take's target never changes."""
        if self._state != "preparing" or not 0 <= index < len(self._targets):
            return
        self._activate(index)
        self._refresh()

    def _can_prepare(self) -> bool:
        return self._state == "ready" or (self._state == "stopped" and self._saved)

    def _activate(self, index: int) -> None:
        """Make target ``index`` the task's active one (its ``active`` and its target) and draw it emphasized."""
        if self._task is None:
            return
        self._task.params["active"] = index  # saved with each take under [extra.playback.task]
        apply_active_target(self._task)
        self._active = index
        self._draw_targets()

    def _draw_targets(self) -> None:
        """Draw a multi-target task's active target filled and the others as dashed rings."""
        if self._active is None:
            return
        pos, _, color, tolerance = self._targets[self._active]
        self.canvas.target = np.asarray(pos, dtype=np.float64)
        self.canvas.target_color = QColor(color)
        self.canvas.target_tolerance = tolerance
        self.canvas.secondary_targets = [
            (np.asarray(other, dtype=np.float64), QColor(other_color))
            for i, (other, _, other_color, _) in enumerate(self._targets)
            if i != self._active
        ]
        self.canvas.update()

    def _target_name(self, index: int) -> str:
        """Return the target's label, or its number (from 1) when it has none."""
        label = self._targets[index][1]
        return str(index + 1) if label is None else label

    def _add_preparation_controls(self, controls: QVBoxLayout, hint: QLabel) -> None:
        """With ``prepare``: explain E in the hint, and add a button per target with the keys 1 to 9 to choose it.

        The Prepare button is checked while preparing; the target buttons and keys act only then.
        """
        if not self._prepare:
            return
        keys = min(len(self._targets), _NUMBER_KEYS)
        choose = f", 1-{keys} choose its target," if self._targets else ""
        hint.setText(f"{hint.text()} E prepares a take: drag the tip to pose its start{choose} and E again to finish.")
        self.buttons["prepare"].setCheckable(True)
        if not self._targets:
            return
        controls.addWidget(QLabel("Target, chosen while preparing:"))
        grid = QGridLayout()
        group = QButtonGroup(self)  # exclusive: the checked button is the active target
        for index, (_, label, _, _) in enumerate(self._targets):
            button = QPushButton(str(index + 1) if label is None else f"{index + 1}: {label}")
            button.setCheckable(True)
            button.setFocusPolicy(Qt.FocusPolicy.NoFocus)  # Space must never click a target
            group.addButton(button, index)
            grid.addWidget(button, index // _TARGET_COLUMNS, index % _TARGET_COLUMNS)
            self.target_buttons.append(button)
            if index < _NUMBER_KEYS:
                shortcut = QShortcut(QKeySequence(str(index + 1)), self)
                shortcut.setContext(Qt.ShortcutContext.WindowShortcut)
                shortcut.setAutoRepeat(False)
                shortcut.activated.connect(functools.partial(self.select_target, index))
                self.shortcuts[f"target_{index + 1}"] = shortcut
        group.idClicked.connect(self.select_target)
        controls.addLayout(grid)

    def _pose(self) -> None:
        """Pose the arm toward the cursor by IK in either mode, at rest; nothing is logged and the clock stands."""
        self._step_ik()
        self._ticks += 1
        if self._ticks % self._display_every == 0:
            self.canvas.update_skeleton()
            self.status_label.setText(self._status_text())  # the posture being posed

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
        """Continue a take after a dialog: the pause enters neither the timestamps nor the simulated time."""
        if self._state == "recording":
            self._t0 += self._clock() - pause_start
        if self._run_timer:
            self._timer.start(self.tick_ms)

    # --------------------------------------------------------------- display ---

    def _on_overlay_toggled(self, _checked: bool) -> None:  # noqa: FBT001
        """Redraw after a checkbox change; the overlays never touch the log or the robot."""
        self._refresh_saved_trails()
        self._refresh_trails()
        self.canvas.update_skeleton()

    def _refresh_saved_trails(self) -> None:
        """Hand the canvas the faint saved trails (by display history); it caches them until the next change."""
        past = self._drawn_history() if self.show_past_trails else ()
        self.canvas.static_trails = [TrailOverlay(t.points, _PAST_TRAIL_COLOR, _PAST_TRAIL_WIDTH_PX) for t in past]

    def _refresh_trails(self) -> None:
        """Rebuild the current trail, drawn in front of the saved ones on every repaint."""
        show = self.show_tip_trail and len(self.log) >= _MIN_SAMPLES
        current = TrailOverlay(self.current_trail(), _CURRENT_TRAIL_COLOR, _CURRENT_TRAIL_WIDTH_PX) if show else None
        self.canvas.trails = [current] if current is not None else []

    def _refresh(self) -> None:
        """Repaint the arm with its overlays and update the status readout and button states."""
        self._refresh_trails()
        self.canvas.show_drag_arrow = self._mode == "dynamics" and self._state != "preparing"  # posing is kinematic
        self.canvas.update_skeleton()
        self._set_status(self._status_text())

    def _set_status(self, text: str) -> None:
        self.status_label.setText(text)
        empty = self._take_is_empty()
        self.buttons["start"].setEnabled(self._state == "ready")
        self.buttons["save"].setEnabled(not empty and not self._saved)
        self.buttons["save_next"].setEnabled(not empty)
        if self._prepare:
            self.buttons["prepare"].setEnabled(self._state == "preparing" or self._can_prepare())
            self.buttons["prepare"].setChecked(self._state == "preparing")
            for button in self.target_buttons:
                button.setEnabled(self._state == "preparing")
            if self.target_buttons and self._active is not None:
                self.target_buttons[self._active].setChecked(True)

    def _status_text(self) -> str:
        target = "" if self._active is None else f"; target {self._target_name(self._active)}"
        posture = f"; {_posture_text(self.skeleton.q)}" if self._prepare else ""  # ready, or being posed
        if self._state == "preparing":
            keys = min(len(self._targets), _NUMBER_KEYS)
            choose = f"1-{keys} choose the target, " if self._targets else ""
            return (
                f"PREPARING take {self._take_number:03d} — drag the tip to pose the start; "
                f"{choose}E finishes, R cancels{posture}{target}"
            )
        if self._state == "ready":
            how = "grab the tip" if self._start_on_grab else "press Space"
            return f"READY — take {self._take_number:03d} → {self.output_path.name}; {how} to start{posture}{target}"
        if self._state == "recording":
            cap = "" if self._duration is None else f" / {self._duration:.1f}"
            return (
                f"RECORDING take {self._take_number:03d}  t = {self.time:.2f}{cap} s,  {len(self.log)} samples"
                f"{target}{self._notice}"
            )
        if self._saved:
            return f"SAVED take {self._saved_take_number():03d} → {self._last_saved_path}"
        return (
            f"STOPPED (unsaved) take {self._take_number:03d}, {len(self.log)} samples — S saves, R discards"
            f"{self._notice}"
        )

    def show_plot(self) -> None:
        """Plot the taught tip path, final pose, the task's target or curve, and joint angles (blocking)."""
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
        path = self.canvas.overlay_path
        if path is not None:  # plotted directly: plot_trajectory would toggle the grid again
            ax_path.plot(path[:, 0], path[:, 1], color="gray", linestyle="--", label="reference")
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
    """Parse ``--sample-rate``, a best-effort request that must be positive."""
    rate = float(text)
    try:
        tick_period_ms(rate)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(str(exc)) from exc
    return rate


def _grab_radius_argument(text: str) -> float:
    """Parse ``--grab-radius``, a distance that must be positive."""
    radius = float(text)
    if radius <= 0:
        msg = f"the grab radius must be positive, got {radius:g}"
        raise argparse.ArgumentTypeError(msg)
    return radius


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
        "--prepare",
        action="store_true",
        help="add a preparation phase, toggled by E, to pose each take's start by dragging the tip and, "
        "with a multi_target_reaching task, to choose its target with 1-9",
    )
    parser.add_argument(
        "--sample-rate",
        type=_sample_rate_argument,
        default=_SAMPLE_RATE,
        help="requested logger sampling rate in Hz, a best effort; one timer tick per sample (default: 50)",
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
        "--grab-radius",
        type=_grab_radius_argument,
        default=_GRAB_RADIUS,
        help=f"a press grabs the tip only within this distance of it, in meters (default: {_GRAB_RADIUS:g})",
    )
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
    parser.add_argument(
        "--task",
        type=Path,
        default=None,
        help="TOML file whose [task] is drawn as a guide: a target, or a periodic_curve task's curve",
    )
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
        If ``--pose`` length mismatches the DOF, a ``--task`` file lacks ``[task]``, or a
        multi-target task's ``active`` index is out of range.
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
    _targets_of(task)  # applies a multi-target task's active target here, so a bad index is a usage error
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
        grab_radius=args.grab_radius,
        stiffness=args.stiffness,
        friction=args.friction,
        task=task,
        show_com=args.show_com,
        enforce_limits=not args.no_joint_limits,
        prepare=args.prepare,
    )
    window.show()
    window.canvas.setFocus()
    app.exec()

    if window.saved and args.plot:
        window.show_plot()


if __name__ == "__main__":
    main()
