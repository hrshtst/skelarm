# Copyright (C) 2025-2026 Hiroshi Atsuta <atsuta@ieee.org>
# SPDX-License-Identifier: GPL-3.0-only

"""Tests for the interactive trajectory recorder (tools/trajectory_recorder.py)."""

from __future__ import annotations

import os
import subprocess
import sys
import time
import weakref
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np
import pytest

# Importing the tool pulls in PyQt6; run headless.
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from skelarm import Skeleton
from skelarm.recording import StateLog
from tools.player import PlaybackWindow
from tools.trajectory_recorder import (
    _DISPLAY_MS,
    _DURATION,
    CloseChoice,
    RecorderWindow,
    build_parser,
    load_setup,
    numbered_output,
)

if TYPE_CHECKING:
    from collections.abc import Iterator

    from numpy.typing import NDArray

pytestmark = pytest.mark.integration

_FOUR_DOF = Path(__file__).resolve().parents[1] / "examples" / "four_dof_robot.toml"

_SCENARIO_WITH_TASK = (
    "[skeleton]\n"
    "[[skeleton.link]]\nlength = 1.0\nmass = 1.0\ninertia = 0.1\ncom = [0.5, 0.0]\nlimits = [-180.0, 180.0]\n"
    "[[skeleton.link]]\nlength = 0.8\nmass = 0.8\ninertia = 0.05\ncom = [0.4, 0.0]\nlimits = [-180.0, 180.0]\n"
    "[initial]\nq = [34.4, 57.3]\n"
    '[task]\ntype = "reaching"\ntarget = { pos = [0.55, 1.21], tolerance = 0.02 }\n'
)


@pytest.fixture(scope="module")
def qapp():  # noqa: ANN201
    """Provide a single QApplication instance for the GUI tests."""
    from PyQt6.QtWidgets import QApplication

    return QApplication.instance() or QApplication([])


class _Prompt:
    """A stand-in for the unsaved-take dialog that records how often it was shown."""

    def __init__(self, choice: CloseChoice) -> None:
        self.choice = choice
        self.calls = 0

    def __call__(self) -> CloseChoice:
        self.calls += 1
        return self.choice


class _FakeClock:
    """A deterministic stand-in for the wall clock that tests advance by hand."""

    def __init__(self) -> None:
        self.now = 1000.0

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


_CLOCKS: weakref.WeakKeyDictionary[RecorderWindow, _FakeClock] = weakref.WeakKeyDictionary()


def _window(tmp_path: Path, **overrides: object) -> RecorderWindow:
    """Build a headless 4-DOF recorder in multi-take mode at 100 Hz on a fake clock, without the live timer."""
    clock = _FakeClock()
    options: dict[str, object] = {
        "mode": "ik",
        "sample_rate": 100.0,
        "duration": 0.0,
        "output": tmp_path / "take.sklog.npz",
        "multi_take": True,
        "run_timer": False,
        "clock": clock,
    }
    options.update(overrides)
    window = RecorderWindow(Skeleton.from_toml(_FOUR_DOF), **options)  # type: ignore[arg-type]
    _CLOCKS[window] = clock
    return window


def _clock(window: RecorderWindow) -> _FakeClock:
    return _CLOCKS[window]


def _tick(window: RecorderWindow, ticks: int) -> None:
    """Advance the fake clock by one period and tick, ``ticks`` times, without dragging."""
    for _ in range(ticks):
        _clock(window).advance(window.tick_ms / 1000.0)
        window.tick()


def _cursor(window: RecorderWindow) -> Iterator[tuple[float, float]]:
    """Yield cursor positions circling near the tip, one per tick."""
    tip = window.skeleton.links[-1]
    base = (tip.xe, tip.ye)
    k = 0
    while True:
        yield (base[0] + 0.1 * np.sin(0.05 * k), base[1] + 0.1 * np.cos(0.05 * k))
        k += 1


def _move(window: RecorderWindow, ticks: int) -> None:
    """Drag the tip along a moving cursor for ``ticks`` ticks, the fake clock keeping real time."""
    cursor = _cursor(window)
    for _ in range(ticks):
        window.canvas.drag_point = next(cursor)
        _clock(window).advance(window.tick_ms / 1000.0)
        window.tick()


def _press(window: RecorderWindow, key: object, modifier: object = None) -> None:
    """Fire one key press at the window through Qt's event system."""
    from PyQt6.QtCore import Qt
    from PyQt6.QtTest import QTest
    from PyQt6.QtWidgets import QApplication

    window.show()
    window.activateWindow()
    QApplication.processEvents()
    mod = Qt.KeyboardModifier.NoModifier if modifier is None else modifier
    QTest.keyClick(window, key, mod)  # type: ignore[call-overload]  # PyQt6 stubs type QTest methods as bound
    QApplication.processEvents()


def _recorded_take(window: RecorderWindow, ticks: int = 30) -> None:
    """Start a take and drag through it (leaves the window recording)."""
    window.start()
    _move(window, ticks)


def _saved_path(window: RecorderWindow) -> Path:
    """Return the path of the last saved take, asserting one exists."""
    path = window.last_saved_path
    assert path is not None
    return path


# ----------------------------------------------------------------------------------------------
# Parser and setup
# ----------------------------------------------------------------------------------------------


def test_parser_requires_config() -> None:
    """The config argument is required."""
    with pytest.raises(SystemExit):
        build_parser().parse_args([])


def test_load_setup_robot_only() -> None:
    """A plain robot config loads the skeleton and no task."""
    skeleton, task = load_setup(build_parser().parse_args([str(_FOUR_DOF)]))
    assert skeleton.num_joints == 4  # noqa: PLR2004
    assert task is None


def test_load_setup_reads_optional_task(tmp_path: Path) -> None:
    """A config with a [task] section yields the target to draw."""
    config = tmp_path / "s.toml"
    config.write_text(_SCENARIO_WITH_TASK, encoding="utf-8")
    skeleton, task = load_setup(build_parser().parse_args([str(config)]))
    assert skeleton.num_joints == 2  # noqa: PLR2004
    assert task is not None
    assert task.target == pytest.approx([0.55, 1.21])
    assert task.tolerance == pytest.approx(0.02)


def test_parser_multi_take_and_start_on_grab_are_opt_in() -> None:
    """Numbered outputs and the first-grab start are off unless requested."""
    parser = build_parser()
    args = parser.parse_args([str(_FOUR_DOF)])
    assert args.multi_take is False
    assert args.start_on_grab is False
    args = parser.parse_args([str(_FOUR_DOF), "--multi-take", "--start-on-grab"])
    assert args.multi_take is True
    assert args.start_on_grab is True


def test_no_joint_limits_flag_disables_enforcement() -> None:
    """The ``--no-joint-limits`` flag parses and defaults to enforcing limits."""
    parser = build_parser()
    assert parser.parse_args([str(_FOUR_DOF)]).no_joint_limits is False
    assert parser.parse_args([str(_FOUR_DOF), "--no-joint-limits"]).no_joint_limits is True


def test_plot_flag_is_opt_in() -> None:
    """The trajectory plot is off by default and enabled by ``--plot``; ``--no-plot`` is gone."""
    parser = build_parser()
    assert parser.parse_args([str(_FOUR_DOF)]).plot is False
    assert parser.parse_args([str(_FOUR_DOF), "--plot"]).plot is True
    with pytest.raises(SystemExit):
        parser.parse_args([str(_FOUR_DOF), "--no-plot"])


def test_runs_as_a_standalone_script() -> None:
    """Running the file directly (script mode) must resolve all of its imports."""
    script = Path(__file__).resolve().parents[1] / "tools" / "trajectory_recorder.py"
    result = subprocess.run(  # noqa: S603  # trusted: our own interpreter and script path
        [sys.executable, str(script), "--help"],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    assert "trajectory" in result.stdout.lower()


def test_numbered_output_keeps_the_double_suffix() -> None:
    """``reach.sklog.npz`` numbers as ``reach_001.sklog.npz``; other suffixes keep theirs."""
    assert numbered_output(Path("out/reach.sklog.npz"), 1) == Path("out/reach_001.sklog.npz")
    assert numbered_output(Path("reach.npz"), 12) == Path("reach_012.npz")


# ----------------------------------------------------------------------------------------------
# Acquisition clock
# ----------------------------------------------------------------------------------------------


def test_any_positive_sample_rate_is_accepted_as_best_effort(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """The requested rate sets the timer period rounded to whole milliseconds (at least 1 ms); only <= 0 is refused."""
    assert _window(tmp_path, sample_rate=100.0).tick_ms == 10  # noqa: PLR2004
    assert _window(tmp_path, sample_rate=60.0).tick_ms == 17  # noqa: PLR2004
    assert _window(tmp_path, sample_rate=5000.0).tick_ms == 1
    assert build_parser().parse_args([str(_FOUR_DOF), "--sample-rate", "60"]).sample_rate == 60.0  # noqa: PLR2004
    with pytest.raises(ValueError, match="positive"):
        _window(tmp_path, sample_rate=0.0)
    with pytest.raises(SystemExit):
        build_parser().parse_args([str(_FOUR_DOF), "--sample-rate", "-5"])


def test_one_pose_update_per_sample(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """At 100 Hz every logged sample carries its own pose update: no repeated frames, 10 ms apart."""
    window = _window(tmp_path)
    window.start()
    _move(window, 40)
    times = window.log.times
    assert len(times) == 41  # noqa: PLR2004  # t = 0 plus one sample per tick
    assert np.allclose(np.diff(times), 0.01)
    q = window.log.channel("q")
    assert not np.any(np.all(np.isclose(q[1:], q[:-1]), axis=1))  # a moving cursor never repeats a pose


def test_display_refresh_is_throttled_independently_of_sampling(qapp, tmp_path: Path, monkeypatch) -> None:  # noqa: ANN001, ARG001
    """Repainting follows the display period; sampling follows the tick, so 100 Hz samples repaint at 50 Hz."""
    window = _window(tmp_path)
    window.start()
    repaints: list[int] = []
    monkeypatch.setattr(window.canvas, "update_skeleton", lambda: repaints.append(1))
    assert _DISPLAY_MS == 20  # noqa: PLR2004
    _move(window, 8)
    assert len(repaints) == 4  # noqa: PLR2004


def test_saved_take_reports_the_requested_and_achieved_rate(qapp, tmp_path: Path, capsys) -> None:  # noqa: ANN001, ARG001
    """A take whose ticks run slower than requested says so, in the log and on the terminal."""
    window = _window(tmp_path)  # 100 Hz requested
    window.start()
    for _ in range(20):  # every tick arrives 20 ms after the previous one: 50 Hz achieved
        _clock(window).advance(0.02)
        window.tick()
    assert window.save_take()
    meta = StateLog.load(_saved_path(window)).extra["acquisition"]
    assert meta == {"requested_rate_hz": pytest.approx(100.0), "achieved_rate_hz": pytest.approx(50.0)}
    assert "50.0 Hz of 100 Hz requested" in capsys.readouterr().out


def test_sample_times_are_the_actual_elapsed_time(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """A delayed tick is recorded as a longer interval, so a replay follows the real motion."""
    window = _window(tmp_path)
    window.start()
    _move(window, 5)
    _clock(window).advance(0.19)  # this tick arrives 200 ms after the previous one
    _move(window, 1)
    _move(window, 5)
    times = window.log.times
    assert np.allclose(np.diff(times), [0.01] * 5 + [0.2] + [0.01] * 5)
    assert window.time == pytest.approx(0.3)
    assert set(window.log.channel_names) == {"q", "tip"}


def test_cancelled_close_dialog_is_excluded_from_timing(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """The time spent in the unsaved-take warning does not enter the timestamps."""

    def cancel_after_ten_seconds() -> CloseChoice:
        _clock(window).advance(10.0)
        return CloseChoice.CANCEL

    window = _window(tmp_path, unsaved_prompt=cancel_after_ten_seconds)
    _recorded_take(window, ticks=15)
    assert not window.close()
    _move(window, 5)
    assert np.allclose(np.diff(window.log.times), 0.01)
    assert window.time == pytest.approx(0.2)


def test_output_without_npz_suffix_is_written_where_it_is_checked(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """``--output reach`` numbers, checks, and writes ``reach_001.npz``, so a second session never overwrites it."""
    window = _window(tmp_path, output=tmp_path / "reach")
    assert window.output_path == tmp_path / "reach_001.npz"
    _recorded_take(window)
    assert window.save_take()
    assert window.last_saved_path == tmp_path / "reach_001.npz"
    assert sorted(p.name for p in tmp_path.iterdir()) == ["reach_001.npz"]
    payload = (tmp_path / "reach_001.npz").read_bytes()

    second = _window(tmp_path, output=tmp_path / "reach")
    assert second.take_number == 2  # noqa: PLR2004
    _recorded_take(second)
    assert second.save_take()
    assert (tmp_path / "reach_001.npz").read_bytes() == payload
    assert (tmp_path / "reach_002.npz").exists()

    single = _window(tmp_path, output=tmp_path / "exact.sklog", multi_take=False)
    assert single.output_path == tmp_path / "exact.sklog.npz"


def test_partial_write_failure_leaves_no_file_and_allows_retry(qapp, tmp_path: Path, monkeypatch) -> None:  # noqa: ANN001, ARG001
    """A write that fails half-way leaves no file behind; the retry succeeds."""
    window = _window(tmp_path)
    _recorded_take(window, ticks=12)
    original = StateLog.save

    def failing(_self: StateLog, path: str | Path) -> None:
        Path(path).write_bytes(b"partial archive")
        raise OSError("disk full")  # noqa: EM101, TRY003

    monkeypatch.setattr(StateLog, "save", failing)
    assert not window.save_take()
    assert not window.output_path.exists()
    assert list(tmp_path.iterdir()) == []
    monkeypatch.setattr(StateLog, "save", original)
    assert window.save_take()
    assert sorted(p.name for p in tmp_path.iterdir()) == ["take_001.sklog.npz"]
    assert len(StateLog.load(window.output_path.with_name("take_001.sklog.npz"))) == 13  # noqa: PLR2004


def _disk_full(_self: StateLog, _path: str | Path) -> None:
    """Stand in for ``StateLog.save`` when the disk is full."""
    raise OSError("disk full")  # noqa: EM101, TRY003


def _unsupported_link(_src: str | Path, _dst: str | Path) -> None:
    """Stand in for ``os.link`` on a file system without hard links (FAT/exFAT, some network mounts)."""
    import errno

    raise OSError(errno.EPERM, "Operation not permitted")


def test_take_is_written_straight_to_its_name(qapp, tmp_path: Path, monkeypatch) -> None:  # noqa: ANN001, ARG001
    """The take goes directly to its output name, with no temporary, link, or rename, so FAT drives work too."""
    window = _window(tmp_path)
    _recorded_take(window, ticks=12)
    target = window.output_path
    written: list[Path] = []
    original = StateLog.save

    def recording_save(self: StateLog, path: str | Path) -> None:
        written.append(Path(path))
        original(self, path)

    monkeypatch.setattr(StateLog, "save", recording_save)
    monkeypatch.setattr(os, "link", _unsupported_link)
    assert window.save_take()
    assert written == [target]
    assert sorted(p.name for p in tmp_path.iterdir()) == [target.name]
    assert len(StateLog.load(target)) == 13  # noqa: PLR2004


# ----------------------------------------------------------------------------------------------
# Start
# ----------------------------------------------------------------------------------------------


def test_arm_stays_at_reset_until_started(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """In explicit-start mode a drag before Start neither moves the arm nor records anything."""
    window = _window(tmp_path)
    reset = window.skeleton.q.copy()
    _move(window, 20)
    assert window.state == "ready"
    assert np.array_equal(window.skeleton.q, reset)
    assert len(window.log) == 0
    assert not window.saved


def test_space_starts_from_the_stationary_reset_state(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """Space starts recording immediately: the reset state at t = 0, then the stationary pre-roll."""
    from PyQt6.QtCore import Qt

    window = _window(tmp_path)
    reset = window.skeleton.q.copy()
    _press(window, Qt.Key.Key_Space)
    assert window.state == "recording"
    assert window.time == 0.0
    _tick(window, 10)  # no drag yet: the pre-roll is logged as stationary samples
    q = window.log.channel("q")
    assert len(window.log) == 11  # noqa: PLR2004
    assert np.allclose(q, reset)
    assert window.log.times[0] == 0.0


def test_space_during_recording_never_restarts(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """A second Space (or key auto-repeat) neither resets the clock nor adds a take."""
    from PyQt6.QtCore import Qt

    window = _window(tmp_path)
    _press(window, Qt.Key.Key_Space)
    _move(window, 15)
    before = (window.time, len(window.log), window.take_number)
    _press(window, Qt.Key.Key_Space)
    assert (window.time, len(window.log), window.take_number) == before
    assert window.state == "recording"
    assert not window.shortcuts["start"].autoRepeat()


def test_start_button_starts_and_buttons_never_take_focus(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """Buttons mirror the shortcuts with visible key hints and cannot swallow Space by holding focus."""
    from PyQt6.QtCore import Qt

    window = _window(tmp_path)
    hints = {"start": "(Space)", "save": "(S)", "save_next": "(Shift+S)", "reset": "(R)"}
    for name, hint in hints.items():
        button = window.buttons[name]
        assert hint in button.text()
        assert button.focusPolicy() == Qt.FocusPolicy.NoFocus
    window.buttons["start"].click()
    assert window.state == "recording"


def test_start_on_grab_mode_is_optional(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """The legacy first-grab start still exists as an opt-in mode."""
    window = _window(tmp_path, start_on_grab=True)
    _move(window, 5)
    assert window.state == "recording"
    assert len(window.log) == 5  # noqa: PLR2004  # t = 0 at the grab tick, then one sample per later tick


def test_start_on_grab_ignores_a_button_held_through_a_reset(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """After R clears the drag, moving with the button still held neither drags nor starts the next take."""
    from PyQt6.QtCore import QEvent, QPointF, Qt
    from PyQt6.QtGui import QMouseEvent

    window = _window(tmp_path, start_on_grab=True)
    canvas = window.canvas
    canvas.resize(400, 400)
    tip = window.skeleton.links[-1]

    def mouse(kind: QEvent.Type, button: Qt.MouseButton, target: tuple[float, float]) -> QMouseEvent:
        px = canvas.width() / 2 + target[0] * canvas.scale_factor
        py = canvas.height() / 2 - target[1] * canvas.scale_factor
        return QMouseEvent(kind, QPointF(px, py), button, Qt.MouseButton.LeftButton, Qt.KeyboardModifier.NoModifier)

    canvas.mousePressEvent(mouse(QEvent.Type.MouseButtonPress, Qt.MouseButton.LeftButton, (tip.xe, tip.ye)))
    _tick(window, 5)
    assert window.state == "recording"
    window.reset_take()
    canvas.mouseMoveEvent(mouse(QEvent.Type.MouseMove, Qt.MouseButton.NoButton, (tip.xe + 0.1, tip.ye)))
    _tick(window, 5)
    assert canvas.drag_point is None
    assert window.state == "ready"


def test_grab_tick_only_starts_the_take(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """The tick that sees the first grab logs the reset state at t = 0; the first pose update is the next tick's."""
    window = _window(tmp_path, start_on_grab=True)
    reset_q = window.skeleton.q.copy()
    _move(window, 1)
    assert window.state == "recording"
    assert len(window.log) == 1
    assert np.allclose(window.skeleton.q, reset_q)  # no IK step toward the cursor yet
    _move(window, 4)
    assert np.allclose(np.diff(window.log.times), 0.01)  # no near-coincident first pair


def test_start_restarts_the_tick_timer(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """Starting a take restarts the tick timer, so the first sample follows t = 0 by one full period."""
    from PyQt6.QtCore import QTimer

    window = _window(tmp_path, sample_rate=1.0, run_timer=True)  # a one-second period
    (timer,) = window.findChildren(QTimer)
    time.sleep(0.3)  # the free-running timer is now well into its period
    window.start()
    try:
        assert timer.remainingTime() > 0.9 * window.tick_ms
    finally:
        timer.stop()


# ----------------------------------------------------------------------------------------------
# Save, save-and-next, reset
# ----------------------------------------------------------------------------------------------


def test_s_saves_and_keeps_the_take_visible(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """S stops and saves the take, keeps the window open and the arm where it is; no reset."""
    from PyQt6.QtCore import Qt

    window = _window(tmp_path)
    reset = window.skeleton.q.copy()
    _recorded_take(window)
    _press(window, Qt.Key.Key_S)
    assert window.state == "stopped"
    assert window.saved
    assert not window.closed
    assert window.last_saved_path == tmp_path / "take_001.sklog.npz"
    assert _saved_path(window).exists()
    assert not np.allclose(window.skeleton.q, reset)
    assert "take_001.sklog.npz" in window.status_label.text()
    PlaybackWindow(StateLog.load(_saved_path(window)))  # replayable in the player


def test_shift_s_saves_and_prepares_the_next_take(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """Shift+S saves, then resets posture, velocity, drag, log, and clock, and waits for Space."""
    from PyQt6.QtCore import Qt

    window = _window(tmp_path)
    reset = window.skeleton.q.copy()
    _recorded_take(window)
    _press(window, Qt.Key.Key_S, Qt.KeyboardModifier.ShiftModifier)
    assert (tmp_path / "take_001.sklog.npz").exists()
    assert window.state == "ready"
    assert window.take_number == 2  # noqa: PLR2004
    assert window.output_path == tmp_path / "take_002.sklog.npz"
    assert np.array_equal(window.skeleton.q, reset)
    assert np.array_equal(window.skeleton.dq, np.zeros(4))
    assert window.canvas.drag_point is None
    assert len(window.log) == 0
    assert window.time == 0.0
    assert not window.saved
    window.tick()  # ready: ticking does not record
    assert len(window.log) == 0


def test_plain_s_and_shift_s_are_distinct(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """One keypress fires exactly one of Save and Save-and-next: S only Save, Shift+S only Save-and-next."""
    from PyQt6.QtCore import Qt

    window = _window(tmp_path)
    fired: list[str] = []
    for name in ("save", "save_next"):
        window.shortcuts[name].activated.connect(lambda name=name: fired.append(name))
    _recorded_take(window)
    _press(window, Qt.Key.Key_S)
    assert fired == ["save"]
    assert window.state == "stopped"  # plain S did not reset
    window.reset_take()
    _recorded_take(window)
    _press(window, Qt.Key.Key_S, Qt.KeyboardModifier.ShiftModifier)
    assert fired == ["save", "save_next"]
    assert window.state == "ready"  # Shift+S saved and reset
    assert window.takes_saved == 2  # noqa: PLR2004


def test_s_then_r_equals_shift_s(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """S followed by R produces the same file, numbering, reset state, and paused recording as Shift+S."""
    a = _window(tmp_path / "a")
    b = _window(tmp_path / "b")
    for window in (a, b):
        window.output_path.parent.mkdir()
        _recorded_take(window, ticks=20)
    assert a.save_take()
    a.reset_take()
    assert b.save_and_next()

    saved_a, saved_b = StateLog.load(_saved_path(a)), StateLog.load(_saved_path(b))
    assert np.array_equal(saved_a.channel("q"), saved_b.channel("q"))
    assert np.array_equal(saved_a.times, saved_b.times)
    for window in (a, b):
        assert window.state == "ready"
        assert window.takes_saved == 1
        assert window.take_number == 2  # noqa: PLR2004
        assert window.output_path.name == "take_002.sklog.npz"
        assert len(window.log) == 0
        assert window.time == 0.0
        assert not window.saved


def test_r_discards_only_an_unsaved_take(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """R during recording discards the samples without a file; after a save it keeps the file."""
    from PyQt6.QtCore import Qt

    window = _window(tmp_path)
    _recorded_take(window)
    _press(window, Qt.Key.Key_R)
    assert window.state == "ready"
    assert len(window.log) == 0
    assert window.take_number == 1  # a discarded take consumes no number
    assert not list(tmp_path.glob("*.npz"))

    _recorded_take(window)
    assert window.save_take()
    saved = tmp_path / "take_001.sklog.npz"
    payload = saved.read_bytes()
    _press(window, Qt.Key.Key_R)
    assert saved.read_bytes() == payload
    assert window.state == "ready"
    assert window.take_number == 2  # noqa: PLR2004
    _press(window, Qt.Key.Key_R)  # R while already ready changes nothing
    assert window.take_number == 2  # noqa: PLR2004
    assert window.takes_saved == 1
    assert saved.read_bytes() == payload


def test_empty_take_is_never_written_and_consumes_no_number(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """Saving with nothing recorded creates no file and keeps the take number."""
    window = _window(tmp_path)
    assert not window.save_take()
    assert not list(tmp_path.glob("*.npz"))
    assert window.take_number == 1
    assert not window.save_and_next()
    assert window.state == "ready"
    assert window.take_number == 1
    window.start()  # only the t = 0 frame, nothing else
    assert not window.save_and_next()
    assert window.state == "ready"
    assert window.take_number == 1
    assert not list(tmp_path.glob("*.npz"))


def test_numbering_continues_after_existing_takes(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """Existing numbered files are never overwritten: numbering resumes after the highest one."""
    existing = tmp_path / "take_002.sklog.npz"
    existing.write_bytes(b"keep me")
    window = _window(tmp_path)
    assert window.take_number == 3  # noqa: PLR2004
    _recorded_take(window)
    assert window.save_take()
    assert window.last_saved_path == tmp_path / "take_003.sklog.npz"
    assert existing.read_bytes() == b"keep me"


def test_numbering_continues_after_existing_takes_of_a_base_with_glob_characters(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """A base such as ``run[1]`` is matched literally, not as a glob pattern, so its takes are found."""
    existing = tmp_path / "run[1]_004.sklog.npz"
    existing.write_bytes(b"keep me")
    window = _window(tmp_path, output=tmp_path / "run[1].sklog.npz")
    assert window.take_number == 5  # noqa: PLR2004
    _recorded_take(window)
    assert window.save_take()
    assert window.last_saved_path == tmp_path / "run[1]_005.sklog.npz"
    assert existing.read_bytes() == b"keep me"


def test_already_saved_message_names_the_saved_take(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """Saving a saved take again reports that take's number, in single-file and multi-take mode alike."""
    single = _window(tmp_path, output=tmp_path / "exact.sklog.npz", multi_take=False)
    _recorded_take(single)
    assert single.save_take()
    assert single.save_take()
    assert "take 001 is already saved" in single.status_label.text()

    numbered = _window(tmp_path)
    _recorded_take(numbered)
    assert numbered.save_take()
    assert numbered.save_take()
    assert "take 001 is already saved" in numbered.status_label.text()


def test_multi_take_collision_moves_on_to_the_next_free_number(qapp, tmp_path: Path, capsys) -> None:  # noqa: ANN001, ARG001
    """Files appearing under the next numbers mid-session are skipped, never overwritten, and never block saving."""
    window = _window(tmp_path)
    _recorded_take(window)
    (tmp_path / "take_001.sklog.npz").write_bytes(b"another session's take")
    (tmp_path / "take_002.sklog.npz").write_bytes(b"and another")
    assert window.save_take()
    assert window.last_saved_path == tmp_path / "take_003.sklog.npz"
    assert (tmp_path / "take_001.sklog.npz").read_bytes() == b"another session's take"
    assert (tmp_path / "take_002.sklog.npz").read_bytes() == b"and another"
    assert window.take_number == 4  # noqa: PLR2004
    assert [trail.take for trail in window.saved_trails()] == [3]
    out = capsys.readouterr().out
    assert "take_001.sklog.npz exists" in out
    assert "saved take 003" in out


def test_single_file_collision_is_refused_before_writing(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """In single-file mode an existing output makes the save fail, byte for byte untouched, and retryable."""
    window = _window(tmp_path, output=tmp_path / "exact.sklog.npz", multi_take=False)
    _recorded_take(window)
    target = window.output_path
    target.write_bytes(b"someone else's take")
    assert not window.save_take()
    assert target.read_bytes() == b"someone else's take"
    assert window.state == "stopped"
    assert not window.saved
    assert "exists" in window.status_label.text()
    target.unlink()
    assert window.save_take()  # the unsaved take was retained for the retry
    assert window.saved


def test_save_failure_retains_the_take_for_retry(qapp, tmp_path: Path, monkeypatch) -> None:  # noqa: ANN001, ARG001
    """An I/O failure leaves the take unsaved and the recording data intact."""
    window = _window(tmp_path)
    _recorded_take(window, ticks=12)
    samples = len(window.log)
    original = StateLog.save

    def failing(_self: StateLog, _path: str | Path) -> None:
        raise OSError("disk full")  # noqa: EM101, TRY003

    monkeypatch.setattr(StateLog, "save", failing)
    assert not window.save_take()
    assert not window.saved
    assert window.state == "stopped"
    assert len(window.log) == samples
    assert not list(tmp_path.glob("*.npz"))
    monkeypatch.setattr(StateLog, "save", original)
    assert window.save_take()
    assert window.takes_saved == 1


def test_single_take_output_is_exact_and_never_overwritten(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """Without --multi-take the exact --output name is used once; a second take is refused, not overwritten."""
    out = tmp_path / "exact.sklog.npz"
    window = _window(tmp_path, output=out, multi_take=False)
    assert window.output_path == out
    _recorded_take(window)
    assert window.save_and_next()
    payload = out.read_bytes()
    assert window.state == "ready"
    _recorded_take(window)
    assert not window.save_take()
    assert out.read_bytes() == payload


def test_duration_cap_saves_and_leaves_the_window_open(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """Reaching the cap stops and saves the take but never closes the window."""
    window = _window(tmp_path, duration=0.2)
    window.start()
    _move(window, 50)  # far past the cap
    assert window.state == "stopped"
    assert window.saved
    assert not window.closed
    assert len(window.log) == 21  # noqa: PLR2004  # t = 0 .. 0.20 s at 10 ms
    assert window.time == pytest.approx(0.2)


def test_nonpositive_duration_records_until_saved(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """A zero or negative duration never auto-stops; S ends and saves the recording."""
    window = _window(tmp_path, duration=-1.0, sample_rate=50.0)
    window.start()
    _move(window, int(1.5 * _DURATION * 1000 / window.tick_ms))
    assert window.state == "recording"
    assert window.time > _DURATION
    assert "/" not in window.status_label.text()  # no "t / duration" cap in the readout
    assert window.save_take()


def test_f_has_no_action(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """The former Finish binding is gone: F changes nothing and writes nothing."""
    from PyQt6.QtCore import Qt

    window = _window(tmp_path)
    _recorded_take(window)
    before = (window.state, len(window.log), window.time)
    _press(window, Qt.Key.Key_F)
    assert (window.state, len(window.log), window.time) == before
    assert not window.closed
    assert not list(tmp_path.glob("*.npz"))
    assert not hasattr(window, "finish_button")


# ----------------------------------------------------------------------------------------------
# Close
# ----------------------------------------------------------------------------------------------


def test_q_shortcut_bound_without_auto_repeat(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """The recorder binds Q to close, which runs the same unsaved-take check as the window's X."""
    window = _window(tmp_path)
    assert window.quit_shortcut.key().toString() == "Q"
    assert not window.quit_shortcut.autoRepeat()


def test_close_without_unsaved_data_asks_nothing_and_writes_nothing(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """An empty or already-saved take closes silently and never writes a second file."""
    prompt = _Prompt(CloseChoice.CANCEL)
    window = _window(tmp_path, unsaved_prompt=prompt)
    assert window.close()
    assert window.closed
    assert prompt.calls == 0

    window = _window(tmp_path, unsaved_prompt=prompt)
    _recorded_take(window)
    assert window.save_take()
    payload = _saved_path(window).read_bytes()
    assert window.close()
    assert prompt.calls == 0
    assert _saved_path(window).read_bytes() == payload
    assert len(list(tmp_path.glob("*.npz"))) == 1


def test_cancel_keeps_the_take_and_adds_no_time(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """Cancel (and dismissing the dialog) leaves the window open and the recording exactly where it was."""
    prompt = _Prompt(CloseChoice.CANCEL)
    window = _window(tmp_path, unsaved_prompt=prompt)
    _recorded_take(window, ticks=15)
    before = (window.state, len(window.log), window.time)
    assert not window.close()
    assert prompt.calls == 1
    assert not window.closed
    assert (window.state, len(window.log), window.time) == before
    _tick(window, 1)  # recording resumes seamlessly
    assert len(window.log) == before[1] + 1
    assert not list(tmp_path.glob("*.npz"))


def test_save_and_close_uses_the_same_save_path(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """Save and close writes through the ordinary save and closes only on success."""
    prompt = _Prompt(CloseChoice.SAVE)
    window = _window(tmp_path, unsaved_prompt=prompt)
    _recorded_take(window)
    assert window.close()
    assert prompt.calls == 1
    assert window.closed
    assert window.last_saved_path == tmp_path / "take_001.sklog.npz"
    assert _saved_path(window).exists()


def test_save_and_close_failure_keeps_the_window(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """A failing save during close leaves the window and the unsaved take available for retry."""
    prompt = _Prompt(CloseChoice.SAVE)
    window = _window(tmp_path, unsaved_prompt=prompt, multi_take=False)
    _recorded_take(window)
    window.output_path.write_bytes(b"collision")
    assert not window.close()
    assert not window.closed
    assert window.state == "stopped"
    assert not window.saved
    assert window.output_path.read_bytes() == b"collision"


def test_discard_and_close_writes_nothing(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """Discard and close needs that explicit choice and then leaves no file behind."""
    prompt = _Prompt(CloseChoice.DISCARD)
    window = _window(tmp_path, unsaved_prompt=prompt)
    _recorded_take(window)
    assert window.close()
    assert window.closed
    assert prompt.calls == 1
    assert not list(tmp_path.glob("*.npz"))


def test_q_key_runs_the_close_check(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """Q goes through the same handler as the window close button."""
    from PyQt6.QtCore import Qt

    prompt = _Prompt(CloseChoice.CANCEL)
    window = _window(tmp_path, unsaved_prompt=prompt)
    _recorded_take(window)
    _press(window, Qt.Key.Key_Q)
    assert prompt.calls == 1
    assert not window.closed
    assert window.state == "recording"


# ----------------------------------------------------------------------------------------------
# Modes and channels
# ----------------------------------------------------------------------------------------------


def test_ik_mode_records_q_and_tip_and_replays(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """IK mode records joint angles + tip at the sample rate; the log replays."""
    out = tmp_path / "ik.sklog.npz"
    window = _window(tmp_path, output=out, multi_take=False, sample_rate=50.0, duration=1.0)
    assert not window.canvas.show_drag_arrow  # no force cue in the kinematic IK drag
    window.start()
    _move(window, 200)
    assert window.saved
    assert out.exists()
    assert set(window.log.channel_names) == {"q", "tip"}
    assert len(window.log) == 51  # noqa: PLR2004  # t = 0 plus 50 samples up to the 1 s cap
    PlaybackWindow(StateLog.load(out))  # replayable in the player


def test_dynamics_mode_records_force_and_replays(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """Dynamics mode also records dq and the external force; the log replays."""
    out = tmp_path / "dyn.sklog.npz"
    window = _window(tmp_path, output=out, multi_take=False, mode="dynamics", sample_rate=100.0, duration=0.4)
    assert window.canvas.show_drag_arrow  # the force arrow is shown in dynamics mode
    window.start()
    _move(window, 100)
    assert window.saved
    assert {"q", "tip", "dq", "ext_force"} <= set(window.log.channel_names)
    assert len(window.log) == 41  # noqa: PLR2004
    PlaybackWindow(StateLog.load(out))


def test_dynamics_mode_simulates_the_elapsed_time(qapp, tmp_path: Path, monkeypatch) -> None:  # noqa: ANN001, ARG001
    """The physics covers the real time since the previous sample in fixed substeps, capped after a stall.

    So ``dq`` agrees with the logged times, a stall cannot snowball into ever longer catch-up work,
    and a cancelled close dialog is not simulated.
    """
    from skelarm import integrate_with_limits

    steps: list[float] = []

    def counting(
        skeleton: Skeleton,
        tau: NDArray[np.float64],
        dt: float,
        lower: NDArray[np.float64] | None,
        upper: NDArray[np.float64] | None,
    ) -> None:
        steps.append(dt)
        integrate_with_limits(skeleton, tau, dt, lower, upper)

    def cancel_after_ten_seconds() -> CloseChoice:
        _clock(window).advance(10.0)
        return CloseChoice.CANCEL

    monkeypatch.setattr("tools.trajectory_recorder.integrate_with_limits", counting)
    window = _window(tmp_path, mode="dynamics", unsaved_prompt=cancel_after_ten_seconds)  # 100 Hz: 2.5 ms substeps
    window.start()
    for elapsed, substeps in ((0.01, 4), (0.02, 8), (0.2, 12)):  # on time, one period late, a stall (capped)
        steps.clear()
        _clock(window).advance(elapsed)
        window.tick()
        assert len(steps) == substeps
        assert np.allclose(steps, 0.0025)
    assert not window.close()
    steps.clear()
    _clock(window).advance(0.01)
    window.tick()
    assert len(steps) == 4  # noqa: PLR2004  # the ten seconds in the dialog are not simulated


def test_dynamics_mode_records_the_friction_channel(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """Dynamics mode records the applied viscous friction; ik mode has no such channel."""
    window = _window(tmp_path, mode="dynamics", friction=0.3)
    window.start()
    _move(window, 10)
    assert window.log.channel("friction")[-1] == pytest.approx(0.3)

    ik = _window(tmp_path, mode="ik")
    ik.start()
    _move(ik, 10)
    assert "friction" not in ik.log.channel_names


def test_dynamics_enforce_limits_toggles_the_hard_stop(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """``enforce_limits`` controls whether the integrator gets joint bounds (default on)."""
    enforced = _window(tmp_path, mode="dynamics", enforce_limits=True)
    assert enforced._lower is not None  # noqa: SLF001
    assert enforced._upper is not None  # noqa: SLF001

    free = _window(tmp_path, mode="dynamics", enforce_limits=False)
    assert free._lower is None  # noqa: SLF001
    assert free._upper is None  # noqa: SLF001


def test_control_panel_width_is_fixed(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """The side panel keeps a constant width, independent of the status text."""
    from tools.trajectory_recorder import _PANEL_WIDTH_PX

    window = _window(tmp_path)
    panel = window.controls_panel
    assert panel.minimumWidth() == panel.maximumWidth() == _PANEL_WIDTH_PX


# ----------------------------------------------------------------------------------------------
# Tip trails (current take and faint saved history)
# ----------------------------------------------------------------------------------------------


def test_trail_flags_are_opt_in() -> None:
    """Both overlays are off unless requested, independently."""
    parser = build_parser()
    args = parser.parse_args([str(_FOUR_DOF)])
    assert args.show_tip_trail is False
    assert args.show_past_trails is False
    args = parser.parse_args([str(_FOUR_DOF), "--show-tip-trail"])
    assert (args.show_tip_trail, args.show_past_trails) == (True, False)
    args = parser.parse_args([str(_FOUR_DOF), "--show-past-trails"])
    assert (args.show_tip_trail, args.show_past_trails) == (False, True)


def test_current_trail_is_the_logged_tip_path(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """The current trail is exactly the logged (FK) tip samples, not the cursor path."""
    window = _window(tmp_path, show_tip_trail=True)
    _recorded_take(window, ticks=20)
    tip = window.log.channel("tip")
    assert np.array_equal(window.current_trail(), tip)
    assert len(window.canvas.trails) == 1
    assert np.array_equal(window.canvas.trails[0].points, tip)
    assert window.canvas.trails[0].color.alpha() > 200  # noqa: PLR2004  # the current take is drawn clearly

    hidden = _window(tmp_path / "hidden")
    hidden.output_path.parent.mkdir()
    _recorded_take(hidden, ticks=20)
    assert np.array_equal(hidden.current_trail(), hidden.log.channel("tip"))  # always derivable
    assert hidden.canvas.trails == []  # but not drawn unless asked


def test_only_saved_takes_enter_the_history_without_duplicates(qapp, tmp_path: Path, monkeypatch) -> None:  # noqa: ANN001, ARG001
    """History gains one entry per successful save, never for discarded, failed, or re-saved takes."""
    window = _window(tmp_path, show_past_trails=True)
    _recorded_take(window)
    window.reset_take()  # unsaved: discarded
    assert window.saved_trails() == ()

    _recorded_take(window)
    monkeypatch.setattr(StateLog, "save", _disk_full)
    assert not window.save_take()  # failed: nothing enters the history
    assert window.saved_trails() == ()
    monkeypatch.undo()
    assert window.save_take()
    (first,) = window.saved_trails()
    assert first.take == 1
    assert first.path == tmp_path / "take_001.sklog.npz"
    assert np.array_equal(first.points, StateLog.load(first.path).channel("tip"))
    assert window.save_and_next()  # already saved: advances without a second entry
    window.reset_take()
    window.reset_take()
    assert [trail.take for trail in window.saved_trails()] == [1]

    _recorded_take(window)
    assert window.save_and_next()
    assert [trail.take for trail in window.saved_trails()] == [1, 2]


def test_s_then_r_and_shift_s_leave_identical_history(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """Both paths add the saved trail exactly once, with the same geometry and take number."""
    a = _window(tmp_path / "a", show_past_trails=True, show_tip_trail=True)
    b = _window(tmp_path / "b", show_past_trails=True, show_tip_trail=True)
    for window in (a, b):
        window.output_path.parent.mkdir()
        _recorded_take(window, ticks=20)
    assert a.save_take()
    a.reset_take()
    assert b.save_and_next()
    for window in (a, b):
        (trail,) = window.saved_trails()
        assert trail.take == 1
        assert len(window.canvas.trails) == 1  # the faint saved trail; no current trail while ready
        assert window.canvas.trails[0].color.alpha() < 100  # noqa: PLR2004  # faint
    assert np.array_equal(a.saved_trails()[0].points, b.saved_trails()[0].points)


def test_reset_clears_only_the_unsaved_trail(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """Saved trails survive reset; an unsaved trail disappears with its take."""
    window = _window(tmp_path, show_tip_trail=True, show_past_trails=True)
    _recorded_take(window)
    assert window.save_take()
    window.reset_take()
    assert [np.array_equal(t.points, window.saved_trails()[0].points) for t in window.canvas.trails] == [True]
    _recorded_take(window, ticks=10)
    assert len(window.canvas.trails) == 2  # noqa: PLR2004  # past (behind) then current (in front)
    assert window.canvas.trails[0].color.alpha() < window.canvas.trails[1].color.alpha()
    window.reset_take()
    assert len(window.canvas.trails) == 1
    assert len(window.saved_trails()) == 1


def test_display_toggles_never_change_logs_or_pose(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """The checkboxes only change what is drawn: samples, timestamps, and posture stay untouched."""
    from PyQt6.QtCore import Qt

    window = _window(tmp_path)
    _recorded_take(window, ticks=25)
    q, tip, times = window.log.channel("q").copy(), window.log.channel("tip").copy(), window.log.times.copy()
    pose = window.skeleton.q.copy()
    for name in ("tip_trail", "past_trails"):
        box = window.checkboxes[name]
        assert box.focusPolicy() == Qt.FocusPolicy.NoFocus  # Space must never toggle a box
        box.setChecked(True)
        box.setChecked(False)
        box.setChecked(True)
    assert len(window.canvas.trails) == 1  # tip trail on, no history yet
    assert np.array_equal(window.log.channel("q"), q)
    assert np.array_equal(window.log.channel("tip"), tip)
    assert np.array_equal(window.log.times, times)
    assert np.array_equal(window.skeleton.q, pose)
    assert window.state == "recording"
    window.checkboxes["tip_trail"].setChecked(False)
    assert window.canvas.trails == []


def test_saved_takes_carry_no_display_metadata(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """The overlays are a drawing aid only: a saved log holds the samples, not what was on screen."""
    window = _window(tmp_path, show_tip_trail=True, show_past_trails=True)
    _save_takes(window, 20, 20)
    for number in (1, 2):
        assert "display" not in StateLog.load(tmp_path / f"take_{number:03d}.sklog.npz").extra


def test_fresh_session_starts_without_history(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """Files already on disk only advance the numbering; they are never shown as past trails."""
    (tmp_path / "take_002.sklog.npz").write_bytes(b"practice")
    window = _window(tmp_path, show_past_trails=True, show_tip_trail=True)
    assert window.take_number == 3  # noqa: PLR2004
    assert window.saved_trails() == ()
    assert window.canvas.trails == []


def test_trails_are_drawn_and_can_be_hidden(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """Showing the trail changes the rendered pixels; hiding it restores the plain arm."""
    window = _window(tmp_path)
    window.resize(640, 480)
    window.show()
    _recorded_take(window, ticks=40)
    plain = window.canvas.grab().toImage()
    window.checkboxes["tip_trail"].setChecked(True)
    with_trail = window.canvas.grab().toImage()
    window.checkboxes["tip_trail"].setChecked(False)
    hidden_again = window.canvas.grab().toImage()
    assert with_trail != plain
    assert hidden_again == plain


# ----------------------------------------------------------------------------------------------
# Display history of the faint saved trails (all saved takes, or only the last one)
# ----------------------------------------------------------------------------------------------


def _save_takes(window: RecorderWindow, *ticks: int, save_then_reset: bool = False) -> None:
    """Record and save one take per entry of ``ticks`` (drag ticks per take), by Shift+S or by S then R."""
    for count in ticks:
        _recorded_take(window, ticks=count)
        if save_then_reset:
            assert window.save_take()
            window.reset_take()
        else:
            assert window.save_and_next()


def test_past_trail_history_option_defaults_to_all(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """The display history is ``all`` unless ``last`` is requested; any other value is rejected."""
    parser = build_parser()
    assert parser.parse_args([str(_FOUR_DOF)]).past_trail_history == "all"
    assert parser.parse_args([str(_FOUR_DOF), "--past-trail-history", "last"]).past_trail_history == "last"
    with pytest.raises(SystemExit):
        parser.parse_args([str(_FOUR_DOF), "--past-trail-history", "recent"])
    assert _window(tmp_path).past_trail_history == "all"
    assert _window(tmp_path, past_trail_history="last").past_trail_history == "last"
    with pytest.raises(ValueError, match="past_trail_history"):
        _window(tmp_path, past_trail_history="recent")


def test_last_history_draws_only_the_latest_saved_trail(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """After three saves the canvas draws one faint trail, the latest saved take, behind the current trail."""
    window = _window(tmp_path, show_tip_trail=True, show_past_trails=True, past_trail_history="last")
    _save_takes(window, 20, 25, 30)
    assert [trail.take for trail in window.saved_trails()] == [1, 2, 3]  # the session history is unchanged
    assert sorted(path.name for path in tmp_path.glob("*.npz")) == [f"take_{n:03d}.sklog.npz" for n in (1, 2, 3)]
    latest = window.saved_trails()[-1]
    assert np.array_equal(latest.points, StateLog.load(latest.path).channel("tip"))
    assert [np.array_equal(t.points, latest.points) for t in window.canvas.trails] == [True]  # ready: past only

    _recorded_take(window, ticks=16)  # an even count: the display refreshes every second tick at 100 Hz
    assert len(window.canvas.trails) == 2  # noqa: PLR2004  # one past trail plus the current trail
    past, current = window.canvas.trails
    assert np.array_equal(past.points, latest.points)
    assert np.array_equal(current.points, window.log.channel("tip"))
    assert past.color.alpha() < current.color.alpha()  # faint behind, clear in front


def test_last_history_after_s_then_r_equals_shift_s(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """S then R and Shift+S leave the same history, with only the last saved take drawn."""
    a = _window(tmp_path / "a", show_past_trails=True, past_trail_history="last")
    b = _window(tmp_path / "b", show_past_trails=True, past_trail_history="last")
    for window, save_then_reset in ((a, True), (b, False)):
        window.output_path.parent.mkdir()
        _save_takes(window, 20, 20, 20, save_then_reset=save_then_reset)
    for window in (a, b):
        assert window.state == "ready"
        assert [trail.take for trail in window.saved_trails()] == [1, 2, 3]
        assert [np.array_equal(t.points, window.saved_trails()[-1].points) for t in window.canvas.trails] == [True]


def test_last_history_failed_save_does_not_change_the_visible_source(qapp, tmp_path: Path, monkeypatch) -> None:  # noqa: ANN001, ARG001
    """A failed save adds nothing to draw: the last successfully saved take stays the visible source."""
    window = _window(tmp_path, show_past_trails=True, past_trail_history="last")
    _save_takes(window, 20, 25)
    last_saved = window.saved_trails()[-1]
    _recorded_take(window, ticks=30)
    monkeypatch.setattr(StateLog, "save", _disk_full)
    assert not window.save_take()
    assert [trail.take for trail in window.saved_trails()] == [1, 2]
    assert [np.array_equal(t.points, last_saved.points) for t in window.canvas.trails] == [True]
    window.reset_take()  # discard the take whose save failed
    monkeypatch.undo()

    _recorded_take(window, ticks=30)
    assert window.save_take()
    assert [trail.take for trail in window.saved_trails()] == [1, 2, 3]


def test_last_history_reset_of_an_unsaved_take_keeps_the_last_saved_trail(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """R drops only the unsaved current trail; the last saved trail stays drawn and the history stays whole."""
    window = _window(tmp_path, show_tip_trail=True, show_past_trails=True, past_trail_history="last")
    _save_takes(window, 20, 25)
    last_saved = window.saved_trails()[-1]
    _recorded_take(window, ticks=30)
    assert len(window.canvas.trails) == 2  # noqa: PLR2004  # the last saved trail and the unsaved current trail
    for _ in range(2):  # the second R, while ready, changes nothing
        window.reset_take()
        assert [np.array_equal(t.points, last_saved.points) for t in window.canvas.trails] == [True]
    assert [trail.take for trail in window.saved_trails()] == [1, 2]
    assert not (tmp_path / "take_003.sklog.npz").exists()


def test_last_history_toggles_never_change_logs(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """In ``last`` mode the checkboxes only change the drawing, never the samples or the saved log."""
    window = _window(tmp_path, past_trail_history="last")
    _save_takes(window, 20, 25)
    _recorded_take(window, ticks=25)  # both overlays hidden when the take starts
    q, tip, times = window.log.channel("q").copy(), window.log.channel("tip").copy(), window.log.times.copy()
    pose = window.skeleton.q.copy()
    for name in ("tip_trail", "past_trails"):
        box = window.checkboxes[name]
        box.setChecked(True)
        box.setChecked(False)
        box.setChecked(True)
    assert len(window.canvas.trails) == 2  # noqa: PLR2004
    past, current = window.canvas.trails
    assert np.array_equal(past.points, window.saved_trails()[-1].points)
    assert np.array_equal(current.points, tip)
    assert np.array_equal(window.log.channel("q"), q)
    assert np.array_equal(window.log.channel("tip"), tip)
    assert np.array_equal(window.log.times, times)
    assert np.array_equal(window.skeleton.q, pose)
    assert window.state == "recording"

    assert window.save_take()
    saved = StateLog.load(tmp_path / "take_003.sklog.npz")
    assert np.array_equal(saved.channel("q"), q)


def test_all_history_still_draws_every_saved_trail(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """The default ``all`` mode draws every saved take behind the current trail."""
    window = _window(tmp_path, show_tip_trail=True, show_past_trails=True)
    _save_takes(window, 20, 25, 30)
    _recorded_take(window, ticks=16)  # an even count: the display refreshes every second tick at 100 Hz
    *past, current = window.canvas.trails
    assert len(past) == 3  # noqa: PLR2004
    assert all(np.array_equal(p.points, s.points) for p, s in zip(past, window.saved_trails(), strict=True))
    assert np.array_equal(current.points, window.log.channel("tip"))
