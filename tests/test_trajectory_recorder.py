# Copyright (C) 2025-2026 Hiroshi Atsuta <atsuta@ieee.org>
# SPDX-License-Identifier: GPL-3.0-only

"""Tests for the interactive trajectory recorder (tools/trajectory_recorder.py)."""

from __future__ import annotations

import os
import subprocess
import sys
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


def test_sample_rate_must_give_a_whole_millisecond_tick(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """The timer tick equals the sample period, so the period must be a whole number of ms."""
    assert _window(tmp_path, sample_rate=100.0).tick_ms == 10  # noqa: PLR2004
    assert _window(tmp_path, sample_rate=50.0).tick_ms == 20  # noqa: PLR2004
    with pytest.raises(ValueError, match="millisecond"):
        _window(tmp_path, sample_rate=60.0)
    with pytest.raises(SystemExit):
        build_parser().parse_args([str(_FOUR_DOF), "--sample-rate", "60"])


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


def test_acquisition_metadata_records_the_clock_and_tick_timing(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """Every saved take declares the nominal tick clock and the realized wall-clock tick spacing."""
    window = _window(tmp_path)
    _recorded_take(window, ticks=25)
    assert window.save_take()
    meta = StateLog.load(_saved_path(window)).extra["acquisition"]
    assert meta["clock"] == "wall-clock"
    assert meta["tick_period_s"] == pytest.approx(0.01)
    assert meta["pose_updates_per_sample"] == 1
    assert meta["ticks"] == len(window.log) - 1
    assert meta["wall_max_tick_s"] >= meta["wall_mean_tick_s"] >= 0.0
    assert meta["late_ticks"] >= 0


def test_sample_times_are_the_actual_elapsed_time(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """A delayed tick is recorded as a longer interval; the nominal tick clock is kept as its own channel."""
    window = _window(tmp_path)
    window.start()
    _move(window, 5)
    _clock(window).advance(0.19)  # this tick arrives 200 ms after the previous one
    _move(window, 1)
    _move(window, 5)
    times = window.log.times
    assert np.allclose(np.diff(times), [0.01] * 5 + [0.2] + [0.01] * 5)
    assert np.allclose(np.diff(window.log.channel("nominal_time")), 0.01)
    assert window.time == pytest.approx(0.3)
    assert window.save_take()
    meta = StateLog.load(_saved_path(window)).extra["acquisition"]
    assert meta["late_ticks"] == 1
    assert meta["wall_max_tick_s"] == pytest.approx(0.2)


def test_cancelled_close_dialog_is_excluded_from_timing(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """The time spent in the unsaved-take warning enters neither the timestamps nor the tick statistics."""

    def cancel_after_ten_seconds() -> CloseChoice:
        _clock(window).advance(10.0)
        return CloseChoice.CANCEL

    window = _window(tmp_path, unsaved_prompt=cancel_after_ten_seconds)
    _recorded_take(window, ticks=15)
    assert not window.close()
    _move(window, 5)
    assert np.allclose(np.diff(window.log.times), 0.01)
    assert window.time == pytest.approx(0.2)
    assert window.save_take()
    meta = StateLog.load(_saved_path(window)).extra["acquisition"]
    assert meta["late_ticks"] == 0
    assert meta["wall_max_tick_s"] == pytest.approx(0.01)


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
    assert len(window.log) == 6  # noqa: PLR2004


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
    """One keypress invokes exactly one of Save and Save-and-next."""
    from PyQt6.QtCore import Qt

    window = _window(tmp_path)
    _recorded_take(window)
    _press(window, Qt.Key.Key_S)
    assert window.state == "stopped"  # plain S did not reset
    assert window.shortcuts["save"].key().toString() == "S"
    assert window.shortcuts["save_next"].key().toString() == "Shift+S"


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


def test_collision_is_refused_before_writing(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """A file appearing under the next name makes the save fail, byte for byte untouched, and retryable."""
    window = _window(tmp_path)
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
    window = _window(tmp_path, unsaved_prompt=prompt)
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
    assert set(window.log.channel_names) == {"q", "tip", "nominal_time"}
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
