# Copyright (C) 2025-2026 Hiroshi Atsuta <atsuta@ieee.org>
# SPDX-License-Identifier: GPL-3.0-only

"""Tests for the playback/analysis tool (tools/player.py)."""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

# Importing the tool pulls in PyQt6; run headless.
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from skelarm.recording import StateLog
from skelarm.skeleton import LinkProp, Skeleton
from tools.player import PlaybackWindow, build_parser

pytestmark = pytest.mark.integration


@pytest.fixture(scope="module")
def qapp():  # noqa: ANN201
    """Provide a single QApplication instance for the GUI tests."""
    from PyQt6.QtWidgets import QApplication

    return QApplication.instance() or QApplication([])


def _log(frames: int = 5) -> StateLog:
    """Build a small two-link log whose pose changes each frame."""
    link_props = [LinkProp(length=1.0, m=1.0, i=0.1, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi) for _ in range(2)]
    log = StateLog(
        Skeleton(link_props),
        producer="test",
        channel_meta={"q": {"unit": "rad", "columns": ["q1", "q2"]}, "tau": {"unit": "N*m"}},
    )
    for k in range(frames):
        log.record(0.1 * k, q=[0.1 * k, -0.05 * k], dq=[0.0, 0.0], tau=[0.0, 0.0])
    return log


_EXAMPLES = Path(__file__).resolve().parents[1] / "examples"


def _embedded_log(config_name: str, tmp_path: Path) -> StateLog:
    """Run an example scenario headlessly to get a log embedding its task config."""
    from tools._scenario_cli import build_scenario, save_scenario_run

    out = tmp_path / "run.sklog.npz"
    save_scenario_run(build_scenario(_EXAMPLES / config_name), out, duration=0.2, enforce_limits=True)
    return StateLog.load(out)


def test_player_draws_curve_overlay_and_toggles(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """A periodic-curve log draws the reference path, toggleable via the checkbox."""
    window = PlaybackWindow(_embedded_log("periodic_curve.toml", tmp_path))
    assert window._has_reference  # noqa: SLF001
    assert window.canvas.overlay_path is not None
    assert window.canvas.overlay_path.shape[1] == 2  # noqa: PLR2004
    window.reference_checkbox.setChecked(False)
    assert window.canvas.show_overlay_path is False


def test_player_emphasizes_the_active_target(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """A multi-target log draws every candidate with exactly one active."""
    window = PlaybackWindow(_embedded_log("multi_target.toml", tmp_path))
    assert window._has_targets  # noqa: SLF001
    assert len(window.canvas.overlay_targets) == 3  # noqa: PLR2004
    assert sum(1 for *_rest, active in window.canvas.overlay_targets if active) == 1
    window.target_checkbox.setChecked(False)
    assert window.canvas.show_overlay_targets is False


def test_replay_follows_recorded_active_target_switches(qapp) -> None:  # noqa: ANN001, ARG001
    """The overlay's active marker follows the recorded active_target channel per frame."""
    from skelarm.scenario import load_scenario
    from tools.multi_target_simulator import MultiTargetReachSimulator

    sim = MultiTargetReachSimulator(load_scenario(_EXAMPLES / "multi_target.toml"))
    sim.step()
    sim.switch_to(2)
    sim.step()
    assert sim.state_log is not None

    window = PlaybackWindow(sim.state_log)
    window._show_frame(0)  # noqa: SLF001
    assert [active for *_rest, active in window.canvas.overlay_targets] == [True, False, False]
    window._show_frame(len(sim.state_log) - 1)  # noqa: SLF001
    assert [active for *_rest, active in window.canvas.overlay_targets] == [False, False, True]


def _force_log(frames: int = 5) -> StateLog:
    """A small two-link log that also records an external tip force per frame."""
    link_props = [LinkProp(length=1.0, m=1.0, i=0.1, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi) for _ in range(2)]
    log = StateLog(
        Skeleton(link_props),
        producer="test",
        channel_meta={
            "q": {"unit": "rad", "columns": ["q1", "q2"]},
            "ext_force": {"unit": "N", "columns": ["fx", "fy"]},
        },
    )
    for k in range(frames):
        log.record(0.1 * k, q=[0.1 * k, -0.05 * k], ext_force=[0.5 * k, -0.2 * k])
    return log


def test_parser_requires_logfile() -> None:
    """The logfile argument is required."""
    with pytest.raises(SystemExit):
        build_parser().parse_args([])


def test_player_starts_at_first_frame(qapp) -> None:  # noqa: ANN001, ARG001
    """The player opens on frame 0 with the slider spanning all frames."""
    log = _log()
    window = PlaybackWindow(log)

    assert window.frame == 0
    assert window.skeleton.q == pytest.approx(log.channel("q")[0])
    assert (window.slider.minimum(), window.slider.maximum()) == (0, len(log) - 1)


def test_control_panel_width_is_fixed_regardless_of_time_text(qapp) -> None:  # noqa: ANN001, ARG001
    """The side panel keeps a constant width even as the time/frame readout grows."""
    from tools.player import _PANEL_WIDTH_PX

    window = PlaybackWindow(_log())
    panel = window.controls_panel
    assert panel.minimumWidth() == panel.maximumWidth() == _PANEL_WIDTH_PX

    # A long time/frame readout (the widest control) must not widen the fixed panel.
    window.time_label.setText("t = 123456.78 s   (frame 999999/999999)")
    assert panel.minimumWidth() == panel.maximumWidth() == _PANEL_WIDTH_PX


def test_set_frame_updates_pose(qapp) -> None:  # noqa: ANN001, ARG001
    """Selecting a frame drives the reconstructed arm to that recorded pose."""
    log = _log()
    window = PlaybackWindow(log)
    target = 3
    window.set_frame(target)

    assert window.frame == target
    assert window.skeleton.q == pytest.approx(log.channel("q")[target])


def test_slider_scrubs_frame(qapp) -> None:  # noqa: ANN001, ARG001
    """Moving the timeline slider scrubs to that frame."""
    log = _log()
    window = PlaybackWindow(log)
    target = 2
    window.slider.setValue(target)

    assert window.frame == target
    assert window.skeleton.q == pytest.approx(log.channel("q")[target])


def test_advance_progresses_through_frames(qapp) -> None:  # noqa: ANN001, ARG001
    """Advancing playback time moves to the frame at that time."""
    log = _log()
    window = PlaybackWindow(log)
    window.advance(0.25)  # 0.25 s -> last frame with t <= 0.25 is t = 0.2
    expected_frame = 2
    assert window.frame == expected_frame


def test_speed_scales_advance(qapp) -> None:  # noqa: ANN001, ARG001
    """A higher speed advances proportionally more log time per real second."""
    log = _log()
    window = PlaybackWindow(log)
    window.speed = 2.0
    window.advance(0.1)  # 0.1 s * 2 = 0.2 s of log time
    expected_frame = 2
    assert window.frame == expected_frame


def test_play_pause_toggles(qapp) -> None:  # noqa: ANN001, ARG001
    """Play starts the timeline; pause stops it; the toggle button stays in sync."""
    log = _log()
    window = PlaybackWindow(log)
    assert window.is_playing is False
    assert window.play_button.isChecked() is False
    window.play()
    assert window.is_playing is True
    assert window.play_button.isChecked() is True
    window.pause()
    assert window.is_playing is False
    assert window.play_button.isChecked() is False


def test_transport_bar_sits_under_the_timeline(qapp) -> None:  # noqa: ANN001, ARG001
    """The transport bar is placed directly below the timeline slider."""
    window = PlaybackWindow(_log())
    layout = window.controls_panel.layout()
    assert layout is not None
    assert layout.indexOf(window.transport_bar) == layout.indexOf(window.slider) + 1


def test_play_button_is_an_icon_toggle(qapp) -> None:  # noqa: ANN001, ARG001
    """The play control is the bar's checkable icon-only tool button."""
    from PyQt6.QtWidgets import QToolButton

    window = PlaybackWindow(_log())
    assert window.play_button is window.transport_bar.play_button
    assert isinstance(window.play_button, QToolButton)
    assert window.play_button.isCheckable()
    assert window.play_button.toolTip() == "Play (Space)"

    window.play_button.click()
    assert window.is_playing is True
    assert window.play_button.toolTip() == "Pause (Space)"


def test_auto_pause_at_the_end_syncs_the_toggle(qapp) -> None:  # noqa: ANN001, ARG001
    """Reaching the end of the timeline pauses and unchecks the toggle cleanly."""
    window = PlaybackWindow(_log())
    window.play()
    window.advance(10.0)  # way past the last frame -> auto-pause
    assert window.frame == len(window.log) - 1
    assert window.is_playing is False
    assert window.play_button.isChecked() is False


def test_step_button_advances_one_frame_while_paused(qapp) -> None:  # noqa: ANN001, ARG001
    """The step button ('Next frame') moves forward a single frame while paused."""
    window = PlaybackWindow(_log())
    assert window.step_button.toolTip() == "Next frame (→/F)"
    assert window.step_button.isEnabled()  # paused from the start
    window.step_button.click()
    assert window.frame == 1
    window.step_button.click()
    assert window.frame == 2  # noqa: PLR2004


def test_reset_button_returns_to_the_first_frame(qapp) -> None:  # noqa: ANN001, ARG001
    """The reset button ('Back to start') jumps back to frame 0."""
    window = PlaybackWindow(_log())
    assert window.reset_button.toolTip() == "Back to start (R)"
    window.set_frame(3)
    window.reset_button.click()
    assert window.frame == 0


def test_reset_button_pauses_playback(qapp) -> None:  # noqa: ANN001, ARG001
    """Clicking 'Back to start' during playback pauses at frame 0 with the toggle unchecked."""
    window = PlaybackWindow(_log())
    window.play()
    window.reset_button.click()
    assert window.frame == 0
    assert window.is_playing is False
    assert window.play_button.isChecked() is False


def test_plot_button_keeps_text_and_gains_icon(qapp) -> None:  # noqa: ANN001, ARG001
    """The plot button stays a labeled button but shows a leading icon."""
    window = PlaybackWindow(_log())
    assert window.plot_button.text() == "Plot channels…"
    assert not window.plot_button.icon().isNull()


def test_requires_q_channel(qapp) -> None:  # noqa: ANN001, ARG001
    """A log without a q channel cannot be animated."""
    link_prop = LinkProp(length=1.0, m=1.0, i=0.1, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi)
    log = StateLog(Skeleton([link_prop]))
    log.record(0.0, energy=0.0)
    with pytest.raises(ValueError, match="q"):
        PlaybackWindow(log)


def test_build_channel_figure_has_one_axis_per_channel(qapp) -> None:  # noqa: ANN001, ARG001
    """The analysis figure plots every recorded channel."""
    log = _log()
    window = PlaybackWindow(log)
    figure = window.build_channel_figure()
    assert len(figure.axes) == len(log.channel_names)


def test_force_arrow_set_when_log_records_force(qapp) -> None:  # noqa: ANN001, ARG001
    """A log with an ext_force channel drives the canvas force arrow and auto-scales it."""
    log = _force_log()
    window = PlaybackWindow(log)
    window.set_frame(3)
    np.testing.assert_array_equal(window.canvas.tip_force, log.channel("ext_force")[3])
    assert window.canvas.force_scale > 0.0


def test_no_force_arrow_without_force_channel(qapp) -> None:  # noqa: ANN001, ARG001
    """A log without an ext_force channel leaves the canvas force arrow unset."""
    window = PlaybackWindow(_log())
    window.set_frame(2)
    assert window.canvas.tip_force is None


def test_force_toggle_hides_arrow(qapp) -> None:  # noqa: ANN001, ARG001
    """Unchecking 'Show external force' clears the arrow on the canvas."""
    window = PlaybackWindow(_force_log())
    window.force_checkbox.setChecked(False)
    window.set_frame(2)
    assert window.canvas.tip_force is None


def test_parser_accepts_export_and_fps() -> None:
    """``--export`` takes an output path and ``--fps`` a frame rate."""
    args = build_parser().parse_args(["run.sklog.npz", "--export", "out.mp4", "--fps", "24"])
    assert args.export == Path("out.mp4")
    assert args.fps == pytest.approx(24.0)


@pytest.mark.parametrize("suffix", ["gif", "mp4"])
def test_export_writes_a_nonempty_animation(qapp, tmp_path: Path, suffix: str) -> None:  # noqa: ANN001, ARG001
    """Exporting replays the motion to a real video/gif on disk, no GUI shown."""
    window = PlaybackWindow(_log())
    out = tmp_path / f"replay.{suffix}"
    frames = window.export(out, fps=30.0)
    assert out.exists()
    assert out.stat().st_size > 0
    # The 5-frame, 0.4 s log yields several output frames at 30 fps.
    assert frames > 1


def test_export_rejects_unsupported_format(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """An extension other than .mp4/.gif is rejected before any rendering."""
    window = PlaybackWindow(_log())
    with pytest.raises(ValueError, match="format"):
        window.export(tmp_path / "replay.avi")


def test_export_renders_at_the_requested_size(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """Frames are rendered at the requested square size, not the window layout's size."""
    import imageio.v3 as iio

    window = PlaybackWindow(_log())
    out = tmp_path / "replay.mp4"
    size = 128  # a multiple of 16, so ffmpeg won't rescale the frame
    window.export(out, size=size)
    frame = iio.imread(out, index=0)
    assert frame.shape[0] == size
    assert frame.shape[1] == size


def test_parser_accepts_panel_flag() -> None:
    """``--panel`` opts the export into the simulator-style side panel."""
    args = build_parser().parse_args(["run.sklog.npz", "--export", "out.gif", "--panel"])
    assert args.panel is True
    assert build_parser().parse_args(["run.sklog.npz"]).panel is False


def test_export_with_panel_widens_the_frame(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """``panel=True`` composites a simulator-style side panel next to the canvas frame."""
    import imageio.v3 as iio

    from tools.player import _EXPORT_PANEL_WIDTH_PX

    window = PlaybackWindow(_log())
    out = tmp_path / "replay.mp4"
    size = 128  # 128 + 304 = 432, a multiple of 16, so ffmpeg won't rescale
    window.export(out, size=size, panel=True)
    frame = iio.imread(out, index=0)
    assert frame.shape[0] == size
    assert frame.shape[1] == size + _EXPORT_PANEL_WIDTH_PX
    panel_region = frame[:, size:, :]
    assert panel_region.std() > 1.0  # text and sliders rendered, not a uniform background


def test_panel_export_pads_only_for_mp4(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """A non-aligned size stays exact for GIFs; mp4 pads the width up to a multiple of 16."""
    import imageio.v3 as iio

    from tools.player import _EXPORT_PANEL_WIDTH_PX

    window = PlaybackWindow(_log())
    gif = tmp_path / "replay.gif"
    window.export(gif, size=127, panel=True)  # 127 + 304 = 431: no GIF codec constraint
    frame = iio.imread(gif, index=0)
    assert frame.shape[1] == 127 + _EXPORT_PANEL_WIDTH_PX

    mp4 = tmp_path / "replay.mp4"
    window.export(mp4, size=120, panel=True)  # 120 + 304 = 424 -> padded to 432
    frame = iio.imread(mp4, index=0)
    assert frame.shape[1] == 432  # noqa: PLR2004


def test_export_runs_headless_via_cli(tmp_path: Path) -> None:
    """``--export`` runs the tool headless (no window) end-to-end and writes the file."""
    log_path = tmp_path / "run.sklog.npz"
    _log().save(log_path)
    out = tmp_path / "out.gif"
    script = Path(__file__).resolve().parents[1] / "tools" / "player.py"
    # Deliberately do NOT pass QT_QPA_PLATFORM: export mode must arrange headless rendering itself.
    env = {k: v for k, v in os.environ.items() if k != "QT_QPA_PLATFORM"}
    result = subprocess.run(  # noqa: S603  # trusted: our own interpreter and script path
        [sys.executable, str(script), str(log_path), "--export", str(out), "--fps", "20"],
        capture_output=True,
        text=True,
        check=False,
        env=env,
    )
    assert result.returncode == 0, result.stderr
    assert out.exists()
    assert out.stat().st_size > 0


def test_runs_as_a_standalone_script() -> None:
    """Running the file directly (script mode) must resolve all of its imports."""
    script = Path(__file__).resolve().parents[1] / "tools" / "player.py"
    result = subprocess.run(  # noqa: S603  # trusted: our own interpreter and script path
        [sys.executable, str(script), "--help"],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    assert "replay" in result.stdout.lower()


def _activate(window) -> None:  # noqa: ANN001
    """Make ``window`` the active window so WindowShortcut QShortcuts fire offscreen."""
    from PyQt6.QtWidgets import QApplication

    window.show()
    window.activateWindow()
    QApplication.processEvents()


def test_back_button_present_with_tooltip(qapp) -> None:  # noqa: ANN001, ARG001
    """The player's bar has a 'Previous frame' back button, aliased on the window."""
    window = PlaybackWindow(_log())
    assert window.back_button is window.transport_bar.back_button
    assert window.back_button is not None
    assert window.back_button.toolTip() == "Previous frame (←/B)"


def test_back_button_steps_backward_while_paused(qapp) -> None:  # noqa: ANN001, ARG001
    """The back button moves one frame backward while paused and clamps at frame 0."""
    window = PlaybackWindow(_log())
    window.set_frame(3)
    window.back_button.click()
    window.back_button.click()
    assert window.frame == 1
    window.back_button.click()
    window.back_button.click()
    assert window.frame == 0  # clamped


def test_back_button_disabled_while_playing(qapp) -> None:  # noqa: ANN001, ARG001
    """The back button is only enabled while paused."""
    window = PlaybackWindow(_log())
    window.play()
    assert not window.back_button.isEnabled()
    window.pause()
    assert window.back_button.isEnabled()


def test_home_end_shortcuts(qapp) -> None:  # noqa: ANN001, ARG001
    """Home jumps (paused) to the first frame, End to the last."""
    from PyQt6.QtCore import Qt
    from PyQt6.QtTest import QTest

    window = PlaybackWindow(_log())
    assert window.home_shortcut.key().toString() == "Home"
    assert window.end_shortcut.key().toString() == "End"

    _activate(window)
    window.play()
    QTest.keyClick(window, Qt.Key.Key_End)  # type: ignore[call-overload]  # PyQt6 stubs type QTest methods as bound
    assert window.is_playing is False
    assert window.frame == len(window.log) - 1

    QTest.keyClick(window, Qt.Key.Key_Home)  # type: ignore[call-overload]  # PyQt6 stubs type QTest methods as bound
    assert window.frame == 0
    assert window.is_playing is False


def test_q_shortcut_closes_the_window(qapp) -> None:  # noqa: ANN001, ARG001
    """Pressing Q closes the player window."""
    from PyQt6.QtCore import Qt
    from PyQt6.QtTest import QTest
    from PyQt6.QtWidgets import QApplication

    window = PlaybackWindow(_log())
    assert window.quit_shortcut.key().toString() == "Q"

    _activate(window)
    QTest.keyClick(window, Qt.Key.Key_Q)  # type: ignore[call-overload]  # PyQt6 stubs type QTest methods as bound
    QApplication.processEvents()
    assert not window.isVisible()


def _playback_extra(target: object) -> dict[str, object]:
    """Playback-only metadata carrying a task the log cannot re-run."""
    return {"playback": {"task": {"type": "reaching", "target": target}}}


def test_player_draws_playback_only_task_metadata(qapp) -> None:  # noqa: ANN001, ARG001
    """A log with only ``extra.playback.task`` renders the target marker with its tolerance."""
    log = _log()
    log.extra.update(_playback_extra({"pos": [0.4, 0.3], "tolerance": 0.01}))
    assert "source_config" not in log.extra  # playback metadata never advertises a rerunnable scenario
    window = PlaybackWindow(log)
    assert window._has_targets  # noqa: SLF001
    assert not window._has_reference  # noqa: SLF001
    ((pos, _color, tolerance, active),) = window.canvas.overlay_targets
    assert pos.tolist() == [0.4, 0.3]
    assert tolerance == 0.01  # noqa: PLR2004
    assert active


def test_player_prefers_the_full_source_config_task(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """When a log embeds a full ``source_config``, playback metadata must not override it."""
    log = _embedded_log("multi_target.toml", tmp_path)
    log.extra.update(_playback_extra({"pos": [9.0, 9.0]}))
    window = PlaybackWindow(log)
    assert len(window.canvas.overlay_targets) == 3  # noqa: PLR2004
    assert all(pos.tolist() != [9.0, 9.0] for pos, *_rest in window.canvas.overlay_targets)


@pytest.mark.parametrize(
    "task",
    [
        {"type": "no_such_task", "target": {"pos": [0.1, 0.2]}},
        {"type": "reaching"},
        {"type": "reaching", "target": {"pos": [0.1, 0.2, 0.3]}},
        {"target": {"pos": [0.1, 0.2]}},
    ],
    ids=["unknown-type", "missing-target", "bad-shape", "missing-type"],
)
def test_player_rejects_malformed_playback_metadata(qapp, task: dict[str, object]) -> None:  # noqa: ANN001, ARG001
    """Malformed playback-only metadata is an explicit error, not a silently empty overlay."""
    log = _log()
    log.extra.update({"playback": {"task": task}})
    with pytest.raises(ValueError, match=r"extra\.playback\.task"):
        PlaybackWindow(log)


def test_player_ignores_a_missing_playback_table(qapp) -> None:  # noqa: ANN001, ARG001
    """Logs without playback metadata behave exactly as before (no overlays, no error)."""
    log = _log()
    log.extra.update({"playback": {}})
    window = PlaybackWindow(log)
    assert not window._has_targets  # noqa: SLF001
    assert not window.canvas.overlay_targets


@pytest.mark.parametrize(
    "extra",
    [
        {"playback": "not-a-table"},
        {"playback": ["task"]},
        {"playback": {"task": "reaching"}},
        {"playback": {"task": [1, 2]}},
    ],
    ids=["playback-string", "playback-list", "task-string", "task-list"],
)
def test_player_rejects_non_mapping_playback_metadata(qapp, extra: dict[str, object]) -> None:  # noqa: ANN001, ARG001
    """A playback table of the wrong shape is a clear error, never an AttributeError."""
    log = _log()
    log.extra.update(extra)
    with pytest.raises(ValueError, match=r"extra\.playback"):
        PlaybackWindow(log)


# ----------------------------------------------------------------------------------------------
# Switching logs in place (the basis of the playlist)
# ----------------------------------------------------------------------------------------------


def _three_link_log(frames: int = 4) -> StateLog:
    """A three-link log, so switching to it changes the robot's joint count."""
    link_props = [LinkProp(length=0.6, m=1.0, i=0.1, rgx=0.3, rgy=0.0, qmin=-np.pi, qmax=np.pi) for _ in range(3)]
    log = StateLog(Skeleton(link_props), producer="three-link")
    for k in range(frames):
        log.record(0.2 * k, q=[0.1 * k, 0.2 * k, -0.1 * k])
    return log


def test_load_log_switches_the_window_to_another_log(qapp) -> None:  # noqa: ANN001, ARG001
    """``load_log`` replaces the replayed log in place: frames, slider, pose, header, and file name."""
    window = PlaybackWindow(_log(5), name="first.sklog.npz")
    window.set_frame(3)
    other = _three_link_log(4)
    window.load_log(other, name="second.sklog.npz")
    assert window.log is other
    assert window.frame == 0
    assert (window.slider.minimum(), window.slider.maximum()) == (0, 3)
    assert window.skeleton.num_joints == 3  # noqa: PLR2004
    assert window.canvas.skeleton is window.skeleton
    assert window.skeleton.q == pytest.approx(other.channel("q")[0])
    window.set_frame(2)
    assert window.skeleton.q == pytest.approx(other.channel("q")[2])
    assert "three-link" in window.header_label.text()
    assert window.file_label.text() == "second.sklog.npz"


def test_load_log_shows_only_the_toggles_the_new_log_supports(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """Force and overlay toggles appear and disappear with the data of the loaded log."""
    window = PlaybackWindow(_force_log())
    assert not window.force_checkbox.isHidden()
    window.load_log(_log())
    assert window.force_checkbox.isHidden()
    assert window.force_label.isHidden()
    assert window.canvas.tip_force is None

    window.load_log(_embedded_log("periodic_curve.toml", tmp_path))
    assert not window.reference_checkbox.isHidden()
    assert window.canvas.overlay_path is not None
    window.load_log(_embedded_log("multi_target.toml", tmp_path))
    assert not window.target_checkbox.isHidden()
    assert len(window.canvas.overlay_targets) == 3  # noqa: PLR2004
    window.load_log(_log())
    assert window.target_checkbox.isHidden()
    assert window.reference_checkbox.isHidden()
    assert window.canvas.overlay_targets == []
    assert window.canvas.overlay_path is None


def test_load_log_keeps_the_viewer_settings(qapp) -> None:  # noqa: ANN001, ARG001
    """Speed, centers of mass, and a hidden force arrow carry over to the next log."""
    window = PlaybackWindow(_force_log())
    window.speed_spin.setValue(2.0)
    window.com_checkbox.setChecked(True)
    window.force_checkbox.setChecked(False)
    window.load_log(_force_log(8))
    assert window.speed == pytest.approx(2.0)
    assert window.canvas.show_com
    window.set_frame(3)
    assert window.canvas.tip_force is None  # still hidden


def test_load_log_pauses_playback(qapp) -> None:  # noqa: ANN001, ARG001
    """Loading another log stops the running timeline; the caller decides whether to play it."""
    window = PlaybackWindow(_log())
    window.play()
    window.load_log(_log(8))
    assert window.is_playing is False
    assert window.play_button.isChecked() is False


@pytest.mark.parametrize("broken", ["no-q", "bad-playback"])
def test_load_log_rejects_a_bad_log_and_keeps_the_current_one(qapp, broken: str) -> None:  # noqa: ANN001, ARG001
    """A log that cannot be replayed raises before anything changes: the current log stays on screen."""
    current = _force_log()
    window = PlaybackWindow(current, name="good.sklog.npz")
    window.set_frame(2)
    if broken == "no-q":
        bad = StateLog(
            Skeleton([LinkProp(length=1.0, m=1.0, i=0.1, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi)]), producer="bad"
        )
        bad.record(0.0, tau=[0.0])
    else:
        bad = _log()
        bad.extra.update({"playback": {"task": {"type": "reaching"}}})
    with pytest.raises(ValueError, match=r"'q' channel|playback"):
        window.load_log(bad, name="bad.sklog.npz")
    assert window.log is current
    assert window.frame == 2  # noqa: PLR2004
    assert window.file_label.text() == "good.sklog.npz"
    assert not window.force_checkbox.isHidden()


def test_playback_finished_fires_only_at_a_natural_end(qapp) -> None:  # noqa: ANN001, ARG001
    """The signal marks playback running into the last frame, not jumps or steps to it."""
    window = PlaybackWindow(_log())
    finished: list[int] = []
    window.playback_finished.connect(lambda: finished.append(window.frame))
    window.set_frame(len(window.log) - 1)  # a jump to the end
    window.set_frame(0)
    for _ in range(len(window.log)):
        window.step_button.click()  # stepping onto the last frame while paused
    assert finished == []
    window.play()
    window.advance(10.0)
    assert finished == [len(window.log) - 1]
    assert window.is_playing is False


def test_playing_runs_the_timeline_on_its_own_to_the_end(qapp) -> None:  # noqa: ANN001, ARG001
    """While playing, the window's own timer advances the timeline until the natural end."""
    from PyQt6.QtTest import QSignalSpy

    window = PlaybackWindow(_log())  # five frames over 0.4 s
    window.speed = 10.0  # a 20 ms tick advances 0.2 s of log time
    finished = QSignalSpy(window.playback_finished)
    window.play()
    assert finished.wait(5000)
    assert window.frame == len(window.log) - 1
    assert window.is_playing is False
    assert window.play_button.isChecked() is False


def test_file_name_is_shown_in_the_side_panel(qapp) -> None:  # noqa: ANN001, ARG001
    """The playing file's name is shown in the panel (and the title); without one the label is hidden."""
    named = PlaybackWindow(_log(), name="take_001.sklog.npz")
    assert named.file_label.text() == "take_001.sklog.npz"
    assert not named.file_label.isHidden()
    assert "take_001.sklog.npz" in named.windowTitle()
    unnamed = PlaybackWindow(_log())
    assert unnamed.file_label.isHidden()


# ----------------------------------------------------------------------------------------------
# Playlist (several log files)
# ----------------------------------------------------------------------------------------------


def _write_logs(tmp_path: Path, *logs: StateLog | bytes) -> list[Path]:
    """Save each log (or raw bytes, for a broken file) as ``take_<k>.sklog.npz`` and return the paths."""
    paths = []
    for k, log in enumerate(logs, start=1):
        path = tmp_path / f"take_{k}.sklog.npz"
        if isinstance(log, bytes):
            path.write_bytes(log)
        else:
            log.save(path)
        paths.append(path)
    return paths


def _double_click(playlist, row: int) -> None:  # noqa: ANN001
    """Double-click the playlist row ``row`` through Qt's event system."""
    from PyQt6.QtCore import Qt
    from PyQt6.QtTest import QTest
    from PyQt6.QtWidgets import QApplication

    playlist.window().show()  # the player window holding the dock (or the floating dock itself)
    QApplication.processEvents()
    widget = playlist.list_widget
    center = widget.visualItemRect(widget.item(row)).center()
    QTest.mouseClick(widget.viewport(), Qt.MouseButton.LeftButton, pos=center)  # type: ignore[call-overload]  # PyQt6 stubs type QTest methods as bound
    QTest.mouseDClick(widget.viewport(), Qt.MouseButton.LeftButton, pos=center)  # type: ignore[call-overload]  # PyQt6 stubs type QTest methods as bound


def _entry(playlist, row: int):  # noqa: ANN001, ANN202  # QListWidgetItem (lazy PyQt import)
    """Return the playlist entry at ``row``, asserting it exists."""
    item = playlist.list_widget.item(row)
    assert item is not None
    return item


def _marked(playlist) -> list[int]:  # noqa: ANN001
    """Return the rows carrying the now-playing marker."""
    return [row for row in range(playlist.list_widget.count()) if _entry(playlist, row).text().startswith("▶")]


def test_playlist_lists_the_files_and_opens_the_first_paused(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """Every file is listed by name (full path as tooltip); the first is loaded, marked, and paused."""
    from tools.player import open_playlist

    paths = _write_logs(tmp_path, _log(5), _force_log(7), _three_link_log(4))
    player, playlist = open_playlist(paths)
    widget = playlist.list_widget
    assert widget.count() == 3  # noqa: PLR2004
    assert [_entry(playlist, row).toolTip() for row in range(3)] == [str(path) for path in paths]
    assert _entry(playlist, 1).text() == "take_2.sklog.npz"
    assert _marked(playlist) == [0]
    assert playlist.current == 0
    assert len(player.log) == 5  # noqa: PLR2004
    assert player.file_label.text() == "take_1.sklog.npz"
    assert player.is_playing is False


def test_double_click_loads_and_plays_that_file(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """Double-clicking a row loads that file into the player and starts playing it."""
    from tools.player import open_playlist

    player, playlist = open_playlist(_write_logs(tmp_path, _log(5), _force_log(7), _three_link_log(4)))
    _double_click(playlist, 2)
    assert playlist.current == 2  # noqa: PLR2004
    assert _marked(playlist) == [2]
    assert player.skeleton.num_joints == 3  # noqa: PLR2004
    assert player.file_label.text() == "take_3.sklog.npz"
    assert player.is_playing is True
    player.pause()


def test_enter_loads_the_selected_file(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """Enter is the keyboard equivalent of the double-click."""
    from PyQt6.QtCore import Qt
    from PyQt6.QtTest import QTest
    from PyQt6.QtWidgets import QApplication

    from tools.player import open_playlist

    player, playlist = open_playlist(_write_logs(tmp_path, _log(5), _force_log(7)))
    _activate(player)
    playlist.list_widget.setFocus()
    playlist.list_widget.setCurrentRow(1)
    QApplication.processEvents()
    QTest.keyClick(playlist.list_widget, Qt.Key.Key_Return)  # type: ignore[call-overload]  # PyQt6 stubs type QTest methods as bound
    assert playlist.current == 1
    assert len(player.log) == 7  # noqa: PLR2004
    assert player.is_playing is True
    player.pause()


@pytest.mark.parametrize("floating", [False, True], ids=["docked", "floating"])
def test_space_in_the_playlist_plays_and_pauses_the_player(qapp, tmp_path: Path, floating: bool) -> None:  # noqa: ANN001, ARG001, FBT001
    """With the playlist focused, docked or floating, Space plays and pauses the loaded log exactly once."""
    from PyQt6.QtCore import Qt
    from PyQt6.QtTest import QTest
    from PyQt6.QtWidgets import QApplication

    from tools.player import open_playlist

    player, playlist = open_playlist(_write_logs(tmp_path, _log(5), _force_log(7)))
    _activate(player)
    playlist.setFloating(floating)
    _activate(playlist.window())
    playlist.list_widget.setFocus()
    QApplication.processEvents()
    QTest.keyClick(playlist.list_widget, Qt.Key.Key_Space)  # type: ignore[call-overload]  # PyQt6 stubs type QTest methods as bound
    assert player.is_playing is True
    QTest.keyClick(playlist.list_widget, Qt.Key.Key_Space)  # type: ignore[call-overload]  # PyQt6 stubs type QTest methods as bound
    assert player.is_playing is False
    assert playlist.current == 0  # Space does not load anything


def test_playback_moves_on_to_the_next_file_and_stops_after_the_last(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """When a log finishes, the next one loads and plays; the end of the list just stops."""
    from tools.player import open_playlist

    player, playlist = open_playlist(_write_logs(tmp_path, _log(5), _force_log(7)))
    player.play()
    player.advance(10.0)  # run into the end of the first log
    assert playlist.current == 1
    assert len(player.log) == 7  # noqa: PLR2004
    assert player.is_playing is True
    player.advance(10.0)  # and of the last one
    assert playlist.current == 1
    assert player.frame == len(player.log) - 1
    assert player.is_playing is False


def test_files_that_fail_to_load_are_marked_and_skipped(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """A broken file is greyed out with the reason and skipped; double-clicking it keeps the current log."""
    from PyQt6.QtCore import Qt

    from tools.player import open_playlist

    player, playlist = open_playlist(_write_logs(tmp_path, _log(5), b"not a log", _force_log(7)))
    player.play()
    player.advance(10.0)
    assert playlist.current == 2  # take_2 was skipped  # noqa: PLR2004
    assert len(player.log) == 7  # noqa: PLR2004
    broken = _entry(playlist, 1)
    assert "could not load" in broken.toolTip()
    assert not broken.flags() & Qt.ItemFlag.ItemIsEnabled
    player.pause()
    _double_click(playlist, 1)
    assert playlist.current == 2  # noqa: PLR2004
    assert len(player.log) == 7  # noqa: PLR2004


def test_playlist_starts_at_the_first_loadable_file(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """A broken first file is skipped at startup; a list with nothing loadable is an error."""
    from tools.player import open_playlist

    player, playlist = open_playlist(_write_logs(tmp_path, b"broken", _force_log(7)))
    assert playlist.current == 1
    assert len(player.log) == 7  # noqa: PLR2004
    unloadable = tmp_path / "unloadable"
    unloadable.mkdir()
    with pytest.raises(ValueError, match="none of the logs"):
        open_playlist(_write_logs(unloadable, b"a", b"b"))


def test_plot_channels_uses_the_loaded_log(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """Plot channels follows the playlist: it plots the log that is loaded, not the first one."""
    from tools.player import open_playlist

    player, playlist = open_playlist(_write_logs(tmp_path, _log(5), _force_log(7)))
    first = player.build_channel_figure()
    assert len(first.axes) == len(_log().channel_names)
    playlist.play_index(1, play=False)
    second = player.build_channel_figure()
    assert len(second.axes) == len(_force_log().channel_names)


def test_playlist_is_docked_beside_the_player(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """The playlist docks on the player's right, the window widening so the canvas keeps its room; it can float."""
    from PyQt6.QtCore import Qt
    from PyQt6.QtWidgets import QDockWidget

    from tools.player import open_playlist

    single = PlaybackWindow(_log())
    _activate(single)
    player, playlist = open_playlist(_write_logs(tmp_path, _log(5), _force_log(7)))
    _activate(player)
    central, single_central = player.centralWidget(), single.centralWidget()
    assert central is not None
    assert single_central is not None
    assert isinstance(playlist, QDockWidget)
    assert player.dockWidgetArea(playlist) == Qt.DockWidgetArea.RightDockWidgetArea
    assert playlist.x() >= central.geometry().right()  # beside, never over, the canvas and panel
    assert player.width() > single.width()
    assert central.width() == single_central.width()  # the canvas and side panel keep their size
    assert playlist.features() & QDockWidget.DockWidgetFeature.DockWidgetFloatable
    playlist.setFloating(True)
    assert playlist.isFloating()
    playlist.setFloating(False)
    assert player.dockWidgetArea(playlist) == Qt.DockWidgetArea.RightDockWidgetArea


def test_playlist_button_shows_and_hides_the_dock(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """The Playlist button toggles the dock and follows it when the dock's own close button hides it."""
    from PyQt6.QtWidgets import QApplication

    from tools.player import open_playlist

    player, playlist = open_playlist(_write_logs(tmp_path, _log(5), _force_log(7)))
    _activate(player)
    button = player.playlist_button
    assert not button.isHidden()
    assert button.isCheckable()
    assert button.isChecked()
    button.click()
    QApplication.processEvents()
    assert playlist.isHidden()
    assert not button.isChecked()
    button.click()
    QApplication.processEvents()
    assert playlist.isVisible()
    assert button.isChecked()
    playlist.close()  # the dock's own close button
    QApplication.processEvents()
    assert playlist.isHidden()
    assert not button.isChecked()


@pytest.mark.parametrize("floating", [False, True], ids=["docked", "floating"])
def test_closing_the_player_closes_the_playlist(qapp, tmp_path: Path, floating: bool) -> None:  # noqa: ANN001, ARG001, FBT001
    """Closing the player leaves no playlist behind, docked or floating."""
    from PyQt6.QtWidgets import QApplication

    from tools.player import open_playlist

    player, playlist = open_playlist(_write_logs(tmp_path, _log(5), _force_log(7)))
    _activate(player)
    playlist.setFloating(floating)
    QApplication.processEvents()
    assert playlist.isVisible()
    player.close()
    QApplication.processEvents()
    assert not playlist.isVisible()


def test_single_file_player_has_no_playlist_button(qapp) -> None:  # noqa: ANN001, ARG001
    """Without a playlist the Playlist button stays hidden."""
    assert PlaybackWindow(_log()).playlist_button.isHidden()


def test_parser_accepts_several_log_files() -> None:
    """Several log files are accepted; one is still the plain single-log player."""
    args = build_parser().parse_args(["a.sklog.npz", "b.sklog.npz"])
    assert args.logfile == [Path("a.sklog.npz"), Path("b.sklog.npz")]
    assert build_parser().parse_args(["a.sklog.npz"]).logfile == [Path("a.sklog.npz")]


def test_export_refuses_several_log_files(tmp_path: Path) -> None:
    """Headless export renders one log; several files are a clear command-line error."""
    paths = _write_logs(tmp_path, _log(5), _log(6))
    script = Path(__file__).resolve().parents[1] / "tools" / "player.py"
    result = subprocess.run(  # noqa: S603  # trusted: our own interpreter and script path
        [sys.executable, str(script), *map(str, paths), "--export", str(tmp_path / "out.gif")],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 2  # noqa: PLR2004
    assert "--export takes a single log" in result.stderr
    assert not (tmp_path / "out.gif").exists()


def _press(window, key) -> None:  # noqa: ANN001
    """Press ``key`` in the (activated) ``window`` through Qt's event system."""
    from PyQt6.QtTest import QTest

    _activate(window)
    QTest.keyClick(window, key)  # type: ignore[call-overload]  # PyQt6 stubs type QTest methods as bound


def test_n_and_p_load_the_next_and_previous_files(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """N/P step through the playlist, keep a paused player paused and a playing one playing, and stop at the ends."""
    from PyQt6.QtCore import Qt

    from tools.player import open_playlist

    player, playlist = open_playlist(_write_logs(tmp_path, _log(5), _force_log(7), _three_link_log(4)))
    assert player.next_shortcut.key().toString() == "N"
    assert player.previous_shortcut.key().toString() == "P"
    _press(player, Qt.Key.Key_N)
    assert playlist.current == 1
    assert player.is_playing is False  # paused stays paused
    player.play()
    _press(player, Qt.Key.Key_N)
    assert playlist.current == 2  # noqa: PLR2004
    assert player.is_playing is True  # playing keeps playing
    _press(player, Qt.Key.Key_N)  # already the last file
    assert playlist.current == 2  # noqa: PLR2004
    _press(player, Qt.Key.Key_P)
    _press(player, Qt.Key.Key_P)
    assert playlist.current == 0
    assert len(player.log) == 5  # noqa: PLR2004
    _press(player, Qt.Key.Key_P)  # already the first file
    assert playlist.current == 0
    player.pause()


def test_n_and_p_work_with_the_playlist_focused(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """The dock lives in the player window, so N / P step files even while the list has the focus."""
    from PyQt6.QtCore import Qt
    from PyQt6.QtTest import QTest
    from PyQt6.QtWidgets import QApplication

    from tools.player import open_playlist

    player, playlist = open_playlist(_write_logs(tmp_path, _log(5), _force_log(7)))
    _activate(player)
    playlist.list_widget.setFocus()
    QApplication.processEvents()
    QTest.keyClick(playlist.list_widget, Qt.Key.Key_N)  # type: ignore[call-overload]  # PyQt6 stubs type QTest methods as bound
    assert playlist.current == 1
    QTest.keyClick(playlist.list_widget, Qt.Key.Key_P)  # type: ignore[call-overload]  # PyQt6 stubs type QTest methods as bound
    assert playlist.current == 0


def test_n_and_p_skip_files_that_fail_to_load(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """A broken file between two good ones is skipped in both directions."""
    from PyQt6.QtCore import Qt

    from tools.player import open_playlist

    player, playlist = open_playlist(_write_logs(tmp_path, _log(5), b"broken", _force_log(7)))
    _press(player, Qt.Key.Key_N)
    assert playlist.current == 2  # noqa: PLR2004
    _press(player, Qt.Key.Key_P)
    assert playlist.current == 0


def test_n_and_p_do_nothing_without_a_playlist(qapp) -> None:  # noqa: ANN001, ARG001
    """A single-log player has no next/previous shortcuts to fire."""
    from PyQt6.QtCore import Qt

    log = _log()
    window = PlaybackWindow(log)
    _press(window, Qt.Key.Key_N)
    _press(window, Qt.Key.Key_P)
    assert window.log is log
    assert window.next_shortcut.isEnabled() is False


# ----------------------------------------------------------------------------------------------
# Logs that do not fit the player (rejected up front) and one-frame logs
# ----------------------------------------------------------------------------------------------


def _misfit_log(kind: str) -> StateLog:
    """A two-joint log whose ``kind`` channel has the wrong number of columns for the player."""
    link_props = [LinkProp(length=1.0, m=1.0, i=0.1, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi) for _ in range(2)]
    log = StateLog(Skeleton(link_props), producer="misfit")
    for k in range(4):
        channels: dict[str, list[float]] = {"q": [0.1 * k, 0.2 * k]}
        if kind == "q":
            channels["q"] = [0.1 * k]  # one angle for a two-joint arm
        elif kind == "ext_force":
            channels["ext_force"] = [0.5 * k]  # a force needs (fx, fy)
        elif kind == "active_target":
            channels["active_target"] = [0.0, 1.0]  # one index per frame
        log.record(0.1 * k, **channels)
    return log


_MISFITS = ["q", "ext_force", "active_target"]


@pytest.mark.parametrize("kind", _MISFITS)
def test_player_rejects_a_log_whose_channels_do_not_fit(qapp, kind: str) -> None:  # noqa: ANN001, ARG001
    """A channel of the wrong shape is a clear ValueError up front, not a failure on the first frame."""
    with pytest.raises(ValueError, match=kind):
        PlaybackWindow(_misfit_log(kind))


@pytest.mark.parametrize("kind", _MISFITS)
def test_load_log_rejects_misfit_channels_and_keeps_the_current_log(qapp, kind: str) -> None:  # noqa: ANN001, ARG001
    """The check runs before the window changes: the current log stays and scrubbing keeps working."""
    current = _force_log()
    window = PlaybackWindow(current, name="good.sklog.npz")
    window.set_frame(2)
    window.play()
    with pytest.raises(ValueError, match=kind):
        window.load_log(_misfit_log(kind), name="bad.sklog.npz")
    assert window.log is current
    assert window.frame == 2  # noqa: PLR2004
    assert window.file_label.text() == "good.sklog.npz"
    assert window.is_playing is True  # rejected before pausing
    window.pause()
    window.set_frame(3)
    assert window.skeleton.q == pytest.approx(current.channel("q")[3])


def test_playlist_skips_a_log_whose_channels_do_not_fit(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """A misfit file is marked failed and skipped; the player keeps replaying the good log."""
    from tools.player import open_playlist

    player, playlist = open_playlist(_write_logs(tmp_path, _log(5), _misfit_log("q"), _force_log(7)))
    assert playlist.play_index(1) is False
    assert playlist.current == 0
    assert len(player.log) == 5  # noqa: PLR2004
    player.set_frame(4)  # scrubbing the kept log still works
    assert "could not load" in _entry(playlist, 1).toolTip()
    player.play()
    player.advance(10.0)
    assert playlist.current == 2  # noqa: PLR2004
    player.pause()


def _one_frame_log() -> StateLog:
    """A log holding a single frame."""
    return _log(1)


def test_one_frame_log_finishes_playback(qapp) -> None:  # noqa: ANN001, ARG001
    """Playing a one-frame log completes at once: it pauses and reports the natural end."""
    window = PlaybackWindow(_one_frame_log())
    finished: list[int] = []
    window.playback_finished.connect(lambda: finished.append(window.frame))
    window.play()
    window.advance(0.02)
    assert window.is_playing is False
    assert window.play_button.isChecked() is False
    assert finished == [0]


def test_playlist_moves_past_a_one_frame_log(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """A one-frame log in the playlist finishes and the next file plays, like any other log."""
    from tools.player import open_playlist

    player, playlist = open_playlist(_write_logs(tmp_path, _one_frame_log(), _force_log(7)))
    player.play()
    player.advance(10.0)
    assert playlist.current == 1
    assert player.is_playing is True
    player.pause()


def _central_width(player) -> int:  # noqa: ANN001
    """Return the width of the player's central widget (canvas plus side panel)."""
    central = player.centralWidget()
    assert central is not None
    return central.width()


def test_hiding_and_showing_the_playlist_resizes_the_window_not_the_canvas(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """The window narrows by the dock's room when it hides and widens back when it shows; the canvas keeps its size."""
    from PyQt6.QtWidgets import QApplication

    from tools.player import open_playlist

    player, playlist = open_playlist(_write_logs(tmp_path, _log(5), _force_log(7)))
    _activate(player)
    window, central, dock = player.width(), _central_width(player), playlist.width()
    for hide, show in ((player.playlist_button.click, player.playlist_button.click), (playlist.close, playlist.show)):
        hide()
        QApplication.processEvents()
        assert playlist.isHidden()
        assert player.width() < window - dock + 1  # the dock's width (and its separator) is given back
        assert _central_width(player) == central
        show()
        QApplication.processEvents()
        assert player.width() == window
        assert _central_width(player) == central
        assert playlist.width() == dock


def test_floating_and_redocking_the_playlist_resizes_the_window(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """Floating the playlist off narrows the window; docking it back widens it again, the canvas unchanged."""
    from PyQt6.QtWidgets import QApplication

    from tools.player import open_playlist

    player, playlist = open_playlist(_write_logs(tmp_path, _log(5), _force_log(7)))
    _activate(player)
    window, central = player.width(), _central_width(player)
    playlist.setFloating(True)
    QApplication.processEvents()
    assert player.width() < window - playlist.width() + 1
    assert _central_width(player) == central
    playlist.setFloating(False)
    QApplication.processEvents()
    assert player.width() == window
    assert _central_width(player) == central


def test_a_maximized_player_keeps_its_size_when_the_playlist_hides(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """A maximized window's size belongs to the desktop: hiding the playlist leaves it alone."""
    from PyQt6.QtWidgets import QApplication

    from tools.player import open_playlist

    player, _playlist = open_playlist(_write_logs(tmp_path, _log(5), _force_log(7)))
    player.showMaximized()
    QApplication.processEvents()
    width = player.width()
    player.playlist_button.click()
    QApplication.processEvents()
    assert player.width() == width
    player.close()


@pytest.mark.parametrize("leave", ["hide", "float"])
def test_a_widened_playlist_still_keeps_the_canvas_size(qapp, tmp_path: Path, leave: str) -> None:  # noqa: ANN001, ARG001
    """After the user drags the dock wider, leaving and returning still keep the canvas and the window size."""
    from PyQt6.QtCore import Qt
    from PyQt6.QtWidgets import QApplication

    from tools.player import open_playlist

    player, playlist = open_playlist(_write_logs(tmp_path, _log(5), _force_log(7)))
    _activate(player)
    player.resizeDocks([playlist], [480], Qt.Orientation.Horizontal)  # as dragging the separator does
    QApplication.processEvents()
    assert playlist.width() == 480  # noqa: PLR2004
    window, central = player.width(), _central_width(player)
    if leave == "hide":
        player.playlist_button.click()
    else:
        playlist.setFloating(True)
    QApplication.processEvents()
    assert _central_width(player) == central
    assert player.width() < window - 480 + 1
    if leave == "hide":
        player.playlist_button.click()
    else:
        playlist.setFloating(False)
    QApplication.processEvents()
    assert player.width() == window
    assert _central_width(player) == central
    assert playlist.width() == 480  # noqa: PLR2004


_DELETE_PLAYER = """
import sys
from pathlib import Path

sys.path.insert(0, sys.argv[1])
from PyQt6 import sip
from PyQt6.QtWidgets import QApplication

from tools.player import open_playlist

app = QApplication([])
player, playlist = open_playlist([Path(path) for path in sys.argv[3:]])
player.show()
app.processEvents()
if sys.argv[2] == "floating":
    playlist.setFloating(True)
elif sys.argv[2] == "hidden":
    player.playlist_button.click()
app.processEvents()
sip.delete(player)  # what the garbage collector does to a window nobody holds any more
app.processEvents()
print("deleted cleanly")
"""


@pytest.mark.parametrize("dock", ["shown", "floating", "hidden"])
def test_deleting_the_player_with_its_playlist_does_not_abort(tmp_path: Path, dock: str) -> None:
    """Destroying a player (as garbage collection or app exit does) never runs playlist slots on a dead window.

    A slot raising inside Qt aborts the whole process, so this runs in a subprocess.
    """
    paths = _write_logs(tmp_path, _log(5), _force_log(7))
    repo = Path(__file__).resolve().parents[1]
    result = subprocess.run(  # noqa: S603  # trusted: our own interpreter and inline script
        [sys.executable, "-c", _DELETE_PLAYER, str(repo), dock, *map(str, paths)],
        capture_output=True,
        text=True,
        check=False,
        env={**os.environ, "QT_QPA_PLATFORM": "offscreen"},
    )
    assert result.returncode == 0, result.stderr
    assert "deleted cleanly" in result.stdout
    assert "has been deleted" not in result.stderr
