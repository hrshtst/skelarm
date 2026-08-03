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
