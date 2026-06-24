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
    """Play starts the timeline; pause stops it."""
    log = _log()
    window = PlaybackWindow(log)
    assert window.is_playing is False
    window.play()
    assert window.is_playing is True
    window.pause()
    assert window.is_playing is False


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
