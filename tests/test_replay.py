"""Tests for the playback/analysis tool (tools/replay.py)."""

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
from tools.replay import PlaybackWindow, build_parser

pytestmark = pytest.mark.integration

_SCENARIO_TOML = (
    "[skeleton]\n"
    "[[skeleton.link]]\nlength = 1.0\nmass = 1.0\ninertia = 0.1\ncom = [0.5, 0.0]\nlimits = [-180.0, 180.0]\n"
    "[[skeleton.link]]\nlength = 0.8\nmass = 0.8\ninertia = 0.05\ncom = [0.4, 0.0]\nlimits = [-180.0, 180.0]\n"
    "[initial]\nq = [34.4, 57.3]\n"
    "[task]\ntarget = [0.55, 1.21]\nduration = 0.03\ndt = 0.01\n"
    '[controller]\ntype = "computed_torque"\nkp = 200.0\nkd = 30.0\n'
)


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


def test_runs_as_a_standalone_script() -> None:
    """Running the file directly (script mode) must resolve all of its imports."""
    script = Path(__file__).resolve().parents[1] / "tools" / "replay.py"
    result = subprocess.run(  # noqa: S603  # trusted: our own interpreter and script path
        [sys.executable, str(script), "--help"],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    assert "replay" in result.stdout.lower()


def test_export_config_cli_writes_loadable_scenario(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """`replay.py --export-config` writes an editable, loadable scenario config and exits."""
    from skelarm.scenario import load_scenario, run_scenario
    from tools.replay import main

    config = tmp_path / "reach.toml"
    config.write_text(_SCENARIO_TOML, encoding="utf-8")
    log_path = tmp_path / "run.sklog.npz"
    run_scenario(load_scenario(config)).save(log_path)

    exported = tmp_path / "exported.toml"
    monkeypatch.setattr(sys, "argv", ["replay", str(log_path), "--export-config", str(exported)])
    main()  # returns before the GUI starts

    assert exported.exists()
    assert type(load_scenario(exported).controller).__name__ == "ComputedTorque"
