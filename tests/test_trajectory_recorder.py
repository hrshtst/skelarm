"""Tests for the interactive trajectory recorder (tools/trajectory_recorder.py)."""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

# Importing the tool pulls in PyQt6; run headless.
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from skelarm import Skeleton
from skelarm.recording import StateLog
from tools.player import PlaybackWindow
from tools.trajectory_recorder import RecorderWindow, build_parser, load_setup

pytestmark = pytest.mark.integration

_FOUR_DOF = Path(__file__).resolve().parents[1] / "examples" / "four_dof_robot.toml"

_SCENARIO_WITH_TASK = (
    "[skeleton]\n"
    "[[skeleton.link]]\nlength = 1.0\nmass = 1.0\ninertia = 0.1\ncom = [0.5, 0.0]\nlimits = [-180.0, 180.0]\n"
    "[[skeleton.link]]\nlength = 0.8\nmass = 0.8\ninertia = 0.05\ncom = [0.4, 0.0]\nlimits = [-180.0, 180.0]\n"
    "[initial]\nq = [34.4, 57.3]\n"
    "[task]\ntarget = { pos = [0.55, 1.21], tolerance = 0.02 }\n"
)


@pytest.fixture(scope="module")
def qapp():  # noqa: ANN201
    """Provide a single QApplication instance for the GUI tests."""
    from PyQt6.QtWidgets import QApplication

    return QApplication.instance() or QApplication([])


def _drive(window: RecorderWindow, frames: int = 3000) -> None:
    """Inject a moving cursor near the tip and tick the window until it stops."""
    tip = window.skeleton.links[-1]
    base = (tip.xe, tip.ye)
    for k in range(frames):
        window.canvas.drag_point = (base[0] + 0.1 * np.sin(0.05 * k), base[1] + 0.1 * np.cos(0.05 * k))
        window.tick()
        if window.finished:
            break


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


def test_ik_mode_records_q_and_tip_and_replays(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """IK mode records joint angles + tip at the sample rate; the log replays."""
    out = tmp_path / "ik.sklog.npz"
    window = RecorderWindow(Skeleton.from_toml(_FOUR_DOF), mode="ik", sample_rate=50.0, duration=1.0, output=out)
    assert not window.canvas.show_drag_arrow  # no force cue in the kinematic IK drag
    _drive(window)

    assert window.finished
    assert window.saved
    assert out.exists()
    assert set(window.log.channel_names) == {"q", "tip"}
    samples = round(1.0 * 50.0)
    assert samples <= len(window.log) <= samples + 2  # ~duration*rate (+ the t=0 frame)
    PlaybackWindow(StateLog.load(out))  # replayable in the player


def test_dynamics_mode_records_force_and_replays(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """Dynamics mode also records dq and the external force; the log replays."""
    out = tmp_path / "dyn.sklog.npz"
    window = RecorderWindow(Skeleton.from_toml(_FOUR_DOF), mode="dynamics", sample_rate=100.0, duration=0.4, output=out)
    assert window.canvas.show_drag_arrow  # the force arrow is shown in dynamics mode
    _drive(window)

    assert window.saved
    assert {"q", "tip", "dq", "ext_force"} <= set(window.log.channel_names)
    assert len(window.log) >= round(0.4 * 100.0)
    PlaybackWindow(StateLog.load(out))


def test_dynamics_enforce_limits_toggles_the_hard_stop(qapp) -> None:  # noqa: ANN001, ARG001
    """``enforce_limits`` controls whether the integrator gets joint bounds (default on)."""
    skeleton = Skeleton.from_toml(_FOUR_DOF)
    enforced = RecorderWindow(skeleton, mode="dynamics", enforce_limits=True)
    assert enforced._lower is not None  # noqa: SLF001
    assert enforced._upper is not None  # noqa: SLF001

    free = RecorderWindow(Skeleton.from_toml(_FOUR_DOF), mode="dynamics", enforce_limits=False)
    assert free._lower is None  # noqa: SLF001
    assert free._upper is None  # noqa: SLF001


def test_no_joint_limits_flag_disables_enforcement() -> None:
    """The ``--no-joint-limits`` flag parses and defaults to enforcing limits."""
    parser = build_parser()
    assert parser.parse_args([str(_FOUR_DOF)]).no_joint_limits is False
    assert parser.parse_args([str(_FOUR_DOF), "--no-joint-limits"]).no_joint_limits is True


def test_no_recording_without_a_grab(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """Ticking with no grab records nothing and saves nothing."""
    out = tmp_path / "none.sklog.npz"
    window = RecorderWindow(Skeleton.from_toml(_FOUR_DOF), mode="ik", duration=0.2, output=out)
    for _ in range(20):
        window.tick()  # never grabs
    assert not window.saved
    assert not out.exists()


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
