"""Tests for tools/trajectory_tracking_simulator.py (interactive + headless)."""

from __future__ import annotations

import os
import subprocess
import sys
import tomllib
from pathlib import Path

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from skelarm import Skeleton
from skelarm.recording import StateLog
from tools._scenario_cli import build_scenario
from tools.trajectory_tracking_simulator import TrajectoryTrackSimulator, build_parser

pytestmark = pytest.mark.integration

_ROOT = Path(__file__).resolve().parents[1]
_SCRIPT = _ROOT / "tools" / "trajectory_tracking_simulator.py"
_SKELETON_TOML = (
    "[skeleton]\n"
    "[[skeleton.link]]\nlength = 1.0\nmass = 1.0\ninertia = 0.1\ncom = [0.5, 0.0]\nlimits = [-180.0, 180.0]\n"
    "[[skeleton.link]]\nlength = 0.8\nmass = 0.8\ninertia = 0.05\ncom = [0.4, 0.0]\nlimits = [-180.0, 180.0]\n"
)
_Q0 = np.deg2rad([20.0, 30.0])
_Q1 = np.deg2rad([45.0, -10.0])


@pytest.fixture(scope="module")
def qapp():  # noqa: ANN201
    """Provide a single QApplication instance for the GUI tests."""
    from PyQt6.QtWidgets import QApplication

    return QApplication.instance() or QApplication([])


def _write_reference(path: Path) -> None:
    """Write a reachable joint+tip reference .sklog.npz for the 2-link robot."""
    skeleton = Skeleton.from_config(tomllib.loads(_SKELETON_TOML))
    log = StateLog(skeleton, channel_meta={"q": {"columns": ["j1", "j2"]}, "tip": {"columns": ["x", "y"]}})
    for t in np.linspace(0.0, 1.0, 21):
        skeleton.q = _Q0 + t * (_Q1 - _Q0)
        log.record(float(t), q=skeleton.q.copy(), tip=np.array([skeleton.links[-1].xe, skeleton.links[-1].ye]))
    log.save(path)


def _write_config(tmp_path: Path, *, task_type: str) -> Path:
    """Write a tracking scenario config referencing a fresh reference file."""
    ref = tmp_path / "ref.sklog.npz"
    _write_reference(ref)
    initial = ", ".join(str(v) for v in np.rad2deg(_Q0))
    config = tmp_path / "track.toml"
    config.write_text(
        _SKELETON_TOML
        + f"[initial]\nq = [{initial}]\n"
        + f'[task]\ntype = "{task_type}"\nfile = "{ref}"\ndt = 0.01\n'
        + '[controller]\ntype = "computed_torque"\nkp = 300.0\nkd = 40.0\n',
        encoding="utf-8",
    )
    return config


def test_parser_requires_config() -> None:
    """The config argument is required."""
    with pytest.raises(SystemExit):
        build_parser().parse_args([])


def test_gui_records_with_embedded_reference(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """The GUI embeds the reference task config and draws the reference path."""
    window = TrajectoryTrackSimulator(build_scenario(_write_config(tmp_path, task_type="trajectory_tracking")))
    assert window.state_log is not None
    assert window.state_log.extra["source_config"]["task"]["type"] == "trajectory_tracking"
    assert window.canvas.overlay_path is not None


def test_joint_tracking_gui_draws_fk_reference(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """A per-joint reference is drawn as a task-space path via forward kinematics."""
    window = TrajectoryTrackSimulator(build_scenario(_write_config(tmp_path, task_type="joint_trajectory_tracking")))
    assert window.canvas.overlay_path is not None
    assert window.canvas.overlay_path.shape[1] == 2  # noqa: PLR2004


def test_headless_save_is_replayable(tmp_path: Path) -> None:
    """--save writes a replayable log embedding the reference task."""
    config = _write_config(tmp_path, task_type="joint_trajectory_tracking")
    out = tmp_path / "track.sklog.npz"
    result = subprocess.run(  # noqa: S603
        [sys.executable, str(_SCRIPT), str(config), "--save", str(out), "--duration", "0.3"],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    log = StateLog.load(out)
    assert "q" in log.channel_names
    assert "reference_samples" in log.extra["source_config"]["task"]


def test_wrong_task_type_errors(tmp_path: Path) -> None:
    """A non-tracking scenario is rejected."""
    result = subprocess.run(  # noqa: S603
        [sys.executable, str(_SCRIPT), str(_ROOT / "examples" / "reach.toml"), "--save", str(tmp_path / "x.npz")],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode != 0
    assert "trajectory_tracking" in result.stderr


def test_runs_as_a_standalone_script() -> None:
    """Running the file directly resolves all imports (the tools.* bootstrap)."""
    result = subprocess.run(  # noqa: S603
        [sys.executable, str(_SCRIPT), "--help"], capture_output=True, text=True, check=False
    )
    assert result.returncode == 0, result.stderr
    assert "trajectory" in result.stdout.lower()
