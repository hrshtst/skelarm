"""Tests for tools/periodic_curve_simulator.py (interactive + headless)."""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from skelarm.recording import StateLog
from tools._scenario_cli import build_scenario
from tools.periodic_curve_simulator import CurveTraceSimulator, build_parser

pytestmark = pytest.mark.integration

_ROOT = Path(__file__).resolve().parents[1]
_CONFIG = _ROOT / "examples" / "periodic_curve.toml"
_SCRIPT = _ROOT / "tools" / "periodic_curve_simulator.py"


@pytest.fixture(scope="module")
def qapp():  # noqa: ANN201
    """Provide a single QApplication instance for the GUI tests."""
    from PyQt6.QtWidgets import QApplication

    return QApplication.instance() or QApplication([])


def test_parser_requires_config() -> None:
    """The config argument is required."""
    with pytest.raises(SystemExit):
        build_parser().parse_args([])


def test_parser_has_override_and_save_flags() -> None:
    """The shared override flags and --save are present."""
    args = build_parser().parse_args([str(_CONFIG), "--save", "out.npz", "--task", "t.toml", "--no-joint-limits"])
    assert args.save == Path("out.npz")
    assert args.task == Path("t.toml")
    assert args.no_joint_limits is True


def test_gui_records_with_embedded_task_config(qapp) -> None:  # noqa: ANN001, ARG001
    """The GUI auto-records and embeds the scenario config; the curve is drawn."""
    window = CurveTraceSimulator(build_scenario(_CONFIG))
    assert window.state_log is not None
    assert window.state_log.extra["source_config"]["task"]["type"] == "periodic_curve"
    assert window.canvas.overlay_path is not None  # the reference curve

    window.step()  # a step appends a frame; the embedded config is preserved
    assert "q" in window.state_log.channel_names


def test_headless_save_is_replayable(tmp_path: Path) -> None:
    """--save writes a .sklog.npz that loads with q + the embedded curve task."""
    out = tmp_path / "curve.sklog.npz"
    result = subprocess.run(  # noqa: S603
        [sys.executable, str(_SCRIPT), str(_CONFIG), "--save", str(out), "--duration", "0.5"],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    log = StateLog.load(out)
    assert "q" in log.channel_names
    assert log.extra["source_config"]["task"]["curve"] == "ellipse"


def test_wrong_task_type_errors(tmp_path: Path) -> None:
    """A non-periodic_curve scenario is rejected."""
    result = subprocess.run(  # noqa: S603
        [sys.executable, str(_SCRIPT), str(_ROOT / "examples" / "reach.toml"), "--save", str(tmp_path / "x.npz")],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode != 0
    assert "periodic_curve" in result.stderr


def test_runs_as_a_standalone_script() -> None:
    """Running the file directly resolves all imports (the tools.* bootstrap)."""
    result = subprocess.run(  # noqa: S603
        [sys.executable, str(_SCRIPT), "--help"], capture_output=True, text=True, check=False
    )
    assert result.returncode == 0, result.stderr
    assert "periodic-curve" in result.stdout.lower()
