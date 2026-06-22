"""Tests for the reach scenario runner tool (tools/reach.py)."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from skelarm.recording import StateLog
from skelarm.scenario import rerun_log
from tools.reach import build_parser, run_reach

_SCENARIO_TOML = (
    "[skeleton]\n"
    "[[skeleton.link]]\nlength = 1.0\nmass = 1.0\ninertia = 0.1\ncom = [0.5, 0.0]\nlimits = [-180.0, 180.0]\n"
    "[[skeleton.link]]\nlength = 0.8\nmass = 0.8\ninertia = 0.05\ncom = [0.4, 0.0]\nlimits = [-180.0, 180.0]\n"
    "[initial]\nq = [34.4, 57.3]\n"
    "[task]\ntarget = [0.55, 1.21]\nduration = 2.0\ndt = 0.002\n"
    '[controller]\ntype = "computed_torque"\nkp = 200.0\nkd = 30.0\n'
)


def test_parser_requires_config() -> None:
    """The config argument is required."""
    with pytest.raises(SystemExit):
        build_parser().parse_args([])


def test_run_reach_exports_a_replayable_log(tmp_path: Path) -> None:
    """Running a reach writes a .sklog.npz that loads back with the expected channels."""
    config = tmp_path / "reach.toml"
    config.write_text(_SCENARIO_TOML, encoding="utf-8")
    output = tmp_path / "run.sklog.npz"

    written = run_reach(config, output=output)
    assert written == output

    log = StateLog.load(output)
    assert {"q", "dq", "tau", "q_ref", "error"} <= set(log.channel_names)
    final = log.build_skeleton()
    final.q = log.channel("q")[-1]
    tip = np.array([final.links[-1].xe, final.links[-1].ye])
    assert tip == pytest.approx(np.array([0.55, 1.21]), abs=2e-2)


def test_run_reach_exports_a_rerunnable_log(tmp_path: Path) -> None:
    """The exported log embeds the scenario, so it re-simulates to the same motion."""
    config = tmp_path / "reach.toml"
    config.write_text(_SCENARIO_TOML, encoding="utf-8")
    output = tmp_path / "run.sklog.npz"
    run_reach(config, output=output, duration=0.2)

    log = StateLog.load(output)
    assert log.extra["scenario"]["controller"]["type"] == "computed_torque"

    replayed = rerun_log(log)
    np.testing.assert_array_equal(replayed.channel("q"), log.channel("q"))


def test_run_reach_defaults_output_next_to_config(tmp_path: Path) -> None:
    """Without --output the log is written beside the config with a .sklog.npz suffix."""
    config = tmp_path / "myreach.toml"
    config.write_text(_SCENARIO_TOML, encoding="utf-8")

    written = run_reach(config)
    assert written == config.with_suffix(".sklog.npz")
    assert written.exists()


def test_runs_as_a_standalone_script() -> None:
    """Running the file directly (script mode) must resolve all of its imports."""
    script = Path(__file__).resolve().parents[1] / "tools" / "reach.py"
    result = subprocess.run(  # noqa: S603  # trusted: our own interpreter and script path
        [sys.executable, str(script), "--help"],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    assert "reach" in result.stdout.lower()
