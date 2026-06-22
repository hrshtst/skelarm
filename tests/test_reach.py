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
    assert log.extra["source_config"]["controller"]["type"] == "computed_torque"

    replayed = rerun_log(log)
    np.testing.assert_array_equal(replayed.channel("q"), log.channel("q"))


def test_run_reach_defaults_output_next_to_config(tmp_path: Path) -> None:
    """Without --output the log is written beside the config with a .sklog.npz suffix."""
    config = tmp_path / "myreach.toml"
    config.write_text(_SCENARIO_TOML, encoding="utf-8")

    written = run_reach(config)
    assert written == config.with_suffix(".sklog.npz")
    assert written.exists()


def test_run_reach_overrides_controller(tmp_path: Path) -> None:
    """--controller swaps the controller while keeping the base robot and task."""
    config = tmp_path / "reach.toml"
    config.write_text(_SCENARIO_TOML, encoding="utf-8")  # base: computed_torque
    controller = tmp_path / "pd.toml"
    controller.write_text('[controller]\ntype = "joint_pd"\nkp = 300.0\nkd = 40.0\n', encoding="utf-8")

    output = tmp_path / "run.sklog.npz"
    run_reach(config, output=output, duration=0.05, controller=controller)

    log = StateLog.load(output)
    assert log.producer == "JointPD"
    assert log.extra["source_config"]["controller"]["type"] == "joint_pd"
    # The embedded (overridden) config re-runs and reproduces the motion.
    np.testing.assert_array_equal(rerun_log(log).channel("q"), log.channel("q"))


def test_run_reach_overrides_task(tmp_path: Path) -> None:
    """--task swaps the task (e.g. a different target) over the base config."""
    config = tmp_path / "reach.toml"
    config.write_text(_SCENARIO_TOML, encoding="utf-8")  # base target [0.55, 1.21]
    task = tmp_path / "task.toml"
    task.write_text("[task]\ntarget = [0.4, 0.9]\nduration = 0.05\ndt = 0.01\n", encoding="utf-8")

    output = tmp_path / "run.sklog.npz"
    run_reach(config, output=output, task=task)

    log = StateLog.load(output)
    assert log.extra["source_config"]["task"]["target"] == pytest.approx([0.4, 0.9])


def test_run_reach_initial_then_pose_override(tmp_path: Path) -> None:
    """--initial replaces the initial pose; --pose then overrides just q."""
    config = tmp_path / "reach.toml"
    config.write_text(_SCENARIO_TOML, encoding="utf-8")
    initial = tmp_path / "init.toml"
    initial.write_text("[initial]\nq = [10.0, 20.0]\n", encoding="utf-8")

    output = tmp_path / "run.sklog.npz"
    run_reach(config, output=output, duration=0.02, initial=initial, pose="5,15")

    log = StateLog.load(output)
    assert log.extra["source_config"]["initial"]["q"] == pytest.approx([5.0, 15.0])  # --pose wins
    np.testing.assert_allclose(log.channel("q")[0], np.deg2rad([5.0, 15.0]))


def test_run_reach_missing_override_file_errors(tmp_path: Path) -> None:
    """A missing override file raises a clear error."""
    config = tmp_path / "reach.toml"
    config.write_text(_SCENARIO_TOML, encoding="utf-8")
    with pytest.raises(FileNotFoundError, match="override file not found"):
        run_reach(config, duration=0.02, controller=tmp_path / "missing.toml")


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
