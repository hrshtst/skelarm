"""Full-config embedding and re-run reproducibility of recorded scenario logs.

A ``run_scenario`` log embeds the task, controller, initial state, and run
parameters, so ``rerun_log`` can reconstruct the scenario and re-simulate it. The
deterministic controllers reproduce the recorded channels exactly; MPC reproduces
within a small numerical tolerance.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from skelarm.recording import StateLog
from skelarm.scenario import load_scenario, rerun_log, run_scenario, scenario_from_log

_SKELETON_TOML = (
    "[skeleton]\n"
    "[[skeleton.link]]\nlength = 1.0\nmass = 1.0\ninertia = 0.1\ncom = [0.5, 0.0]\nlimits = [-180.0, 180.0]\n"
    "[[skeleton.link]]\nlength = 0.8\nmass = 0.8\ninertia = 0.05\ncom = [0.4, 0.0]\nlimits = [-180.0, 180.0]\n"
)


def _write_scenario(path: Path, controller_block: str) -> None:
    """Write a combined skeleton + initial + task + controller config."""
    path.write_text(
        _SKELETON_TOML
        + "[initial]\nq = [34.4, 57.3]\n"
        + "[task]\ntarget = [0.55, 1.21]\nduration = 2.0\ndt = 0.01\n"
        + controller_block,
        encoding="utf-8",
    )


def _computed_torque(path: Path) -> None:
    """Write a computed-torque scenario config to ``path``."""
    _write_scenario(path, '[controller]\ntype = "computed_torque"\nkp = 200.0\nkd = 30.0\n')


def test_run_scenario_embeds_reproduction_metadata(tmp_path: Path) -> None:
    """A run_scenario log carries the controller, task, initial state, and run params."""
    config = tmp_path / "reach.toml"
    _computed_torque(config)
    scenario = load_scenario(config)

    log = run_scenario(scenario, duration=0.1, dt=0.01)

    repro = log.extra["scenario"]
    assert repro["controller"]["type"] == "computed_torque"
    assert repro["controller"]["kp"] == pytest.approx(200.0)
    assert repro["task"]["target"] == pytest.approx([0.55, 1.21])
    assert len(repro["initial"]["q"]) == scenario.skeleton.num_joints
    assert repro["run"]["duration"] == pytest.approx(0.1)
    assert repro["run"]["dt"] == pytest.approx(0.01)
    assert "skelarm" in log.extra["provenance"]


def test_scenario_from_log_rebuilds_scenario(tmp_path: Path) -> None:
    """scenario_from_log restores the controller, task, and initial pose."""
    config = tmp_path / "reach.toml"
    _computed_torque(config)
    scenario = load_scenario(config)
    log = run_scenario(scenario, duration=0.1, dt=0.01)

    rebuilt, run = scenario_from_log(log)

    assert type(rebuilt.controller).__name__ == "ComputedTorque"
    assert rebuilt.task.target == pytest.approx(scenario.task.target)
    np.testing.assert_array_equal(rebuilt.skeleton.q, scenario.skeleton.q)
    assert run["duration"] == pytest.approx(0.1)


def test_rerun_log_reproduces_computed_torque(tmp_path: Path) -> None:
    """A saved-and-reloaded log re-simulates to exactly the recorded channels."""
    config = tmp_path / "reach.toml"
    _computed_torque(config)
    scenario = load_scenario(config)
    original = run_scenario(scenario, duration=0.2, dt=0.01)

    saved = tmp_path / "run.sklog.npz"
    original.save(saved)
    reloaded = StateLog.load(saved)

    replayed = rerun_log(reloaded)

    np.testing.assert_array_equal(replayed.times, original.times)
    for channel in ("q", "dq", "tau"):
        np.testing.assert_array_equal(replayed.channel(channel), original.channel(channel))


def test_rerun_log_respects_duration_override(tmp_path: Path) -> None:
    """Rerun re-uses the actual run duration, not the task's planned duration."""
    config = tmp_path / "reach.toml"
    _computed_torque(config)
    scenario = load_scenario(config)
    original = run_scenario(scenario, duration=0.15, dt=0.01)  # != task.duration (2.0)

    replayed = rerun_log(original)

    assert len(replayed) == len(original)
    np.testing.assert_array_equal(replayed.channel("q"), original.channel("q"))


def test_rerun_log_without_metadata_raises() -> None:
    """A log lacking reproduction metadata cannot be re-run."""
    log = StateLog()
    with pytest.raises(ValueError, match="reproduction"):
        rerun_log(log)


@pytest.mark.slow
def test_rerun_log_reproduces_mpc_within_tolerance(tmp_path: Path) -> None:
    """MPC re-runs reproduce the recorded motion within a small numerical tolerance."""
    config = tmp_path / "reach.toml"
    _write_scenario(config, '[controller]\ntype = "mpc"\nhorizon = 3\nq_weight = 10.0\n')
    scenario = load_scenario(config)
    original = run_scenario(scenario, duration=0.05, dt=0.01)

    replayed = rerun_log(original)

    np.testing.assert_allclose(replayed.channel("q"), original.channel("q"), rtol=1e-5, atol=1e-8)
