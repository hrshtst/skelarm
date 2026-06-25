"""Full-config embedding, editable export, and re-run reproducibility of logs.

A ``run_scenario`` log embeds the original source config, so ``rerun_log`` and an
exported editable TOML both reconstruct the scenario and re-simulate it. The
deterministic controllers reproduce the recorded channels exactly; MPC reproduces
within a small numerical tolerance.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from skelarm.recording import StateLog
from skelarm.scenario import (
    export_scenario_toml,
    load_scenario,
    rerun_log,
    run_scenario,
    scenario_from_log,
)

_SKELETON_TOML = (
    "[skeleton]\n"
    "[[skeleton.link]]\nlength = 1.0\nmass = 1.0\ninertia = 0.1\ncom = [0.5, 0.0]\nlimits = [-180.0, 180.0]\n"
    "[[skeleton.link]]\nlength = 0.8\nmass = 0.8\ninertia = 0.05\ncom = [0.4, 0.0]\nlimits = [-180.0, 180.0]\n"
)


def _write_scenario(path: Path, controller_block: str, *, duration: float = 2.0, dt: float = 0.01) -> None:
    """Write a combined skeleton + initial + task + controller config."""
    path.write_text(
        _SKELETON_TOML
        + "[initial]\nq = [34.4, 57.3]\n"
        + f'[task]\ntype = "reaching"\ntarget = [0.55, 1.21]\nduration = {duration}\n[simulator]\ndt = {dt}\n'
        + controller_block,
        encoding="utf-8",
    )


def _computed_torque(path: Path, *, duration: float = 2.0) -> None:
    """Write a computed-torque scenario config to ``path``."""
    _write_scenario(path, '[controller]\ntype = "computed_torque"\nkp = 200.0\nkd = 30.0\n', duration=duration)


def test_run_scenario_embeds_the_source_config(tmp_path: Path) -> None:
    """A run_scenario log carries the original config plus the actual run params."""
    config = tmp_path / "reach.toml"
    _computed_torque(config)
    scenario = load_scenario(config)

    log = run_scenario(scenario, duration=0.1, dt=0.01)

    source = log.extra["source_config"]
    assert source["controller"]["type"] == "computed_torque"
    assert source["controller"]["kp"] == pytest.approx(200.0)
    assert source["task"]["target"] == pytest.approx([0.55, 1.21])
    assert source["initial"]["q"] == pytest.approx([34.4, 57.3])  # degrees, exactly as written
    assert log.extra["run"]["duration"] == pytest.approx(0.1)
    assert log.extra["run"]["dt"] == pytest.approx(0.01)
    assert log.extra["run"]["enforce_limits"] is True  # the actual joint-limit setting used
    assert "skelarm" in log.extra["provenance"]


def test_run_scenario_honors_simulator_enforce_limits(tmp_path: Path) -> None:
    """A ``[simulator].enforce_limits = false`` config disables the hard stop and is reproducible."""
    config = tmp_path / "free.toml"
    config.write_text(
        _SKELETON_TOML
        + "[initial]\nq = [34.4, 57.3]\n"
        + '[task]\ntype = "reaching"\ntarget = [0.55, 1.21]\nduration = 0.2\n'
        + "[simulator]\ndt = 0.01\nenforce_limits = false\n"
        + '[controller]\ntype = "computed_torque"\nkp = 200.0\nkd = 30.0\n',
        encoding="utf-8",
    )
    original = run_scenario(load_scenario(config))

    assert original.extra["run"]["enforce_limits"] is False
    replayed = rerun_log(original)
    np.testing.assert_array_equal(replayed.channel("q"), original.channel("q"))


def test_run_scenario_enforce_limits_override_is_recorded_and_reproduced(tmp_path: Path) -> None:
    """A call-time ``enforce_limits`` override (e.g. ``--no-joint-limits``) is captured for re-run."""
    config = tmp_path / "reach.toml"
    _computed_torque(config)  # [task] defaults enforce_limits to True
    original = run_scenario(load_scenario(config), duration=0.1, dt=0.01, enforce_limits=False)

    assert original.extra["run"]["enforce_limits"] is False  # the override, not the task default
    replayed = rerun_log(original)
    np.testing.assert_array_equal(replayed.channel("q"), original.channel("q"))


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


def test_export_scenario_toml_round_trips_exactly(tmp_path: Path) -> None:
    """Run -> save -> reload -> export editable TOML -> re-run reproduces exactly."""
    config = tmp_path / "reach.toml"
    _computed_torque(config, duration=0.05)
    original = run_scenario(load_scenario(config))  # default duration = task.duration = 0.05

    original.save(tmp_path / "run.sklog.npz")
    loaded = StateLog.load(tmp_path / "run.sklog.npz")
    exported = tmp_path / "exported.toml"
    export_scenario_toml(loaded, exported)

    replayed = run_scenario(load_scenario(exported))  # the exported file is a valid scenario config

    np.testing.assert_array_equal(replayed.channel("q"), original.channel("q"))
    np.testing.assert_array_equal(replayed.channel("tau"), original.channel("tau"))


def test_export_scenario_toml_preserves_enforce_limits(tmp_path: Path) -> None:
    """A ``[simulator].enforce_limits = false`` survives the editable export round-trip."""
    config = tmp_path / "free.toml"
    config.write_text(
        _SKELETON_TOML
        + "[initial]\nq = [34.4, 57.3]\n"
        + '[task]\ntype = "reaching"\ntarget = [0.55, 1.21]\nduration = 0.1\n'
        + "[simulator]\ndt = 0.01\nenforce_limits = false\n"
        + '[controller]\ntype = "computed_torque"\nkp = 200.0\nkd = 30.0\n',
        encoding="utf-8",
    )
    original = run_scenario(load_scenario(config))

    exported = tmp_path / "exported.toml"
    export_scenario_toml(original, exported)

    assert load_scenario(exported).simulator.enforce_limits is False  # the bool survived dump_toml
    replayed = run_scenario(load_scenario(exported))
    np.testing.assert_array_equal(replayed.channel("q"), original.channel("q"))


def test_edited_export_changes_the_result(tmp_path: Path) -> None:
    """Editing a gain in the exported config changes the re-run (the comparison use case)."""
    config = tmp_path / "reach.toml"
    _computed_torque(config, duration=0.1)
    original = run_scenario(load_scenario(config))

    exported = tmp_path / "exported.toml"
    export_scenario_toml(original, exported)
    exported.write_text(exported.read_text(encoding="utf-8").replace("kp = 200.0", "kp = 50.0"), encoding="utf-8")

    edited = run_scenario(load_scenario(exported))

    assert not np.allclose(edited.channel("tau"), original.channel("tau"))


def test_export_scenario_toml_requires_metadata(tmp_path: Path) -> None:
    """A log without an embedded scenario config cannot be exported."""
    with pytest.raises(ValueError, match="scenario config"):
        export_scenario_toml(StateLog(), tmp_path / "x.toml")


@pytest.mark.slow
def test_rerun_log_reproduces_mpc_within_tolerance(tmp_path: Path) -> None:
    """MPC re-runs reproduce the recorded motion within a small numerical tolerance."""
    config = tmp_path / "reach.toml"
    _write_scenario(config, '[controller]\ntype = "mpc"\nhorizon = 3\nq_weight = 10.0\n')
    scenario = load_scenario(config)
    original = run_scenario(scenario, duration=0.05, dt=0.01)

    replayed = rerun_log(original)

    np.testing.assert_allclose(replayed.channel("q"), original.channel("q"), rtol=1e-5, atol=1e-8)
