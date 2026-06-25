"""End-to-end tests for the trajectory-tracking task types (task-space and joint-space)."""

from __future__ import annotations

import tomllib
from pathlib import Path

import numpy as np
import pytest

from skelarm.recording import StateLog
from skelarm.scenario import load_scenario, reference_builders, rerun_log, run_scenario
from skelarm.skeleton import Skeleton

_SKELETON_TOML = (
    "[skeleton]\n"
    "[[skeleton.link]]\nlength = 1.0\nmass = 1.0\ninertia = 0.1\ncom = [0.5, 0.0]\nlimits = [-180.0, 180.0]\n"
    "[[skeleton.link]]\nlength = 0.8\nmass = 0.8\ninertia = 0.05\ncom = [0.4, 0.0]\nlimits = [-180.0, 180.0]\n"
)

_Q0 = np.deg2rad([20.0, 30.0])
_Q1 = np.deg2rad([55.0, -15.0])


def _write_reference(path: Path, *, channels: tuple[str, ...], jaggy: bool = False) -> tuple[np.ndarray, np.ndarray]:
    """Write a smooth (or jaggy) reachable reference .sklog.npz; return final (q, tip)."""
    skeleton = Skeleton.from_config(tomllib.loads(_SKELETON_TOML))
    times = np.linspace(0.0, 1.0, 51)
    rng = np.random.default_rng(0)
    meta = {
        "q": {"unit": "rad", "label": "joint angle", "columns": ["j1", "j2"]},
        "tip": {"unit": "m", "label": "tip position", "columns": ["x", "y"]},
    }
    log = StateLog(skeleton, producer="test", channel_meta={c: meta[c] for c in channels})
    for t in times:
        frac = 10.0 * t**3 - 15.0 * t**4 + 6.0 * t**5  # minimum-jerk progress in [0, 1]
        q = _Q0 + frac * (_Q1 - _Q0)
        if jaggy:
            q = q + rng.normal(0.0, np.deg2rad(1.0), size=2)  # measurement-like noise
        skeleton.q = q
        record = {}
        if "q" in channels:
            record["q"] = skeleton.q.copy()
        if "tip" in channels:
            record["tip"] = np.array([skeleton.links[-1].xe, skeleton.links[-1].ye])
        log.record(float(t), **record)
    log.save(path)
    skeleton.q = _Q1
    return _Q1, np.array([skeleton.links[-1].xe, skeleton.links[-1].ye])


def _scenario_toml(task_block: str) -> str:
    """A combined config: the 2-link robot starting at the reference start + a tracking task."""
    initial = ", ".join(str(v) for v in np.rad2deg(_Q0))
    return (
        _SKELETON_TOML
        + f"[initial]\nq = [{initial}]\n"
        + task_block
        + '[controller]\ntype = "computed_torque"\nkp = 300.0\nkd = 40.0\n'
    )


def test_reference_builders_lists_builtin_tracking_types() -> None:
    """The dispatcher exposes the built-in reference task types."""
    builders = reference_builders()
    assert {"reaching", "trajectory_tracking", "joint_trajectory_tracking"} <= set(builders)


def test_joint_trajectory_tracking_follows_reference(tmp_path: Path) -> None:
    """A joint-space reference is tracked to its final joint angles."""
    ref = tmp_path / "joint_ref.sklog.npz"
    q_final, _ = _write_reference(ref, channels=("q",))
    config = tmp_path / "track.toml"
    config.write_text(
        _scenario_toml(f'[task]\ntype = "joint_trajectory_tracking"\nfile = "{ref}"\ndt = 0.005\n'),
        encoding="utf-8",
    )
    log = run_scenario(load_scenario(config))
    assert log.channel("q")[-1] == pytest.approx(q_final, abs=3e-2)


def test_trajectory_tracking_follows_reference(tmp_path: Path) -> None:
    """A task-space (tip) reference is tracked so the tip reaches the final point."""
    ref = tmp_path / "tip_ref.sklog.npz"
    _, tip_final = _write_reference(ref, channels=("tip",))
    config = tmp_path / "track.toml"
    config.write_text(
        _scenario_toml(f'[task]\ntype = "trajectory_tracking"\nfile = "{ref}"\ndt = 0.005\n'),
        encoding="utf-8",
    )
    log = run_scenario(load_scenario(config))
    final = log.build_skeleton()
    final.q = log.channel("q")[-1]
    tip = np.array([final.links[-1].xe, final.links[-1].ye])
    assert tip == pytest.approx(tip_final, abs=3e-2)


def test_jaggy_joint_reference_with_butterworth_filter(tmp_path: Path) -> None:
    """A noisy joint reference smoothed by a Butterworth filter still tracks to the goal."""
    ref = tmp_path / "noisy.sklog.npz"
    q_final, _ = _write_reference(ref, channels=("q",), jaggy=True)
    config = tmp_path / "track.toml"
    config.write_text(
        _scenario_toml(
            '[task]\ntype = "joint_trajectory_tracking"\n'
            f'file = "{ref}"\ndt = 0.005\n'
            'filter = { kind = "butterworth", cutoff_hz = 6.0, order = 3 }\n'
            'interpolator = "cubic_spline"\n'
        ),
        encoding="utf-8",
    )
    log = run_scenario(load_scenario(config))
    assert log.channel("q")[-1] == pytest.approx(q_final, abs=5e-2)


def test_jaggy_joint_reference_with_savgol_filter(tmp_path: Path) -> None:
    """A Savitzky-Golay filter config (window/polyorder keys) smooths a noisy reference."""
    ref = tmp_path / "noisy.sklog.npz"
    q_final, _ = _write_reference(ref, channels=("q",), jaggy=True)
    config = tmp_path / "track.toml"
    config.write_text(
        _scenario_toml(
            '[task]\ntype = "joint_trajectory_tracking"\n'
            f'file = "{ref}"\ndt = 0.005\n'
            'filter = { kind = "savgol", window = 9, polyorder = 3 }\n'
        ),
        encoding="utf-8",
    )
    log = run_scenario(load_scenario(config))
    assert log.channel("q")[-1] == pytest.approx(q_final, abs=5e-2)


def test_joint_reference_dof_mismatch_raises(tmp_path: Path) -> None:
    """A joint reference whose DOF differs from the robot is rejected."""
    ref = tmp_path / "ref.sklog.npz"
    skeleton = Skeleton.from_config(tomllib.loads(_SKELETON_TOML))
    log = StateLog(skeleton, channel_meta={"q": {"columns": ["j1", "j2", "j3"]}})
    log.record(0.0, q=[0.0, 0.0, 0.0])
    log.record(1.0, q=[0.1, 0.1, 0.1])  # 3 joints, robot has 2
    log.save(ref)
    config = tmp_path / "track.toml"
    config.write_text(
        _scenario_toml(f'[task]\ntype = "joint_trajectory_tracking"\nfile = "{ref}"\ndt = 0.01\n'),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="joint columns"):
        run_scenario(load_scenario(config))


def test_tracking_requires_a_reference(tmp_path: Path) -> None:
    """A tracking task with neither a file nor inlined samples is rejected."""
    config = tmp_path / "track.toml"
    config.write_text(
        _scenario_toml('[task]\ntype = "joint_trajectory_tracking"\ndt = 0.01\nduration = 0.2\n'),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="requires a 'file'"):
        run_scenario(load_scenario(config))


def test_tracking_reproduces_with_the_reference_file_deleted(tmp_path: Path) -> None:
    """run_scenario embeds the reference content, so rerun reproduces without the file."""
    ref = tmp_path / "ref.sklog.npz"
    _write_reference(ref, channels=("q",))
    config = tmp_path / "track.toml"
    config.write_text(
        _scenario_toml(f'[task]\ntype = "joint_trajectory_tracking"\nfile = "{ref}"\ndt = 0.01\n'),
        encoding="utf-8",
    )
    original = run_scenario(load_scenario(config))
    assert "reference_samples" in original.extra["source_config"]["task"]  # content embedded

    saved = tmp_path / "run.sklog.npz"
    original.save(saved)
    ref.unlink()  # the external reference is gone
    replayed = rerun_log(StateLog.load(saved))

    np.testing.assert_array_equal(replayed.channel("q"), original.channel("q"))
