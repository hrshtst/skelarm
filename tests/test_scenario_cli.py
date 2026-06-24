"""Tests for the shared tools scenario-CLI helpers (tools/_scenario_cli.py)."""

from __future__ import annotations

import argparse
import os
import tomllib
from pathlib import Path

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from skelarm import Skeleton, Task
from skelarm.recording import StateLog
from tools._scenario_cli import (
    add_override_arguments,
    build_scenario,
    save_scenario_run,
    task_overlays,
)

pytestmark = pytest.mark.integration

_EXAMPLES = Path(__file__).resolve().parents[1] / "examples"
_SKELETON_TOML = (
    "[skeleton]\n"
    "[[skeleton.link]]\nlength = 1.0\nmass = 1.0\ninertia = 0.1\ncom = [0.5, 0.0]\nlimits = [-180.0, 180.0]\n"
    "[[skeleton.link]]\nlength = 0.8\nmass = 0.8\ninertia = 0.05\ncom = [0.4, 0.0]\nlimits = [-180.0, 180.0]\n"
)


def _two_link() -> Skeleton:
    return Skeleton.from_config(tomllib.loads(_SKELETON_TOML))


def test_reaching_overlay_has_one_active_target() -> None:
    """A reaching task yields a single active target marker and no path."""
    scenario = build_scenario(_EXAMPLES / "reach.toml")
    targets, path = task_overlays(scenario.task, scenario.skeleton)
    assert len(targets) == 1
    assert targets[0][3] is True  # active
    assert path is None


def test_multi_target_overlay_flags_the_active() -> None:
    """A multi-target task yields every candidate, exactly one flagged active."""
    scenario = build_scenario(_EXAMPLES / "multi_target.toml")
    targets, path = task_overlays(scenario.task, scenario.skeleton)
    assert len(targets) == 3  # noqa: PLR2004
    assert sum(1 for *_rest, active in targets if active) == 1
    assert targets[0][3] is True  # active = 0 in the example
    assert path is None


def test_periodic_curve_overlay_is_a_closed_path() -> None:
    """A periodic-curve task yields a closed reference polyline and no target."""
    scenario = build_scenario(_EXAMPLES / "periodic_curve.toml")
    targets, path = task_overlays(scenario.task, scenario.skeleton)
    assert targets == []
    assert path is not None
    assert path.shape[1] == 2  # noqa: PLR2004
    assert path[0] == pytest.approx(path[-1], abs=1e-6)  # closed


def test_trajectory_tracking_overlay_is_the_tip_series() -> None:
    """A task-space trajectory yields the recorded tip path verbatim."""
    task = Task.from_dict(
        {
            "type": "trajectory_tracking",
            "reference_samples": {"channel": "tip", "times": [0.0, 1.0], "values": [[1.0, 0.0], [1.1, 0.2]]},
        }
    )
    _, path = task_overlays(task, _two_link())
    assert path == pytest.approx(np.array([[1.0, 0.0], [1.1, 0.2]]))


def test_joint_trajectory_overlay_is_forward_kinematics() -> None:
    """A per-joint trajectory yields the tip path from forward kinematics of q."""
    task = Task.from_dict(
        {
            "type": "joint_trajectory_tracking",
            "reference_samples": {"channel": "q", "times": [0.0, 1.0], "values": [[0.1, 0.2], [0.4, -0.3]]},
        }
    )
    skeleton = _two_link()
    _, path = task_overlays(task, skeleton)
    assert path is not None
    assert path.shape == (2, 2)
    skeleton.q = np.array([0.1, 0.2])  # the first FK point matches
    assert path[0] == pytest.approx([skeleton.links[-1].xe, skeleton.links[-1].ye])


def test_add_override_arguments_parses_the_shared_flags() -> None:
    """The shared override flags are added and parsed."""
    parser = argparse.ArgumentParser()
    parser.add_argument("config")
    add_override_arguments(parser)
    args = parser.parse_args(
        ["c.toml", "--initial", "i.toml", "--pose", "10,20", "--task", "t.toml", "--no-joint-limits"]
    )
    assert args.initial == Path("i.toml")
    assert args.pose == "10,20"
    assert args.task == Path("t.toml")
    assert args.no_joint_limits is True


def test_save_scenario_run_writes_a_replayable_log(tmp_path: Path) -> None:
    """save_scenario_run writes a .sklog.npz embedding the task config."""
    scenario = build_scenario(_EXAMPLES / "periodic_curve.toml")
    out = tmp_path / "run.sklog.npz"
    save_scenario_run(scenario, out, duration=0.2, enforce_limits=True)
    log = StateLog.load(out)
    assert "q" in log.channel_names
    assert log.extra["source_config"]["task"]["type"] == "periodic_curve"
