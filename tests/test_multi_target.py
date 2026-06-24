"""Tests for the multiple-target reaching task and its GUI live-switching."""

from __future__ import annotations

import os
import subprocess
import sys
import tomllib
from pathlib import Path

import numpy as np
import pytest

# Importing the tool pulls in PyQt6; run headless.
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from skelarm.reaching import VirtualSpringDamper
from skelarm.scenario import (
    Task,
    active_target_index,
    apply_active_target,
    multi_target_specs,
    run_scenario,
    scenario_from_config,
)

_EXAMPLE = Path(__file__).resolve().parents[1] / "examples" / "multi_target.toml"


def _example_config() -> dict:
    with _EXAMPLE.open("rb") as f:
        return tomllib.load(f)


def _task(active: int = 0) -> Task:
    targets = [{"pos": [1.0, 0.2]}, {"pos": [0.3, 1.0], "color": "teal"}]
    return Task.from_dict({"type": "multi_target_reaching", "targets": targets, "active": active})


def test_multi_target_specs_and_active_index() -> None:
    """The targets list and active index parse from the task params."""
    task = _task(active=1)
    specs = multi_target_specs(task)
    assert len(specs) == 2  # noqa: PLR2004
    assert specs[1][0] == pytest.approx([0.3, 1.0])
    assert specs[1][2] == "teal"  # color
    assert active_target_index(task) == 1


def test_apply_active_target_sets_first_class_target() -> None:
    """apply_active_target promotes the active candidate to task.target."""
    task = _task(active=1)
    apply_active_target(task)
    assert task.target == pytest.approx([0.3, 1.0])
    assert task.color == "teal"


def test_missing_targets_raises() -> None:
    """A multi-target task without a targets list is rejected."""
    task = Task.from_dict({"type": "multi_target_reaching"})
    with pytest.raises(ValueError, match="requires a non-empty 'targets'"):
        multi_target_specs(task)


def test_out_of_range_active_raises() -> None:
    """An active index outside the targets list is rejected."""
    task = _task(active=5)
    with pytest.raises(ValueError, match="out of range"):
        apply_active_target(task)


def test_scenario_builds_with_the_active_target() -> None:
    """Loading a multi-target scenario sets the controller's target to the active one."""
    scenario = scenario_from_config(_example_config())
    assert isinstance(scenario.controller, VirtualSpringDamper)
    assert scenario.task.target == pytest.approx([1.2, 0.4])  # active = 0
    assert scenario.controller.target == pytest.approx([1.2, 0.4])


def test_headless_run_reaches_the_active_target() -> None:
    """A headless run drives the arm to the configured active target."""
    config = _example_config()
    config["task"]["active"] = 1  # target B
    scenario = scenario_from_config(config)
    log = run_scenario(scenario, duration=3.0, dt=0.005)
    final = log.build_skeleton()
    final.q = log.channel("q")[-1]
    tip = np.array([final.links[-1].xe, final.links[-1].ye])
    assert tip == pytest.approx([0.3, 1.2], abs=3e-2)


@pytest.fixture(scope="module")
def qapp():  # noqa: ANN201
    """Provide a single QApplication instance for the GUI test."""
    from PyQt6.QtWidgets import QApplication

    return QApplication.instance() or QApplication([])


def test_gui_switch_retargets_the_controller(qapp) -> None:  # noqa: ANN001, ARG001
    """Switching the active target in the GUI retargets the reaching controller live."""
    from skelarm.scenario import load_scenario
    from tools.multi_target_simulator import MultiTargetReachSimulator

    window = MultiTargetReachSimulator(load_scenario(_EXAMPLE))
    assert window.active_index == 0
    assert window.canvas.target == pytest.approx([1.2, 0.4])
    assert len(window.canvas.secondary_targets) == 2  # noqa: PLR2004
    assert window.state_log is not None
    assert window.state_log.extra["source_config"]["task"]["type"] == "multi_target_reaching"  # embedded for the player

    window.switch_to(2)  # target C
    assert window.active_index == 2  # noqa: PLR2004
    assert window.canvas.target == pytest.approx([-0.4, 0.9])
    controller = window._controller  # noqa: SLF001
    assert isinstance(controller, VirtualSpringDamper)
    assert controller.target == pytest.approx([-0.4, 0.9])  # live retarget


def test_parser_has_override_and_save_flags() -> None:
    """The multi-target tool gained the shared override flags and --save (like reaching)."""
    from tools.multi_target_simulator import build_parser

    args = build_parser().parse_args([str(_EXAMPLE), "--save", "out.npz", "--controller", "pd.toml"])
    assert args.save == Path("out.npz")
    assert args.controller == Path("pd.toml")
    assert args.no_joint_limits is False


def test_headless_save_writes_a_replayable_log(tmp_path: Path) -> None:
    """The headless save runs the active-target reach and embeds the config."""
    from skelarm.recording import StateLog
    from tools._scenario_cli import run_headless

    out = tmp_path / "mt.sklog.npz"
    run_headless(_EXAMPLE, output=out, duration=0.2)
    log = StateLog.load(out)
    assert "q" in log.channel_names
    assert log.extra["source_config"]["task"]["type"] == "multi_target_reaching"


def test_runs_as_a_standalone_script() -> None:
    """Running the file directly (script mode) must resolve all of its imports."""
    script = Path(__file__).resolve().parents[1] / "tools" / "multi_target_simulator.py"
    result = subprocess.run(  # noqa: S603  # trusted: our own interpreter and script path
        [sys.executable, str(script), "--help"],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    assert "multiple-target" in result.stdout.lower()
