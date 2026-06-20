"""Tests for loading a control scenario (robot + task + controller) from TOML."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from skelarm.control import ComputedTorque, simulate_controlled
from skelarm.reaching import VirtualSpringDamper
from skelarm.scenario import Task, build_controller, load_scenario
from skelarm.skeleton import LinkProp, Skeleton

_SKELETON_TOML = (
    "[skeleton]\n"
    "[[skeleton.link]]\nlength = 1.0\nmass = 1.0\ninertia = 0.1\ncom = [0.5, 0.0]\nlimits = [-180.0, 180.0]\n"
    "[[skeleton.link]]\nlength = 0.8\nmass = 0.8\ninertia = 0.05\ncom = [0.4, 0.0]\nlimits = [-180.0, 180.0]\n"
)


def _two_link() -> Skeleton:
    """A planar two-link arm at a folded pose, matching the scenario fixtures."""
    skeleton = Skeleton(
        [
            LinkProp(length=1.0, m=1.0, i=0.1, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi),
            LinkProp(length=0.8, m=0.8, i=0.05, rgx=0.4, rgy=0.0, qmin=-np.pi, qmax=np.pi),
        ]
    )
    skeleton.q = np.array([0.6, 1.0])
    return skeleton


def _tip(skeleton: Skeleton) -> np.ndarray:
    """Current endpoint position."""
    tip = skeleton.links[-1]
    return np.array([tip.xe, tip.ye])


def _write_scenario(path: Path, controller_block: str, *, target: tuple[float, float] = (0.55, 1.21)) -> None:
    """Write a combined skeleton+initial+task+controller config."""
    text = (
        _SKELETON_TOML
        + "[initial]\nq = [34.4, 57.3]\n"
        + f"[task]\ntarget = [{target[0]}, {target[1]}]\nduration = 2.0\ndt = 0.002\n"
        + controller_block
    )
    path.write_text(text, encoding="utf-8")


def test_task_from_dict_parses_fields() -> None:
    """A [task] mapping yields the target, duration, dt, and schedule."""
    task = Task.from_dict({"target": [0.5, 0.4], "duration": 3.0, "dt": 0.01, "schedule": "quintic"})
    assert task.target == pytest.approx([0.5, 0.4])
    assert task.duration == pytest.approx(3.0)
    assert task.dt == pytest.approx(0.01)
    assert task.schedule == "quintic"


def test_task_requires_two_element_target() -> None:
    """A missing or wrong-shaped target is rejected."""
    with pytest.raises(ValueError, match="target"):
        Task.from_dict({"duration": 1.0})
    with pytest.raises(ValueError, match="target"):
        Task.from_dict({"target": [0.5, 0.4, 0.3]})


def test_task_from_toml_requires_task_section(tmp_path: Path) -> None:
    """Loading a Task from a file without a [task] section raises."""
    config = tmp_path / "no_task.toml"
    config.write_text(_SKELETON_TOML, encoding="utf-8")
    with pytest.raises(ValueError, match="task"):
        Task.from_toml(config)


@pytest.mark.parametrize(
    ("controller_type", "expected"),
    [
        ("computed_torque", "ComputedTorque"),
        ("joint_pd", "JointPD"),
        ("inverse_dynamics_pd", "InverseDynamicsFeedforwardPD"),
        ("virtual_spring_damper", "VirtualSpringDamper"),
        ("time_varying_stiffness", "TimeVaryingStiffness"),
        ("online_shaping", "OnlineReferenceShaping"),
        ("position_dependent_shaping", "PositionDependentShaping"),
        ("adaptive_shaping", "AdaptiveReferenceShaping"),
        ("mpc", "JointSpaceMPC"),
    ],
)
def test_build_controller_dispatches_each_type(controller_type: str, expected: str) -> None:
    """Each controller type name builds the matching controller class."""
    skeleton = _two_link()
    task = Task(target=_tip(skeleton) + np.array([-0.2, -0.1]))
    controller = build_controller({"type": controller_type}, skeleton=skeleton, task=task)
    assert type(controller).__name__ == expected


def test_build_controller_rejects_unknown_type() -> None:
    """An unknown controller type is reported."""
    skeleton = _two_link()
    task = Task(target=np.array([0.5, 0.5]))
    with pytest.raises(ValueError, match="unknown controller type"):
        build_controller({"type": "bogus"}, skeleton=skeleton, task=task)


def test_load_scenario_returns_robot_task_and_controller(tmp_path: Path) -> None:
    """load_scenario reads all three sections from one file."""
    config = tmp_path / "reach.toml"
    _write_scenario(config, '[controller]\ntype = "computed_torque"\nkp = 200.0\nkd = 30.0\n')

    scenario = load_scenario(config)
    assert scenario.skeleton.num_joints == len(_two_link().links) - 1
    assert scenario.task.target == pytest.approx([0.55, 1.21])
    assert isinstance(scenario.controller, ComputedTorque)


def test_scenario_computed_torque_reaches_target(tmp_path: Path) -> None:
    """A computed-torque scenario tracks the planned reach to the target."""
    config = tmp_path / "reach.toml"
    _write_scenario(config, '[controller]\ntype = "computed_torque"\nkp = 200.0\nkd = 30.0\n')

    scenario = load_scenario(config)
    log = simulate_controlled(scenario.skeleton, scenario.controller, duration=scenario.task.duration, dt=0.002)
    final = scenario.skeleton.clone()
    final.q = log.channel("q")[-1]
    assert _tip(final) == pytest.approx(scenario.task.target, abs=2e-2)


def test_scenario_virtual_spring_damper_reaches_target(tmp_path: Path) -> None:
    """A virtual spring-damper scenario settles the endpoint on the target."""
    config = tmp_path / "reach.toml"
    _write_scenario(
        config, '[controller]\ntype = "virtual_spring_damper"\nk_task = 120.0\nd_task = 25.0\nc_joint = 0.5\n'
    )

    scenario = load_scenario(config)
    assert isinstance(scenario.controller, VirtualSpringDamper)
    log = simulate_controlled(scenario.skeleton, scenario.controller, duration=5.0, dt=0.002)
    final = scenario.skeleton.clone()
    final.q = log.channel("q")[-1]
    assert _tip(final) == pytest.approx(scenario.task.target, abs=3e-2)


def test_example_reach_config_loads() -> None:
    """The shipped examples/reach.toml is a valid scenario."""
    config = Path(__file__).resolve().parents[1] / "examples" / "reach.toml"
    scenario = load_scenario(config)
    assert scenario.skeleton.num_joints == len(_two_link().links) - 1
    assert isinstance(scenario.controller, ComputedTorque)
