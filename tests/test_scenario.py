"""Tests for loading a control scenario (robot + task + controller) from TOML."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

import skelarm.scenario as scenario_mod
from skelarm.control import ComputedTorque, Controller, simulate_controlled
from skelarm.reaching import VirtualSpringDamper
from skelarm.scenario import (
    Task,
    build_controller,
    controller_types,
    load_scenario,
    register_controller,
    register_task_type,
    task_types,
)
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
        + f'[task]\ntype = "reaching"\ntarget = [{target[0]}, {target[1]}]\nduration = 2.0\ndt = 0.002\n'
        + controller_block
    )
    path.write_text(text, encoding="utf-8")


def test_task_from_dict_parses_fields() -> None:
    """A [task] mapping yields the target, duration, dt, and schedule, with defaults."""
    task = Task.from_dict(
        {"type": "reaching", "target": [0.5, 0.4], "duration": 3.0, "dt": 0.01, "schedule": "quintic"}
    )
    assert task.target == pytest.approx([0.5, 0.4])
    assert task.duration == pytest.approx(3.0)
    assert task.dt == pytest.approx(0.01)
    assert task.schedule == "quintic"
    # Defaults for the target attributes.
    assert task.type == "reaching"
    assert task.label is None
    assert task.color == "purple"
    assert task.tolerance is None
    assert task.enforce_limits is True  # joint-limit hard stop on by default


def test_task_from_dict_requires_type() -> None:
    """[task] must declare its kind; the type is the required discriminator."""
    with pytest.raises(ValueError, match="requires a 'type'"):
        Task.from_dict({"target": [0.5, 0.4]})


def test_task_from_dict_reads_enforce_limits() -> None:
    """``[task].enforce_limits`` toggles the dynamics joint-limit hard stop (default True)."""
    base = {"type": "reaching", "target": [0.5, 0.4]}
    assert Task.from_dict(base).enforce_limits is True
    assert Task.from_dict({**base, "enforce_limits": False}).enforce_limits is False


def test_task_target_table_parses_attributes() -> None:
    """A [task].target table yields the position plus label, color, and tolerance."""
    task = Task.from_dict(
        {"type": "reaching", "target": {"pos": [0.4, 0.9], "label": "goal", "color": "green", "tolerance": 0.02}}
    )
    assert task.target == pytest.approx([0.4, 0.9])
    assert task.type == "reaching"
    assert task.label == "goal"
    assert task.color == "green"
    assert task.tolerance == pytest.approx(0.02)


def test_reaching_task_requires_a_target() -> None:
    """The target is required for (and specific to) the reaching task."""
    with pytest.raises(ValueError, match="requires a 'target'"):
        Task.from_dict({"type": "reaching"})
    with pytest.raises(ValueError, match="target"):
        Task.from_dict({"type": "reaching", "target": [0.5, 0.4, 0.3]})  # wrong shape
    with pytest.raises(ValueError, match="pos"):
        Task.from_dict({"type": "reaching", "target": {"label": "x"}})


@pytest.mark.usefixtures("_registries")
def test_non_reaching_task_may_omit_target() -> None:
    """A custom (registered) task type without a target loads with ``target=None``."""
    register_task_type("waypoints")
    task = Task.from_dict({"type": "waypoints", "points": [[0.1, 0.2], [0.3, 0.4]]})
    assert task.target is None
    assert task.params["points"] == [[0.1, 0.2], [0.3, 0.4]]
    with pytest.raises(ValueError, match="no target"):
        task.require_target()


def test_task_rejects_unknown_type() -> None:
    """An unimplemented task type is reported."""
    with pytest.raises(ValueError, match="unknown task type"):
        Task.from_dict({"type": "tracking", "target": [0.5, 0.4]})


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
    task = Task(type="reaching", target=_tip(skeleton) + np.array([-0.2, -0.1]))
    controller = build_controller({"type": controller_type}, skeleton=skeleton, task=task)
    assert type(controller).__name__ == expected


def test_build_controller_rejects_unknown_type() -> None:
    """An unknown controller type is reported."""
    skeleton = _two_link()
    task = Task(type="reaching", target=np.array([0.5, 0.5]))
    with pytest.raises(ValueError, match="unknown controller type"):
        build_controller({"type": "bogus"}, skeleton=skeleton, task=task)


@pytest.fixture
def _registries():  # noqa: ANN202
    """Snapshot and restore the controller / task-type registries around a test."""
    builders = dict(scenario_mod._BUILDERS)  # noqa: SLF001
    types = set(scenario_mod._TASK_TYPES)  # noqa: SLF001
    yield
    scenario_mod._BUILDERS.clear()  # noqa: SLF001
    scenario_mod._BUILDERS.update(builders)  # noqa: SLF001
    scenario_mod._TASK_TYPES.clear()  # noqa: SLF001
    scenario_mod._TASK_TYPES.update(types)  # noqa: SLF001


def test_task_from_dict_captures_extra_keys_as_params() -> None:
    """Unrecognized [task] keys are preserved on ``task.params`` for custom tasks/controllers."""
    task = Task.from_dict({"type": "reaching", "target": [0.5, 0.4], "radius": 0.3, "period": 2.0})
    assert task.params["radius"] == pytest.approx(0.3)
    assert task.params["period"] == pytest.approx(2.0)
    assert "duration" not in task.params  # known keys stay off params
    assert "target" not in task.params
    assert "type" not in task.params


@pytest.mark.usefixtures("_registries")
def test_register_task_type_allows_a_new_type() -> None:
    """A registered task type passes validation and carries its custom parameters."""
    register_task_type("tracing")
    assert "tracing" in task_types()

    task = Task.from_dict({"type": "tracing", "target": [0.5, 0.4], "radius": 0.3})
    assert task.type == "tracing"
    assert task.params["radius"] == pytest.approx(0.3)


@pytest.mark.usefixtures("_registries")
def test_register_controller_makes_it_usable_via_config() -> None:
    """A registered builder is dispatched by ``build_controller`` from its ``[controller].type``."""

    class ConstantTorque(Controller):
        def __init__(self, value: float) -> None:
            self.value = value

        def control(self, t: float, skeleton: Skeleton) -> np.ndarray:  # noqa: ARG002
            return np.full(skeleton.num_joints, self.value)

    def build(params, skeleton, task):  # noqa: ANN001, ANN202, ARG001
        return ConstantTorque(params.get("value", 1.0))

    register_controller("constant_torque", build)
    assert "constant_torque" in controller_types()

    skeleton = _two_link()
    task = Task(type="reaching", target=np.array([0.5, 0.5]))
    controller = build_controller({"type": "constant_torque", "value": 2.0}, skeleton=skeleton, task=task)
    assert isinstance(controller, ConstantTorque)
    assert controller.value == pytest.approx(2.0)


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
