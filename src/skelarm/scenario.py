"""Load a full control scenario (robot + task + controller) from one TOML file.

A combined config mirrors the per-section schema used by :meth:`Skeleton.from_toml`:
``[skeleton]`` and ``[initial]`` describe the robot and its start state, ``[task]``
the goal (a task-space target with a movement duration), and ``[controller]`` which
controller drives the reach. :func:`load_scenario` reads all three.

Example::

    [task]
    target = [0.55, 1.21]  # endpoint goal (x, y) in meters
    duration = 2.0  # simulated time (s)
    dt = 0.002  # control step (s)

    [controller]
    type = "computed_torque"
    kp = 200.0
    kd = 30.0
"""

from __future__ import annotations

import tomllib
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any

import numpy as np

from skelarm.control import (
    ComputedTorque,
    Controller,
    InverseDynamicsFeedforwardPD,
    JointPD,
    ik_joint_reference,
)
from skelarm.mpc import JointSpaceMPC
from skelarm.reaching import (
    AdaptiveReferenceShaping,
    OnlineReferenceShaping,
    PositionDependentShaping,
    TimeVaryingStiffness,
    VirtualSpringDamper,
)
from skelarm.skeleton import Skeleton
from skelarm.trajectory import Trajectory

if TYPE_CHECKING:
    from numpy.typing import NDArray

_REFERENCE_DT = 0.02  # sampling step for the IK-converted joint reference (interpolated)
_TASK_DIM = 2  # planar endpoint target (x, y)


@dataclass
class Task:
    """A reaching task: a task-space target and the run conditions for the reach.

    Parameters
    ----------
    target : NDArray[np.float64]
        The endpoint goal ``(x, y)`` in meters.
    duration : float
        Total simulated time (seconds).
    dt : float
        Control / integration step (seconds).
    schedule : str
        Time-scaling schedule for planned-trajectory controllers.
    """

    target: NDArray[np.float64]
    duration: float = 2.0
    dt: float = 0.002
    schedule: str = "minimum_jerk"

    @classmethod
    def from_toml(cls, file_path: str | Path) -> Task:
        """Build a Task from the ``[task]`` table of a TOML file."""
        with Path(file_path).open("rb") as f:
            data = tomllib.load(f)
        if "task" not in data:
            msg = f"no [task] section in {file_path}"
            raise ValueError(msg)
        return cls.from_dict(data["task"])

    @classmethod
    def from_dict(cls, section: Mapping[str, Any]) -> Task:
        """Build a Task from a ``[task]`` mapping.

        Raises
        ------
        ValueError
            If ``target`` is missing or is not a two-element ``(x, y)`` vector.
        """
        if "target" not in section:
            msg = "[task] requires a 'target' endpoint"
            raise ValueError(msg)
        target = np.asarray(section["target"], dtype=np.float64)
        if target.shape != (_TASK_DIM,):
            msg = f"[task].target must have {_TASK_DIM} elements (x, y), got shape {target.shape}"
            raise ValueError(msg)
        return cls(
            target=target,
            duration=float(section.get("duration", 2.0)),
            dt=float(section.get("dt", 0.002)),
            schedule=str(section.get("schedule", "minimum_jerk")),
        )


@dataclass
class Scenario:
    """A loaded control scenario: the robot, the task, and a ready-to-run controller."""

    skeleton: Skeleton
    task: Task
    controller: Controller


def _endpoint(skeleton: Skeleton) -> NDArray[np.float64]:
    """Current endpoint position of the arm."""
    tip = skeleton.links[-1]
    return np.array([tip.xe, tip.ye], dtype=np.float64)


def _joint_reference(skeleton: Skeleton, task: Task) -> Any:  # noqa: ANN401  # SampledJointReference
    """Build a joint reference by converting a planned task-space reach with IK."""
    trajectory = Trajectory(_endpoint(skeleton), task.target, task.duration, task.schedule)
    return ik_joint_reference(skeleton, trajectory, dt=_REFERENCE_DT)


def _build_computed_torque(params: Mapping[str, Any], skeleton: Skeleton, task: Task) -> Controller:
    return ComputedTorque(_joint_reference(skeleton, task), kp=params.get("kp", 200.0), kd=params.get("kd", 30.0))


def _build_joint_pd(params: Mapping[str, Any], skeleton: Skeleton, task: Task) -> Controller:
    return JointPD(_joint_reference(skeleton, task), kp=params.get("kp", 300.0), kd=params.get("kd", 40.0))


def _build_inverse_dynamics_pd(params: Mapping[str, Any], skeleton: Skeleton, task: Task) -> Controller:
    reference = _joint_reference(skeleton, task)
    return InverseDynamicsFeedforwardPD(reference, kp=params.get("kp", 100.0), kd=params.get("kd", 20.0))


def _build_virtual_spring_damper(params: Mapping[str, Any], _skeleton: Skeleton, task: Task) -> Controller:
    return VirtualSpringDamper(
        task.target,
        k_task=params.get("k_task", 150.0),
        d_task=params.get("d_task", 25.0),
        c_joint=params.get("c_joint", 0.0),
    )


def _build_time_varying_stiffness(params: Mapping[str, Any], _skeleton: Skeleton, task: Task) -> Controller:
    return TimeVaryingStiffness(
        task.target,
        k0=params.get("k0", 150.0),
        alpha=params.get("alpha", 6.0),
        zeta1=params.get("zeta1", 0.15),
        c_joint=params.get("c_joint", 0.0),
    )


def _build_online_shaping(params: Mapping[str, Any], _skeleton: Skeleton, task: Task) -> Controller:
    return OnlineReferenceShaping(
        task.target,
        k_task=params.get("k_task", 150.0),
        d_task=params.get("d_task", 25.0),
        c_joint=params.get("c_joint", 0.0),
        r=params.get("r", 0.5),
        t1=params.get("t1", 0.2),
        t2=params.get("t2", 0.2),
    )


def _build_position_dependent_shaping(params: Mapping[str, Any], _skeleton: Skeleton, task: Task) -> Controller:
    return PositionDependentShaping(
        task.target,
        k_task=params.get("k_task", 150.0),
        d_task=params.get("d_task", 25.0),
        c_joint=params.get("c_joint", 0.0),
        a=params.get("a", 0.01),
        t1=params.get("t1", 0.2),
        t2=params.get("t2", 0.2),
    )


def _build_adaptive_shaping(params: Mapping[str, Any], _skeleton: Skeleton, task: Task) -> Controller:
    return AdaptiveReferenceShaping(
        task.target,
        k_task=params.get("k_task", 150.0),
        d_task=params.get("d_task", 25.0),
        c_joint=params.get("c_joint", 0.0),
        epsilon=params.get("epsilon", 0.01),
        t_adapt=params.get("t_adapt", 5.0),
        t1=params.get("t1", 0.2),
        t2=params.get("t2", 0.2),
    )


def _build_mpc(params: Mapping[str, Any], skeleton: Skeleton, task: Task) -> Controller:
    return JointSpaceMPC(
        _joint_reference(skeleton, task),
        horizon=int(params.get("horizon", 6)),
        dt=task.dt,  # the prediction step must match the simulation control step
        q_weight=params.get("q_weight", 10.0),
        dq_weight=params.get("dq_weight", 1.0),
        tau_weight=params.get("tau_weight", 1e-3),
        terminal_weight=params.get("terminal_weight", 50.0),
        tau_max=params.get("tau_max"),
        limit_weight=params.get("limit_weight", 0.0),
        max_iter=int(params.get("max_iter", 20)),
    )


_BUILDERS: dict[str, Callable[[Mapping[str, Any], Skeleton, Task], Controller]] = {
    "computed_torque": _build_computed_torque,
    "joint_pd": _build_joint_pd,
    "inverse_dynamics_pd": _build_inverse_dynamics_pd,
    "virtual_spring_damper": _build_virtual_spring_damper,
    "time_varying_stiffness": _build_time_varying_stiffness,
    "online_shaping": _build_online_shaping,
    "position_dependent_shaping": _build_position_dependent_shaping,
    "adaptive_shaping": _build_adaptive_shaping,
    "mpc": _build_mpc,
}

CONTROLLER_TYPES = tuple(_BUILDERS)


def build_controller(source: str | Path | Mapping[str, Any], *, skeleton: Skeleton, task: Task) -> Controller:
    """Construct the controller described by a ``[controller]`` config.

    Parameters
    ----------
    source : str | Path | Mapping[str, Any]
        A TOML file path (its ``[controller]`` table is read) or the table itself.
    skeleton : Skeleton
        The robot, posed at its start state; used to build joint references.
    task : Task
        The reaching task supplying the target and run conditions.

    Returns
    -------
    Controller
        The configured controller.

    Raises
    ------
    ValueError
        If the config has no ``type`` or an unknown controller ``type``.
    """
    config = source if isinstance(source, Mapping) else _read_controller_section(source)
    controller_type = config.get("type")
    if controller_type not in _BUILDERS:
        msg = f"unknown controller type {controller_type!r}; choose from {list(CONTROLLER_TYPES)}"
        raise ValueError(msg)
    params = {key: value for key, value in config.items() if key != "type"}
    return _BUILDERS[controller_type](params, skeleton, task)


def load_scenario(file_path: str | Path) -> Scenario:
    """Load a robot, task, and controller from one combined TOML file."""
    skeleton = Skeleton.from_toml(file_path)
    task = Task.from_toml(file_path)
    controller = build_controller(file_path, skeleton=skeleton, task=task)
    return Scenario(skeleton=skeleton, task=task, controller=controller)


def _read_controller_section(file_path: str | Path) -> Mapping[str, Any]:
    """Read the ``[controller]`` table from a TOML file."""
    with Path(file_path).open("rb") as f:
        data = tomllib.load(f)
    if "controller" not in data:
        msg = f"no [controller] section in {file_path}"
        raise ValueError(msg)
    return data["controller"]
