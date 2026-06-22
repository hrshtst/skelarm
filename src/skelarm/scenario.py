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

import importlib.metadata
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
    simulate_controlled,
)
from skelarm.mpc import JointSpaceMPC
from skelarm.reaching import (
    AdaptiveReferenceShaping,
    OnlineReferenceShaping,
    PositionDependentShaping,
    TimeVaryingStiffness,
    VirtualSpringDamper,
)
from skelarm.recording import StateLog
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
    """A loaded control scenario: the robot, the task, and a ready-to-run controller.

    ``controller_config`` keeps the raw ``[controller]`` table (``type`` plus gains)
    when the scenario was loaded from a config, so a run can embed it for later
    reproduction; it is ``None`` for controllers built programmatically.
    """

    skeleton: Skeleton
    task: Task
    controller: Controller
    controller_config: Mapping[str, Any] | None = None


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
    controller_config = dict(_read_controller_section(file_path))
    controller = build_controller(controller_config, skeleton=skeleton, task=task)
    return Scenario(skeleton=skeleton, task=task, controller=controller, controller_config=controller_config)


def _read_controller_section(file_path: str | Path) -> Mapping[str, Any]:
    """Read the ``[controller]`` table from a TOML file."""
    with Path(file_path).open("rb") as f:
        data = tomllib.load(f)
    if "controller" not in data:
        msg = f"no [controller] section in {file_path}"
        raise ValueError(msg)
    return data["controller"]


_PROVENANCE_PACKAGES = ("skelarm", "numpy", "scipy")


def _provenance() -> dict[str, str]:
    """Record the installed versions of the packages that affect reproducibility."""
    versions: dict[str, str] = {}
    for package in _PROVENANCE_PACKAGES:
        try:
            versions[package] = importlib.metadata.version(package)
        except importlib.metadata.PackageNotFoundError:  # pragma: no cover - install-dependent
            versions[package] = "unknown"
    return versions


def _reproduction_metadata(
    scenario: Scenario,
    initial_q: NDArray[np.float64],
    initial_dq: NDArray[np.float64],
    *,
    duration: float,
    dt: float,
    grav_vec: NDArray[np.float64],
) -> dict[str, Any] | None:
    """Assemble the ``[extra]`` payload that lets a run be reconstructed and re-run.

    Returns ``None`` when the scenario carries no ``controller_config`` (e.g. a
    programmatically built controller), so the run is recorded without it.
    """
    if scenario.controller_config is None:
        return None
    controller = {key: value for key, value in scenario.controller_config.items() if value is not None}
    return {
        "scenario": {
            "initial": {"q": initial_q.tolist(), "dq": initial_dq.tolist()},
            "task": {
                "target": scenario.task.target.tolist(),
                "duration": float(scenario.task.duration),
                "dt": float(scenario.task.dt),
                "schedule": scenario.task.schedule,
            },
            "controller": controller,
            "run": {"duration": float(duration), "dt": float(dt), "grav_vec": grav_vec.tolist()},
        },
        "provenance": _provenance(),
    }


def run_scenario(
    scenario: Scenario,
    *,
    duration: float | None = None,
    dt: float | None = None,
    grav_vec: NDArray[np.float64] | None = None,
) -> StateLog:
    """Simulate a scenario and embed its full config for later reproduction.

    Runs :func:`~skelarm.control.simulate_controlled` and, when the scenario was
    loaded from a config, stores the task, controller, initial state, and run
    parameters in the log's ``[extra]`` table so :func:`rerun_log` can reconstruct
    and re-simulate it.

    Parameters
    ----------
    scenario : Scenario
        The robot, task, and controller to run.
    duration : float | None, optional
        Simulated duration (seconds); defaults to ``scenario.task.duration``.
    dt : float | None, optional
        Control / integration step; defaults to ``scenario.task.dt``.
    grav_vec : NDArray[np.float64] | None, optional
        Gravity vector; defaults to zero (planar motion).

    Returns
    -------
    StateLog
        The recorded run, with reproduction metadata when available.
    """
    run_duration = scenario.task.duration if duration is None else float(duration)
    run_dt = scenario.task.dt if dt is None else float(dt)
    grav = np.zeros(_TASK_DIM, dtype=np.float64) if grav_vec is None else np.asarray(grav_vec, dtype=np.float64)
    initial_q = scenario.skeleton.q.copy()
    initial_dq = scenario.skeleton.dq.copy()
    extra = _reproduction_metadata(scenario, initial_q, initial_dq, duration=run_duration, dt=run_dt, grav_vec=grav)
    return simulate_controlled(
        scenario.skeleton, scenario.controller, duration=run_duration, dt=run_dt, grav_vec=grav, extra=extra
    )


def scenario_from_log(log: StateLog) -> tuple[Scenario, Mapping[str, Any]]:
    """Reconstruct the scenario and run parameters embedded in a log by :func:`run_scenario`.

    Returns
    -------
    tuple[Scenario, Mapping[str, Any]]
        The rebuilt scenario (skeleton posed at the recorded initial state) and the
        ``run`` parameters (``duration`` / ``dt`` / ``grav_vec``).

    Raises
    ------
    ValueError
        If the log carries no reproduction metadata (was not produced by
        :func:`run_scenario`).
    """
    config = log.extra.get("scenario")
    if not config:
        msg = "log has no reproduction metadata; it was not produced by run_scenario"
        raise ValueError(msg)
    skeleton = log.build_skeleton()
    initial = config["initial"]
    skeleton.set_state(
        q=np.asarray(initial["q"], dtype=np.float64),
        dq=np.asarray(initial["dq"], dtype=np.float64),
    )
    task = Task.from_dict(config["task"])
    controller_config = dict(config["controller"])
    controller = build_controller(controller_config, skeleton=skeleton, task=task)
    scenario = Scenario(skeleton=skeleton, task=task, controller=controller, controller_config=controller_config)
    return scenario, config["run"]


def rerun_log(log: StateLog) -> StateLog:
    """Reconstruct and re-simulate a scenario recorded by :func:`run_scenario`.

    The new run uses the recorded initial state, controller, task, and run
    parameters, so deterministic controllers reproduce the original channels
    exactly (MPC, which calls :func:`scipy.optimize.minimize`, reproduces within a
    small numerical tolerance on the same platform).

    Raises
    ------
    ValueError
        If the log carries no reproduction metadata.
    """
    scenario, run = scenario_from_log(log)
    grav_vec = np.asarray(run["grav_vec"], dtype=np.float64)
    return run_scenario(scenario, duration=run["duration"], dt=run["dt"], grav_vec=grav_vec)
