"""Load a full control scenario (robot + task + controller) from one TOML file.

A combined config mirrors the per-section schema used by :meth:`Skeleton.from_toml`:
``[skeleton]`` and ``[initial]`` describe the robot and its start state, ``[task]``
the goal (a task-space target with a movement duration), and ``[controller]`` which
controller drives the reach. :func:`load_scenario` reads all three.

Example::

    [task]
    type = "reaching"  # the task kind (required)
    target = [0.55, 1.21]  # endpoint goal (x, y) in meters (required for reaching)
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
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Any

import numpy as np

from skelarm.control import (
    ComputedTorque,
    Controller,
    InverseDynamicsFeedforwardPD,
    JointPD,
    JointReference,
    SampledJointReference,
    SampledTaskReference,
    ik_joint_reference,
    simulate_controlled,
)
from skelarm.curves import PeriodicTaskReference, build_curve
from skelarm.filtering import smooth
from skelarm.interpolation import resample_with_derivatives
from skelarm.mpc import JointSpaceMPC
from skelarm.reaching import (
    AdaptiveReferenceShaping,
    OnlineReferenceShaping,
    PositionDependentShaping,
    TimeVaryingStiffness,
    VirtualSpringDamper,
)
from skelarm.recording import StateLog, dump_toml
from skelarm.skeleton import Skeleton
from skelarm.trajectory import Trajectory

if TYPE_CHECKING:
    from numpy.typing import NDArray

_REFERENCE_DT = 0.02  # sampling step for the IK-converted joint reference (interpolated)
_TASK_DIM = 2  # planar endpoint target (x, y)
_REACHING_TYPE = "reaching"  # the built-in task type; the one that requires a target
_MULTI_TARGET_TYPE = "multi_target_reaching"  # several candidate targets; one active, switchable in the GUI
_TASK_TYPES: set[str] = {_REACHING_TYPE}  # supported [task].type values; extend via register_task_type
# Top-level [task] keys consumed by Task; any others are kept on Task.params for custom tasks.
_KNOWN_TASK_KEYS = frozenset({"type", "target", "duration", "dt", "schedule", "enforce_limits"})
_DEFAULT_TARGET_COLOR = "purple"  # marker color when a target omits one


def register_task_type(name: str) -> None:
    """Allow ``name`` as a ``[task].type``.

    Task types are a label that a controller interprets; a custom task carries its
    parameters on :attr:`Task.params` (any extra ``[task]`` keys). Register the type
    so :meth:`Task.from_dict` accepts it, then read ``task.params`` from a matching
    controller built via :func:`register_controller`.

    Parameters
    ----------
    name : str
        The task-type string to allow (idempotent).
    """
    _TASK_TYPES.add(name)


def task_types() -> tuple[str, ...]:
    """Return the registered ``[task].type`` values, sorted."""
    return tuple(sorted(_TASK_TYPES))


@dataclass
class Task:
    """A task and the run conditions for it.

    The required field is ``type`` — the kind of task, which determines what else is
    needed. ``reaching`` (the only built-in) requires a ``target``; other task types
    (see :func:`register_task_type`) carry their own data on :attr:`params` and may
    omit the target.

    Parameters
    ----------
    type : str
        The task kind (the ``[task].type`` config key). Required.
    target : NDArray[np.float64] | None
        The endpoint goal ``(x, y)`` in meters. Required for ``reaching``; ``None``
        for task types that do not use a single point goal.
    label : str | None
        Optional name for the target, used to select it among multiple targets
        (multi-target tasks are defined later).
    color : str
        Marker color for the target (any Qt/SVG color name or ``#rrggbb``).
    tolerance : float | None
        Optional success distance in meters: the reach succeeds when the tip is
        within this distance of the target, and the target marker's ring is drawn
        at this radius. ``None`` applies no success criterion.
    duration : float
        Total simulated time (seconds).
    dt : float
        Control / integration step (seconds).
    schedule : str
        Time-scaling schedule for planned-trajectory controllers.
    enforce_limits : bool
        Apply the joint limits as hard stops in the dynamics (default). When
        ``False``, the limits constrain only the kinematics (posing and inverse
        kinematics), not the simulated motion.
    params : dict[str, Any]
        Any extra ``[task]`` keys not recognized above, kept verbatim. A custom
        task type carries its own parameters here for a matching controller builder
        to read (e.g. ``task.params["radius"]``).
    """

    type: str  # the task kind (the [task].type config key); the required discriminator
    target: NDArray[np.float64] | None = None
    label: str | None = None
    color: str = _DEFAULT_TARGET_COLOR
    tolerance: float | None = None
    duration: float = 2.0
    dt: float = 0.002
    schedule: str = "minimum_jerk"
    enforce_limits: bool = True
    params: dict[str, Any] = field(default_factory=dict)

    def require_target(self) -> NDArray[np.float64]:
        """Return the task target, raising if this task type has none.

        Use this from a controller that needs a point goal (e.g. the reaching
        controllers), so a task wired without a target fails with a clear message.

        Raises
        ------
        ValueError
            If the task has no target.
        """
        if self.target is None:
            msg = f"task type {self.type!r} has no target, but this controller requires one"
            raise ValueError(msg)
        return self.target

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

        ``type`` is required; it selects the task kind. The ``reaching`` type also
        requires a ``target`` — either a ``[x, y]`` array (position only) or a table
        with a ``pos`` and optional ``label`` / ``color`` / ``tolerance``. Other task
        types may omit the target. Any keys beyond the recognized ones are kept on
        :attr:`params` for custom task types.

        Raises
        ------
        ValueError
            If ``type`` is missing or unknown, or a ``reaching`` task has no/ill-shaped
            ``target``.
        """
        if "type" not in section:
            msg = "[task] requires a 'type' (e.g. type = \"reaching\")"
            raise ValueError(msg)
        task_type = str(section["type"])
        if task_type not in _TASK_TYPES:
            msg = f"unknown task type {task_type!r}; choose from {sorted(_TASK_TYPES)} (or register_task_type)"
            raise ValueError(msg)

        pos: NDArray[np.float64] | None
        if "target" in section:
            pos, label, color, tolerance = _parse_target(section["target"])
        elif task_type == _REACHING_TYPE:
            msg = "[task] of type 'reaching' requires a 'target' endpoint"
            raise ValueError(msg)
        else:
            pos, label, color, tolerance = None, None, _DEFAULT_TARGET_COLOR, None

        params = {key: value for key, value in section.items() if key not in _KNOWN_TASK_KEYS}
        return cls(
            type=task_type,
            target=pos,
            label=label,
            color=color,
            tolerance=tolerance,
            duration=float(section.get("duration", 2.0)),
            dt=float(section.get("dt", 0.002)),
            schedule=str(section.get("schedule", "minimum_jerk")),
            params=params,
            enforce_limits=bool(section.get("enforce_limits", True)),
        )


def _parse_target(raw: Any) -> tuple[NDArray[np.float64], str | None, str, float | None]:  # noqa: ANN401
    """Parse a ``[task].target`` (array or table) into ``(pos, label, color, tolerance)``."""
    if isinstance(raw, Mapping):
        if "pos" not in raw:
            msg = "[task].target table requires a 'pos' (x, y)"
            raise ValueError(msg)
        pos = np.asarray(raw["pos"], dtype=np.float64)
        label = None if raw.get("label") is None else str(raw["label"])
        color = str(raw.get("color", _DEFAULT_TARGET_COLOR))
        tolerance = None if raw.get("tolerance") is None else float(raw["tolerance"])
    else:
        pos = np.asarray(raw, dtype=np.float64)
        label, color, tolerance = None, _DEFAULT_TARGET_COLOR, None
    if pos.shape != (_TASK_DIM,):
        msg = f"[task].target must have {_TASK_DIM} elements (x, y), got shape {pos.shape}"
        raise ValueError(msg)
    return pos, label, color, tolerance


def multi_target_specs(task: Task) -> list[tuple[NDArray[np.float64], str | None, str, float | None]]:
    """Parse a multi-target task's ``targets`` list into ``(pos, label, color, tolerance)`` tuples.

    Raises
    ------
    ValueError
        If the task carries no ``targets`` list.
    """
    raw = task.params.get("targets")
    if not raw:
        msg = "[task] of type 'multi_target_reaching' requires a non-empty 'targets' list"
        raise ValueError(msg)
    return [_parse_target(entry) for entry in raw]


def active_target_index(task: Task) -> int:
    """The configured active-target index of a multi-target task (default 0)."""
    return int(task.params.get("active", 0))


def apply_active_target(task: Task) -> None:
    """Set a multi-target task's first-class ``target`` (and marker attrs) to its active candidate.

    Lets the active goal flow through the ordinary reaching pipeline (controller build,
    marker drawing); the GUI switches it live by reassigning ``task.target`` / the
    controller's target.

    Raises
    ------
    ValueError
        If the active index is out of range.
    """
    specs = multi_target_specs(task)
    index = active_target_index(task)
    if not 0 <= index < len(specs):
        msg = f"active target index {index} is out of range for {len(specs)} targets"
        raise ValueError(msg)
    task.target, task.label, task.color, task.tolerance = specs[index]


@dataclass
class Scenario:
    """A loaded control scenario: the robot, the task, and a ready-to-run controller.

    ``source_config`` keeps the original combined config (the parsed ``[skeleton]`` /
    ``[initial]`` / ``[task]`` / ``[controller]`` tables) when the scenario was
    loaded from a file, so a run can embed it for an exact, editable re-run; it is
    ``None`` for scenarios built programmatically.
    """

    skeleton: Skeleton
    task: Task
    controller: Controller
    source_config: Mapping[str, Any] | None = None


def _endpoint(skeleton: Skeleton) -> NDArray[np.float64]:
    """Current endpoint position of the arm."""
    tip = skeleton.links[-1]
    return np.array([tip.xe, tip.ye], dtype=np.float64)


# A reference builder turns a task into the joint reference a tracking controller tracks.
ReferenceBuilder = Callable[[Skeleton, "Task"], JointReference]

_REFERENCE_BUILDERS: dict[str, ReferenceBuilder] = {}
# Which recorded channel a task-space / joint-space tracking task reads from a .sklog.npz.
_TRACKING_CHANNELS = {"trajectory_tracking": "tip", "joint_trajectory_tracking": "q"}


def register_reference_builder(task_type: str, builder: ReferenceBuilder) -> None:
    """Register a joint-reference builder for a ``[task].type``.

    A tracking controller (computed torque, joint PD, inverse-dynamics PD, MPC) builds
    its reference by calling the builder registered for the scenario's task type. Use
    this to add a new trajectory-style task that drives the existing controllers.

    Parameters
    ----------
    task_type : str
        The ``[task].type`` value to bind.
    builder : ReferenceBuilder
        A callable ``(skeleton, task) -> JointReference``.
    """
    _REFERENCE_BUILDERS[task_type] = builder


def reference_builders() -> tuple[str, ...]:
    """Return the task types that provide a joint reference, sorted."""
    return tuple(sorted(_REFERENCE_BUILDERS))


def _joint_reference(skeleton: Skeleton, task: Task) -> JointReference:
    """Build the joint reference for ``task`` by dispatching on its type.

    Raises
    ------
    ValueError
        If the task type provides no joint reference (so it cannot drive a
        trajectory-tracking controller).
    """
    builder = _REFERENCE_BUILDERS.get(task.type)
    if builder is None:
        msg = (
            f"task type {task.type!r} provides no joint reference; it cannot drive a "
            f"trajectory-tracking controller (reference tasks: {list(reference_builders())})"
        )
        raise ValueError(msg)
    return builder(skeleton, task)


def _reaching_reference(skeleton: Skeleton, task: Task) -> SampledJointReference:
    """Joint reference for a reaching task: a planned point-to-point reach via IK."""
    trajectory = Trajectory(_endpoint(skeleton), task.require_target(), task.duration, task.schedule)
    return ik_joint_reference(skeleton, trajectory, dt=_REFERENCE_DT)


def _trajectory_tracking_reference(skeleton: Skeleton, task: Task) -> SampledJointReference:
    """Joint reference for a task-space trajectory: smooth/interpolate the tip path, then IK."""
    times, tip = _resolve_reference_series(task, "tip")
    grid, p, dp, ddp = _smooth_and_resample(times, tip, task, target_dt=_REFERENCE_DT)
    task_ref = SampledTaskReference(grid, p, dp, ddp)
    method = str(task.params.get("ik_method", "lm_sugihara"))
    return ik_joint_reference(skeleton, task_ref, dt=_REFERENCE_DT, method=method)


def _joint_trajectory_tracking_reference(skeleton: Skeleton, task: Task) -> SampledJointReference:
    """Joint reference for a per-joint trajectory: smooth/interpolate ``q(t)`` (no IK)."""
    times, q = _resolve_reference_series(task, "q")  # q is 2-D, shape (N, joints)
    if q.shape[1] != skeleton.num_joints:
        msg = f"joint reference has {q.shape[1]} joint columns but the robot has {skeleton.num_joints}"
        raise ValueError(msg)
    grid, qg, dqg, ddqg = _smooth_and_resample(times, q, task, target_dt=task.dt)
    return SampledJointReference(grid, qg, dqg, ddqg)


def _periodic_curve_reference(skeleton: Skeleton, task: Task) -> SampledJointReference:
    """Joint reference for a periodic curve: trace a closed task-space curve via IK."""
    kind = task.params.get("curve")
    if kind is None:
        msg = "[task] of type 'periodic_curve' requires a 'curve' kind"
        raise ValueError(msg)
    period = float(task.params.get("period", task.duration))
    curve = build_curve(str(kind), task.params)
    curve_ref = PeriodicTaskReference(curve, period=period, duration=task.duration)
    method = str(task.params.get("ik_method", "lm_sugihara"))
    return ik_joint_reference(skeleton, curve_ref, dt=_REFERENCE_DT, method=method)


def _resolve_reference_series(task: Task, channel: str) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Return ``(times, values)`` for a reference (values 2-D), from inlined samples or a file."""
    embedded = task.params.get("reference_samples")
    if embedded is not None:
        times = np.asarray(embedded["times"], dtype=np.float64)
        return times, np.atleast_2d(np.asarray(embedded["values"], dtype=np.float64))
    file = task.params.get("file")
    if file is None:
        msg = f"task type {task.type!r} requires a 'file' (.sklog.npz) or inlined 'reference_samples'"
        raise ValueError(msg)
    log = load_reference_log(file)
    if channel not in log.channel_names:
        msg = f"reference {file} has no {channel!r} channel (has {log.channel_names})"
        raise ValueError(msg)
    return log.times, np.atleast_2d(log.channel(channel))


def _smooth_and_resample(
    times: NDArray[np.float64],
    values: NDArray[np.float64],
    task: Task,
    *,
    target_dt: float,
) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
    """Smooth a sampled series and resample it (with derivatives) onto a uniform grid from ``t=0``."""
    rel = np.asarray(times, dtype=np.float64) - float(times[0])
    smoothed = _smooth_reference(values, rel, task)
    duration = float(rel[-1])
    num = max(round(duration / target_dt) + 1, 2)
    grid = np.linspace(0.0, duration, num)
    method = str(task.params.get("interpolator", "cubic_spline"))
    value, d1, d2 = resample_with_derivatives(rel, smoothed, grid, method=method)
    return grid, value, d1, d2


def _smooth_reference(values: NDArray[np.float64], rel_times: NDArray[np.float64], task: Task) -> NDArray[np.float64]:
    """Apply the task's optional ``filter`` to a uniformly sampled reference series."""
    cfg = task.params.get("filter")
    if not cfg:
        return np.asarray(values, dtype=np.float64)
    dt = float(np.mean(np.diff(rel_times))) if rel_times.shape[0] > 1 else 1.0
    return smooth(
        values,
        dt,
        kind=str(cfg.get("kind", "none")),
        cutoff_hz=cfg.get("cutoff_hz"),
        order=int(cfg.get("order", 2)),
    )


def load_reference_log(path: str | Path) -> StateLog:
    """Load a ``.sklog.npz`` reference trajectory (e.g. from ``tools/trajectory_recorder.py``)."""
    file = Path(path)
    if not file.exists():
        msg = f"reference file not found: {file}"
        raise FileNotFoundError(msg)
    return StateLog.load(file)


register_reference_builder(_REACHING_TYPE, _reaching_reference)
register_task_type("trajectory_tracking")
register_reference_builder("trajectory_tracking", _trajectory_tracking_reference)
register_task_type("joint_trajectory_tracking")
register_reference_builder("joint_trajectory_tracking", _joint_trajectory_tracking_reference)
register_task_type("periodic_curve")
register_reference_builder("periodic_curve", _periodic_curve_reference)
register_task_type(_MULTI_TARGET_TYPE)
register_reference_builder(_MULTI_TARGET_TYPE, _reaching_reference)  # tracks the active target


def _build_computed_torque(params: Mapping[str, Any], skeleton: Skeleton, task: Task) -> Controller:
    return ComputedTorque(_joint_reference(skeleton, task), kp=params.get("kp", 200.0), kd=params.get("kd", 30.0))


def _build_joint_pd(params: Mapping[str, Any], skeleton: Skeleton, task: Task) -> Controller:
    return JointPD(_joint_reference(skeleton, task), kp=params.get("kp", 300.0), kd=params.get("kd", 40.0))


def _build_inverse_dynamics_pd(params: Mapping[str, Any], skeleton: Skeleton, task: Task) -> Controller:
    reference = _joint_reference(skeleton, task)
    return InverseDynamicsFeedforwardPD(reference, kp=params.get("kp", 100.0), kd=params.get("kd", 20.0))


def _build_virtual_spring_damper(params: Mapping[str, Any], _skeleton: Skeleton, task: Task) -> Controller:
    return VirtualSpringDamper(
        task.require_target(),
        k_task=params.get("k_task", 150.0),
        d_task=params.get("d_task", 25.0),
        c_joint=params.get("c_joint", 0.0),
    )


def _build_time_varying_stiffness(params: Mapping[str, Any], _skeleton: Skeleton, task: Task) -> Controller:
    return TimeVaryingStiffness(
        task.require_target(),
        k0=params.get("k0", 150.0),
        alpha=params.get("alpha", 6.0),
        zeta1=params.get("zeta1", 0.15),
        c_joint=params.get("c_joint", 0.0),
    )


def _build_online_shaping(params: Mapping[str, Any], _skeleton: Skeleton, task: Task) -> Controller:
    return OnlineReferenceShaping(
        task.require_target(),
        k_task=params.get("k_task", 150.0),
        d_task=params.get("d_task", 25.0),
        c_joint=params.get("c_joint", 0.0),
        r=params.get("r", 0.5),
        t1=params.get("t1", 0.2),
        t2=params.get("t2", 0.2),
    )


def _build_position_dependent_shaping(params: Mapping[str, Any], _skeleton: Skeleton, task: Task) -> Controller:
    return PositionDependentShaping(
        task.require_target(),
        k_task=params.get("k_task", 150.0),
        d_task=params.get("d_task", 25.0),
        c_joint=params.get("c_joint", 0.0),
        a=params.get("a", 0.01),
        t1=params.get("t1", 0.2),
        t2=params.get("t2", 0.2),
    )


def _build_adaptive_shaping(params: Mapping[str, Any], _skeleton: Skeleton, task: Task) -> Controller:
    return AdaptiveReferenceShaping(
        task.require_target(),
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


# A controller builder turns the [controller] params (plus the robot and task) into a
# Controller. Register your own with register_controller to drive it from a config type.
ControllerBuilder = Callable[[Mapping[str, Any], Skeleton, Task], Controller]

_BUILDERS: dict[str, ControllerBuilder] = {
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


def register_controller(name: str, builder: ControllerBuilder) -> None:
    """Register a controller builder so ``[controller].type = name`` builds it.

    The builder receives the ``[controller]`` table (without ``type``) as a mapping,
    the posed ``Skeleton``, and the ``Task``, and returns a :class:`~skelarm.Controller`.
    Re-registering an existing name replaces it. Custom controllers are also usable
    directly (construct the instance and pass it to :func:`run_scenario` /
    :func:`~skelarm.simulate_controlled`); registration is only needed for the
    config-driven path.

    Parameters
    ----------
    name : str
        The ``[controller].type`` value to bind.
    builder : ControllerBuilder
        A callable ``(params, skeleton, task) -> Controller``.
    """
    _BUILDERS[name] = builder


def controller_types() -> tuple[str, ...]:
    """Return the registered ``[controller].type`` values, sorted."""
    return tuple(sorted(_BUILDERS))


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
        msg = f"unknown controller type {controller_type!r}; choose from {list(controller_types())}"
        raise ValueError(msg)
    params = {key: value for key, value in config.items() if key != "type"}
    return _BUILDERS[controller_type](params, skeleton, task)


def load_scenario(file_path: str | Path) -> Scenario:
    """Load a robot, task, and controller from one combined TOML file."""
    with Path(file_path).open("rb") as f:
        data = tomllib.load(f)
    return scenario_from_config(data)


def scenario_from_config(data: Mapping[str, Any]) -> Scenario:
    """Build a Scenario from a parsed combined config and keep the config for re-runs.

    Reuses the real parsers (:meth:`Skeleton.from_config`, :meth:`Task.from_dict`,
    :func:`build_controller`), so rebuilding from an identical mapping reproduces the
    robot, initial pose, task, and controller exactly.

    Raises
    ------
    ValueError
        If the config has no ``[task]`` or ``[controller]`` table.
    """
    if "task" not in data:
        msg = "no [task] section in scenario config"
        raise ValueError(msg)
    if "controller" not in data:
        msg = "no [controller] section in scenario config"
        raise ValueError(msg)
    resolved = _inline_reference_samples(data)
    skeleton = Skeleton.from_config(resolved)
    task = Task.from_dict(resolved["task"])
    if task.type == _MULTI_TARGET_TYPE:
        apply_active_target(task)  # set the first-class target to the active candidate
    controller = build_controller(resolved["controller"], skeleton=skeleton, task=task)
    return Scenario(skeleton=skeleton, task=task, controller=controller, source_config=dict(resolved))


def _inline_reference_samples(data: Mapping[str, Any]) -> dict[str, Any]:
    """Embed a file-based reference's samples into the config so re-runs are self-contained.

    For a trajectory-tracking task whose ``[task]`` names an external ``file`` but carries
    no inlined ``reference_samples`` yet, this loads the ``.sklog.npz`` once and copies the
    relevant channel + timestamps into the (copied) task table, also defaulting the run
    ``duration`` to the reference length. The embedded content then travels in
    ``source_config``, so ``rerun_log`` and exported configs need no external file.
    """
    result = dict(data)
    task_cfg = result.get("task")
    if not isinstance(task_cfg, Mapping):
        return result
    channel = _TRACKING_CHANNELS.get(str(task_cfg.get("type")))
    if channel is None or "reference_samples" in task_cfg or "file" not in task_cfg:
        return result

    log = load_reference_log(task_cfg["file"])
    if channel not in log.channel_names:
        msg = f"reference {task_cfg['file']} has no {channel!r} channel (has {log.channel_names})"
        raise ValueError(msg)
    times = log.times
    values = np.atleast_2d(log.channel(channel))

    task_copy = dict(task_cfg)
    task_copy["reference_samples"] = {
        "channel": channel,
        "source": str(task_cfg["file"]),
        "times": times.tolist(),
        "values": values.tolist(),
        "columns": list(log.channel_meta.get(channel, {}).get("columns", [])),
    }
    task_copy.setdefault("duration", float(times[-1] - times[0]))  # default sim length to the reference
    result["task"] = task_copy
    return result


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
    *,
    duration: float,
    dt: float,
    grav_vec: NDArray[np.float64],
    enforce_limits: bool,
) -> dict[str, Any] | None:
    """Assemble the ``[extra]`` payload that lets a run be reconstructed and re-run.

    Embeds the original source config (the editable ``[skeleton]`` / ``[initial]`` /
    ``[task]`` / ``[controller]`` tables, exactly as loaded), the actual run
    parameters (including the resolved ``enforce_limits``, so a call-time override is
    captured), and the package versions. Returns ``None`` when the scenario carries
    no ``source_config`` (e.g. a programmatically built controller).
    """
    if scenario.source_config is None:
        return None
    return {
        "source_config": dict(scenario.source_config),
        "run": {
            "duration": float(duration),
            "dt": float(dt),
            "grav_vec": grav_vec.tolist(),
            "enforce_limits": bool(enforce_limits),
        },
        "provenance": _provenance(),
    }


def run_scenario(
    scenario: Scenario,
    *,
    duration: float | None = None,
    dt: float | None = None,
    grav_vec: NDArray[np.float64] | None = None,
    enforce_limits: bool | None = None,
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
    enforce_limits : bool | None, optional
        Apply the joint limits as hard stops in the dynamics. ``None`` (the default)
        uses the scenario's ``task.enforce_limits``; an explicit value overrides it
        (e.g. a ``--no-joint-limits`` CLI flag). The resolved value is recorded in
        the run metadata so the override is reproduced on re-run.

    Returns
    -------
    StateLog
        The recorded run, with reproduction metadata when available.
    """
    run_duration = scenario.task.duration if duration is None else float(duration)
    run_dt = scenario.task.dt if dt is None else float(dt)
    grav = np.zeros(_TASK_DIM, dtype=np.float64) if grav_vec is None else np.asarray(grav_vec, dtype=np.float64)
    run_enforce_limits = scenario.task.enforce_limits if enforce_limits is None else bool(enforce_limits)
    extra = _reproduction_metadata(
        scenario, duration=run_duration, dt=run_dt, grav_vec=grav, enforce_limits=run_enforce_limits
    )
    return simulate_controlled(
        scenario.skeleton,
        scenario.controller,
        duration=run_duration,
        dt=run_dt,
        grav_vec=grav,
        enforce_limits=run_enforce_limits,
        extra=extra,
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
    config = log.extra.get("source_config")
    if not config:
        msg = "log has no reproduction metadata; it was not produced by run_scenario"
        raise ValueError(msg)
    scenario = scenario_from_config(config)
    run = log.extra.get(
        "run",
        {
            "duration": scenario.task.duration,
            "dt": scenario.task.dt,
            "grav_vec": [0.0, 0.0],
            "enforce_limits": scenario.task.enforce_limits,
        },
    )
    return scenario, run


def rerun_log(log: StateLog) -> StateLog:
    """Reconstruct and re-simulate a scenario recorded by :func:`run_scenario`.

    Rebuilds the scenario by reparsing the embedded source config (identical input
    gives identical state) and re-runs with the recorded run parameters, so
    deterministic controllers reproduce the original channels exactly (MPC, which
    calls :func:`scipy.optimize.minimize`, reproduces within a small numerical
    tolerance on the same platform).

    Raises
    ------
    ValueError
        If the log carries no reproduction metadata.
    """
    scenario, run = scenario_from_log(log)
    grav_vec = np.asarray(run["grav_vec"], dtype=np.float64)
    # Older logs may predate the recorded enforce_limits; fall back to the task's value.
    enforce_limits = run.get("enforce_limits", scenario.task.enforce_limits)
    return run_scenario(
        scenario, duration=run["duration"], dt=run["dt"], grav_vec=grav_vec, enforce_limits=enforce_limits
    )


def export_scenario_toml(log: StateLog, path: str | Path) -> None:
    """Write the log's embedded scenario config to an editable TOML file.

    The output is a standard combined config (``[skeleton]`` / ``[initial]`` /
    ``[task]`` / ``[controller]``) that :func:`load_scenario` reads back. Re-running
    the unedited file reproduces the original run exactly for the deterministic
    controllers, and individual values can be edited for comparison studies.

    Parameters
    ----------
    log : StateLog
        A log produced by :func:`run_scenario` (carrying the source config).
    path : str | Path
        Destination ``.toml`` path.

    Raises
    ------
    ValueError
        If the log carries no embedded scenario config.
    """
    config = log.extra.get("source_config")
    if not config:
        msg = "log has no embedded scenario config to export; it was not produced by run_scenario"
        raise ValueError(msg)
    Path(path).write_text(dump_toml(config).strip() + "\n", encoding="utf-8")
