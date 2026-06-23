"""Trajectory-tracking control: references, torque controllers, and a fixed-step loop.

This module turns a planned reference into joint torques. A task-space trajectory
is first converted to a joint-space reference (samplewise inverse kinematics or
resolved motion rate), then a torque controller (joint PD, inverse-dynamics
feedforward plus PD, or computed torque) tracks it. :func:`simulate_controlled`
runs a controller against the dynamics with a fixed time step and returns a
:class:`~skelarm.StateLog` for replay and analysis.

See ``docs/reference/07_control.md`` for the theory.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, Protocol

import numpy as np

from skelarm.dynamics import (
    compute_coriolis_gravity_vector,
    compute_inverse_dynamics,
    compute_mass_matrix,
    integrate_with_limits,
)
from skelarm.kinematics import compute_inverse_kinematics, compute_jacobian
from skelarm.recording import StateLog

if TYPE_CHECKING:
    from collections.abc import Mapping
    from typing import Any

    from numpy.typing import ArrayLike, NDArray

    from skelarm.skeleton import Skeleton
    from skelarm.trajectory import Trajectory


class JointReference(Protocol):
    """A joint-space reference that can be sampled for ``(q_r, dq_r, ddq_r)``."""

    def sample(self, t: float) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
        """Return the reference angles, velocities, and accelerations at time ``t``."""
        ...


class SampledJointReference:
    """A joint reference stored as time-sampled arrays, read by linear interpolation.

    Parameters
    ----------
    times : ArrayLike
        Sample times, shape ``(N,)``, increasing.
    q, dq, ddq : ArrayLike
        Joint angles, velocities, and accelerations, each shape ``(N, num_joints)``.
    """

    def __init__(self, times: ArrayLike, q: ArrayLike, dq: ArrayLike, ddq: ArrayLike) -> None:
        """Store the sampled reference arrays."""
        self.times = np.asarray(times, dtype=np.float64)
        self.q = np.asarray(q, dtype=np.float64)
        self.dq = np.asarray(dq, dtype=np.float64)
        self.ddq = np.asarray(ddq, dtype=np.float64)

    def sample(self, t: float) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
        """Linearly interpolate the reference at time ``t`` (held at the endpoints)."""
        return self._interp(t, self.q), self._interp(t, self.dq), self._interp(t, self.ddq)

    def _interp(self, t: float, data: NDArray[np.float64]) -> NDArray[np.float64]:
        return np.array([np.interp(t, self.times, data[:, j]) for j in range(data.shape[1])], dtype=np.float64)


def ik_joint_reference(
    skeleton: Skeleton,
    task_trajectory: Trajectory,
    *,
    dt: float,
    method: str = "lm_sugihara",
) -> SampledJointReference:
    """Convert a task-space trajectory to a joint reference with samplewise IK.

    Each sample is solved with :func:`compute_inverse_kinematics`, warm-started
    from the previous solution for joint continuity. Velocities and accelerations
    are estimated by finite differences. The input ``skeleton`` is not mutated.

    Parameters
    ----------
    skeleton : Skeleton
        The arm to solve for; its current pose seeds the first IK solve.
    task_trajectory : Trajectory
        The desired endpoint trajectory ``p_r(t)``.
    dt : float
        Sampling interval (seconds).
    method : str, optional
        IK method passed to :func:`compute_inverse_kinematics`.

    Returns
    -------
    SampledJointReference
        The joint reference ``(q_r, dq_r, ddq_r)``.
    """
    model = skeleton.clone()
    duration = task_trajectory.duration
    times = np.linspace(0.0, duration, round(duration / dt) + 1)
    seed = model.q.copy()
    q_samples = []
    for t in times:
        target = task_trajectory.sample(float(t))[0]
        compute_inverse_kinematics(model, target, method=method, q0=seed)
        seed = model.q.copy()  # solution is written back to the skeleton; reuse as the next seed
        q_samples.append(seed.copy())
    q = np.array(q_samples, dtype=np.float64)
    dq = np.gradient(q, times, axis=0)
    ddq = np.gradient(dq, times, axis=0)
    return SampledJointReference(times, q, dq, ddq)


def resolved_rate_joint_reference(
    skeleton: Skeleton,
    task_trajectory: Trajectory,
    *,
    dt: float,
    damping: float = 1e-3,
    k_task: float = 0.0,
) -> SampledJointReference:
    """Convert a task-space trajectory to a joint reference by resolved motion rate.

    Integrates ``q̇_r = Jᵀ(JJᵀ+µI)⁻¹(ṗ_r + K_task(p_r - p))`` forward in time. The
    task-feedback term ``K_task`` corrects drift from the desired path. The input
    ``skeleton`` is not mutated.

    Parameters
    ----------
    skeleton : Skeleton
        The arm to convert for; its current pose is the integration start.
    task_trajectory : Trajectory
        The desired endpoint trajectory.
    dt : float
        Integration step (seconds).
    damping : float, optional
        Damping ``µ`` for the damped pseudoinverse.
    k_task : float, optional
        Task-space position-feedback gain.

    Returns
    -------
    SampledJointReference
        The joint reference ``(q_r, dq_r, ddq_r)``.
    """
    model = skeleton.clone()
    duration = task_trajectory.duration
    times = np.linspace(0.0, duration, round(duration / dt) + 1)
    q = model.q.copy()
    q_samples = []
    dq_samples = []
    for t in times:
        model.q = q  # eager setter refreshes forward kinematics
        tip = model.links[-1]
        position = np.array([tip.xe, tip.ye], dtype=np.float64)
        p_r, dp_r, _ = task_trajectory.sample(float(t))
        jacobian = compute_jacobian(model)
        rhs = dp_r + k_task * (p_r - position)
        qdot = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + damping * np.eye(2), rhs)
        q_samples.append(q.copy())
        dq_samples.append(qdot.copy())
        q = q + qdot * dt
    q_arr = np.array(q_samples, dtype=np.float64)
    dq_arr = np.array(dq_samples, dtype=np.float64)
    ddq_arr = np.gradient(dq_arr, times, axis=0)
    return SampledJointReference(times, q_arr, dq_arr, ddq_arr)


class Controller(ABC):
    """Base class for joint-torque controllers.

    A controller is callable as ``f(t, skeleton) -> tau`` so stateless controllers
    work directly as a :func:`~skelarm.simulate_robot` torque callback. Stateful
    controllers use :meth:`reset` and :meth:`update` together with
    :func:`simulate_controlled`, and may expose internal signals via
    :meth:`log_channels`.
    """

    def reset(self, skeleton: Skeleton) -> None:  # noqa: B027
        """Initialize controller state at movement onset (default: no-op)."""

    @abstractmethod
    def control(self, t: float, skeleton: Skeleton) -> NDArray[np.float64]:
        """Return the joint torque for the current time and skeleton state."""

    def update(self, t: float, skeleton: Skeleton, dt: float) -> None:  # noqa: B027
        """Advance internal state once per fixed control step (default: no-op)."""

    def log_channels(self) -> dict[str, ArrayLike]:
        """Return controller-internal signals to record each step (default: none)."""
        return {}

    def __call__(self, t: float, skeleton: Skeleton) -> NDArray[np.float64]:
        """Evaluate the control law (so the controller is a torque callback)."""
        return self.control(t, skeleton)


class TrackingController(Controller):
    """A controller that tracks a joint reference, logging ``q_ref`` and ``error``."""

    def __init__(self, reference: JointReference, kp: ArrayLike, kd: ArrayLike) -> None:
        """Store the reference and (scalar or per-joint) PD gains."""
        self.reference = reference
        self.kp = np.asarray(kp, dtype=np.float64)
        self.kd = np.asarray(kd, dtype=np.float64)
        self._q_ref: NDArray[np.float64] | None = None
        self._error: NDArray[np.float64] | None = None

    def _store(self, q_ref: NDArray[np.float64], error: NDArray[np.float64]) -> None:
        self._q_ref = q_ref
        self._error = error

    def log_channels(self) -> dict[str, ArrayLike]:
        """Record the joint reference and tracking error once tracking has begun."""
        if self._q_ref is None or self._error is None:
            return {}
        return {"q_ref": self._q_ref, "error": self._error}


class JointPD(TrackingController):
    """Direct joint-space PD control: ``τ = Kp(q_r-q) + Kd(q̇_r-q̇)``."""

    def control(self, t: float, skeleton: Skeleton) -> NDArray[np.float64]:
        """Compute the PD torque for the current state."""
        q_r, dq_r, _ = self.reference.sample(t)
        error = q_r - skeleton.q
        self._store(q_r, error)
        return self.kp * error + self.kd * (dq_r - skeleton.dq)


class InverseDynamicsFeedforwardPD(TrackingController):
    """Inverse-dynamics feedforward plus PD: ``τ = ID(q_r,q̇_r,q̈_r) + Kp e + Kd ė``."""

    def __init__(self, reference: JointReference, kp: ArrayLike, kd: ArrayLike) -> None:
        """Store the reference and gains; the feedforward model is built lazily."""
        super().__init__(reference, kp, kd)
        self._model: Skeleton | None = None

    def control(self, t: float, skeleton: Skeleton) -> NDArray[np.float64]:
        """Compute the feedforward-plus-PD torque for the current state."""
        q_r, dq_r, ddq_r = self.reference.sample(t)
        if self._model is None:
            self._model = skeleton.clone()  # separate model so the reference state does not disturb the plant
        self._model.set_state(q=q_r, dq=dq_r, ddq=ddq_r)
        compute_inverse_dynamics(self._model)
        error = q_r - skeleton.q
        self._store(q_r, error)
        return self._model.tau + self.kp * error + self.kd * (dq_r - skeleton.dq)


class ComputedTorque(TrackingController):
    """Computed torque control: ``τ = H(q)(q̈_r + Kd ė + Kp e) + b(q,q̇)``."""

    def control(self, t: float, skeleton: Skeleton) -> NDArray[np.float64]:
        """Compute the model-cancelling torque for the current state."""
        q_r, dq_r, ddq_r = self.reference.sample(t)
        error = q_r - skeleton.q
        commanded = ddq_r + self.kd * (dq_r - skeleton.dq) + self.kp * error
        self._store(q_r, error)
        return compute_mass_matrix(skeleton) @ commanded + compute_coriolis_gravity_vector(skeleton)


def simulate_controlled(
    skeleton: Skeleton,
    controller: Controller,
    *,
    duration: float,
    dt: float = 1e-3,
    grav_vec: NDArray[np.float64] | None = None,
    enforce_limits: bool = True,
    extra: Mapping[str, Any] | None = None,
) -> StateLog:
    """Run a controller against the dynamics with a fixed time step.

    Uses semi-implicit (symplectic) Euler, the same integration as the GUI
    simulator. A clone of ``skeleton`` is simulated, so the caller's skeleton is
    not mutated. Each frame records ``q``, ``dq``, the applied ``tau``, and any
    channels from ``controller.log_channels()``.

    Parameters
    ----------
    skeleton : Skeleton
        Initial state (``q`` / ``dq``); not mutated.
    controller : Controller
        The torque controller to run.
    duration : float
        Total simulated time (seconds).
    dt : float, optional
        Fixed control/integration step (seconds).
    grav_vec : NDArray[np.float64] | None, optional
        Gravity vector; defaults to zero (planar motion).
    enforce_limits : bool, optional
        Apply the joint limits as hard stops in the dynamics (default). When
        ``False``, the limits no longer constrain the simulation, so they apply
        only to the kinematics (posing and inverse kinematics).
    extra : Mapping[str, Any] | None, optional
        Free-form metadata to embed in the returned log's ``[extra]`` table (e.g.
        the scenario config that produced the run, for later reproduction).

    Returns
    -------
    StateLog
        The recorded run (``duration / dt`` rounded, plus the initial frame).
    """
    model = skeleton.clone()
    lower = upper = None
    if enforce_limits:
        lower = np.array([link.prop.qmin for link in model.links[1:]], dtype=np.float64)
        upper = np.array([link.prop.qmax for link in model.links[1:]], dtype=np.float64)
    joints = [f"j{i + 1}" for i in range(model.num_joints)]
    channel_meta = {
        "q": {"unit": "rad", "label": "joint angle", "columns": joints},
        "dq": {"unit": "rad/s", "label": "joint velocity", "columns": joints},
        "tau": {"unit": "N*m", "label": "applied joint torque", "columns": joints},
    }
    log = StateLog(model, producer=type(controller).__name__, channel_meta=channel_meta, extra=extra)
    controller.reset(model)

    t = 0.0
    for _ in range(round(duration / dt)):
        controller.update(t, model, dt)
        tau = controller.control(t, model)
        log.record(t, q=model.q, dq=model.dq, tau=tau, **controller.log_channels())

        integrate_with_limits(model, tau, dt, lower, upper, grav_vec)
        t += dt

    # Final frame: record the settled state with the torque that would be applied next.
    tau = controller.control(t, model)
    log.record(t, q=model.q, dq=model.dq, tau=tau, **controller.log_channels())
    return log
