"""Endpoint-feedback reaching controllers.

These controllers move the endpoint to a task-space target ``p*`` using a virtual
spring-damper in task space, rather than tracking a preplanned time trajectory.
They range from a constant target spring through time-varying stiffness to online
reference shaping with a shaped equilibrium ``p_s``. All are stateful endpoint
controllers run via :func:`skelarm.control.simulate_controlled`.

Isotropic (scalar) task-space stiffness and damping are used, matching the planar
reaching formulation. See ``docs/reference/08_reaching_control.md`` for the theory.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import numpy as np

from skelarm.control import Controller
from skelarm.kinematics import compute_endpoint_velocity, compute_jacobian

if TYPE_CHECKING:
    from numpy.typing import ArrayLike, NDArray

    from skelarm.skeleton import Skeleton

_MIN_SPAN = 1e-9  # guard against a near-zero start-to-target distance in r(d)


def _endpoint_position(skeleton: Skeleton) -> NDArray[np.float64]:
    """Return the current endpoint position ``(x, y)``."""
    tip = skeleton.links[-1]
    return np.array([tip.xe, tip.ye], dtype=np.float64)


def shaping_ratio(d: float, a: float) -> float:
    """Position-dependent shaping ratio ``r(d)`` (Humanoids 2009), clamped to ``(0, 1]``.

    Parameters
    ----------
    d : float
        Normalized remaining distance ``||p*-p|| / ||p*-p0||``.
    a : float
        Small positive shaping constant (``0 < a << 1``).

    Returns
    -------
    float
        ``r = 0.5(sqrt((d-1)^2 + 4a) - (d-1))`` clamped to ``[0, 1]``. It is about
        ``sqrt(a)`` at the start (``d=1``) and approaches one near the target.
    """
    r = 0.5 * (math.sqrt((d - 1.0) ** 2 + 4.0 * a) - (d - 1.0))
    return float(np.clip(r, 0.0, 1.0))


class EndpointController(Controller):
    """Base class for task-space spring-damper controllers.

    Implements the shared law ``tau = J^T (k (p_eq - p) - D pdot) - Cq qdot`` given
    a spring equilibrium ``p_eq`` and spring stiffness ``k`` chosen by subclasses.

    Parameters
    ----------
    target : ArrayLike
        The task-space target ``p*`` (x, y).
    k_task : float
        Task-space spring stiffness.
    d_task : float
        Task-space damping gain.
    c_joint : float, optional
        Joint viscous damping gain ``Cq``.
    """

    def __init__(self, target: ArrayLike, *, k_task: float, d_task: float, c_joint: float = 0.0) -> None:
        """Store the target and the stiffness / damping gains."""
        self.target = np.asarray(target, dtype=np.float64)
        self.k_task = float(k_task)
        self.d_task = float(d_task)
        self.c_joint = float(c_joint)
        self._endpoint: NDArray[np.float64] | None = None
        self._equilibrium: NDArray[np.float64] | None = None

    def _endpoint_torque(self, skeleton: Skeleton, equilibrium: NDArray[np.float64], k: float) -> NDArray[np.float64]:
        """Compute the spring-damper joint torque toward ``equilibrium`` with stiffness ``k``."""
        position = _endpoint_position(skeleton)
        velocity = compute_endpoint_velocity(skeleton)
        force = k * (equilibrium - position) - self.d_task * velocity
        torque = compute_jacobian(skeleton).T @ force - self.c_joint * skeleton.dq
        self._endpoint = position
        self._equilibrium = equilibrium
        return torque

    def log_channels(self) -> dict[str, ArrayLike]:
        """Record the endpoint and the spring equilibrium once control has run."""
        if self._endpoint is None or self._equilibrium is None:
            return {}
        return {"endpoint": self._endpoint, "equilibrium": self._equilibrium}


class VirtualSpringDamper(EndpointController):
    """Constant virtual spring-damper toward a fixed target (Arimoto-Sekimoto 2006)."""

    def control(self, t: float, skeleton: Skeleton) -> NDArray[np.float64]:  # noqa: ARG002
        """Return the spring-damper torque toward the fixed target."""
        return self._endpoint_torque(skeleton, self.target, self.k_task)


class TimeVaryingStiffness(EndpointController):
    """Virtual spring-damper with a gamma-shaped time-varying stiffness ``k(t)``.

    The stiffness starts at zero and grows to ``k0`` (Sekimoto-Arimoto 2006), giving
    a small initial torque. The damping gain is fixed at ``zeta1 * k0``.

    Parameters
    ----------
    target : ArrayLike
        The task-space target.
    k0 : float
        Saturated stiffness.
    alpha : float
        Rate of the gamma-shaped stiffness rise.
    zeta1 : float
        Damping ratio so the damping gain is ``zeta1 * k0``.
    c_joint : float, optional
        Joint viscous damping gain.
    """

    def __init__(self, target: ArrayLike, *, k0: float, alpha: float, zeta1: float, c_joint: float = 0.0) -> None:
        """Store the saturated stiffness, rate, and damping ratio."""
        super().__init__(target, k_task=k0, d_task=zeta1 * k0, c_joint=c_joint)
        self.k0 = float(k0)
        self.alpha = float(alpha)
        self._k = 0.0

    def stiffness(self, t: float) -> float:
        """Gamma-distribution-shaped stiffness ``k(t)`` (zero at ``t<=0``, ``->k0``)."""
        if t <= 0.0:
            return 0.0
        a = self.alpha
        return self.k0 * (1.0 - (1.0 + a * t + 0.5 * a * a * t * t) * math.exp(-a * t))

    def control(self, t: float, skeleton: Skeleton) -> NDArray[np.float64]:
        """Return the spring-damper torque with the current time-varying stiffness."""
        self._k = self.stiffness(t)
        return self._endpoint_torque(skeleton, self.target, self._k)

    def log_channels(self) -> dict[str, ArrayLike]:
        """Record the endpoint, equilibrium, and the current stiffness."""
        channels = super().log_channels()
        if channels:
            channels["stiffness"] = self._k
        return channels


class OnlineReferenceShaping(EndpointController):
    """Online reference shaping with a fixed ratio ``r`` (Seto-Sugihara IROS 2009).

    The spring equilibrium is a shaped reference ``p_s`` produced by passing
    ``r p* + (1-r) p`` through a second-order lag filter (two cascaded first-order
    lags with time constants ``t1`` and ``t2``). Feeding back the current endpoint
    keeps ``p_s`` near the arm when the endpoint is constrained.

    Parameters
    ----------
    target : ArrayLike
        The task-space target.
    k_task, d_task : float
        Task-space stiffness and damping.
    c_joint : float, optional
        Joint viscous damping gain.
    r : float, optional
        Fixed shaping ratio in ``(0, 1]``.
    t1, t2 : float, optional
        Lag-filter time constants (seconds).
    """

    def __init__(
        self,
        target: ArrayLike,
        *,
        k_task: float,
        d_task: float,
        c_joint: float = 0.0,
        r: float = 0.3,
        t1: float = 0.1,
        t2: float = 0.1,
    ) -> None:
        """Store the shaping ratio and lag-filter time constants."""
        super().__init__(target, k_task=k_task, d_task=d_task, c_joint=c_joint)
        self.r = float(r)
        self.t1 = float(t1)
        self.t2 = float(t2)
        self._filter_state: NDArray[np.float64] | None = None  # intermediate lag output
        self._shaped: NDArray[np.float64] | None = None  # shaped equilibrium p_s

    def reset(self, skeleton: Skeleton) -> None:
        """Initialize the shaped reference to the current endpoint at movement onset."""
        position = _endpoint_position(skeleton)
        self._filter_state = position.copy()
        self._shaped = position.copy()

    def update(self, t: float, skeleton: Skeleton, dt: float) -> None:  # noqa: ARG002
        """Advance the second-order lag filter that produces the shaped equilibrium."""
        if self._filter_state is None or self._shaped is None:
            self.reset(skeleton)
            return
        position = _endpoint_position(skeleton)
        target_input = self.r * self.target + (1.0 - self.r) * position
        self._filter_state = self._filter_state + dt * (target_input - self._filter_state) / self.t1
        self._shaped = self._shaped + dt * (self._filter_state - self._shaped) / self.t2

    def control(self, t: float, skeleton: Skeleton) -> NDArray[np.float64]:  # noqa: ARG002
        """Return the spring-damper torque toward the shaped equilibrium ``p_s``."""
        if self._shaped is None:
            self.reset(skeleton)
        assert self._shaped is not None  # set by reset
        return self._endpoint_torque(skeleton, self._shaped, self.k_task)


class PositionDependentShaping(OnlineReferenceShaping):
    """Online reference shaping with a position-dependent ratio ``r(d)`` (Humanoids 2009).

    Extends :class:`OnlineReferenceShaping` by recomputing the shaping ratio each
    step from the normalized remaining distance, so the reach starts gently and
    converges faster near the target.

    Parameters
    ----------
    target : ArrayLike
        The task-space target.
    k_task, d_task : float
        Task-space stiffness and damping.
    c_joint : float, optional
        Joint viscous damping gain.
    a : float, optional
        Small positive shaping constant for :func:`shaping_ratio`.
    t1, t2 : float, optional
        Lag-filter time constants (seconds).
    """

    def __init__(
        self,
        target: ArrayLike,
        *,
        k_task: float,
        d_task: float,
        c_joint: float = 0.0,
        a: float = 0.01,
        t1: float = 0.1,
        t2: float = 0.1,
    ) -> None:
        """Store the shaping constant; the ratio ``r`` is recomputed each step."""
        super().__init__(target, k_task=k_task, d_task=d_task, c_joint=c_joint, r=0.0, t1=t1, t2=t2)
        self.a = float(a)
        self._initial: NDArray[np.float64] | None = None  # endpoint at movement onset

    def reset(self, skeleton: Skeleton) -> None:
        """Record the onset endpoint used to normalize the remaining distance."""
        super().reset(skeleton)
        self._initial = _endpoint_position(skeleton)

    def update(self, t: float, skeleton: Skeleton, dt: float) -> None:
        """Recompute ``r(d)`` from the remaining distance, then advance the lag filter."""
        if self._initial is None:
            self.reset(skeleton)
        assert self._initial is not None  # set by reset
        position = _endpoint_position(skeleton)
        span = float(np.linalg.norm(self.target - self._initial))
        d = float(np.linalg.norm(self.target - position)) / span if span > _MIN_SPAN else 0.0
        self.r = shaping_ratio(d, self.a)
        super().update(t, skeleton, dt)
