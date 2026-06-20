"""Joint-space model predictive control.

A finite-horizon optimal-control tracker: at each control step it rolls out the
forward dynamics over a horizon, optimizes the torque sequence to track a joint
reference (with torque bounds and soft joint-limit penalties), applies only the
first torque, and warm-starts the next solve from the shifted solution.

This is the stateful, model-based tracker from ``docs/reference/07_control.md``; it
must be run with the fixed-step :func:`skelarm.control.simulate_controlled` loop.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
from scipy.optimize import minimize

from skelarm.control import Controller
from skelarm.dynamics import compute_forward_dynamics
from skelarm.kinematics import compute_forward_kinematics

if TYPE_CHECKING:
    from numpy.typing import ArrayLike, NDArray

    from skelarm.control import JointReference
    from skelarm.skeleton import Skeleton


class JointSpaceMPC(Controller):
    """Receding-horizon joint-space tracker using :func:`scipy.optimize.minimize`.

    The prediction model is semi-implicit Euler over :func:`compute_forward_dynamics`,
    matching the integration in :func:`skelarm.control.simulate_controlled`; run the
    controller with the same ``dt``.

    Parameters
    ----------
    reference : JointReference
        The joint reference providing ``(q_r, dq_r, ddq_r)`` samples.
    horizon : int
        Number of prediction/control steps ``N``.
    dt : float
        Control interval and rollout step (seconds); match ``simulate_controlled``.
    q_weight, dq_weight : float, optional
        Stage weights on the joint-position and joint-velocity tracking error.
    tau_weight : float, optional
        Stage weight (effort penalty) on the torque.
    terminal_weight : float, optional
        Weight on the terminal joint-position error.
    tau_max : float | None, optional
        Symmetric torque bound; ``None`` leaves the torque unbounded.
    limit_weight : float, optional
        Soft joint-limit penalty weight (0 disables it).
    max_iter : int, optional
        Maximum optimizer iterations per control step.
    """

    def __init__(
        self,
        reference: JointReference,
        *,
        horizon: int,
        dt: float,
        q_weight: float = 10.0,
        dq_weight: float = 1.0,
        tau_weight: float = 1e-3,
        terminal_weight: float = 50.0,
        tau_max: float | None = None,
        limit_weight: float = 0.0,
        max_iter: int = 20,
    ) -> None:
        """Store the reference, horizon, weights, and optimizer settings."""
        self.reference = reference
        self.horizon = int(horizon)
        self.dt = float(dt)
        self.q_weight = float(q_weight)
        self.dq_weight = float(dq_weight)
        self.tau_weight = float(tau_weight)
        self.terminal_weight = float(terminal_weight)
        self.tau_max = None if tau_max is None else float(tau_max)
        self.limit_weight = float(limit_weight)
        self.max_iter = int(max_iter)
        self._model: Skeleton | None = None
        self._lower: NDArray[np.float64] | None = None
        self._upper: NDArray[np.float64] | None = None
        self._warm: NDArray[np.float64] | None = None
        self._q_ref: NDArray[np.float64] | None = None
        self._error: NDArray[np.float64] | None = None

    def reset(self, skeleton: Skeleton) -> None:
        """Build the prediction model and clear the warm-start torque sequence."""
        self._model = skeleton.clone()
        self._lower = np.array([link.prop.qmin for link in skeleton.links[1:]], dtype=np.float64)
        self._upper = np.array([link.prop.qmax for link in skeleton.links[1:]], dtype=np.float64)
        self._warm = np.zeros((self.horizon, skeleton.num_joints), dtype=np.float64)

    def control(self, t: float, skeleton: Skeleton) -> NDArray[np.float64]:
        """Optimize the horizon torque sequence and return only the first torque."""
        if self._model is None or self._warm is None:
            self.reset(skeleton)
        assert self._warm is not None  # set by reset

        num_joints = skeleton.num_joints
        q0 = skeleton.q
        dq0 = skeleton.dq
        q_ref = np.array([self.reference.sample(t + k * self.dt)[0] for k in range(self.horizon + 1)])
        dq_ref = np.array([self.reference.sample(t + k * self.dt)[1] for k in range(self.horizon + 1)])

        bounds = None
        if self.tau_max is not None:
            bounds = [(-self.tau_max, self.tau_max)] * (self.horizon * num_joints)
        result = minimize(
            self._rollout_cost,
            self._warm.flatten(),
            args=(q0, dq0, q_ref, dq_ref),
            method="L-BFGS-B",
            bounds=bounds,
            options={"maxiter": self.max_iter},
        )
        torques = result.x.reshape(self.horizon, num_joints)
        # Warm-start the next solve from the shifted sequence (repeat the last torque).
        self._warm = np.vstack([torques[1:], torques[-1:]])
        self._q_ref = q_ref[0]
        self._error = q_ref[0] - q0
        return torques[0].copy()

    def _rollout_cost(
        self,
        tau_flat: NDArray[np.float64],
        q0: NDArray[np.float64],
        dq0: NDArray[np.float64],
        q_ref: NDArray[np.float64],
        dq_ref: NDArray[np.float64],
    ) -> float:
        """Roll out the prediction model and accumulate the tracking-plus-effort cost."""
        num_joints = q0.shape[0]
        torques = tau_flat.reshape(self.horizon, num_joints)
        q = q0.copy()
        dq = dq0.copy()
        cost = 0.0
        for k in range(self.horizon):
            error_q = q - q_ref[k]
            error_dq = dq - dq_ref[k]
            cost += self.q_weight * error_q @ error_q
            cost += self.dq_weight * error_dq @ error_dq
            cost += self.tau_weight * torques[k] @ torques[k]
            if self.limit_weight and self._lower is not None and self._upper is not None:
                overshoot = np.maximum(q - self._upper, 0.0) + np.maximum(self._lower - q, 0.0)
                cost += self.limit_weight * overshoot @ overshoot
            ddq = self._predict(q, dq, torques[k])
            dq = dq + self.dt * ddq
            q = q + self.dt * dq
        terminal = q - q_ref[self.horizon]
        cost += self.terminal_weight * terminal @ terminal
        return float(cost)

    def _predict(
        self, q: NDArray[np.float64], dq: NDArray[np.float64], tau: NDArray[np.float64]
    ) -> NDArray[np.float64]:
        """Forward-dynamics prediction at unclamped state ``(q, dq)`` under torque ``tau``."""
        assert self._model is not None  # set by reset
        for link, q_value, dq_value in zip(self._model.links[1:], q, dq, strict=True):
            link.q = float(q_value)
            link.dq = float(dq_value)
        compute_forward_kinematics(self._model)
        return compute_forward_dynamics(self._model, tau)

    def log_channels(self) -> dict[str, ArrayLike]:
        """Record the current reference and tracking error once control has run."""
        if self._q_ref is None or self._error is None:
            return {}
        return {"q_ref": self._q_ref, "error": self._error}
