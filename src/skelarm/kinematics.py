"""Provides functions for robot arm kinematics."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from skelarm.skeleton import Skeleton


def compute_forward_kinematics(skeleton: Skeleton) -> None:
    """Compute the forward kinematics for the given skeleton.

    Updates the (x, y) positions of each link's end-effector (tip) and joints,
    as well as angular, joint-origin, tip, and COM velocities and accelerations.

    The arm is rooted at the origin (0, 0). The fixed base link (``links[0]``)
    carries no joint motion, so it simply offsets the first joint to
    ``(base_length, 0)`` with zero velocity and acceleration.

    Parameters
    ----------
    skeleton : Skeleton
        The Skeleton object containing the robot arm's links and joint angles.
    """
    current_x = 0.0
    current_y = 0.0
    current_angle = 0.0  # Absolute angle of the current link

    prev_w = 0.0
    prev_dw = 0.0
    prev_v = np.array([0.0, 0.0], dtype=np.float64)
    prev_dv = np.array([0.0, 0.0], dtype=np.float64)

    for link in skeleton.links:
        # Store the start of the link (joint position)
        link.x = current_x
        link.y = current_y

        # Add the current joint angle to the absolute angle
        current_angle += link.q
        link.q_absolute = current_angle

        cos_angle = np.cos(current_angle)
        sin_angle = np.sin(current_angle)

        # Compute angular velocity and acceleration (scalar for 2D)
        link.w = prev_w + link.dq
        link.dw = prev_dw + link.ddq

        link.v = prev_v
        link.dv = prev_dv

        link_vec = np.array([link.prop.length * cos_angle, link.prop.length * sin_angle], dtype=np.float64)
        cross_dw_link = np.array([-link.dw * link_vec[1], link.dw * link_vec[0]], dtype=np.float64)
        centripetal_link = np.array([-(link.w**2) * link_vec[0], -(link.w**2) * link_vec[1]], dtype=np.float64)
        tip_v = link.v + np.array([-link.w * link_vec[1], link.w * link_vec[0]], dtype=np.float64)
        tip_dv = link.dv + cross_dw_link + centripetal_link

        current_x += link_vec[0]
        current_y += link_vec[1]

        # Store the end-effector position of the current link
        link.xe = current_x
        link.ye = current_y
        link.vx = tip_v[0]
        link.vy = tip_v[1]
        link.ax = tip_dv[0]
        link.ay = tip_dv[1]

        # Vector from current joint to COM (relative to link frame)
        rc_curr = np.array([link.prop.rgx, link.prop.rgy])

        # Rotate rc_curr to base frame
        r_curr_to_base = np.array(
            [
                [cos_angle, -sin_angle],
                [sin_angle, cos_angle],
            ]
        )
        rc_curr_base_frame = r_curr_to_base @ rc_curr
        link.xg = link.x + rc_curr_base_frame[0]
        link.yg = link.y + rc_curr_base_frame[1]

        # Compute linear velocity of COM
        # vc_i = v_i + w_i x rc_i
        cross_w_rc = np.array([-link.w * rc_curr_base_frame[1], link.w * rc_curr_base_frame[0]])
        link.vc = link.v + cross_w_rc
        cross_dw_rc = np.array([-link.dw * rc_curr_base_frame[1], link.dw * rc_curr_base_frame[0]])
        centripetal_rc = np.array([-(link.w**2) * rc_curr_base_frame[0], -(link.w**2) * rc_curr_base_frame[1]])
        link.dvc = link.dv + cross_dw_rc + centripetal_rc
        link.agx = link.dvc[0]
        link.agy = link.dvc[1]

        # Update for next iteration
        prev_w = link.w
        prev_dw = link.dw
        prev_v = tip_v
        prev_dv = tip_dv


def compute_jacobian(skeleton: Skeleton) -> NDArray[np.float64]:
    r"""Compute the endpoint Jacobian relating joint and endpoint velocities.

    The Jacobian ``J`` is the ``2 x num_joints`` matrix that maps the joint
    velocities to the linear velocity of the endpoint, ``(xdot, ydot) = J qdot``.
    Each column is obtained geometrically from the lever arm between the joint
    and the endpoint,

    .. math::
        j_{x i} = -(y_n - y_{i-1}), \qquad j_{y i} = x_n - x_{i-1},

    where :math:`(x_n, y_n)` is the endpoint and :math:`(x_{i-1}, y_{i-1})` the
    origin of joint :math:`i`. The per-joint columns are also cached on each
    movable link as ``link.jx`` / ``link.jy``.

    Notes
    -----
    ``compute_forward_kinematics`` must have been run first so that the joint and
    endpoint positions are current.

    Parameters
    ----------
    skeleton : Skeleton
        The Skeleton object with up-to-date forward kinematics.

    Returns
    -------
    NDArray[np.float64]
        The ``2 x num_joints`` endpoint Jacobian matrix.
    """
    jacobian = np.zeros((2, skeleton.num_joints), dtype=np.float64)
    if skeleton.num_joints == 0:
        return jacobian

    tip = skeleton.links[-1]
    for column, link in enumerate(skeleton.links[1:]):
        link.jx = -(tip.ye - link.y)
        link.jy = tip.xe - link.x
        jacobian[0, column] = link.jx
        jacobian[1, column] = link.jy
    return jacobian


def compute_coriolis_basis(skeleton: Skeleton) -> NDArray[np.float64]:
    r"""Compute the centripetal/Coriolis acceleration basis of the endpoint.

    The basis ``H`` is the ``2 x num_joints`` matrix whose columns are the time
    derivatives of the Jacobian columns,

    .. math::
        h_{x i} = -(\dot{y}_n - \dot{y}_{i-1}), \qquad
        h_{y i} = \dot{x}_n - \dot{x}_{i-1}.

    Together with the Jacobian it gives the endpoint acceleration as
    ``(xddot, yddot) = J qddot + H qdot``. The per-joint columns are cached on
    each movable link as ``link.hx`` / ``link.hy``.

    Notes
    -----
    ``compute_forward_kinematics`` must have been run first so that the joint and
    endpoint velocities are current.

    Parameters
    ----------
    skeleton : Skeleton
        The Skeleton object with up-to-date forward kinematics.

    Returns
    -------
    NDArray[np.float64]
        The ``2 x num_joints`` centripetal/Coriolis basis matrix.
    """
    basis = np.zeros((2, skeleton.num_joints), dtype=np.float64)
    if skeleton.num_joints == 0:
        return basis

    tip = skeleton.links[-1]
    for column, link in enumerate(skeleton.links[1:]):
        link.hx = -(tip.vy - link.v[1])
        link.hy = tip.vx - link.v[0]
        basis[0, column] = link.hx
        basis[1, column] = link.hy
    return basis


def compute_endpoint_velocity(skeleton: Skeleton) -> NDArray[np.float64]:
    """Compute the endpoint linear velocity from the Jacobian.

    Evaluates ``(xdot, ydot) = J qdot`` using the current joint velocities. This
    offers an independent cross-check against the velocity propagated directly by
    ``compute_forward_kinematics`` (stored on the tip link as ``vx`` / ``vy``).

    Notes
    -----
    ``compute_forward_kinematics`` must have been run first.

    Parameters
    ----------
    skeleton : Skeleton
        The Skeleton object with up-to-date forward kinematics.

    Returns
    -------
    NDArray[np.float64]
        The 2-element endpoint velocity ``[xdot, ydot]``.
    """
    jacobian = compute_jacobian(skeleton)
    return (jacobian @ skeleton.dq).astype(np.float64)


def compute_endpoint_acceleration(skeleton: Skeleton) -> NDArray[np.float64]:
    """Compute the endpoint linear acceleration from the Jacobian and Coriolis basis.

    Evaluates ``(xddot, yddot) = J qddot + H qdot`` using the current joint
    velocities and accelerations. This offers an independent cross-check against
    the acceleration propagated directly by ``compute_forward_kinematics``
    (stored on the tip link as ``ax`` / ``ay``).

    Notes
    -----
    ``compute_forward_kinematics`` must have been run first.

    Parameters
    ----------
    skeleton : Skeleton
        The Skeleton object with up-to-date forward kinematics.

    Returns
    -------
    NDArray[np.float64]
        The 2-element endpoint acceleration ``[xddot, yddot]``.
    """
    jacobian = compute_jacobian(skeleton)
    basis = compute_coriolis_basis(skeleton)
    return (jacobian @ skeleton.ddq + basis @ skeleton.dq).astype(np.float64)


# === Numerical inverse kinematics ===


@dataclass
class IKResult:
    """Outcome of a numerical inverse-kinematics solve.

    Attributes
    ----------
    q : NDArray[np.float64]
        Final joint angles (radians), one per movable joint.
    position : NDArray[np.float64]
        Final endpoint position ``[x, y]``.
    residual : NDArray[np.float64]
        Final residual ``target - position``.
    residual_norm : float
        Euclidean norm of ``residual``.
    iterations : int
        Number of iterations performed.
    status : str
        Why the solver stopped: ``"converged"``, ``"stalled"``,
        ``"max_iterations"``, or ``"singular"``.
    joint_limits_hit : bool
        Whether any proposed step was clamped to a joint limit.
    success : bool
        ``True`` when ``residual_norm`` is within the position tolerance.
    """

    q: NDArray[np.float64]
    position: NDArray[np.float64]
    residual: NDArray[np.float64]
    residual_norm: float
    iterations: int
    status: str
    joint_limits_hit: bool
    success: bool


def _ik_step(
    method: str,
    jacobian: NDArray[np.float64],
    error: NDArray[np.float64],
    damping: float,
) -> NDArray[np.float64]:
    """Compute one joint-space step ``delta q`` for the selected IK method.

    The endpoint task weight ``W_e`` is the identity, so the gradient is
    ``g = J^T e`` and the Levenberg-Marquardt system is ``(J^T J + W_n) dq = g``.

    Parameters
    ----------
    method : str
        Step rule. Currently ``"lm"`` (``W_n = damping * I``) and ``"lm_sugihara"``
        (``W_n = (E_k + damping) * I`` with residual energy ``E_k``).
    jacobian : NDArray[np.float64]
        The ``2 x num_joints`` endpoint Jacobian at the current pose.
    error : NDArray[np.float64]
        The endpoint residual ``target - position``.
    damping : float
        Damping term: ``mu`` for ``"lm"`` or the bias ``w_bar`` for ``"lm_sugihara"``.

    Returns
    -------
    NDArray[np.float64]
        The joint-space step ``delta q``.

    Raises
    ------
    ValueError
        If ``method`` is not a recognized step rule.
    """
    num_joints = jacobian.shape[1]
    gradient = jacobian.T @ error
    if method == "lm":
        weight = damping
    elif method == "lm_sugihara":
        residual_energy = 0.5 * float(error @ error)
        weight = residual_energy + damping
    else:
        error_msg = f"Unknown inverse-kinematics method: {method!r}"
        raise ValueError(error_msg)
    hessian = jacobian.T @ jacobian + weight * np.eye(num_joints)
    return np.linalg.solve(hessian, gradient).astype(np.float64)


def compute_inverse_kinematics(
    skeleton: Skeleton,
    target: NDArray[np.float64] | tuple[float, float],
    *,
    method: str = "lm_sugihara",
    q0: NDArray[np.float64] | None = None,
    max_iterations: int = 100,
    position_tolerance: float = 1e-6,
    step_tolerance: float = 1e-9,
    damping: float = 1e-3,
    step_scale: float = 1.0,
) -> IKResult:
    """Solve endpoint position inverse kinematics by iterative refinement.

    Starting from a seed pose, the solver repeatedly linearizes the endpoint
    residual ``e = target - p(q)`` and takes a damped step until the residual is
    within tolerance, the step stalls, or the iteration limit is reached. The
    skeleton is driven to the final pose as a side effect. Each proposed step is
    clamped to the joint limits, so the returned pose is always feasible.

    Parameters
    ----------
    skeleton : Skeleton
        The arm to solve for. Its joint state is updated to the result pose.
    target : NDArray[np.float64] | tuple[float, float]
        Desired endpoint position ``[x, y]``.
    method : str, optional
        Step rule; ``"lm_sugihara"`` (the recommended default) or ``"lm"``.
    q0 : NDArray[np.float64] | None, optional
        Seed joint angles. Defaults to the skeleton's current pose.
    max_iterations : int, optional
        Maximum number of iterations.
    position_tolerance : float, optional
        Success threshold on the residual norm.
    step_tolerance : float, optional
        Stalls when the (clamped) joint step norm falls to or below this value.
    damping : float, optional
        Damping term passed to the step rule (``w_bar`` for ``"lm_sugihara"``).
    step_scale : float, optional
        Scale ``alpha`` applied to each step, in ``(0, 1]``.

    Returns
    -------
    IKResult
        The final pose, endpoint, residual, iteration count, and status.
    """
    target_xy = np.asarray(target, dtype=np.float64)
    lower = np.array([link.prop.qmin for link in skeleton.links[1:]], dtype=np.float64)
    upper = np.array([link.prop.qmax for link in skeleton.links[1:]], dtype=np.float64)

    q = skeleton.q if q0 is None else np.asarray(q0, dtype=np.float64)
    q = np.clip(q, lower, upper)
    skeleton.q = q  # eager setter runs forward kinematics; q is within limits

    joint_limits_hit = False
    status = "max_iterations"
    iterations = 0
    while iterations < max_iterations:
        iterations += 1
        tip = skeleton.links[-1]
        position = np.array([tip.xe, tip.ye], dtype=np.float64)
        error = target_xy - position
        if float(np.linalg.norm(error)) <= position_tolerance:
            status = "converged"
            break

        jacobian = compute_jacobian(skeleton)
        try:
            delta = _ik_step(method, jacobian, error, damping)
        except np.linalg.LinAlgError:
            status = "singular"
            break

        q_proposed = q + step_scale * delta
        q_next = np.clip(q_proposed, lower, upper)
        if not np.array_equal(q_next, q_proposed):
            joint_limits_hit = True
        if float(np.linalg.norm(q_next - q)) <= step_tolerance:
            status = "stalled"
            q = q_next
            skeleton.q = q
            break

        q = q_next
        skeleton.q = q

    tip = skeleton.links[-1]
    position = np.array([tip.xe, tip.ye], dtype=np.float64)
    residual = target_xy - position
    residual_norm = float(np.linalg.norm(residual))
    return IKResult(
        q=q.copy(),
        position=position,
        residual=residual,
        residual_norm=residual_norm,
        iterations=iterations,
        status=status,
        joint_limits_hit=joint_limits_hit,
        success=residual_norm <= position_tolerance,
    )
