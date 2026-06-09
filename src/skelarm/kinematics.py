"""Provides functions for robot arm kinematics."""

from __future__ import annotations

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
