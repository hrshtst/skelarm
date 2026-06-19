"""Reference trajectories for motion planning.

A :class:`Trajectory` moves a vector from ``start`` to ``end`` over a fixed
duration following a smooth time-scaling schedule ``s(t)``, returning position,
velocity, and acceleration samples. It is dimension-agnostic, so the same class
produces task-space endpoint references ``p_r(t)`` and joint-space references
``q_r(t)``.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from numpy.typing import ArrayLike, NDArray

# Each schedule maps a normalized time u = t / duration in [0, 1] to the progress
# s(u) and its first two derivatives with respect to u. Chain rule then scales
# them into time derivatives (divide by duration once / twice).
_SCHEDULES: dict[str, Callable[[float], tuple[float, float, float]]] = {
    "linear": lambda u: (u, 1.0, 0.0),
    "cubic": lambda u: (3 * u**2 - 2 * u**3, 6 * u - 6 * u**2, 6 - 12 * u),
    "quintic": lambda u: (
        10 * u**3 - 15 * u**4 + 6 * u**5,
        30 * u**2 - 60 * u**3 + 30 * u**4,
        60 * u - 180 * u**2 + 120 * u**3,
    ),
}
# Rest-to-rest minimum jerk is the quintic time law (see docs/reference/07_control.md).
_SCHEDULES["minimum_jerk"] = _SCHEDULES["quintic"]

SCHEDULES = tuple(_SCHEDULES)


def evaluate_schedule(name: str, t: float, duration: float) -> tuple[float, float, float]:
    """Evaluate a time-scaling schedule and its time derivatives.

    Parameters
    ----------
    name : str
        One of :data:`SCHEDULES` (``linear``, ``cubic``, ``quintic``, ``minimum_jerk``).
    t : float
        Time. Values outside ``[0, duration]`` hold at rest (0 before, 1 after).
    duration : float
        Total motion duration; must be positive.

    Returns
    -------
    tuple[float, float, float]
        The progress ``s`` in ``[0, 1]`` and its time derivatives ``ds``, ``dds``.

    Raises
    ------
    ValueError
        If ``name`` is not a known schedule or ``duration`` is not positive.
    """
    if name not in _SCHEDULES:
        msg = f"unknown schedule {name!r}; choose from {SCHEDULES}"
        raise ValueError(msg)
    if duration <= 0.0:
        msg = f"duration must be positive, got {duration}"
        raise ValueError(msg)
    if t <= 0.0:
        return 0.0, 0.0, 0.0
    if t >= duration:
        return 1.0, 0.0, 0.0
    s, ds_du, dds_du = _SCHEDULES[name](t / duration)
    return s, ds_du / duration, dds_du / duration**2


class Trajectory:
    """A smooth point-to-point reference for a vector quantity.

    Parameters
    ----------
    start, end : ArrayLike
        The initial and final vectors (same shape).
    duration : float
        Motion duration; must be positive.
    schedule : str, optional
        The time-scaling schedule (see :data:`SCHEDULES`).
    """

    def __init__(self, start: ArrayLike, end: ArrayLike, duration: float, schedule: str = "quintic") -> None:
        """Initialize the trajectory and validate its schedule and duration."""
        if schedule not in _SCHEDULES:
            msg = f"unknown schedule {schedule!r}; choose from {SCHEDULES}"
            raise ValueError(msg)
        if duration <= 0.0:
            msg = f"duration must be positive, got {duration}"
            raise ValueError(msg)
        self.start = np.asarray(start, dtype=np.float64)
        self.end = np.asarray(end, dtype=np.float64)
        self.duration = float(duration)
        self.schedule = schedule

    def sample(self, t: float) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
        """Return ``(y, dy, ddy)`` at time ``t`` (position, velocity, acceleration)."""
        s, ds, dds = evaluate_schedule(self.schedule, t, self.duration)
        delta = self.end - self.start
        return self.start + s * delta, ds * delta, dds * delta
