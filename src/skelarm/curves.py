"""Closed periodic task-space curves for repeated tracing.

A periodic-curve task makes the robot's tip trace a closed planar curve over and over.
Each curve is a position function of a phase angle ``theta`` (``2*pi``-periodic);
:class:`PeriodicTaskReference` maps simulated time to phase (``theta = omega * t`` with
``omega = 2*pi / period``) and exposes the ``TaskReference`` interface, so
:func:`~skelarm.ik_joint_reference` converts it to a joint reference for the tracking
controllers. The built-in curves are ``circle``, ``ellipse``, ``lemniscate`` (the
horizontal Bernoulli infinity), ``vertical_lemniscate`` (an upright figure-eight), and
``rose`` (a ``k``-petal rhodonea ``r = a*cos(k*theta)``). Register your own with
:func:`register_curve`.

See ``docs/reference/09_trajectory_filtering.md`` and the
[Control Configuration](../guides/control_configuration.md) guide.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from collections.abc import Mapping
    from typing import Any

    from numpy.typing import NDArray

# A curve maps a phase angle theta to a planar position (x, y); a factory builds one
# from the task params (center, radius, ...).
CurveFunction = Callable[[float], "NDArray[np.float64]"]
CurveFactory = Callable[["Mapping[str, Any]"], CurveFunction]

_CURVES: dict[str, CurveFactory] = {}


def register_curve(name: str, factory: CurveFactory) -> None:
    """Register a closed-curve factory under ``name`` for the ``periodic_curve`` task.

    Parameters
    ----------
    name : str
        The ``[task].curve`` value to bind.
    factory : CurveFactory
        A callable ``(params) -> (theta -> position)`` building a ``2*pi``-periodic
        position function from the task parameters.
    """
    _CURVES[name] = factory


def curve_kinds() -> tuple[str, ...]:
    """Return the registered curve kinds, sorted."""
    return tuple(sorted(_CURVES))


def build_curve(kind: str, params: Mapping[str, Any]) -> CurveFunction:
    """Build the position function for curve ``kind`` from ``params``.

    Raises
    ------
    ValueError
        If ``kind`` is not a registered curve.
    """
    if kind not in _CURVES:
        msg = f"unknown curve {kind!r}; choose from {curve_kinds()}"
        raise ValueError(msg)
    return _CURVES[kind](params)


class PeriodicTaskReference:
    """A task-space reference that traces a closed curve with a fixed period.

    Implements the ``TaskReference`` interface (``sample`` + ``duration``). The
    velocity and acceleration are obtained by central differences in time (they are not
    needed by :func:`~skelarm.ik_joint_reference`, which uses only the position).

    Parameters
    ----------
    curve : CurveFunction
        A ``2*pi``-periodic position function ``theta -> (x, y)``.
    period : float
        Time for one full loop (seconds).
    duration : float
        Total trace time (seconds); ``duration / period`` loops are traced.
    """

    def __init__(self, curve: CurveFunction, *, period: float, duration: float) -> None:
        """Store the curve and its time mapping."""
        if period <= 0.0:
            msg = f"period must be positive, got {period}"
            raise ValueError(msg)
        self._curve = curve
        self.period = float(period)
        self.duration = float(duration)
        self._omega = 2.0 * np.pi / self.period

    def position(self, t: float) -> NDArray[np.float64]:
        """The curve position at time ``t``."""
        return self._curve(self._omega * t)

    def sample(self, t: float) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
        """Return ``(p, dp, ddp)`` at time ``t`` (position and time derivatives)."""
        h = 1e-4 * self.period
        p = self.position(t)
        forward = self.position(t + h)
        backward = self.position(t - h)
        dp = (forward - backward) / (2.0 * h)
        ddp = (forward - 2.0 * p + backward) / (h * h)
        return p, dp, ddp


def _center(params: Mapping[str, Any]) -> NDArray[np.float64]:
    """Read the optional curve center (default origin)."""
    return np.asarray(params.get("center", (0.0, 0.0)), dtype=np.float64)


def _circle(params: Mapping[str, Any]) -> CurveFunction:
    """Circle of ``radius`` about ``center`` (``phase`` shifts the start)."""
    center = _center(params)
    radius = float(params.get("radius", 1.0))
    phase = float(params.get("phase", 0.0))

    def position(theta: float) -> NDArray[np.float64]:
        return center + radius * np.array([np.cos(theta + phase), np.sin(theta + phase)])

    return position


def _ellipse(params: Mapping[str, Any]) -> CurveFunction:
    """Axis-aligned ellipse with semi-axes ``a`` (x) and ``b`` (y) about ``center``."""
    center = _center(params)
    a = float(params.get("a", 1.0))
    b = float(params.get("b", 0.5))
    phase = float(params.get("phase", 0.0))

    def position(theta: float) -> NDArray[np.float64]:
        return center + np.array([a * np.cos(theta + phase), b * np.sin(theta + phase)])

    return position


def _lemniscate(params: Mapping[str, Any]) -> CurveFunction:
    """Bernoulli lemniscate (horizontal infinity) of scale ``a`` about ``center``."""
    center = _center(params)
    a = float(params.get("a", 1.0))

    def position(theta: float) -> NDArray[np.float64]:
        denom = 1.0 + np.sin(theta) ** 2
        return center + (a / denom) * np.array([np.cos(theta), np.sin(theta) * np.cos(theta)])

    return position


def _vertical_lemniscate(params: Mapping[str, Any]) -> CurveFunction:
    """Upright figure-eight (Gerono, rotated) of width ``a`` and height ``b`` about ``center``."""
    center = _center(params)
    a = float(params.get("a", 0.5))
    b = float(params.get("b", 1.0))

    def position(theta: float) -> NDArray[np.float64]:
        return center + np.array([a * np.sin(theta) * np.cos(theta), b * np.cos(theta)])

    return position


def _rose(params: Mapping[str, Any]) -> CurveFunction:
    """Rhodonea (rose) ``r = a*cos(k*theta)`` about ``center`` (``k`` petals if odd, ``2k`` if even)."""
    center = _center(params)
    a = float(params.get("a", 1.0))
    k = int(params.get("k", 3))

    def position(theta: float) -> NDArray[np.float64]:
        r = a * np.cos(k * theta)
        return center + r * np.array([np.cos(theta), np.sin(theta)])

    return position


register_curve("circle", _circle)
register_curve("ellipse", _ellipse)
register_curve("lemniscate", _lemniscate)
register_curve("vertical_lemniscate", _vertical_lemniscate)
register_curve("rose", _rose)
