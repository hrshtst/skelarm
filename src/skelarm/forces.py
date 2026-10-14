# Copyright (C) 2026 Hiroshi Atsuta <atsuta@ieee.org>
# SPDX-License-Identifier: GPL-3.0-only

"""Replay a recorded tip force.

A run recorded with an external tip force, such as an arm pushed by hand in an
interactive simulator or by a force that :func:`~skelarm.control.simulate_controlled`
scripted, carries the force in its ``ext_force`` channel. :class:`RecordedForce`
replays that history by time, as an ``external_force`` for
:func:`~skelarm.control.simulate_controlled`: the same force at the same times,
whatever the arm does, so that runs of different controllers meet the same pushes.

The record is a zero-order hold, as :func:`~skelarm.control.simulate_controlled`
applies a force: the sample recorded at ``t_k`` acts over ``[t_k, t_k+1)``. The last
sample marks the end of the record, as a run's final frame does, so no force acts
from it on, nor before the first sample.

Example::

    log = StateLog.load("pushed.sklog.npz")
    pushes = RecordedForce.from_log(log)
    again = simulate_controlled(skeleton, controller, duration=2.0, dt=0.002, external_force=pushes)

With a step other than the recording's, :meth:`RecordedForce.stepped` gives each step
the mean recorded force over it, which keeps every push's impulse.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from collections.abc import Callable

    from numpy.typing import ArrayLike, NDArray

    from skelarm.recording import StateLog
    from skelarm.skeleton import Skeleton

# Times this close (s) are the same instant, so that a clock that adds up its steps meets each recorded sample.
_TOLERANCE = 1e-9
_PLANAR = 2  # a tip force (fx, fy)


class RecordedForce:
    """A recorded tip force, replayed by time as an external force.

    Parameters
    ----------
    times : ArrayLike
        The sample times (s), increasing; at least two.
    forces : ArrayLike
        The tip force ``(fx, fy)`` (N, base frame) of each sample, shaped ``(len(times), 2)``.

    Raises
    ------
    ValueError
        If the times do not increase, or the forces are not one ``(fx, fy)`` per time.
    """

    def __init__(self, times: ArrayLike, forces: ArrayLike) -> None:
        t = np.asarray(times, dtype=np.float64)
        f = np.asarray(forces, dtype=np.float64)
        if t.ndim != 1 or t.size < _PLANAR:
            msg = "a recorded force needs at least two sample times"
            raise ValueError(msg)
        if f.shape != (t.size, _PLANAR):
            msg = f"the forces must be one (fx, fy) per sample time, shape ({t.size}, 2), got {f.shape}"
            raise ValueError(msg)
        if np.any(np.diff(t) <= 0):
            msg = "the sample times must increase"
            raise ValueError(msg)
        self.times: NDArray[np.float64] = t
        self.forces: NDArray[np.float64] = f

    @classmethod
    def from_log(cls, log: StateLog, channel: str = "ext_force", time_offset: float = 0.0) -> RecordedForce:
        """Take the recorded force of a log.

        Parameters
        ----------
        log : StateLog
            A recorded run with a tip force channel.
        channel : str, optional
            The channel to replay, ``ext_force`` by default.
        time_offset : float, optional
            Added to every sample time (s), to replay the log on another clock.

        Returns
        -------
        RecordedForce
            The log's force history.

        Raises
        ------
        ValueError
            If the log has no such channel.
        """
        if channel not in log.channel_names:
            msg = f"the log has no {channel!r} channel to replay"
            raise ValueError(msg)
        return cls(np.asarray(log.times, dtype=np.float64) + time_offset, log.channel(channel))

    @property
    def start(self) -> float:
        """The time of the first sample (s), from which the record acts."""
        return float(self.times[0])

    @property
    def stop(self) -> float:
        """The time of the last sample (s), the end of the record."""
        return float(self.times[-1])

    def __call__(self, t: float, skeleton: Skeleton | None = None) -> NDArray[np.float64]:  # noqa: ARG002
        """The recorded force acting at time ``t``, as ``simulate_controlled``'s ``external_force``.

        Parameters
        ----------
        t : float
            The time (s).
        skeleton : Skeleton | None, optional
            The arm's state, which a recorded force does not depend on.

        Returns
        -------
        NDArray[np.float64]
            The tip force ``(fx, fy)`` (N): the latest sample at or before ``t``, or zero
            outside the record.
        """
        k = int(np.searchsorted(self.times, t + _TOLERANCE, side="right")) - 1
        if 0 <= k < self.times.size - 1:
            return self.forces[k].copy()
        return np.zeros(_PLANAR)

    def mean(self, begin: float, end: float) -> NDArray[np.float64]:
        """The mean recorded force over ``[begin, end)``, each sample weighed by how long it acts within it.

        Parameters
        ----------
        begin, end : float
            The interval (s), ``end`` after ``begin``.

        Returns
        -------
        NDArray[np.float64]
            The mean tip force ``(fx, fy)`` (N); exactly the sample when one alone acts
            over the whole interval.

        Raises
        ------
        ValueError
            If ``end`` is not after ``begin``.
        """
        if end <= begin:
            msg = f"the interval must end after it begins, got [{begin}, {end})"
            raise ValueError(msg)
        # Only the samples whose spans can overlap the interval: from the one acting at its beginning to the one
        # acting at its end, so that each call costs the same however long the record.
        first = max(int(np.searchsorted(self.times, begin, side="right")) - 1, 0)
        last = min(int(np.searchsorted(self.times, end, side="left")), self.times.size - 1)
        if last <= first:
            return np.zeros(_PLANAR)
        overlap = np.minimum(self.times[first + 1 : last + 1], end) - np.maximum(self.times[first:last], begin)
        overlap[overlap < _TOLERANCE] = 0.0  # a sliver from the clock's rounding is no overlap
        acting = np.flatnonzero(overlap)
        if acting.size == 1 and overlap[acting[0]] >= end - begin - _TOLERANCE:
            return self.forces[first + acting[0]].copy()
        return (overlap[:, np.newaxis] * self.forces[first:last]).sum(axis=0) / (end - begin)

    def stepped(self, dt: float) -> Callable[[float, Skeleton], NDArray[np.float64]]:
        """An external force for a simulation that steps by ``dt``: the mean recorded force over each step.

        A step ``[t, t + dt)`` gets the mean of the record over it, so every push keeps its
        impulse whatever the step. With the recording's own step, aligned with its
        samples, each step gets its sample exactly, as :meth:`__call__` gives it.

        Parameters
        ----------
        dt : float
            The simulation step (s).

        Returns
        -------
        Callable[[float, Skeleton], NDArray[np.float64]]
            ``f(t, skeleton) -> (fx, fy)``, for ``simulate_controlled``'s ``external_force``.

        Raises
        ------
        ValueError
            If ``dt`` is not positive.
        """
        if dt <= 0:
            msg = f"the step must be positive, got {dt}"
            raise ValueError(msg)

        def force(t: float, _skeleton: Skeleton) -> NDArray[np.float64]:
            return self.mean(t, t + dt)

        return force

    def spans(self, threshold: float, min_gap: float = 0.0) -> list[tuple[float, float]]:
        """When the force acts: the spans of time its magnitude exceeds ``threshold``.

        Parameters
        ----------
        threshold : float
            The magnitude (N) above which the force counts as acting.
        min_gap : float, optional
            Spans separated by less than this (s) are joined into one, so that a push
            that lulls for a moment counts once.

        Returns
        -------
        list[tuple[float, float]]
            Each span's ``(begin, end)`` (s): from the first sample above the threshold
            to the end of the last, in time order.
        """
        above = np.linalg.norm(self.forces[:-1], axis=1) > threshold
        spans: list[tuple[float, float]] = []
        k = 0
        while k < above.size:
            if not above[k]:
                k += 1
                continue
            j = k
            while j < above.size and above[j]:
                j += 1
            begin, end = float(self.times[k]), float(self.times[j])
            if spans and begin - spans[-1][1] < min_gap - _TOLERANCE:
                spans[-1] = (spans[-1][0], end)
            else:
                spans.append((begin, end))
            k = j
        return spans
