"""Tests for trajectory time-scaling schedules and reference trajectories."""

from __future__ import annotations

import numpy as np
import pytest

from skelarm.trajectory import Trajectory, evaluate_schedule

_DURATION = 2.0


def test_schedule_endpoints_are_rest_to_rest_progress() -> None:
    """Every schedule starts at s=0 and ends at s=1."""
    for name in ("linear", "cubic", "quintic", "minimum_jerk"):
        s0, _, _ = evaluate_schedule(name, 0.0, _DURATION)
        s1, _, _ = evaluate_schedule(name, _DURATION, _DURATION)
        assert s0 == pytest.approx(0.0)
        assert s1 == pytest.approx(1.0)


def test_quintic_and_minimum_jerk_are_identical() -> None:
    """Rest-to-rest minimum jerk reduces to the quintic time law."""
    for t in (0.3 * _DURATION, 0.5 * _DURATION, 0.8 * _DURATION):
        assert evaluate_schedule("minimum_jerk", t, _DURATION) == pytest.approx(
            evaluate_schedule("quintic", t, _DURATION)
        )


def test_quintic_has_zero_endpoint_velocity_and_acceleration() -> None:
    """Quintic velocity and acceleration vanish toward both ends."""
    near_start = evaluate_schedule("quintic", 1e-6, _DURATION)
    near_end = evaluate_schedule("quintic", _DURATION - 1e-6, _DURATION)
    mid_velocity = abs(evaluate_schedule("quintic", 0.5 * _DURATION, _DURATION)[1])
    assert abs(near_start[1]) < mid_velocity  # velocity ramps up from ~0
    assert abs(near_end[1]) < mid_velocity
    assert abs(near_start[2]) < abs(evaluate_schedule("cubic", 1e-6, _DURATION)[2])  # accel starts gentler than cubic


def test_linear_has_constant_nonzero_interior_velocity() -> None:
    """Linear interpolation moves at a constant, non-zero speed inside the interval."""
    early = evaluate_schedule("linear", 0.1 * _DURATION, _DURATION)[1]
    late = evaluate_schedule("linear", 0.9 * _DURATION, _DURATION)[1]
    assert early == pytest.approx(late)
    assert early == pytest.approx(1.0 / _DURATION)


def test_unknown_schedule_and_bad_duration_raise() -> None:
    """Misuse is reported rather than silently accepted."""
    with pytest.raises(ValueError, match="schedule"):
        evaluate_schedule("bogus", 0.0, _DURATION)
    with pytest.raises(ValueError, match="duration"):
        evaluate_schedule("quintic", 0.0, 0.0)


def test_trajectory_interpolates_between_endpoints() -> None:
    """A Trajectory maps the schedule onto the start->end vector."""
    start = np.array([0.2, -0.1])
    end = np.array([0.5, 0.4])
    traj = Trajectory(start, end, _DURATION, schedule="quintic")

    y0, dy0, ddy0 = traj.sample(0.0)
    y_end, dy_end, _ = traj.sample(_DURATION)
    ymid, _, _ = traj.sample(0.5 * _DURATION)

    assert y0 == pytest.approx(start)
    assert y_end == pytest.approx(end)
    assert ymid == pytest.approx(0.5 * (start + end))  # quintic is symmetric at the midpoint
    assert dy0 == pytest.approx(np.zeros(2))
    assert dy_end == pytest.approx(np.zeros(2))
    assert ddy0 == pytest.approx(np.zeros(2))


def test_trajectory_holds_outside_the_time_window() -> None:
    """Before t=0 the trajectory rests at the start; after the duration it rests at the end."""
    start = np.array([0.0, 0.0])
    end = np.array([1.0, 2.0])
    traj = Trajectory(start, end, _DURATION)

    before_y, before_dy, _ = traj.sample(-1.0)
    after_y, after_dy, _ = traj.sample(_DURATION + 5.0)
    assert before_y == pytest.approx(start)
    assert before_dy == pytest.approx(np.zeros(2))
    assert after_y == pytest.approx(end)
    assert after_dy == pytest.approx(np.zeros(2))
