# Copyright (C) 2026 Hiroshi Atsuta <atsuta@ieee.org>
# SPDX-License-Identifier: GPL-3.0-only

"""Tests for replaying a recorded tip force."""

from __future__ import annotations

import math

import numpy as np
import pytest
from hypothesis import given, settings
from hypothesis import strategies as st

from skelarm.control import Controller, JointPD, simulate_controlled
from skelarm.forces import RecordedForce
from skelarm.recording import StateLog
from skelarm.skeleton import LinkProp, Skeleton
from skelarm.trajectory import Trajectory


def _two_link() -> Skeleton:
    """A planar two-link arm at a bent, non-singular pose."""
    link_props = [
        LinkProp(length=1.0, m=1.0, i=0.1, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi),
        LinkProp(length=0.8, m=0.8, i=0.05, rgx=0.4, rgy=0.0, qmin=-np.pi, qmax=np.pi),
    ]
    skeleton = Skeleton(link_props)
    skeleton.q = np.array([0.3, 0.3])
    return skeleton


def _controller(skeleton: Skeleton) -> Controller:
    reference = Trajectory(skeleton.q, skeleton.q + np.array([0.6, 0.4]), duration=0.6)
    return JointPD(reference, kp=40.0, kd=6.0)


def _pushes(t: float, _skeleton: Skeleton) -> np.ndarray:
    """Two pushes, the second in another direction."""
    if 0.1 <= t < 0.25:  # noqa: PLR2004
        return np.array([3.0, -1.0])
    if 0.5 <= t < 0.6:  # noqa: PLR2004
        return np.array([-2.0, 2.5])
    return np.zeros(2)


def test_each_sample_acts_until_the_next_one() -> None:
    """Sample ``k`` acts over ``[t_k, t_k+1)``; there is no force before the first sample or from the last one on."""
    force = RecordedForce([0.0, 0.1, 0.2, 0.3], [[1.0, 0.0], [2.0, 0.0], [3.0, 0.0], [4.0, 0.0]])

    # The last sample is the end of the record, as simulate_controlled's final frame is.
    times = [-0.01, 0.0, 0.099, 0.1, 0.25, 0.3, 0.5]
    assert [float(force(t)[0]) for t in times] == [0.0, 1.0, 1.0, 2.0, 3.0, 0.0, 0.0]
    assert (force.start, force.stop) == (0.0, 0.3)


def test_a_clock_that_adds_up_its_steps_still_finds_each_sample() -> None:
    """A simulation clock accumulates rounding as it adds ``dt``; each step still meets its own sample."""
    dt = 0.002
    times = dt * np.arange(2001)
    values = np.arange(2001, dtype=np.float64)
    force = RecordedForce(times, np.column_stack([values, -values]))

    t = 0.0
    for k in range(2000):
        assert force(t)[0] == values[k]
        t += dt


def test_a_simulated_run_is_reproduced_exactly_by_replaying_its_force() -> None:
    """Replaying a run's recorded ``ext_force`` with the same step gives the very same run."""
    skeleton = _two_link()
    original = simulate_controlled(skeleton, _controller(skeleton), duration=0.8, dt=0.002, external_force=_pushes)
    replay = RecordedForce.from_log(original)

    again = simulate_controlled(skeleton, _controller(skeleton), duration=0.8, dt=0.002, external_force=replay)

    assert np.array_equal(again.channel("ext_force"), original.channel("ext_force"))
    assert np.array_equal(again.channel("q"), original.channel("q"))
    assert np.array_equal(again.channel("tau"), original.channel("tau"))


def test_a_time_offset_moves_the_record_on_the_clock() -> None:
    """``time_offset`` shifts every sample: a log on one clock is replayed on another."""
    skeleton = _two_link()
    log = simulate_controlled(skeleton, _controller(skeleton), duration=0.8, dt=0.002, external_force=_pushes)

    later = RecordedForce.from_log(log, time_offset=1.0)

    assert later.start == pytest.approx(1.0)
    assert later(1.15) == pytest.approx([3.0, -1.0])
    assert later(0.15) == pytest.approx([0.0, 0.0])


def test_a_log_without_the_channel_is_refused() -> None:
    """A log recorded without a tip force has nothing to replay."""
    log = simulate_controlled(_two_link(), _controller(_two_link()), duration=0.05, dt=0.01)

    with pytest.raises(ValueError, match="ext_force"):
        RecordedForce.from_log(log)


@pytest.mark.parametrize(
    ("times", "forces"),
    [
        ([0.0, 0.1, 0.1], [[1.0, 0.0]] * 3),  # times not increasing
        ([0.0, 0.1], [[1.0, 0.0, 0.0]] * 2),  # not a planar force
        ([0.0, 0.1], [[1.0, 0.0]]),  # a force per time
        ([0.0], [[1.0, 0.0]]),  # one sample covers no time
    ],
)
def test_an_inconsistent_record_is_refused(times: list[float], forces: list[list[float]]) -> None:
    """Times that do not increase, or forces that are not one ``(fx, fy)`` per time, are refused."""
    with pytest.raises(ValueError):  # noqa: PT011
        RecordedForce(times, forces)


def test_stepping_with_the_recorded_step_gives_each_sample_exactly() -> None:
    """With the recording's own step, ``stepped`` gives back the recorded samples bit for bit."""
    times = 0.002 * np.arange(11)
    forces = np.random.default_rng(0).normal(size=(11, 2))
    stepped = RecordedForce(times, forces).stepped(0.002)

    t = 0.0
    for k in range(10):
        assert np.array_equal(stepped(t, _two_link()), forces[k])
        t += 0.002


@settings(max_examples=40, deadline=None)
@given(
    dt=st.sampled_from([0.001, 0.0015, 0.003, 0.0025, 0.01]),
    seed=st.integers(min_value=0, max_value=10_000),
)
def test_stepping_with_another_step_keeps_the_impulse(dt: float, seed: int) -> None:
    """With another step, each step gets the mean of the recorded force over it, so the impulse is kept."""
    rng = np.random.default_rng(seed)
    times = 0.002 * np.arange(101)
    forces = rng.normal(size=(101, 2))
    recorded = RecordedForce(times, forces)
    stepped = recorded.stepped(dt)

    steps = math.ceil(0.2 / dt - 1e-9)  # enough steps to cover the record
    impulse = sum(stepped(k * dt, _two_link()) * dt for k in range(steps))

    assert impulse == pytest.approx((forces[:-1] * 0.002).sum(axis=0), abs=1e-12)


def test_the_mean_over_an_interval_weighs_each_sample_by_its_overlap() -> None:
    """The mean over an interval counts each sample for as long as it acts within it."""
    force = RecordedForce([0.0, 0.1, 0.2, 0.3], [[1.0, 0.0], [3.0, 0.0], [5.0, 0.0], [0.0, 0.0]])

    assert force.mean(0.05, 0.15)[0] == pytest.approx(2.0)  # half of each of the first two
    assert force.mean(0.0, 0.3)[0] == pytest.approx(3.0)
    assert force.mean(0.25, 0.45)[0] == pytest.approx(1.25)  # 0.05 s of 5 N, then nothing
    assert force.mean(-0.2, -0.1)[0] == 0.0


def test_the_pushes_are_the_spans_of_force_above_a_threshold() -> None:
    """``spans`` finds when the force exceeds a threshold, joining spans that a short lull separates."""
    times = 0.01 * np.arange(101)
    forces = np.zeros((101, 2))
    forces[10:25] = [2.0, 0.0]  # 0.10 to 0.25 s
    forces[27:30] = [0.0, 1.5]  # a lull of 0.02 s, then on to 0.30 s
    forces[60:70] = [0.5, 0.5]  # 0.60 to 0.70 s, about 0.7 N
    forces[80:82] = [0.05, 0.0]  # too weak to count
    recorded = RecordedForce(times, forces)

    every = np.array([(0.10, 0.25), (0.27, 0.30), (0.60, 0.70)])
    joined = np.array([(0.10, 0.30), (0.60, 0.70)])
    strong = np.array([(0.10, 0.25), (0.27, 0.30)])
    assert np.array(recorded.spans(threshold=0.1)) == pytest.approx(every)
    assert np.array(recorded.spans(threshold=0.1, min_gap=0.05)) == pytest.approx(joined)
    assert np.array(recorded.spans(threshold=1.0)) == pytest.approx(strong)


def test_a_recorded_force_round_trips_through_a_saved_log(tmp_path) -> None:  # noqa: ANN001
    """A log saved and loaded again replays the same force."""
    skeleton = _two_link()
    log = simulate_controlled(skeleton, _controller(skeleton), duration=0.8, dt=0.002, external_force=_pushes)
    log.save(tmp_path / "pushed.sklog.npz")

    loaded = RecordedForce.from_log(StateLog.load(tmp_path / "pushed.sklog.npz"))

    assert loaded(0.2) == pytest.approx([3.0, -1.0])
    assert loaded(0.55) == pytest.approx([-2.0, 2.5])
