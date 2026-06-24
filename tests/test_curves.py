"""Tests for periodic task-space curves and the periodic_curve task type."""

from __future__ import annotations

import tomllib
from pathlib import Path

import numpy as np
import pytest

from skelarm import Skeleton, compute_inverse_kinematics
from skelarm.curves import PeriodicTaskReference, build_curve, curve_kinds
from skelarm.scenario import load_scenario, run_scenario

_SKELETON_TOML = (
    "[skeleton]\n"
    "[[skeleton.link]]\nlength = 1.0\nmass = 1.0\ninertia = 0.1\ncom = [0.5, 0.0]\nlimits = [-180.0, 180.0]\n"
    "[[skeleton.link]]\nlength = 0.8\nmass = 0.8\ninertia = 0.05\ncom = [0.4, 0.0]\nlimits = [-180.0, 180.0]\n"
)

_ALL_CURVES = ("circle", "ellipse", "lemniscate", "vertical_lemniscate", "rose")


def test_curve_kinds_lists_builtins() -> None:
    """The built-in curve kinds are registered."""
    assert set(_ALL_CURVES) <= set(curve_kinds())


def test_build_unknown_curve_raises() -> None:
    """An unknown curve kind is reported."""
    with pytest.raises(ValueError, match="unknown curve"):
        build_curve("spiral", {})


def test_circle_lies_on_its_radius() -> None:
    """Every point of a circle curve is at the configured radius from the center."""
    curve = build_curve("circle", {"center": [1.0, 0.5], "radius": 0.3})
    ref = PeriodicTaskReference(curve, period=2.0, duration=2.0)
    pts = np.array([ref.position(t) for t in np.linspace(0.0, 2.0, 200)])
    d = np.linalg.norm(pts - np.array([1.0, 0.5]), axis=1)
    assert d == pytest.approx(0.3, abs=1e-9)


@pytest.mark.parametrize("kind", _ALL_CURVES)
def test_curve_is_closed_and_velocity_continuous(kind: str) -> None:
    """Each curve returns to its start after one period with continuous velocity."""
    ref = PeriodicTaskReference(
        build_curve(kind, {"a": 0.5, "b": 0.3, "radius": 0.4, "k": 3}), period=3.0, duration=3.0
    )
    p_start, dp_start, _ = ref.sample(0.0)
    p_end, dp_end, _ = ref.sample(3.0)
    assert p_end == pytest.approx(p_start, abs=1e-6)  # closed
    assert dp_end == pytest.approx(dp_start, abs=1e-4)  # C^1 across the seam


@pytest.mark.parametrize(("k", "expected"), [(2, 4), (3, 3), (4, 8), (5, 5)])
def test_rose_petal_count(k: int, expected: int) -> None:
    """A rhodonea has k petals for odd k and 2k for even k."""
    curve = build_curve("rose", {"a": 1.0, "k": k, "center": [0.0, 0.0]})
    ref = PeriodicTaskReference(curve, period=1.0, duration=1.0)
    n = 4000
    pts = np.array([ref.position(t) for t in np.linspace(0.0, 1.0, n, endpoint=False)])
    d = np.linalg.norm(pts, axis=1)
    # Local maxima of the radius are the petal tips; distinct tip positions are the petals.
    peaks = [i for i in range(n) if d[i] > d[i - 1] and d[i] >= d[(i + 1) % n] and d[i] > 0.5 * d.max()]
    petals = {tuple(np.round(pts[i], 2)) for i in peaks}
    assert len(petals) == expected


def test_periodic_reference_rejects_nonpositive_period() -> None:
    """A non-positive period is rejected."""
    with pytest.raises(ValueError, match="period must be positive"):
        PeriodicTaskReference(build_curve("circle", {}), period=0.0, duration=1.0)


def test_example_periodic_curve_config_loads() -> None:
    """The shipped examples/periodic_curve.toml is a valid periodic-curve scenario."""
    config = Path(__file__).resolve().parents[1] / "examples" / "periodic_curve.toml"
    scenario = load_scenario(config)
    assert scenario.task.type == "periodic_curve"
    assert scenario.task.params["curve"] == "ellipse"


def test_periodic_curve_task_traces_a_circle(tmp_path: Path) -> None:
    """A periodic_curve scenario drives the tip around the configured circle."""
    center = (0.9, 0.4)
    radius = 0.18
    # Start the arm at the curve's t=0 point so it locks on quickly.
    skeleton = Skeleton.from_config(tomllib.loads(_SKELETON_TOML))
    compute_inverse_kinematics(skeleton, np.array([center[0] + radius, center[1]]))
    q0 = ", ".join(str(v) for v in np.rad2deg(skeleton.q))

    config = tmp_path / "trace.toml"
    config.write_text(
        _SKELETON_TOML
        + f"[initial]\nq = [{q0}]\n"
        + '[task]\ntype = "periodic_curve"\ncurve = "circle"\n'
        + f"center = [{center[0]}, {center[1]}]\nradius = {radius}\nperiod = 1.0\nduration = 2.0\ndt = 0.005\n"
        + '[controller]\ntype = "computed_torque"\nkp = 400.0\nkd = 50.0\n',
        encoding="utf-8",
    )
    log = run_scenario(load_scenario(config))

    arm = log.build_skeleton()
    tip_points = []
    for q in log.channel("q"):
        arm.q = q
        tip_points.append([arm.links[-1].xe, arm.links[-1].ye])
    tip = np.array(tip_points)
    second_loop = tip[len(tip) // 2 :]  # skip the lock-on transient
    d = np.linalg.norm(second_loop - np.array(center), axis=1)
    assert d == pytest.approx(radius, abs=2e-2)  # tip stays on the circle
