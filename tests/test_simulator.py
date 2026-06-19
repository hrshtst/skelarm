"""Tests for the real-time simulator widgets (skelarm.simulator)."""

from __future__ import annotations

import os

import numpy as np
import pytest

# Run Qt without a display so the test works headless (CI and local).
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from skelarm.skeleton import LinkProp, Skeleton

pytestmark = pytest.mark.integration


@pytest.fixture(scope="module")
def qapp():  # noqa: ANN201
    """Provide a single QApplication instance for the GUI tests."""
    from PyQt6.QtWidgets import QApplication

    return QApplication.instance() or QApplication([])


def _simulator(qmin: float = -np.pi, qmax: float = np.pi, q: tuple[float, ...] = (0.3, 0.3)):  # noqa: ANN202
    """Build a two-link simulator at rest in the given pose."""
    from skelarm.simulator import SkelarmSimulator

    link_props = [LinkProp(length=1.0, m=1.0, i=0.1, rgx=0.5, rgy=0.0, qmin=qmin, qmax=qmax) for _ in range(2)]
    skeleton = Skeleton(link_props)
    skeleton.q = np.array(q)
    skeleton.dq = np.zeros(2)
    return SkelarmSimulator(skeleton)


def _press(canvas, target: tuple[float, float]) -> None:  # noqa: ANN001
    """Dispatch a left-button press at the pixel mapping back to ``target`` (world)."""
    from PyQt6.QtCore import QEvent, QPointF, Qt
    from PyQt6.QtGui import QMouseEvent

    canvas.resize(400, 400)
    px = canvas.width() / 2 + target[0] * canvas.scale_factor
    py = canvas.height() / 2 - target[1] * canvas.scale_factor
    canvas.mousePressEvent(
        QMouseEvent(
            QEvent.Type.MouseButtonPress,
            QPointF(px, py),
            Qt.MouseButton.LeftButton,
            Qt.MouseButton.LeftButton,
            Qt.KeyboardModifier.NoModifier,
        )
    )


def test_external_force_is_zero_without_drag(qapp) -> None:  # noqa: ANN001, ARG001
    """With no active drag the tip force is exactly zero."""
    sim = _simulator()
    assert sim.canvas.external_force(5.0) == pytest.approx(np.zeros(2))


def test_left_press_applies_spring_force_toward_cursor(qapp) -> None:  # noqa: ANN001, ARG001
    """A left press makes the tip force point toward the cursor with magnitude k*distance."""
    sim = _simulator()
    tip = sim.skeleton.links[-1]
    target = (tip.xe + 0.2, tip.ye + 0.1)
    _press(sim.canvas, target)

    expected = 2.0 * (np.array(target) - np.array([tip.xe, tip.ye]))
    assert sim.canvas.external_force(2.0) == pytest.approx(expected, abs=1e-3)


def test_step_pulls_tip_toward_drag_point(qapp) -> None:  # noqa: ANN001, ARG001
    """Stepping under a tip force moves the tip closer to the drag point and advances time."""
    sim = _simulator()
    tip = sim.skeleton.links[-1]
    target = np.array([tip.xe - 0.3, tip.ye + 0.2])
    _press(sim.canvas, (target[0], target[1]))

    start_dist = float(np.hypot(tip.xe - target[0], tip.ye - target[1]))
    min_dist = start_dist
    for _ in range(15):
        sim.step()
        min_dist = min(min_dist, float(np.hypot(tip.xe - target[0], tip.ye - target[1])))

    assert sim.time > 0.0
    assert np.all(np.isfinite(sim.skeleton.q))
    assert min_dist < start_dist


def test_step_keeps_arm_static_without_force(qapp) -> None:  # noqa: ANN001, ARG001
    """At rest with no tip force the pose is unchanged while time still advances."""
    sim = _simulator()
    q0 = sim.skeleton.q.copy()
    for _ in range(10):
        sim.step()

    assert sim.time == pytest.approx(0.2)
    assert sim.skeleton.q == pytest.approx(q0, abs=1e-9)


def test_joint_sliders_are_read_only(qapp) -> None:  # noqa: ANN001, ARG001
    """The sliders display the simulated angles and cannot be dragged by the user."""
    sim = _simulator()
    assert sim.sliders
    assert all(not slider.isEnabled() for slider in sim.sliders)


def test_com_checkbox_toggles_overlay(qapp) -> None:  # noqa: ANN001, ARG001
    """Toggling the checkbox flips the canvas center-of-mass overlay."""
    sim = _simulator()
    assert sim.canvas.show_com is False
    sim.com_checkbox.setChecked(True)
    assert sim.canvas.show_com is True
    sim.com_checkbox.setChecked(False)
    assert sim.canvas.show_com is False


def test_step_respects_joint_limits(qapp) -> None:  # noqa: ANN001, ARG001
    """Joint limits act as hard stops even under a strong, persistent tip force."""
    limit = np.deg2rad(15.0)
    sim = _simulator(qmin=-limit, qmax=limit, q=(0.0, 0.0))
    tip = sim.skeleton.links[-1]
    _press(sim.canvas, (tip.xe + 5.0, tip.ye + 5.0))  # far away -> strong pull into the limits

    for _ in range(100):
        sim.step()

    assert np.all(sim.skeleton.q >= -limit - 1e-9)
    assert np.all(sim.skeleton.q <= limit + 1e-9)
