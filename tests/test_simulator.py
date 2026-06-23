"""Tests for the real-time simulator widgets (skelarm.simulator)."""

from __future__ import annotations

import os

import numpy as np
import pytest

# Run Qt without a display so the test works headless (CI and local).
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from skelarm.dynamics import compute_kinetic_energy
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


def test_grab_radius_blocks_a_far_press(qapp) -> None:  # noqa: ANN001, ARG001
    """With a grab radius set, a press far from the tip does not start a drag."""
    sim = _simulator()
    sim.canvas.grab_radius = 0.1
    tip = sim.skeleton.links[-1]
    _press(sim.canvas, (tip.xe + 0.5, tip.ye))
    assert sim.canvas.drag_point is None


def test_grab_radius_allows_a_near_press(qapp) -> None:  # noqa: ANN001, ARG001
    """A press within the grab radius of the tip starts a drag."""
    sim = _simulator()
    sim.canvas.grab_radius = 0.1
    tip = sim.skeleton.links[-1]
    _press(sim.canvas, (tip.xe + 0.02, tip.ye))
    assert sim.canvas.drag_point is not None


def test_drag_point_is_settable(qapp) -> None:  # noqa: ANN001, ARG001
    """The drag point can be driven programmatically (scripting/tests)."""
    sim = _simulator()
    assert sim.canvas.drag_point is None
    sim.canvas.drag_point = (0.4, 0.5)
    assert sim.canvas.drag_point == pytest.approx((0.4, 0.5))
    sim.canvas.drag_point = None
    assert sim.canvas.drag_point is None


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


def test_add_control_inserts_widget_before_the_stretch(qapp) -> None:  # noqa: ANN001, ARG001
    """Subclasses can append controls, which land just above the trailing stretch."""
    from PyQt6.QtWidgets import QLabel

    sim = _simulator()
    widget = QLabel("extra")
    sim.add_control(widget)
    layout = sim.controls_layout
    assert layout.indexOf(widget) == layout.count() - 2  # last item before the stretch


def test_stiffness_property_round_trips(qapp) -> None:  # noqa: ANN001, ARG001
    """The drag stiffness is publicly readable and writable."""
    sim = _simulator()
    sim.stiffness = 3.5
    assert sim.stiffness == pytest.approx(3.5)


def test_pause_and_resume_toggle_the_loop(qapp) -> None:  # noqa: ANN001, ARG001
    """The simulation runs after construction; pause/resume flip the running state."""
    sim = _simulator()
    assert sim.running is True
    sim.pause()
    assert sim.running is False
    sim.resume()
    assert sim.running is True


def test_reset_restores_initial_pose_velocity_and_clock(qapp) -> None:  # noqa: ANN001, ARG001
    """Reset returns the arm to its initial pose, zeros velocity, and zeros the clock."""
    sim = _simulator()
    q0 = sim.skeleton.q.copy()
    tip = sim.skeleton.links[-1]
    _press(sim.canvas, (tip.xe - 0.3, tip.ye + 0.2))
    for _ in range(10):
        sim.step()
    assert not np.allclose(sim.skeleton.q, q0)  # the arm moved

    sim.reset()
    assert sim.skeleton.q == pytest.approx(q0)
    assert sim.skeleton.dq == pytest.approx(np.zeros_like(sim.skeleton.dq))
    assert sim.time == pytest.approx(0.0)


def test_friction_property_round_trips_and_defaults_to_zero(qapp) -> None:  # noqa: ANN001, ARG001
    """Viscous friction is zero by default and is publicly readable/writable."""
    sim = _simulator()
    assert sim.friction == pytest.approx(0.0)
    sim.friction = 0.4
    assert sim.friction == pytest.approx(0.4)


def test_viscous_friction_dissipates_kinetic_energy(qapp) -> None:  # noqa: ANN001, ARG001
    """With positive friction and no external force, kinetic energy decays."""
    sim = _simulator()
    sim.skeleton.dq = np.array([1.0, -0.8])  # set the arm in motion
    sim.friction = 0.5
    energy_start = compute_kinetic_energy(sim.skeleton)
    for _ in range(50):
        sim.step()
    assert compute_kinetic_energy(sim.skeleton) < energy_start


def test_zero_friction_retains_more_energy_than_positive_friction(qapp) -> None:  # noqa: ANN001, ARG001
    """Zero friction (the default) conserves energy where positive friction dissipates it."""

    def final_energy(friction: float) -> float:
        sim = _simulator()
        sim.skeleton.dq = np.array([1.0, -0.8])
        sim.friction = friction
        for _ in range(50):
            sim.step()
        return compute_kinetic_energy(sim.skeleton)

    assert final_energy(0.5) < final_energy(0.0)


def test_recording_is_off_by_default(qapp) -> None:  # noqa: ANN001, ARG001
    """No state log is created unless recording is explicitly started."""
    sim = _simulator()
    assert sim.is_recording is False
    assert sim.state_log is None
    sim.step()
    assert sim.state_log is None


def test_start_recording_captures_initial_frame_and_each_step(qapp) -> None:  # noqa: ANN001, ARG001
    """Recording seeds an initial frame and appends one frame per step."""
    sim = _simulator()
    sim.start_recording()
    assert sim.is_recording is True
    assert sim.state_log is not None
    assert len(sim.state_log) == 1  # initial frame
    steps = 3
    for _ in range(steps):
        sim.step()
    assert len(sim.state_log) == steps + 1
    assert set(sim.state_log.channel_names) == {"q", "dq", "tau", "ext_force"}
    assert sim.state_log.channel("q").shape == (steps + 1, 2)


def test_stop_recording_halts_capture(qapp) -> None:  # noqa: ANN001, ARG001
    """After stop_recording, further steps are not logged."""
    sim = _simulator()
    sim.start_recording()
    sim.step()
    assert sim.state_log is not None
    count = len(sim.state_log)
    sim.stop_recording()
    assert sim.is_recording is False
    sim.step()
    assert len(sim.state_log) == count


def test_reset_restarts_active_recording(qapp) -> None:  # noqa: ANN001, ARG001
    """Resetting while recording starts a fresh log seeded at t = 0."""
    sim = _simulator()
    sim.start_recording()
    for _ in range(3):
        sim.step()
    sim.reset()
    assert sim.is_recording is True
    assert sim.state_log is not None
    assert len(sim.state_log) == 1
    assert sim.state_log.times[0] == pytest.approx(0.0)
