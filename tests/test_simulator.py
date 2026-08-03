# Copyright (C) 2025-2026 Hiroshi Atsuta <atsuta@ieee.org>
# SPDX-License-Identifier: GPL-3.0-only

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


def _simulator(qmin: float = -np.pi, qmax: float = np.pi, q: tuple[float, ...] = (0.3, 0.3), **kwargs):  # noqa: ANN202, ANN003
    """Build a two-link simulator at rest in the given pose."""
    from skelarm.simulator import SkelarmSimulator

    link_props = [LinkProp(length=1.0, m=1.0, i=0.1, rgx=0.5, rgy=0.0, qmin=qmin, qmax=qmax) for _ in range(2)]
    skeleton = Skeleton(link_props)
    skeleton.q = np.array(q)
    skeleton.dq = np.zeros(2)
    return SkelarmSimulator(skeleton, **kwargs)


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


def test_dt_parameter_sets_substep_size_and_tick_duration(qapp) -> None:  # noqa: ANN001, ARG001
    """A configured dt sets the physics substep; one render tick advances substeps * dt."""
    sim = _simulator(dt=0.002)
    assert sim.dt == pytest.approx(0.002)
    sim.step()
    assert sim.time == pytest.approx(0.02)  # 10 substeps of 2 ms fill the 20 ms render tick


def test_dt_larger_than_frame_period_runs_one_substep_per_tick(qapp) -> None:  # noqa: ANN001, ARG001
    """A dt above the render period gets one substep per tick and a matching timer interval."""
    sim = _simulator(dt=0.05)
    sim.step()
    assert sim.time == pytest.approx(0.05)
    sim.resume()
    try:
        assert sim._timer.interval() == 50  # noqa: SLF001, PLR2004
    finally:
        sim.pause()


def test_controller_update_runs_once_per_substep_at_dt(qapp) -> None:  # noqa: ANN001, ARG001
    """The controller sees every physics substep at the configured dt."""
    from skelarm.control import Controller

    calls: list[tuple[float, float]] = []

    class _Probe(Controller):
        def update(self, t: float, skeleton, dt: float) -> None:  # noqa: ANN001, ARG002
            calls.append((t, dt))

        def control(self, t: float, skeleton) -> np.ndarray:  # noqa: ANN001, ARG002
            return np.zeros(2)

    sim = _simulator(controller=_Probe(), dt=0.01)
    sim.step()
    assert [dt for _, dt in calls] == pytest.approx([0.01, 0.01])  # 2 substeps per 20 ms tick


def test_non_positive_dt_raises(qapp) -> None:  # noqa: ANN001, ARG001
    """A zero or negative dt is rejected at construction."""
    for bad in (0.0, -0.005):
        with pytest.raises(ValueError, match="dt"):
            _simulator(dt=bad)


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


def test_enforce_limits_false_lets_the_arm_pass_its_joint_limits(qapp) -> None:  # noqa: ANN001, ARG001
    """With ``enforce_limits=False`` the dynamics ignore the configured joint limits."""
    from skelarm.simulator import SkelarmSimulator

    limit = np.deg2rad(15.0)
    link_props = [LinkProp(length=1.0, m=1.0, i=0.1, rgx=0.5, rgy=0.0, qmin=-limit, qmax=limit) for _ in range(2)]
    skeleton = Skeleton(link_props)
    skeleton.q = np.zeros(2)
    skeleton.dq = np.zeros(2)
    sim = SkelarmSimulator(skeleton, enforce_limits=False)
    tip = sim.skeleton.links[-1]
    _press(sim.canvas, (tip.xe + 5.0, tip.ye + 5.0))  # far away -> strong pull past the limits

    for _ in range(100):
        sim.step()

    assert np.any(np.abs(sim.skeleton.q) > limit + 1e-3)  # at least one joint sailed past its bound


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
    """The simulation launches paused; resume/pause flip the running state."""
    sim = _simulator()
    assert sim.running is False
    sim.resume()
    assert sim.running is True
    sim.pause()
    assert sim.running is False


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


def test_recording_embeds_log_extra(qapp) -> None:  # noqa: ANN001, ARG001
    """The ``log_extra`` constructor metadata lands in the recorded log's ``extra``."""
    from skelarm.simulator import SkelarmSimulator

    link_props = [LinkProp(length=1.0, m=1.0, i=0.1, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi) for _ in range(2)]
    skeleton = Skeleton(link_props)
    skeleton.q = np.array([0.2, 0.2])
    sim = SkelarmSimulator(skeleton, log_extra={"source_config": {"task": {"type": "reaching"}}})
    sim.start_recording()
    assert sim.state_log is not None
    assert sim.state_log.extra["source_config"]["task"]["type"] == "reaching"


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
    assert set(sim.state_log.channel_names) == {"q", "dq", "tau", "ext_force", "friction"}
    assert sim.state_log.channel("q").shape == (steps + 1, 2)


def test_recording_includes_the_friction_channel(qapp) -> None:  # noqa: ANN001, ARG001
    """The viscous-friction coefficient is recorded per frame, tracking live changes."""
    sim = _simulator(friction=0.2)
    sim.start_recording()
    sim.step()
    sim.friction = 0.5  # a live change, as from the dynamics tool's spin box
    sim.step()
    assert sim.state_log is not None
    friction = sim.state_log.channel("friction")
    assert friction[0] == pytest.approx(0.2)
    assert friction[-1] == pytest.approx(0.5)


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


def test_control_panel_width_is_fixed_regardless_of_time_text(qapp) -> None:  # noqa: ANN001, ARG001
    """The side panel keeps a constant width even as the time readout grows."""
    from skelarm.simulator import _PANEL_WIDTH_PX

    sim = _simulator()
    panel = sim.controls_panel
    assert panel.minimumWidth() == panel.maximumWidth() == _PANEL_WIDTH_PX

    # A long time readout (the widest control) must not widen the fixed panel.
    sim.time_label.setText("t = 123456.78 s")
    assert panel.minimumWidth() == panel.maximumWidth() == _PANEL_WIDTH_PX


def test_transport_bar_sits_below_the_title(qapp) -> None:  # noqa: ANN001, ARG001
    """The transport bar is the first control, right after the panel title."""
    sim = _simulator()
    assert sim.controls_layout.indexOf(sim.transport_bar) == 1


def test_transport_bar_starts_paused(qapp) -> None:  # noqa: ANN001, ARG001
    """The simulator launches paused: toggle unchecked, tooltip 'Play', step enabled."""
    sim = _simulator()
    assert sim.running is False
    assert not sim.pause_button.isChecked()
    assert sim.pause_button.toolTip() == "Play (Space)"
    assert sim.step_button.isEnabled()


def test_transport_buttons_alias_the_bar(qapp) -> None:  # noqa: ANN001, ARG001
    """The historical button attributes point at the bar's tool buttons."""
    sim = _simulator()
    assert sim.pause_button is sim.transport_bar.play_button
    assert sim.step_button is sim.transport_bar.step_button
    assert sim.reset_button is sim.transport_bar.reset_button


def test_play_button_click_starts_and_stops_the_loop(qapp) -> None:  # noqa: ANN001, ARG001
    """Clicking the toggle starts the loop and disables step; clicking again pauses."""
    sim = _simulator()
    sim.pause_button.click()
    assert sim.running is True
    assert not sim.step_button.isEnabled()
    sim.pause_button.click()
    assert sim.running is False
    assert sim.step_button.isEnabled()


def test_step_button_advances_one_tick_while_paused(qapp) -> None:  # noqa: ANN001, ARG001
    """While paused (as launched), the step button advances the clock by one render tick."""
    sim = _simulator()
    t0 = sim.time
    sim.step_button.click()
    assert sim.time == pytest.approx(t0 + 0.02)


def test_reset_button_pauses_and_restores_pose_and_clock(qapp) -> None:  # noqa: ANN001, ARG001
    """The reset button pauses the loop and restores the initial pose and clock."""
    sim = _simulator()
    q0 = sim.skeleton.q.copy()
    tip = sim.skeleton.links[-1]
    _press(sim.canvas, (tip.xe - 0.3, tip.ye + 0.2))
    for _ in range(10):
        sim.step()
    assert not np.allclose(sim.skeleton.q, q0)

    sim.resume()
    sim.reset_button.click()
    assert sim.running is False
    assert not sim.pause_button.isChecked()
    assert sim.step_button.isEnabled()
    assert sim.skeleton.q == pytest.approx(q0)
    assert sim.time == pytest.approx(0.0)


def test_programmatic_reset_does_not_pause(qapp) -> None:  # noqa: ANN001, ARG001
    """Calling reset() directly restores the state but leaves the loop running."""
    sim = _simulator()
    sim.resume()
    sim.reset()
    assert sim.running is True


def test_programmatic_pause_syncs_the_transport_bar(qapp) -> None:  # noqa: ANN001, ARG001
    """Calling resume()/pause() directly keeps the toggle and step button in sync."""
    sim = _simulator()
    sim.resume()
    assert sim.pause_button.isChecked()
    assert not sim.step_button.isEnabled()
    sim.pause()
    assert not sim.pause_button.isChecked()
    assert sim.step_button.isEnabled()


def test_q_shortcut_closes_the_window(qapp) -> None:  # noqa: ANN001, ARG001
    """Pressing Q closes the simulator window."""
    from PyQt6.QtCore import Qt
    from PyQt6.QtTest import QTest
    from PyQt6.QtWidgets import QApplication

    sim = _simulator()
    assert sim.quit_shortcut.key().toString() == "Q"

    sim.show()
    sim.activateWindow()
    QApplication.processEvents()
    QTest.keyClick(sim, Qt.Key.Key_Q)  # type: ignore[call-overload]  # PyQt6 stubs type QTest methods as bound
    QApplication.processEvents()
    assert not sim.isVisible()
