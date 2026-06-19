"""Tests for the dynamics simulator tool (CLI and the simulator window)."""

from __future__ import annotations

import os

import numpy as np
import pytest

# Importing the tool pulls in PyQt6; run headless.
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from skelarm.skeleton import LinkProp, Skeleton
from tools.dynamics_simulator import DynamicsSimulator, build_parser

pytestmark = pytest.mark.integration


@pytest.fixture(scope="module")
def qapp():  # noqa: ANN201
    """Provide a single QApplication instance for the GUI tests."""
    from PyQt6.QtWidgets import QApplication

    return QApplication.instance() or QApplication([])


def _simulator(stiffness: float | None = None) -> DynamicsSimulator:
    """Build a two-link dynamics simulator at rest."""
    link_props = [LinkProp(length=1.0, m=1.0, i=0.1, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi) for _ in range(2)]
    skeleton = Skeleton(link_props)
    skeleton.q = np.array([0.3, 0.3])
    skeleton.dq = np.zeros(2)
    return DynamicsSimulator(skeleton, stiffness=stiffness)


def test_parser_requires_config() -> None:
    """The config argument is required."""
    with pytest.raises(SystemExit):
        build_parser().parse_args([])


def test_parser_parses_stiffness_and_flags() -> None:
    """The dynamics-specific options are parsed."""
    args = build_parser().parse_args(["robot.toml", "--stiffness", "0.25", "--show-com", "--no-plot"])
    assert args.stiffness == pytest.approx(0.25)
    assert args.show_com is True
    assert args.no_plot is True


def test_stiffness_option_overrides_default(qapp) -> None:  # noqa: ANN001, ARG001
    """An explicit --stiffness flows through to the simulator and its spin box."""
    sim = _simulator(stiffness=0.5)
    assert sim.stiffness == pytest.approx(0.5)
    assert sim.stiffness_spin.value() == pytest.approx(0.5)


def test_stiffness_spin_box_updates_stiffness(qapp) -> None:  # noqa: ANN001, ARG001
    """Changing the spin box live-updates the simulation stiffness."""
    sim = _simulator()
    sim.stiffness_spin.setValue(2.0)
    assert sim.stiffness == pytest.approx(2.0)


def test_trajectory_is_recorded_on_each_step(qapp) -> None:  # noqa: ANN001, ARG001
    """The tip trajectory seeds with the start point and grows one point per step."""
    sim = _simulator()
    xs0, _ = sim.trajectory
    assert len(xs0) == 1  # starting point
    n_steps = 5
    for _ in range(n_steps):
        sim.step()
    xs, ys = sim.trajectory
    assert len(xs) == len(ys) == n_steps + 1  # start + one point per step


def test_reset_clears_trajectory_and_restores_state(qapp) -> None:  # noqa: ANN001, ARG001
    """Reset returns to the initial pose, zeros the clock, and reseeds the trajectory."""
    sim = _simulator()
    q0 = sim.skeleton.q.copy()
    tip = sim.skeleton.links[-1]
    from PyQt6.QtCore import QEvent, QPointF, Qt
    from PyQt6.QtGui import QMouseEvent

    sim.canvas.resize(400, 400)
    target = (tip.xe - 0.3, tip.ye + 0.2)
    px = sim.canvas.width() / 2 + target[0] * sim.canvas.scale_factor
    py = sim.canvas.height() / 2 - target[1] * sim.canvas.scale_factor
    sim.canvas.mousePressEvent(
        QMouseEvent(
            QEvent.Type.MouseButtonPress,
            QPointF(px, py),
            Qt.MouseButton.LeftButton,
            Qt.MouseButton.LeftButton,
            Qt.KeyboardModifier.NoModifier,
        )
    )
    for _ in range(10):
        sim.step()

    sim.reset()
    xs, _ = sim.trajectory
    assert len(xs) == 1
    assert sim.skeleton.q == pytest.approx(q0)
    assert sim.time == pytest.approx(0.0)


def test_pause_button_toggles_running_and_step_button(qapp) -> None:  # noqa: ANN001, ARG001
    """Pausing stops the loop and enables single-stepping; resuming reverses it."""
    sim = _simulator()
    assert sim.running is True
    assert sim.step_button.isEnabled() is False

    sim.pause_button.click()
    assert sim.running is False
    assert sim.pause_button.text() == "Resume"
    assert sim.step_button.isEnabled() is True

    sim.pause_button.click()
    assert sim.running is True
    assert sim.pause_button.text() == "Pause"
    assert sim.step_button.isEnabled() is False


def test_single_step_advances_only_while_paused(qapp) -> None:  # noqa: ANN001, ARG001
    """The step button advances exactly one tick and only when paused."""
    sim = _simulator()
    sim.pause_button.click()  # pause
    t0 = sim.time
    n0 = len(sim.trajectory[0])
    sim.step_button.click()
    assert sim.time > t0
    assert len(sim.trajectory[0]) == n0 + 1


def test_status_label_reports_energy_and_tip(qapp) -> None:  # noqa: ANN001, ARG001
    """The status panel shows tip position, speed, and kinetic energy, and updates."""
    sim = _simulator(stiffness=5.0)
    text0 = sim.status_label.text()
    assert "Tip:" in text0
    assert "Kinetic energy" in text0

    tip = sim.skeleton.links[-1]
    from PyQt6.QtCore import QEvent, QPointF, Qt
    from PyQt6.QtGui import QMouseEvent

    sim.canvas.resize(400, 400)
    target = (tip.xe + 0.3, tip.ye + 0.2)
    px = sim.canvas.width() / 2 + target[0] * sim.canvas.scale_factor
    py = sim.canvas.height() / 2 - target[1] * sim.canvas.scale_factor
    sim.canvas.mousePressEvent(
        QMouseEvent(
            QEvent.Type.MouseButtonPress,
            QPointF(px, py),
            Qt.MouseButton.LeftButton,
            Qt.MouseButton.LeftButton,
            Qt.KeyboardModifier.NoModifier,
        )
    )
    for _ in range(10):
        sim.step()
    assert sim.status_label.text() != text0  # the readout changed as the arm moved
