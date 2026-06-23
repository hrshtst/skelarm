"""Tests for the dynamics simulator tool (CLI and the simulator window)."""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

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


def _simulator(stiffness: float | None = None, friction: float = 0.0) -> DynamicsSimulator:
    """Build a two-link dynamics simulator at rest."""
    link_props = [LinkProp(length=1.0, m=1.0, i=0.1, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi) for _ in range(2)]
    skeleton = Skeleton(link_props)
    skeleton.q = np.array([0.3, 0.3])
    skeleton.dq = np.zeros(2)
    return DynamicsSimulator(skeleton, stiffness=stiffness, friction=friction)


def test_parser_requires_config() -> None:
    """The config argument is required."""
    with pytest.raises(SystemExit):
        build_parser().parse_args([])


def test_parser_parses_stiffness_friction_and_flags() -> None:
    """The dynamics-specific options are parsed."""
    args = build_parser().parse_args(
        ["robot.toml", "--stiffness", "0.25", "--friction", "0.3", "--show-com", "--no-plot"]
    )
    assert args.stiffness == pytest.approx(0.25)
    assert args.friction == pytest.approx(0.3)
    assert args.show_com is True
    assert args.no_plot is True


def test_friction_defaults_to_zero() -> None:
    """Friction is frictionless by default."""
    args = build_parser().parse_args(["robot.toml"])
    assert args.friction == pytest.approx(0.0)


def test_no_joint_limits_flag_defaults_off_and_parses() -> None:
    """Joint limits are enforced unless ``--no-joint-limits`` is given."""
    assert build_parser().parse_args(["robot.toml"]).no_joint_limits is False
    assert build_parser().parse_args(["robot.toml", "--no-joint-limits"]).no_joint_limits is True


def test_runs_as_a_standalone_script() -> None:
    """Running the file directly (script mode) must resolve all of its imports.

    Regression test: ``python tools/dynamics_simulator.py`` only puts the script's
    own directory on ``sys.path`` (not the repo root), so an import of the ``tools``
    package fails even though it resolves fine under pytest's ``pythonpath``.
    """
    script = Path(__file__).resolve().parents[1] / "tools" / "dynamics_simulator.py"
    result = subprocess.run(  # noqa: S603  # trusted: our own interpreter and script path
        [sys.executable, str(script), "--help"],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    assert "dynamics" in result.stdout.lower()


def test_stiffness_option_overrides_default(qapp) -> None:  # noqa: ANN001, ARG001
    """An explicit --stiffness flows through to the simulator."""
    sim = _simulator(stiffness=0.5)
    assert sim.stiffness == pytest.approx(0.5)


def test_friction_option_seeds_simulator_and_spin_box(qapp) -> None:  # noqa: ANN001, ARG001
    """An explicit --friction flows through to the simulator and its spin box."""
    sim = _simulator(friction=0.3)
    assert sim.friction == pytest.approx(0.3)
    assert sim.friction_spin.value() == pytest.approx(0.3)


def test_friction_spin_box_updates_friction(qapp) -> None:  # noqa: ANN001, ARG001
    """Changing the spin box live-updates the joint viscous friction."""
    sim = _simulator()
    assert sim.friction == pytest.approx(0.0)  # frictionless by default
    sim.friction_spin.setValue(0.5)
    assert sim.friction == pytest.approx(0.5)


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


def test_records_state_by_default(qapp) -> None:  # noqa: ANN001, ARG001
    """The tool records states from the start; stepping appends frames."""
    sim = _simulator()
    assert sim.is_recording is True
    assert sim.state_log is not None
    n0 = len(sim.state_log)
    for _ in range(3):
        sim.step()
    assert len(sim.state_log) == n0 + 3


def test_record_checkbox_toggles_recording(qapp) -> None:  # noqa: ANN001, ARG001
    """Unchecking the record box stops capture; rechecking restarts it."""
    sim = _simulator()
    sim.record_checkbox.setChecked(False)
    assert sim.is_recording is False
    sim.record_checkbox.setChecked(True)
    assert sim.is_recording is True


def test_export_writes_loadable_npz(qapp, tmp_path) -> None:  # noqa: ANN001, ARG001
    """Export produces a .sklog.npz that loads back with the robot and channels."""
    from skelarm import StateLog

    sim = _simulator()
    for _ in range(4):
        sim.step()
    path = tmp_path / "run.sklog.npz"
    sim.export(path)

    assert sim.state_log is not None
    loaded = StateLog.load(path)
    assert loaded.producer == "skelarm_simulator"
    assert len(loaded) == len(sim.state_log)
    assert set(loaded.channel_names) == {"q", "dq", "tau", "ext_force"}
    assert loaded.build_skeleton().num_joints == sim.skeleton.num_joints


def test_export_toml_round_trips(qapp, tmp_path) -> None:  # noqa: ANN001, ARG001
    """Export to a .toml path writes a parseable TOML document."""
    import tomllib

    sim = _simulator()
    sim.step()
    path = tmp_path / "run.toml"
    sim.export(path)
    data = tomllib.loads(path.read_text(encoding="utf-8"))
    assert data["producer"] == "skelarm_simulator"
    assert "q" in data["data"]
