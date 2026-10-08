# Copyright (C) 2025-2026 Hiroshi Atsuta <atsuta@ieee.org>
# SPDX-License-Identifier: GPL-3.0-only

"""Tests for the kinematics inspector tool (CLI and the inspector viewer)."""

from __future__ import annotations

import os
from pathlib import Path

import numpy as np
import pytest

# Importing the tool pulls in PyQt6; run headless.
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from skelarm import Task
from skelarm.skeleton import LinkProp, Skeleton
from tools.kinematics_inspector import KinematicsInspector, build_parser, load_skeleton, load_task

pytestmark = pytest.mark.integration


@pytest.fixture(scope="module")
def qapp():  # noqa: ANN201
    """Provide a single QApplication instance for the GUI tests."""
    from PyQt6.QtWidgets import QApplication

    return QApplication.instance() or QApplication([])


_REACHING_TASK = '[task]\ntype = "reaching"\ntarget = { pos = [0.5, 1.2], tolerance = 0.02 }\n'
_MULTI_TARGET_TASK = (
    '[task]\ntype = "multi_target_reaching"\nactive = 1\ntargets = [\n'
    '  { pos = [1.2, 0.4], label = "A", tolerance = 0.03 },\n'
    '  { pos = [0.3, 1.2], label = "B", tolerance = 0.03 },\n'
    "]\n"
)


def _inspector(num_links: int, task: Task | None = None) -> KinematicsInspector:
    """Build a KinematicsInspector for a uniform arm, seeded at a non-singular pose."""
    link_props = [
        LinkProp(length=1.0, m=1.0, i=0.1, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi) for _ in range(num_links)
    ]
    skeleton = Skeleton(link_props)
    skeleton.q = np.full(num_links, 0.3)  # bent (non-singular) seed
    return KinematicsInspector(skeleton, task=task)


def _combo_methods(inspector: KinematicsInspector) -> list[str]:
    """List the IK methods offered by the inspector's method combo box."""
    return [inspector.method_combo.itemText(i) for i in range(inspector.method_combo.count())]


def _write_two_joint_config(path: Path, initial_q: tuple[float, float] | None = None) -> None:
    """Write a minimal two-joint robot config, optionally with an [initial] table."""
    text = (
        "[skeleton]\n"
        "[[skeleton.link]]\nlength = 1.0\nmass = 1.0\ninertia = 0.1\ncom = [0.5, 0.0]\nlimits = [-180.0, 180.0]\n"
        "[[skeleton.link]]\nlength = 1.0\nmass = 1.0\ninertia = 0.1\ncom = [0.5, 0.0]\nlimits = [-180.0, 180.0]\n"
    )
    if initial_q is not None:
        text += f"[initial]\nq = [{initial_q[0]}, {initial_q[1]}]\n"
    path.write_text(text, encoding="utf-8")


def test_parser_requires_config() -> None:
    """The config argument is required."""
    with pytest.raises(SystemExit):
        build_parser().parse_args([])


def test_initial_file_overrides_config_initial(tmp_path: Path) -> None:
    """--initial overrides the [initial] table baked into the config."""
    config = tmp_path / "robot.toml"
    _write_two_joint_config(config, initial_q=(5.0, 5.0))
    initial = tmp_path / "init.toml"
    initial.write_text("[initial]\nq = [15.0, 25.0]\n", encoding="utf-8")
    args = build_parser().parse_args([str(config), "--initial", str(initial)])

    skeleton = load_skeleton(args)

    assert skeleton.q == pytest.approx(np.deg2rad([15.0, 25.0]))


def test_pose_overrides_initial(tmp_path: Path) -> None:
    """--pose takes precedence over --initial when both are given."""
    config = tmp_path / "robot.toml"
    _write_two_joint_config(config)
    initial = tmp_path / "init.toml"
    initial.write_text("[initial]\nq = [15.0, 25.0]\n", encoding="utf-8")
    args = build_parser().parse_args([str(config), "--initial", str(initial), "--pose", "30,-45"])

    skeleton = load_skeleton(args)

    assert skeleton.q == pytest.approx(np.deg2rad([30.0, -45.0]))


def test_load_skeleton_applies_pose(tmp_path: Path) -> None:
    """--pose sets the initial joint angles (degrees)."""
    config = tmp_path / "robot.toml"
    _write_two_joint_config(config)
    args = build_parser().parse_args([str(config), "--pose", "30,-45"])

    skeleton = load_skeleton(args)

    assert skeleton.q == pytest.approx(np.deg2rad([30.0, -45.0]))


def test_load_skeleton_pose_length_mismatch_raises(tmp_path: Path) -> None:
    """A --pose with the wrong number of values raises a clear error."""
    config = tmp_path / "robot.toml"
    _write_two_joint_config(config)
    args = build_parser().parse_args([str(config), "--pose", "30,-45,10"])

    with pytest.raises(ValueError, match="--pose"):
        load_skeleton(args)


def test_load_skeleton_applies_initial_file(tmp_path: Path) -> None:
    """--initial applies an [initial] table from a separate TOML file."""
    config = tmp_path / "robot.toml"
    _write_two_joint_config(config)
    initial = tmp_path / "init.toml"
    initial.write_text("[initial]\nq = [15.0, 25.0]\n", encoding="utf-8")
    args = build_parser().parse_args([str(config), "--initial", str(initial)])

    skeleton = load_skeleton(args)

    assert skeleton.q == pytest.approx(np.deg2rad([15.0, 25.0]))


def test_load_skeleton_missing_config_raises(tmp_path: Path) -> None:
    """A missing config path raises FileNotFoundError."""
    args = build_parser().parse_args([str(tmp_path / "nope.toml")])

    with pytest.raises(FileNotFoundError):
        load_skeleton(args)


def test_load_task_reads_the_config_task(tmp_path: Path) -> None:
    """The config's [task] table is the task the inspector draws."""
    config = tmp_path / "robot.toml"
    _write_two_joint_config(config)
    config.write_text(config.read_text(encoding="utf-8") + _REACHING_TASK, encoding="utf-8")

    task = load_task(build_parser().parse_args([str(config)]))

    assert task is not None
    assert task.target == pytest.approx([0.5, 1.2])
    assert task.tolerance == pytest.approx(0.02)


def test_load_task_without_a_task_table_is_none(tmp_path: Path) -> None:
    """A config without a [task] table draws no task."""
    config = tmp_path / "robot.toml"
    _write_two_joint_config(config)

    assert load_task(build_parser().parse_args([str(config)])) is None


def test_task_file_overrides_the_config_task(tmp_path: Path) -> None:
    """--task draws the [task] of a separate file instead of the config's."""
    config = tmp_path / "robot.toml"
    _write_two_joint_config(config)
    config.write_text(config.read_text(encoding="utf-8") + _REACHING_TASK, encoding="utf-8")
    other = tmp_path / "task.toml"
    other.write_text('[task]\ntype = "reaching"\ntarget = { pos = [-0.3, 0.9] }\n', encoding="utf-8")

    task = load_task(build_parser().parse_args([str(config), "--task", str(other)]))

    assert task is not None
    assert task.target == pytest.approx([-0.3, 0.9])


def test_task_file_without_a_task_table_raises(tmp_path: Path) -> None:
    """A --task file must hold a [task] table."""
    config = tmp_path / "robot.toml"
    _write_two_joint_config(config)
    other = tmp_path / "task.toml"
    other.write_text("[initial]\nq = [15.0, 25.0]\n", encoding="utf-8")

    with pytest.raises(ValueError, match=r"\[task\]"):
        load_task(build_parser().parse_args([str(config), "--task", str(other)]))


def test_load_task_applies_the_active_target_of_a_multi_target_task(tmp_path: Path) -> None:
    """A multi-target task's configured active candidate becomes its target, as when a scenario is loaded."""
    config = tmp_path / "robot.toml"
    _write_two_joint_config(config)
    config.write_text(config.read_text(encoding="utf-8") + _MULTI_TARGET_TASK, encoding="utf-8")

    task = load_task(build_parser().parse_args([str(config)]))

    assert task is not None
    assert task.target == pytest.approx([0.3, 1.2])
    assert task.label == "B"
    assert task.tolerance == pytest.approx(0.03)


def test_load_task_rejects_an_active_target_out_of_range(tmp_path: Path) -> None:
    """A multi-target task whose active index names no candidate is rejected."""
    config = tmp_path / "robot.toml"
    _write_two_joint_config(config)
    task_table = _MULTI_TARGET_TASK.replace("active = 1", "active = 2")
    config.write_text(config.read_text(encoding="utf-8") + task_table, encoding="utf-8")

    with pytest.raises(ValueError, match="out of range"):
        load_task(build_parser().parse_args([str(config)]))


def test_missing_task_file_raises(tmp_path: Path) -> None:
    """A missing --task file raises FileNotFoundError."""
    config = tmp_path / "robot.toml"
    _write_two_joint_config(config)

    with pytest.raises(FileNotFoundError):
        load_task(build_parser().parse_args([str(config), "--task", str(tmp_path / "nope.toml")]))


# === KinematicsInspector (the tool's feature-rich viewer) ===


def test_com_checkbox_toggles_canvas_flag(qapp) -> None:  # noqa: ANN001, ARG001
    """The 'show center of mass' checkbox flips the canvas overlay flag."""
    inspector = _inspector(2)

    assert inspector.canvas.show_com is False
    inspector.com_checkbox.setChecked(True)
    assert inspector.canvas.show_com is True
    inspector.com_checkbox.setChecked(False)
    assert inspector.canvas.show_com is False


def test_status_label_reports_ik_result(qapp) -> None:  # noqa: ANN001, ARG001
    """The status label shows the endpoint and IK status after a solve."""
    inspector = _inspector(2)

    inspector.canvas.solve_to_world(0.5, 1.2)

    text = inspector.status_label.text()
    assert "Tip:" in text
    assert "converged" in text


def test_reset_button_restores_initial_pose(qapp) -> None:  # noqa: ANN001, ARG001
    """The reset button returns the arm to its initial pose and clears IK state."""
    inspector = _inspector(2)
    initial = inspector.skeleton.q.copy()

    inspector.canvas.solve_to_world(0.5, 1.2)
    assert not np.allclose(inspector.skeleton.q, initial)

    inspector.reset_button.click()

    assert np.allclose(inspector.skeleton.q, initial)
    assert inspector.canvas.last_ik_result is None


def test_reset_button_keeps_text_and_gains_icon(qapp) -> None:  # noqa: ANN001, ARG001
    """The reset button stays a labeled button but shows a leading icon."""
    inspector = _inspector(2)
    assert inspector.reset_button.text() == "Reset pose"
    assert not inspector.reset_button.icon().isNull()


def test_reset_pose_r_shortcut(qapp) -> None:  # noqa: ANN001, ARG001
    """Pressing R triggers the reset-pose button and restores the initial pose."""
    from PyQt6.QtCore import Qt
    from PyQt6.QtTest import QTest
    from PyQt6.QtWidgets import QApplication

    inspector = _inspector(2)
    shortcut = inspector.reset_button.shortcut()
    assert shortcut is not None
    assert shortcut.toString() == "R"
    assert inspector.reset_button.toolTip() == "Reset pose (R)"

    initial = inspector.skeleton.q.copy()
    inspector.canvas.solve_to_world(0.5, 1.2)
    assert not np.allclose(inspector.skeleton.q, initial)

    inspector.show()
    inspector.activateWindow()
    QApplication.processEvents()
    QTest.keyClick(inspector, Qt.Key.Key_R)  # type: ignore[call-overload]  # PyQt6 stubs type QTest methods as bound
    QTest.qWait(200)  # type: ignore[call-arg, arg-type]  # bound-style stubs; animateClick fires ~100 ms later
    assert np.allclose(inspector.skeleton.q, initial)


def test_method_combo_excludes_nr_for_redundant_arm(qapp) -> None:  # noqa: ANN001, ARG001
    """Newton-Raphson (square-only) is not offered for a redundant arm."""
    methods = _combo_methods(_inspector(3))
    assert "nr" not in methods
    assert "lm_sugihara" in methods


def test_method_combo_includes_nr_for_two_dof(qapp) -> None:  # noqa: ANN001, ARG001
    """Newton-Raphson is offered for a square (two-joint) arm."""
    assert "nr" in _combo_methods(_inspector(2))


def test_selecting_method_routes_to_solver(qapp) -> None:  # noqa: ANN001, ARG001
    """Choosing a method routes it to the canvas solver, which runs and records a result."""
    inspector = _inspector(2)

    inspector.method_combo.setCurrentText("sr_inverse")
    assert inspector.canvas.ik_method == "sr_inverse"

    inspector.canvas.solve_to_world(0.5, 1.2)
    assert inspector.canvas.last_ik_result is not None


def test_q_shortcut_closes_the_window(qapp) -> None:  # noqa: ANN001, ARG001
    """Pressing Q closes the inspector window (inherited from the viewer base)."""
    from PyQt6.QtCore import Qt
    from PyQt6.QtTest import QTest
    from PyQt6.QtWidgets import QApplication

    inspector = _inspector(2)
    assert inspector.quit_shortcut.key().toString() == "Q"

    inspector.show()
    inspector.activateWindow()
    QApplication.processEvents()
    QTest.keyClick(inspector, Qt.Key.Key_Q)  # type: ignore[call-overload]  # PyQt6 stubs type QTest methods as bound
    QApplication.processEvents()
    assert not inspector.isVisible()


def test_the_method_box_never_keeps_the_reset_key(qapp) -> None:  # noqa: ANN001, ARG001
    """The method box is picked with the mouse and never takes the keyboard focus, so R always resets."""
    from PyQt6.QtCore import Qt

    inspector = _inspector(2)

    assert inspector.method_combo.focusPolicy() == Qt.FocusPolicy.NoFocus


def test_the_inspector_draws_the_task_target(qapp) -> None:  # noqa: ANN001, ARG001
    """A reaching task's target is drawn with its tolerance ring, and no path."""
    task = Task.from_dict({"type": "reaching", "target": {"pos": [0.5, 1.2], "tolerance": 0.02}})

    inspector = _inspector(2, task=task)

    ((pos, _, tolerance, active),) = inspector.canvas.overlay_targets
    assert pos == pytest.approx([0.5, 1.2])
    assert tolerance == pytest.approx(0.02)
    assert active
    assert inspector.canvas.overlay_path is None


def test_the_inspector_draws_the_task_path(qapp) -> None:  # noqa: ANN001, ARG001
    """A task with a reference path, such as a periodic curve, has it drawn."""
    task = Task.from_dict(
        {"type": "periodic_curve", "curve": "ellipse", "center": [0.9, 0.0], "a": 0.4, "b": 0.25, "period": 2.0}
    )

    inspector = _inspector(2, task=task)

    path = inspector.canvas.overlay_path
    assert path is not None
    assert path[:, 0].max() == pytest.approx(1.3)  # the center plus the x semi-axis
    assert path[:, 1].max() == pytest.approx(0.25, abs=1e-3)  # the y semi-axis, between two samples


def test_the_inspector_without_a_task_draws_none(qapp) -> None:  # noqa: ANN001, ARG001
    """Without a task, no target or path is drawn."""
    inspector = _inspector(2)

    assert inspector.canvas.overlay_targets == []
    assert inspector.canvas.overlay_path is None


def test_status_label_reports_the_tip_distance_to_the_target(qapp) -> None:  # noqa: ANN001, ARG001
    """The status label shows the target and how far the tip is from it, as the pose changes."""
    task = Task.from_dict({"type": "reaching", "target": {"pos": [0.5, 1.2], "tolerance": 0.02}})
    inspector = _inspector(2, task=task)
    tip = inspector.skeleton.links[-1]
    distance = np.hypot(tip.xe - 0.5, tip.ye - 1.2)

    assert f"Target: (0.500, 1.200) m, {distance:.3f} m from the tip" in inspector.status_label.text()

    inspector.canvas.solve_to_world(0.5, 1.2)

    assert "Target: (0.500, 1.200) m, 0.000 m from the tip" in inspector.status_label.text()


def test_status_label_reports_the_distance_to_the_active_target(qapp, tmp_path: Path) -> None:  # noqa: ANN001, ARG001
    """With a multi-target task, every candidate is drawn and the distance is to the active one."""
    config = tmp_path / "robot.toml"
    _write_two_joint_config(config)
    config.write_text(config.read_text(encoding="utf-8") + _MULTI_TARGET_TASK, encoding="utf-8")
    inspector = _inspector(2, task=load_task(build_parser().parse_args([str(config)])))

    assert [active for _, _, _, active in inspector.canvas.overlay_targets] == [False, True]
    inspector.canvas.solve_to_world(0.3, 1.2)

    assert "Target: (0.300, 1.200) m, 0.000 m from the tip" in inspector.status_label.text()
