"""Tests for the kinematics inspector tool (CLI and the inspector viewer)."""

from __future__ import annotations

import os
from pathlib import Path

import numpy as np
import pytest

# Importing the tool pulls in PyQt6; run headless.
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from skelarm.skeleton import LinkProp, Skeleton
from tools.kinematics_inspector import KinematicsInspector, build_parser, load_skeleton

pytestmark = pytest.mark.integration


@pytest.fixture(scope="module")
def qapp():  # noqa: ANN201
    """Provide a single QApplication instance for the GUI tests."""
    from PyQt6.QtWidgets import QApplication

    return QApplication.instance() or QApplication([])


def _inspector(num_links: int) -> KinematicsInspector:
    """Build a KinematicsInspector for a uniform arm, seeded at a non-singular pose."""
    link_props = [
        LinkProp(length=1.0, m=1.0, i=0.1, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi) for _ in range(num_links)
    ]
    skeleton = Skeleton(link_props)
    skeleton.q = np.full(num_links, 0.3)  # bent (non-singular) seed
    return KinematicsInspector(skeleton)


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
