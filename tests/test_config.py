"""Tests for configuration loading."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from skelarm.skeleton import Skeleton


def test_load_skeleton_from_toml(tmp_path: Path) -> None:
    """Test loading a Skeleton from a TOML file."""
    toml_content = """
    [[link]]
    length = 1.0
    mass = 2.0
    inertia = 0.5
    com = [0.5, 0.0]
    limits = [-180.0, 180.0]

    [[link]]
    length = 0.8
    mass = 1.5
    inertia = 0.3
    rgx = 0.4
    rgy = 0.1
    qmin = -90.0
    qmax = 90.0
    """

    config_file = tmp_path / "robot.toml"
    config_file.write_text(toml_content, encoding="utf-8")

    skeleton = Skeleton.from_toml(config_file)

    expected_num_joints = 2
    assert skeleton.num_joints == expected_num_joints

    # Check first movable link (links[0] is the fixed base link)
    link1 = skeleton.links[1]
    assert link1.prop.length == pytest.approx(1.0)
    assert link1.prop.m == pytest.approx(2.0)
    assert link1.prop.i == pytest.approx(0.5)
    assert link1.prop.rgx == pytest.approx(0.5)
    assert link1.prop.rgy == pytest.approx(0.0)
    assert link1.prop.qmin == pytest.approx(-np.pi)
    assert link1.prop.qmax == pytest.approx(np.pi)

    # Check second movable link
    link2 = skeleton.links[2]
    assert link2.prop.length == pytest.approx(0.8)
    assert link2.prop.m == pytest.approx(1.5)
    assert link2.prop.i == pytest.approx(0.3)
    assert link2.prop.rgx == pytest.approx(0.4)
    assert link2.prop.rgy == pytest.approx(0.1)
    assert link2.prop.qmin == pytest.approx(-np.pi / 2)
    assert link2.prop.qmax == pytest.approx(np.pi / 2)


def test_load_skeleton_with_initial_angles(tmp_path: Path) -> None:
    """A per-link q0 (degrees) sets the initial joint angle; omitting it defaults to zero."""
    toml_content = """
    [[link]]
    length = 1.0
    mass = 2.0
    inertia = 0.5
    com = [0.5, 0.0]
    limits = [-180.0, 180.0]
    q0 = 30.0

    [[link]]
    length = 0.8
    mass = 1.5
    inertia = 0.3
    com = [0.4, 0.0]
    limits = [-90.0, 90.0]
    """

    config_file = tmp_path / "robot.toml"
    config_file.write_text(toml_content, encoding="utf-8")

    skeleton = Skeleton.from_toml(config_file)

    assert skeleton.q == pytest.approx(np.array([np.deg2rad(30.0), 0.0]))


def test_load_skeleton_initial_pose_reflected_in_link_positions(tmp_path: Path) -> None:
    """from_toml must return a skeleton whose link positions already match the q0 pose."""
    toml_content = """
    [[link]]
    length = 1.0
    mass = 1.0
    inertia = 0.1
    com = [0.5, 0.0]
    limits = [-180.0, 180.0]
    q0 = 90.0
    """

    config_file = tmp_path / "robot.toml"
    config_file.write_text(toml_content, encoding="utf-8")

    skeleton = Skeleton.from_toml(config_file)

    tip = skeleton.links[-1]
    assert tip.q_absolute == pytest.approx(np.pi / 2)
    assert tip.xe == pytest.approx(0.0, abs=1e-12)
    assert tip.ye == pytest.approx(1.0)


def test_load_skeleton_with_base_length(tmp_path: Path) -> None:
    """A top-level base_length is loaded as the fixed base (zeroth) link."""
    toml_content = """
    base_length = 0.5

    [[link]]
    length = 1.0
    mass = 1.0
    inertia = 0.1
    com = [0.5, 0.0]
    limits = [-180.0, 180.0]
    """

    config_file = tmp_path / "robot.toml"
    config_file.write_text(toml_content, encoding="utf-8")

    skeleton = Skeleton.from_toml(config_file)

    assert skeleton.num_joints == 1
    assert skeleton.base_length == pytest.approx(0.5)
    assert skeleton.links[0].prop.length == pytest.approx(0.5)


def test_omitted_limits_default_to_plus_minus_180_degrees(tmp_path: Path) -> None:
    """A link without limits defaults to [-180, 180] degrees (an enforced full-revolution cap)."""
    toml_content = """
    [skeleton]

    [[skeleton.link]]
    length = 1.0
    mass = 1.0
    inertia = 0.1
    com = [0.5, 0.0]
    """

    config_file = tmp_path / "robot.toml"
    config_file.write_text(toml_content, encoding="utf-8")

    skeleton = Skeleton.from_toml(config_file)

    assert skeleton.links[1].prop.qmin == pytest.approx(-np.pi)
    assert skeleton.links[1].prop.qmax == pytest.approx(np.pi)


def test_load_skeleton_from_nested_skeleton_section(tmp_path: Path) -> None:
    """A ``[skeleton]`` section with ``[[skeleton.link]]`` is the canonical layout."""
    toml_content = """
    [skeleton]
    base_length = 0.5

    [[skeleton.link]]
    length = 1.0
    mass = 2.0
    inertia = 0.5
    com = [0.5, 0.0]
    limits = [-180.0, 180.0]
    q0 = 30.0

    [[skeleton.link]]
    length = 0.8
    mass = 1.5
    inertia = 0.3
    com = [0.4, 0.0]
    limits = [-90.0, 90.0]
    """

    config_file = tmp_path / "robot.toml"
    config_file.write_text(toml_content, encoding="utf-8")

    skeleton = Skeleton.from_toml(config_file)

    expected_num_joints = 2
    assert skeleton.num_joints == expected_num_joints
    assert skeleton.base_length == pytest.approx(0.5)
    assert skeleton.links[1].prop.length == pytest.approx(1.0)
    assert skeleton.links[2].prop.length == pytest.approx(0.8)
    assert skeleton.q == pytest.approx(np.array([np.deg2rad(30.0), 0.0]))


def test_load_skeleton_ignores_sibling_sections_in_combined_file(tmp_path: Path) -> None:
    """A combined file with ``[task]``/``[controller]`` siblings loads only ``[skeleton]``."""
    toml_content = """
    [skeleton]
    base_length = 0.3

    [[skeleton.link]]
    length = 1.0
    mass = 1.0
    inertia = 0.1
    com = [0.5, 0.0]
    limits = [-180.0, 180.0]

    [task]
    type = "reach"
    target = [1.2, 0.4]

    [controller]
    type = "pd"
    kp = 10.0
    kd = 1.0
    """

    config_file = tmp_path / "experiment.toml"
    config_file.write_text(toml_content, encoding="utf-8")

    skeleton = Skeleton.from_toml(config_file)

    assert skeleton.num_joints == 1
    assert skeleton.base_length == pytest.approx(0.3)


# === [initial] section: posture/velocity as a run condition ===


def _two_link_skeleton_toml(initial_block: str = "", *, q0_lines: tuple[str, str] = ("", "")) -> str:
    """Build a two-joint skeleton config with an optional ``[initial]`` block."""
    return f"""
    [skeleton]

    [[skeleton.link]]
    length = 1.0
    mass = 1.0
    inertia = 0.1
    com = [0.5, 0.0]
    limits = [-180.0, 180.0]
    {q0_lines[0]}

    [[skeleton.link]]
    length = 0.8
    mass = 0.8
    inertia = 0.05
    com = [0.4, 0.0]
    limits = [-180.0, 180.0]
    {q0_lines[1]}
    {initial_block}
    """


def test_initial_section_sets_posture(tmp_path: Path) -> None:
    """An ``[initial]`` section with ``q`` (degrees) sets the whole posture at once."""
    toml_content = _two_link_skeleton_toml("[initial]\n    q = [30.0, -45.0]")
    config_file = tmp_path / "robot.toml"
    config_file.write_text(toml_content, encoding="utf-8")

    skeleton = Skeleton.from_toml(config_file)

    assert skeleton.q == pytest.approx(np.deg2rad([30.0, -45.0]))


def test_initial_section_overrides_per_link_q0(tmp_path: Path) -> None:
    """``[initial].q`` is a run condition that wins over per-link ``q0`` defaults."""
    toml_content = _two_link_skeleton_toml(
        "[initial]\n    q = [30.0, -45.0]",
        q0_lines=("q0 = 10.0", "q0 = 20.0"),
    )
    config_file = tmp_path / "robot.toml"
    config_file.write_text(toml_content, encoding="utf-8")

    skeleton = Skeleton.from_toml(config_file)

    assert skeleton.q == pytest.approx(np.deg2rad([30.0, -45.0]))


def test_initial_section_sets_velocity(tmp_path: Path) -> None:
    """``[initial].dq`` (deg/s) sets the initial joint velocities."""
    toml_content = _two_link_skeleton_toml("[initial]\n    q = [10.0, 20.0]\n    dq = [5.0, -3.0]")
    config_file = tmp_path / "robot.toml"
    config_file.write_text(toml_content, encoding="utf-8")

    skeleton = Skeleton.from_toml(config_file)

    assert skeleton.q == pytest.approx(np.deg2rad([10.0, 20.0]))
    assert skeleton.dq == pytest.approx(np.deg2rad([5.0, -3.0]))


def test_initial_q_length_mismatch_raises(tmp_path: Path) -> None:
    """A ``q`` whose length differs from the DOF count raises a clear error."""
    toml_content = _two_link_skeleton_toml("[initial]\n    q = [30.0, -45.0, 10.0]")
    config_file = tmp_path / "robot.toml"
    config_file.write_text(toml_content, encoding="utf-8")

    with pytest.raises(ValueError, match=r"\[initial\]\.q"):
        Skeleton.from_toml(config_file)


def test_initial_dq_length_mismatch_raises(tmp_path: Path) -> None:
    """A ``dq`` whose length differs from the DOF count raises a clear error."""
    toml_content = _two_link_skeleton_toml("[initial]\n    dq = [1.0]")
    config_file = tmp_path / "robot.toml"
    config_file.write_text(toml_content, encoding="utf-8")

    with pytest.raises(ValueError, match=r"\[initial\]\.dq"):
        Skeleton.from_toml(config_file)
