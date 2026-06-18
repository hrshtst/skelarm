"""Tests for the interactive kinematics tool's command-line handling."""

from __future__ import annotations

import os
from pathlib import Path

import numpy as np
import pytest

# Importing the tool pulls in PyQt6; run headless.
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from tools.interactive_kinematics import build_parser, load_skeleton

pytestmark = pytest.mark.integration


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
