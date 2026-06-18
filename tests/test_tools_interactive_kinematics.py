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


def _write_two_joint_config(path: Path) -> None:
    """Write a minimal two-joint robot config to ``path``."""
    path.write_text(
        "[skeleton]\n"
        "[[skeleton.link]]\nlength = 1.0\nmass = 1.0\ninertia = 0.1\ncom = [0.5, 0.0]\nlimits = [-180.0, 180.0]\n"
        "[[skeleton.link]]\nlength = 1.0\nmass = 1.0\ninertia = 0.1\ncom = [0.5, 0.0]\nlimits = [-180.0, 180.0]\n",
        encoding="utf-8",
    )


def test_parser_requires_config() -> None:
    """The config argument is required."""
    with pytest.raises(SystemExit):
        build_parser().parse_args([])


def test_parser_pose_and_initial_are_mutually_exclusive() -> None:
    """--pose and --initial cannot be combined."""
    with pytest.raises(SystemExit):
        build_parser().parse_args(["robot.toml", "--pose", "1,2", "--initial", "init.toml"])


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
