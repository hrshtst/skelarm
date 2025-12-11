"""Tests for configuration loading."""

from __future__ import annotations

from pathlib import Path

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
    limits = [-3.14, 3.14]

    [[link]]
    length = 0.8
    mass = 1.5
    inertia = 0.3
    rgx = 0.4
    rgy = 0.1
    qmin = -1.57
    qmax = 1.57
    """

    config_file = tmp_path / "robot.toml"
    config_file.write_text(toml_content, encoding="utf-8")

    skeleton = Skeleton.from_toml(config_file)

    expected_num_links = 2
    assert skeleton.num_links == expected_num_links

    # Check first link
    link1 = skeleton.links[0]
    assert link1.prop.length == pytest.approx(1.0)
    assert link1.prop.m == pytest.approx(2.0)
    assert link1.prop.i == pytest.approx(0.5)
    assert link1.prop.rgx == pytest.approx(0.5)
    assert link1.prop.rgy == pytest.approx(0.0)
    assert link1.prop.qmin == pytest.approx(-3.14)
    assert link1.prop.qmax == pytest.approx(3.14)

    # Check second link
    link2 = skeleton.links[1]
    assert link2.prop.length == pytest.approx(0.8)
    assert link2.prop.m == pytest.approx(1.5)
    assert link2.prop.i == pytest.approx(0.3)
    assert link2.prop.rgx == pytest.approx(0.4)
    assert link2.prop.rgy == pytest.approx(0.1)
    assert link2.prop.qmin == pytest.approx(-1.57)
    assert link2.prop.qmax == pytest.approx(1.57)
