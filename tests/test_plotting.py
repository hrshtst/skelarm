"""Tests for the plotting helpers."""

from __future__ import annotations

import os

# Use a non-interactive backend so the tests run headless (CI and local).
os.environ.setdefault("MPLBACKEND", "Agg")

import matplotlib.pyplot as plt
import numpy as np
import pytest
from matplotlib.colors import to_rgba
from matplotlib.patches import Circle

from skelarm.plotting import draw_skeleton, draw_target, plot_trajectory
from skelarm.skeleton import LinkProp, Skeleton


def _arm(num_links: int = 1, base_length: float = 0.0) -> Skeleton:
    """Build a simple unit-link arm for plotting tests."""
    props = [LinkProp(length=1.0, m=1.0, i=0.1, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi) for _ in range(num_links)]
    return Skeleton(props, base_length=base_length)


def test_axes_fit_data_not_symmetric_about_origin() -> None:
    """An arm offset to +x is framed to its bounding box, not a symmetric origin box."""
    skeleton = _arm(num_links=1, base_length=2.0)
    skeleton.q = np.array([0.0])  # arm lies along +x from (0, 0) to (3, 0)

    fig, ax = plt.subplots()
    try:
        draw_skeleton(ax, skeleton)
        xlim = ax.get_xlim()
    finally:
        plt.close(fig)

    tip_x = 3.0
    lower_bound = -1.0
    assert xlim[1] >= tip_x  # includes the tip
    assert xlim[0] > lower_bound  # not the old symmetric box reaching ~ -3.3


def test_overlaid_trajectory_is_not_clipped() -> None:
    """Drawing the skeleton must not fix limits that clip a later trajectory."""
    skeleton = _arm(num_links=1)
    skeleton.q = np.array([0.0])

    far = 5.0
    fig, ax = plt.subplots()
    try:
        draw_skeleton(ax, skeleton)
        plot_trajectory(ax, np.array([0.0, far]), np.array([0.0, far]))
        xlim = ax.get_xlim()
        ylim = ax.get_ylim()
    finally:
        plt.close(fig)

    assert xlim[1] >= far
    assert ylim[1] >= far


def test_degenerate_skeleton_has_valid_limits() -> None:
    """A zero-extent skeleton must still yield a non-degenerate axis range."""
    skeleton = _arm(num_links=0, base_length=0.0)  # only the base link, at the origin

    fig, ax = plt.subplots()
    try:
        draw_skeleton(ax, skeleton)
        xlim = ax.get_xlim()
        ylim = ax.get_ylim()
    finally:
        plt.close(fig)

    assert xlim[0] < xlim[1]
    assert ylim[0] < ylim[1]


def test_plot_trajectory_custom_label_and_title() -> None:
    """plot_trajectory honors a caller-supplied legend label and title."""
    fig, ax = plt.subplots()
    try:
        plot_trajectory(ax, np.array([0.0, 1.0]), np.array([0.0, 1.0]), label="path", title="My Title")
        legend = ax.get_legend()
        assert legend is not None
        labels = [t.get_text() for t in legend.get_texts()]
        title = ax.get_title()
    finally:
        plt.close(fig)

    assert labels == ["path"]
    assert title == "My Title"


def test_draw_skeleton_label_adds_single_legend_entry() -> None:
    """A labeled skeleton contributes exactly one legend entry, not one per link."""
    fig, ax = plt.subplots()
    try:
        draw_skeleton(ax, _arm(num_links=2), label="Arm")
        legend = ax.get_legend()
        assert legend is not None
        labels = [t.get_text() for t in legend.get_texts()]
    finally:
        plt.close(fig)

    assert labels == ["Arm"]


def test_base_link_drawn_distinctly_from_movable_links() -> None:
    """The fixed base link is drawn in a distinct color from the movable links."""
    fig, ax = plt.subplots()
    try:
        draw_skeleton(ax, _arm(num_links=1, base_length=1.0), color="blue", base_color="gray")
        base_color = ax.lines[0].get_color()
        movable_color = ax.lines[1].get_color()
    finally:
        plt.close(fig)

    assert base_color == "gray"
    assert movable_color == "blue"


def test_joints_render_above_link_lines() -> None:
    """Joint/origin markers sit in dedicated layers above the (markerless) link lines."""
    fig, ax = plt.subplots()
    try:
        draw_skeleton(ax, _arm(num_links=2, base_length=1.0))
        link_lines = [ln for ln in ax.lines if ln.get_linestyle() == "-"]
        marker_layers = [ln for ln in ax.lines if ln.get_linestyle() == "None"]
    finally:
        plt.close(fig)

    assert marker_layers  # at least one marker layer
    assert all(ml.get_marker() == "o" for ml in marker_layers)
    # Every marker layer is drawn above every link line, and the lines carry no markers.
    assert all(ml.get_zorder() > ln.get_zorder() for ml in marker_layers for ln in link_lines)
    assert all(ln.get_marker() in ("", "None") for ln in link_lines)


def test_colors_match_the_gui_canvas() -> None:
    """Default link/joint/origin colors mirror the GUI canvas."""
    from skelarm.plotting import _BASE_LINK_COLOR, _JOINT_COLOR, _LINK_COLOR, _ORIGIN_COLOR

    fig, ax = plt.subplots()
    try:
        draw_skeleton(ax, _arm(num_links=2, base_length=1.0))
        link_colors = [ln.get_color() for ln in ax.lines if ln.get_linestyle() == "-"]
        marker_colors = {ln.get_color() for ln in ax.lines if ln.get_linestyle() == "None"}
    finally:
        plt.close(fig)

    assert link_colors[0] == _BASE_LINK_COLOR  # base link gray
    assert link_colors[1] == _LINK_COLOR  # movable links blue
    assert marker_colors == {_JOINT_COLOR, _ORIGIN_COLOR}  # green joints, black origin


def test_show_com_draws_larger_markers_at_link_coms() -> None:
    """show_com adds a green marker, larger than the joints, at each movable link's COM."""
    from skelarm.plotting import _COM_COLOR, _COM_MARKER_SIZE, _JOINT_MARKER_SIZE

    skeleton = _arm(num_links=2, base_length=1.0)
    skeleton.q = np.array([0.3, 0.4])

    fig, ax = plt.subplots()
    try:
        draw_skeleton(ax, skeleton, show_com=True)
        com_layers = [ln for ln in ax.lines if ln.get_linestyle() == "None" and ln.get_markersize() == _COM_MARKER_SIZE]
        assert len(com_layers) == 1
        com = com_layers[0]
        com_x, com_y = com.get_data()
    finally:
        plt.close(fig)

    assert com.get_color() == _COM_COLOR
    assert com.get_markersize() > _JOINT_MARKER_SIZE
    assert list(np.asarray(com_x)) == pytest.approx([link.xg for link in skeleton.links[1:]])
    assert list(np.asarray(com_y)) == pytest.approx([link.yg for link in skeleton.links[1:]])


def test_com_not_drawn_by_default() -> None:
    """Without show_com, no COM markers are drawn."""
    from skelarm.plotting import _COM_MARKER_SIZE

    fig, ax = plt.subplots()
    try:
        draw_skeleton(ax, _arm(num_links=2, base_length=1.0))
        com_layers = [ln for ln in ax.lines if ln.get_linestyle() == "None" and ln.get_markersize() == _COM_MARKER_SIZE]
    finally:
        plt.close(fig)

    assert com_layers == []


def test_titles_compose_without_clobbering() -> None:
    """Passing title=None to one helper lets the other own the axes title."""
    fig, ax = plt.subplots()
    try:
        draw_skeleton(ax, _arm(num_links=1), title=None)
        plot_trajectory(ax, np.array([0.0, 1.0]), np.array([0.0, 1.0]), title="Arm and path")
        title = ax.get_title()
    finally:
        plt.close(fig)

    assert title == "Arm and path"


def _dot_and_rings(ax) -> tuple[list, list]:  # noqa: ANN001
    """Split a target's artists into filled-dot markers and ring outlines (markers + patches)."""
    dots = [ln for ln in ax.lines if ln.get_marker() == "o" and to_rgba(ln.get_markerfacecolor())[3] > 0]
    marker_rings = [ln for ln in ax.lines if ln.get_marker() == "o" and to_rgba(ln.get_markerfacecolor())[3] == 0]
    circle_rings = [p for p in ax.patches if isinstance(p, Circle)]
    return dots, marker_rings + circle_rings


def test_draw_target_default_color_matches_the_player() -> None:
    """The target color defaults to the GUI/player purple, not matplotlib's '*' default."""
    from skelarm.plotting import _GOAL_COLOR

    assert _GOAL_COLOR == "purple"  # matches scenario._DEFAULT_TARGET_COLOR and the canvas overlay
    fig, ax = plt.subplots()
    try:
        draw_target(ax, np.array([1.0, 0.5]))
        dots, _ = _dot_and_rings(ax)
        assert len(dots) == 1
        assert to_rgba(dots[0].get_markerfacecolor()) == to_rgba("purple")
        # No '*' star markers — the player draws a round dot, not a star.
        assert all(ln.get_marker() != "*" for ln in ax.lines)
    finally:
        plt.close(fig)


def test_draw_target_active_with_tolerance_is_dot_plus_data_radius_ring() -> None:
    """An active target with a tolerance is a filled dot inside a hollow ring of that radius."""
    fig, ax = plt.subplots()
    try:
        draw_target(ax, np.array([1.0, 0.5]), tolerance=0.2, color="red")
        dots, rings = _dot_and_rings(ax)
        assert len(dots) == 1
        assert list(dots[0].get_data()[0]) == pytest.approx([1.0])
        assert list(dots[0].get_data()[1]) == pytest.approx([0.5])
        circles = [r for r in rings if isinstance(r, Circle)]
        assert len(circles) == 1
        circle = circles[0]
        assert circle.get_radius() == pytest.approx(0.2)  # ring sized to the success tolerance
        assert circle.center == pytest.approx((1.0, 0.5))
        assert not circle.get_fill()  # hollow ring
        assert to_rgba(circle.get_edgecolor()) == to_rgba("red")
    finally:
        plt.close(fig)


def test_draw_target_active_without_tolerance_uses_a_fixed_ring_marker() -> None:
    """Without a tolerance there is no data-space circle; a fixed-size ring marker is used."""
    fig, ax = plt.subplots()
    try:
        draw_target(ax, np.array([0.0, 0.0]))
        dots, rings = _dot_and_rings(ax)
        assert len(dots) == 1  # the filled center dot
        assert not any(isinstance(r, Circle) for r in rings)  # no data-space tolerance circle
        marker_rings = [r for r in rings if not isinstance(r, Circle)]
        assert len(marker_rings) == 1  # one hollow ring marker
        assert marker_rings[0].get_markersize() > dots[0].get_markersize()  # ring larger than the dot
    finally:
        plt.close(fig)


def test_draw_target_inactive_is_a_dashed_ring_without_a_dot() -> None:
    """An inactive multi-target candidate is a dashed hollow ring with no filled center dot."""
    fig, ax = plt.subplots()
    try:
        draw_target(ax, np.array([1.0, 0.5]), tolerance=0.2, active=False)
        dots, rings = _dot_and_rings(ax)
        assert dots == []  # no fill for an inactive candidate
        circles = [r for r in rings if isinstance(r, Circle)]
        assert len(circles) == 1
        assert circles[0].get_linestyle() in ("--", "dashed")  # dashed, like the canvas
        assert not circles[0].get_fill()
    finally:
        plt.close(fig)


def test_draw_target_label_appears_in_the_legend() -> None:
    """A target label is added once so the legend shows a single 'target' entry."""
    fig, ax = plt.subplots()
    try:
        draw_target(ax, np.array([1.0, 0.5]), tolerance=0.2, label="target")
        labels = [text.get_text() for text in ax.legend().get_texts()]
    finally:
        plt.close(fig)

    assert labels.count("target") == 1
