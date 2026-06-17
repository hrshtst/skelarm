"""Tests for the plotting helpers."""

from __future__ import annotations

import os

# Use a non-interactive backend so the tests run headless (CI and local).
os.environ.setdefault("MPLBACKEND", "Agg")

import matplotlib.pyplot as plt
import numpy as np

from skelarm.plotting import draw_skeleton, plot_trajectory
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
        draw_skeleton(ax, _arm(num_links=1, base_length=1.0), color="blue")
        base_color = ax.lines[0].get_color()
        movable_color = ax.lines[1].get_color()
    finally:
        plt.close(fig)

    assert base_color == "gray"
    assert movable_color == "blue"


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
