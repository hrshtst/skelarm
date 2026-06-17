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
