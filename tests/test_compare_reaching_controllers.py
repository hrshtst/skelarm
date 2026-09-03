# Copyright (C) 2025-2026 Hiroshi Atsuta <atsuta@ieee.org>
# SPDX-License-Identifier: GPL-3.0-only

"""Tests for the reaching-controller trajectory comparison example."""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pytest

from examples.compare_reaching_controllers import load_comparison, plot_comparison
from skelarm import StateLog


def _save_log(path: Path, times: np.ndarray, q: np.ndarray, q_ref: np.ndarray) -> None:
    """Write the channels needed by the comparison example."""
    log = StateLog()
    for time, angles, reference in zip(times, q, q_ref, strict=True):
        log.record(float(time), q=angles, q_ref=reference)
    log.save(path)


def test_load_comparison_returns_aligned_joint_series(tmp_path: Path) -> None:
    """Two logs with a shared reference load as one comparison dataset."""
    times = np.array([0.0, 0.1, 0.2])
    reference = np.array([[0.0, 0.1], [0.2, 0.3], [0.4, 0.5]])
    joint_pd = reference + 0.02
    computed_torque = reference - 0.01
    pd_path = tmp_path / "pd.sklog.npz"
    ct_path = tmp_path / "ct.sklog.npz"
    _save_log(pd_path, times, joint_pd, reference)
    _save_log(ct_path, times, computed_torque, reference)

    loaded_times, loaded_reference, loaded_pd, loaded_ct = load_comparison(pd_path, ct_path)

    np.testing.assert_array_equal(loaded_times, times)
    np.testing.assert_array_equal(loaded_reference, reference)
    np.testing.assert_array_equal(loaded_pd, joint_pd)
    np.testing.assert_array_equal(loaded_ct, computed_torque)


def test_load_comparison_rejects_different_references(tmp_path: Path) -> None:
    """The plot cannot call two recorded trajectories a common reference when they differ."""
    times = np.array([0.0, 0.1])
    q = np.zeros((2, 2))
    pd_path = tmp_path / "pd.sklog.npz"
    ct_path = tmp_path / "ct.sklog.npz"
    _save_log(pd_path, times, q, q)
    _save_log(ct_path, times, q, q + 0.1)

    with pytest.raises(ValueError, match="common reference"):
        load_comparison(pd_path, ct_path)


def test_plot_comparison_draws_degrees_for_each_joint() -> None:
    """Each joint gets a time-series panel with the reference and both controllers."""
    times = np.array([0.0, 0.1])
    reference = np.array([[0.0, np.pi / 2], [np.pi, -np.pi / 2]])

    figure, axes = plot_comparison(times, reference, reference + 0.1, reference - 0.1)

    assert len(axes) == reference.shape[1]
    assert [line.get_label() for line in axes[0].lines] == ["reference", "joint PD", "computed torque"]
    plotted_reference = np.asarray(axes[1].lines[0].get_ydata(), dtype=np.float64)
    np.testing.assert_allclose(plotted_reference, [90.0, -90.0])
    assert axes[-1].get_xlabel() == "time (s)"
    plt.close(figure)
