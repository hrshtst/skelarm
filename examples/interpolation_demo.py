# Copyright (C) 2025-2026 Hiroshi Atsuta <atsuta@ieee.org>
# SPDX-License-Identifier: GPL-3.0-only

"""Demonstrate the differences between the trajectory interpolators on artificial data.

Compares the three interpolators in ``skelarm.interpolation`` (piecewise linear,
natural cubic spline, barycentric Lagrange) on a known smooth reference sampled
coarsely, so every reconstruction error is measurable against the analytic truth:

1. **Reconstruction** — each interpolant through the same coarse nodes, with the
   absolute error over time (RMSE and maximum error printed).
2. **Derivatives** — the first derivative from ``resample_with_derivatives``
   against the analytic derivative: the cubic spline's is analytic and smooth,
   while linear (piecewise-constant slope) and Lagrange fall back to finite
   differences on the query grid.
3. **Node-count sweep** — the maximum error as the sampling gets denser: linear
   and the spline converge steadily, while the equispaced high-degree Lagrange
   polynomial suffers wild Runge-type edge oscillations at intermediate node
   counts (peaking around 26 absolute error here, orders of magnitude above the
   spline) before eventually converging for this analytic reference — which is
   why it is reserved for short series.

Run from the repository root:

    uv run python examples/interpolation_demo.py
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import matplotlib.pyplot as plt
import numpy as np

from skelarm import INTERPOLATORS, interpolate, resample_with_derivatives

if TYPE_CHECKING:
    from matplotlib.axes import Axes
    from numpy.typing import NDArray

_DURATION = 5.0  # seconds of reference motion
_DEMO_NODES = 21  # coarse samples for the reconstruction/derivative panels
_NODE_COUNTS = (6, 9, 13, 17, 21, 26, 31, 41)  # sweep for the convergence panel
_FINE = np.linspace(0.0, _DURATION, 1001)  # query grid (inside the node range)


def _truth(t: NDArray[np.float64]) -> NDArray[np.float64]:
    """The known smooth reference (same family as the filtering demo)."""
    return 0.8 * np.sin(2.0 * np.pi * 0.5 * t) + 0.25 * np.sin(2.0 * np.pi * 1.2 * t)


def _truth_dot(t: NDArray[np.float64]) -> NDArray[np.float64]:
    """Analytic first derivative of :func:`_truth`."""
    return 0.8 * np.pi * np.cos(2.0 * np.pi * 0.5 * t) + 0.6 * np.pi * np.cos(2.0 * np.pi * 1.2 * t)


def _demo_reconstruction(ax_interp: Axes, ax_error: Axes) -> None:
    """Interpolate coarse samples of the known signal and measure the error."""
    nodes = np.linspace(0.0, _DURATION, _DEMO_NODES)
    samples = _truth(nodes)
    truth = _truth(_FINE)

    print(f"Reconstruction from {_DEMO_NODES} equispaced nodes (error against the analytic truth):")
    ax_interp.plot(_FINE, truth, "k--", lw=1.2, label="truth")
    ax_interp.plot(nodes, samples, "ko", ms=4, label="coarse samples")
    for method in INTERPOLATORS:
        values = interpolate(nodes, samples, _FINE, method=method)
        error = np.abs(values - truth)
        print(f"  {method:<14} RMSE {np.sqrt(np.mean(error**2)):.5f}, max {error.max():.5f}")
        ax_interp.plot(_FINE, values, lw=1.2, label=method)
        ax_error.semilogy(_FINE, np.maximum(error, 1e-12), lw=1.0, label=method)

    ax_interp.set_ylim(-1.35, 1.35)  # the Lagrange edge oscillation (±19) is clipped, not hidden
    ax_interp.set_title(f"Interpolants through {_DEMO_NODES} coarse samples (lagrange clipped)")
    ax_interp.set_xlabel("time (s)")
    ax_interp.set_ylabel("value")
    ax_interp.legend(fontsize=7, loc="lower left")
    ax_error.set_title("Absolute reconstruction error")
    ax_error.set_xlabel("time (s)")
    ax_error.set_ylabel("|error|")
    ax_error.legend(fontsize=7, loc="lower center")


def _demo_derivatives(ax: Axes) -> None:
    """Compare each interpolator's first derivative with the analytic one."""
    nodes = np.linspace(0.0, _DURATION, _DEMO_NODES)
    samples = _truth(nodes)

    ax.plot(_FINE, _truth_dot(_FINE), "k--", lw=1.2, label="analytic derivative")
    for method in INTERPOLATORS:
        _, d_dt, _ = resample_with_derivatives(nodes, samples, _FINE, method=method)
        ax.plot(_FINE, d_dt, lw=1.2, label=method)
    ax.set_ylim(-6.0, 6.0)  # keep the Lagrange edge swings from swamping the panel
    ax.set_title("First derivative (spline: analytic; linear/lagrange: finite differences)")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("d/dt")
    ax.legend(fontsize=7, loc="lower left")


def _demo_node_sweep(ax: Axes) -> None:
    """Show convergence vs the Runge phenomenon as the node count grows."""
    print("\nMaximum error vs node count (equispaced nodes):")
    header = "  nodes " + "".join(f"{method:>14}" for method in INTERPOLATORS)
    print(header)
    errors: dict[str, list[float]] = {method: [] for method in INTERPOLATORS}
    for count in _NODE_COUNTS:
        nodes = np.linspace(0.0, _DURATION, count)
        samples = _truth(nodes)
        truth = _truth(_FINE)
        row = f"  {count:>5} "
        for method in INTERPOLATORS:
            worst = float(np.abs(interpolate(nodes, samples, _FINE, method=method) - truth).max())
            errors[method].append(worst)
            row += f"{worst:>14.3g}"
        print(row)

    for method, err in errors.items():
        ax.semilogy(_NODE_COUNTS, err, "o-", lw=1.2, label=method)
    ax.set_title("Max error vs node count (equispaced nodes)")
    ax.set_xlabel("number of equispaced nodes")
    ax.set_ylabel("max |error|")
    ax.legend(fontsize=7, loc="center left")


def main() -> None:
    """Run all three demonstrations and show the summary figure."""
    fig, axes = plt.subplots(2, 2, figsize=(12.5, 9))
    _demo_reconstruction(axes[0, 0], axes[0, 1])
    _demo_derivatives(axes[1, 0])
    _demo_node_sweep(axes[1, 1])
    fig.suptitle("skelarm trajectory interpolators on a known reference")
    fig.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
