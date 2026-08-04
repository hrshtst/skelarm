# Copyright (C) 2025-2026 Hiroshi Atsuta <atsuta@ieee.org>
# SPDX-License-Identifier: GPL-3.0-only

"""Demonstrate the trajectory-smoothing filters on synthetic and hand-taught data.

Two demonstrations of every filter in ``skelarm.filtering`` (first-order low-pass,
Butterworth, moving average, Savitzky-Golay — all zero-phase):

1. **Artificial noisy data** — a known smooth reference corrupted by seeded white
   noise, so each filter's error against the ground truth is measurable; the RMSE
   is aggregated over several noise seeds (mean shown in the legend, mean +/- std
   printed).
2. **A taught trajectory** — the hand-demonstrated recording committed at
   ``docs/assets/teach.sklog.npz``: the taught tip path with each filter
   applied, next to a zoomed view of the slow final approach where the raw
   pointer quantization is visible. The smoothness gain (tip acceleration RMS
   reduction) and the deviation from the demonstrated path are printed per
   filter.

The window-based filters are specified by a window length in seconds so their
nominal time scale matches the 4 Hz IIR filters on both datasets despite the
differing sample rates. The final responses still differ by kind: the zero-phase
forward-backward application squares each magnitude response, pulling the
effective -3 dB cutoff below the configured value (furthest for the first-order
low-pass). The comparison is of representative settings matched by nominal
cutoff / window time scale, not a strict ranking.

Run from the repository root:

    uv run python examples/filtering_demo.py
"""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

import matplotlib.pyplot as plt
import numpy as np

from skelarm import StateLog, smooth

if TYPE_CHECKING:
    from matplotlib.axes import Axes
    from numpy.typing import NDArray

_TEACH_LOG = Path(__file__).resolve().parents[1] / "docs" / "assets" / "teach.sklog.npz"

_ZOOM_WINDOW = (2.3, 3.3)  # synthetic zoom bounds (s), around one peak and descent
_TIP_ZOOM_FROM_S = 6.0  # taught tip-path zoom starts at the slow final approach
_SEEDS = range(10)  # synthetic noise realizations aggregated into the RMSE statistics

# The demonstrated filters. Window lengths are given in SECONDS (``window_s``) and
# converted to an odd sample count per dataset, so every filter keeps the same
# nominal ~4 Hz time scale at either sample rate — a fixed sample count would
# remove twice the bandwidth at 50 Hz that it removes at 100 Hz. (The zero-phase
# double pass still shifts each kind's final -3 dB point differently.)
_FILTERS: dict[str, dict[str, float | int | str]] = {
    "lowpass": {"kind": "lowpass", "cutoff_hz": 4.0},
    "butterworth": {"kind": "butterworth", "cutoff_hz": 4.0, "order": 4},
    "moving_average": {"kind": "moving_average", "window_s": 0.10},
    "savgol": {"kind": "savgol", "window_s": 0.26, "polyorder": 3},
}


def _resolve(params: dict[str, float | int | str], dt: float) -> tuple[dict[str, float | int | str], str]:
    """Turn a filter spec into ``smooth()`` keyword arguments and a display label.

    ``window_s`` (seconds) becomes an odd ``window`` (samples) for this dataset's
    sample period, keeping the effective cutoff sample-rate-independent.
    """
    kwargs = dict(params)
    window_s = kwargs.pop("window_s", None)
    if window_s is not None:
        window = round(float(window_s) / dt)
        kwargs["window"] = window + 1 if window % 2 == 0 else window
    kind = str(kwargs["kind"])
    if kind == "lowpass":
        label = f"lowpass ({kwargs['cutoff_hz']:g} Hz)"
    elif kind == "butterworth":
        label = f"butterworth ({kwargs['cutoff_hz']:g} Hz, order {kwargs['order']})"
    elif kind == "moving_average":
        label = f"moving_average ({kwargs['window']})"
    else:
        label = f"savgol ({kwargs['window']}, poly {kwargs['polyorder']})"
    return kwargs, label


def _rms(values: NDArray[np.float64]) -> float:
    """Root-mean-square of a series."""
    return float(np.sqrt(np.mean(np.square(values))))


def _acceleration_rms(values: NDArray[np.float64], dt: float) -> float:
    """RMS of the finite-difference acceleration magnitude — a roughness measure."""
    accel = np.diff(values, n=2, axis=0) / dt**2
    if accel.ndim > 1:
        accel = np.linalg.norm(accel, axis=1)
    return _rms(accel)


def _demo_synthetic(ax_full: Axes, ax_zoom: Axes) -> None:
    """Filter a known signal under seeded noise; aggregate the RMSE over noise seeds."""
    dt = 0.01  # 100 Hz
    times = np.arange(0.0, 5.0, dt)
    truth = 0.8 * np.sin(2.0 * np.pi * 0.5 * times) + 0.25 * np.sin(2.0 * np.pi * 1.2 * times)
    realizations = [truth + np.random.default_rng(seed).normal(0.0, 0.08, times.shape) for seed in _SEEDS]
    shown = realizations[0]  # one realization is plotted; the statistics cover them all

    print(f"Synthetic signal (RMSE against the ground truth, mean +/- std over {len(realizations)} noise seeds):")
    raw_rmse = [_rms(noisy - truth) for noisy in realizations]
    print(f"  {'unfiltered':<28} {np.mean(raw_rmse):.4f} +/- {np.std(raw_rmse):.4f}")
    for ax in (ax_full, ax_zoom):
        ax.plot(times, shown, color="0.8", lw=0.8, label="noisy input")
        ax.plot(times, truth, "k--", lw=1.2, label="ground truth")
    for params in _FILTERS.values():
        kwargs, label = _resolve(params, dt)
        rmse = [_rms(smooth(noisy, dt, **kwargs) - truth) for noisy in realizations]  # type: ignore[arg-type]
        print(f"  {label:<28} {np.mean(rmse):.4f} +/- {np.std(rmse):.4f}")
        for ax in (ax_full, ax_zoom):
            smoothed = smooth(shown, dt, **kwargs)  # type: ignore[arg-type]
            ax.plot(times, smoothed, lw=1.2, label=f"{label}, RMSE {np.mean(rmse):.3f}")

    ax_full.set_title("Artificial noisy signal (one of the noise realizations)")
    ax_full.set_xlabel("time (s)")
    ax_full.set_ylabel("value")
    ax_full.legend(fontsize=7, loc="lower left")
    window = (times >= _ZOOM_WINDOW[0]) & (times <= _ZOOM_WINDOW[1])
    ax_zoom.set_xlim(*_ZOOM_WINDOW)
    ax_zoom.set_ylim(float(shown[window].min()) - 0.05, float(shown[window].max()) + 0.05)
    ax_zoom.set_title("Zoom: attenuation and shape/ripple differences")
    ax_zoom.set_xlabel("time (s)")


def _demo_taught(ax_path: Axes, ax_zoom: Axes) -> None:
    """Filter the committed hand-taught tip path and report the smoothness gain."""
    from matplotlib.patches import Rectangle

    log = StateLog.load(_TEACH_LOG)
    times = log.times
    dt = float(np.mean(np.diff(times)))
    tip = log.channel("tip")

    print(f"\nTaught trajectory {_TEACH_LOG.name} (tip path, {len(times)} samples at {1 / dt:.0f} Hz):")
    print(f"  {'unfiltered':<28} tip accel RMS {_acceleration_rms(tip, dt):7.3f} m/s^2")
    for ax in (ax_path, ax_zoom):
        ax.plot(tip[:, 0], tip[:, 1], color="0.7", lw=0.9, label="taught tip path (raw)")
    for params in _FILTERS.values():
        kwargs, label = _resolve(params, dt)
        smoothed = smooth(tip, dt, **kwargs)  # type: ignore[arg-type]
        accel = _acceleration_rms(smoothed, dt)
        deviation = _rms(np.linalg.norm(smoothed - tip, axis=1)) * 1000.0
        print(f"  {label:<28} tip accel RMS {accel:7.3f} m/s^2, deviation RMS {deviation:.2f} mm")
        ax_path.plot(smoothed[:, 0], smoothed[:, 1], lw=1.2, label=f"{label}, accel RMS {accel:.2f}")
        ax_zoom.plot(smoothed[:, 0], smoothed[:, 1], lw=1.2)

    # Zoom bounds: the slow final approach, where the millimeter-scale jitter is visible.
    segment = tip[times >= _TIP_ZOOM_FROM_S]
    x_lo, x_hi = float(segment[:, 0].min()) - 0.005, float(segment[:, 0].max()) + 0.005
    y_lo, y_hi = float(segment[:, 1].min()) - 0.003, float(segment[:, 1].max()) + 0.003

    ax_path.add_patch(Rectangle((x_lo, y_lo), x_hi - x_lo, y_hi - y_lo, fill=False, ec="k", lw=0.8, ls=":"))
    ax_path.set_title("Taught tip path (dotted box: zoom region)")
    ax_path.set_xlabel("x (m)")
    ax_path.set_ylabel("y (m)")
    ax_path.set_aspect("equal", adjustable="datalim")
    ax_path.legend(fontsize=7, loc="best")

    ax_zoom.set_xlim(x_lo, x_hi)
    ax_zoom.set_ylim(y_lo, y_hi)
    ax_zoom.set_title("Zoom: final approach, raw pointer jitter vs smoothed")
    ax_zoom.set_xlabel("x (m)")
    ax_zoom.set_ylabel("y (m)")


def main() -> None:
    """Run both demonstrations and show the summary figure."""
    fig, axes = plt.subplots(2, 2, figsize=(12.5, 9))
    _demo_synthetic(axes[0, 0], axes[0, 1])
    _demo_taught(axes[1, 0], axes[1, 1])
    fig.suptitle("skelarm trajectory filters: synthetic comparison and hand-taught data")
    fig.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
