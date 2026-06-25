"""Tests for the from-scratch smoothing filters (src/skelarm/filtering.py)."""

from __future__ import annotations

import numpy as np
import pytest

from skelarm.filtering import (
    FILTERS,
    butterworth_lowpass,
    lowpass_first_order,
    moving_average,
    savitzky_golay,
    smooth,
)


def _two_tone(dt: float, n: int, f_low: float, f_high: float) -> tuple[np.ndarray, np.ndarray]:
    """A low-frequency signal plus a high-frequency component to be attenuated."""
    t = np.arange(n) * dt
    low = np.sin(2.0 * np.pi * f_low * t)
    high = 0.5 * np.sin(2.0 * np.pi * f_high * t)
    return t, low + high


def test_butterworth_attenuates_high_frequency() -> None:
    """A Butterworth low-pass removes most of a component above the cutoff."""
    dt = 1.0 / 500.0
    t, signal = _two_tone(dt, 2000, f_low=2.0, f_high=80.0)
    low = np.sin(2.0 * np.pi * 2.0 * t)
    out = butterworth_lowpass(signal, dt, cutoff_hz=10.0, order=4)
    # Compare on the interior to avoid edge transients.
    sl = slice(200, -200)
    err_filtered = np.std(out[sl] - low[sl])
    err_raw = np.std(signal[sl] - low[sl])
    assert err_filtered < 0.2 * err_raw


def test_butterworth_is_zero_phase_on_a_low_tone() -> None:
    """Zero-phase filtering leaves an in-band sine essentially un-shifted."""
    dt = 1.0 / 500.0
    t = np.arange(2000) * dt
    low = np.sin(2.0 * np.pi * 2.0 * t)
    out = butterworth_lowpass(low, dt, cutoff_hz=20.0, order=2)
    sl = slice(300, -300)
    assert out[sl] == pytest.approx(low[sl], abs=0.05)  # amplitude kept, no lag


def test_butterworth_preserves_a_constant() -> None:
    """A constant (DC) signal passes through unchanged."""
    out = butterworth_lowpass(np.full(500, 3.0), 0.01, cutoff_hz=5.0, order=3)
    assert out == pytest.approx(np.full(500, 3.0), abs=1e-6)


def test_butterworth_lower_cutoff_smooths_more() -> None:
    """A lower cutoff removes more high-frequency energy."""
    dt = 1.0 / 500.0
    _, signal = _two_tone(dt, 2000, f_low=2.0, f_high=60.0)
    tight = butterworth_lowpass(signal, dt, cutoff_hz=5.0, order=3)
    loose = butterworth_lowpass(signal, dt, cutoff_hz=40.0, order=3)
    sl = slice(200, -200)
    assert np.std(np.diff(tight[sl])) < np.std(np.diff(loose[sl]))


def test_first_order_lowpass_reduces_noise_without_bias() -> None:
    """A first-order low-pass cuts noise variance while preserving a ramp's mean trend."""
    rng = np.random.default_rng(0)
    dt = 0.001
    t = np.arange(3000) * dt
    clean = 2.0 * t  # a slow ramp
    noisy = clean + rng.normal(0.0, 0.1, size=t.shape)
    out = lowpass_first_order(noisy, dt, cutoff_hz=5.0)
    sl = slice(300, -300)
    assert np.std(out[sl] - clean[sl]) < np.std(noisy[sl] - clean[sl])
    assert np.mean(out[sl] - clean[sl]) == pytest.approx(0.0, abs=2e-2)  # zero-phase: no bias/lag


def test_smooth_multicolumn_and_none() -> None:
    """smooth() dispatches by kind, handles multi-column input, and passes through 'none'."""
    dt = 1.0 / 200.0
    t = np.arange(400) * dt
    y = np.stack([np.sin(2.0 * np.pi * 1.0 * t), np.cos(2.0 * np.pi * 1.0 * t)], axis=1)
    out = smooth(y, dt, kind="butterworth", cutoff_hz=10.0, order=2)
    assert out.shape == (400, 2)
    passthrough = smooth(y, dt, kind="none")
    assert passthrough == pytest.approx(y)


def test_smooth_unknown_kind_raises() -> None:
    """An unknown filter kind is reported."""
    with pytest.raises(ValueError, match="unknown filter"):
        smooth(np.zeros(10), 0.01, kind="bogus")


def test_moving_average_preserves_a_constant() -> None:
    """A centered moving average leaves a constant (and the edges) unchanged."""
    out = moving_average(np.full(50, 2.5), window=7)
    assert out == pytest.approx(np.full(50, 2.5), abs=1e-9)


def test_moving_average_preserves_a_line_in_the_interior() -> None:
    """The centered window reproduces a linear ramp away from the edges (zero-phase)."""
    x = np.linspace(0.0, 9.0, 100)  # a line
    out = moving_average(x, window=9)
    sl = slice(10, -10)
    assert out[sl] == pytest.approx(x[sl], abs=1e-9)


def test_moving_average_reduces_noise() -> None:
    """Averaging a noisy ramp cuts the residual variance without biasing the trend."""
    rng = np.random.default_rng(0)
    clean = np.linspace(0.0, 5.0, 600)
    noisy = clean + rng.normal(0.0, 0.2, size=clean.shape)
    out = moving_average(noisy, window=15)
    sl = slice(30, -30)
    assert np.std(out[sl] - clean[sl]) < np.std(noisy[sl] - clean[sl])
    assert np.mean(out[sl] - clean[sl]) == pytest.approx(0.0, abs=2e-2)


def test_moving_average_requires_odd_window() -> None:
    """An even (or non-positive) window is rejected."""
    with pytest.raises(ValueError, match="odd"):
        moving_average(np.zeros(20), window=4)


def test_savgol_reproduces_a_quadratic_in_the_interior() -> None:
    """Savitzky-Golay with polyorder 2 reproduces a quadratic exactly at interior points."""
    z = np.linspace(-3.0, 3.0, 80)
    y = 1.0 + 2.0 * z - 0.5 * z**2
    out = savitzky_golay(y, window=11, polyorder=2)
    sl = slice(8, -8)
    assert out[sl] == pytest.approx(y[sl], abs=1e-9)


def test_savgol_reduces_noise_better_than_it_distorts() -> None:
    """SG smoothing of a noisy sine lands closer to the clean signal in the interior."""
    rng = np.random.default_rng(1)
    t = np.linspace(0.0, 2.0 * np.pi, 500)
    clean = np.sin(t)
    noisy = clean + rng.normal(0.0, 0.15, size=t.shape)
    out = savitzky_golay(noisy, window=21, polyorder=3)
    sl = slice(30, -30)
    assert np.std(out[sl] - clean[sl]) < 0.5 * np.std(noisy[sl] - clean[sl])


def test_savgol_rejects_window_not_above_polyorder() -> None:
    """The window must be odd and larger than the polynomial order."""
    with pytest.raises(ValueError, match=r"polyorder|odd"):
        savitzky_golay(np.zeros(20), window=3, polyorder=4)


def test_smooth_dispatches_moving_average_and_savgol() -> None:
    """smooth() routes the two new kinds (multi-column), and a missing window is reported."""
    assert {"moving_average", "savgol"} <= set(FILTERS)
    y = np.stack([np.linspace(0.0, 1.0, 60), np.linspace(1.0, 0.0, 60)], axis=1)
    assert smooth(y, 0.01, kind="moving_average", window=5).shape == (60, 2)
    assert smooth(y, 0.01, kind="savgol", window=7, polyorder=2).shape == (60, 2)
    with pytest.raises(ValueError, match="window"):
        smooth(y, 0.01, kind="moving_average")
