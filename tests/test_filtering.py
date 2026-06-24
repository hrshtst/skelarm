"""Tests for the from-scratch smoothing filters (src/skelarm/filtering.py)."""

from __future__ import annotations

import numpy as np
import pytest

from skelarm.filtering import butterworth_lowpass, lowpass_first_order, smooth


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
