"""From-scratch smoothing filters for noisy reference trajectories.

A hand-taught or coarsely sampled reference is often jagged; differentiating it for a
velocity/acceleration reference then amplifies the noise. These low-pass filters
smooth a **uniformly sampled** series before resampling/differentiation. They are
implemented without SciPy:

- ``lowpass_first_order`` — a one-pole RC/exponential filter.
- ``butterworth_lowpass`` — an order-``n`` Butterworth filter built from its analog
  prototype poles via the bilinear transform.

Both are applied **zero-phase** (forward then backward) so the smoothed signal is not
time-shifted, and both are seeded with steady-state initial conditions so a constant
(DC) signal passes through unchanged. A series may be 1-D ``(N,)`` or multi-column
``(N, k)`` (filtered column-wise).

See ``docs/reference/09_trajectory_filtering.md`` for the theory.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from numpy.typing import ArrayLike, NDArray

FILTERS = ("none", "lowpass", "butterworth", "moving_average", "savgol")
"""The supported filter kinds for :func:`smooth`."""


def smooth(
    values: ArrayLike,
    dt: float,
    *,
    kind: str = "none",
    cutoff_hz: float | None = None,
    order: int = 2,
    window: int | None = None,
    polyorder: int = 2,
) -> NDArray[np.float64]:
    """Smooth a uniformly sampled series by the named filter ``kind``.

    Parameters
    ----------
    values : ArrayLike
        The series, shape ``(N,)`` or ``(N, k)``.
    dt : float
        Sample period in seconds (the series must be uniformly sampled).
    kind : str, optional
        One of :data:`FILTERS`. ``"none"`` returns the series unchanged.
    cutoff_hz : float | None, optional
        Cutoff frequency in Hz; required for ``"lowpass"`` and ``"butterworth"``.
    order : int, optional
        Butterworth order (ignored otherwise); keep small (``<= 4``).
    window : int | None, optional
        Window length in samples (odd); required for ``"moving_average"`` and ``"savgol"``.
    polyorder : int, optional
        Savitzky-Golay polynomial order (``"savgol"`` only); must be ``< window``.

    Returns
    -------
    NDArray[np.float64]
        The smoothed series, same shape as ``values``.

    Raises
    ------
    ValueError
        If ``kind`` is unknown or a parameter required by the chosen kind is missing.
    """
    if kind not in FILTERS:
        msg = f"unknown filter kind {kind!r}; choose from {FILTERS}"
        raise ValueError(msg)
    arr = np.asarray(values, dtype=np.float64)
    if kind == "none":
        return arr.copy()
    if kind in {"lowpass", "butterworth"}:
        if cutoff_hz is None:
            msg = f"filter kind {kind!r} requires a cutoff_hz"
            raise ValueError(msg)
        if kind == "lowpass":
            return lowpass_first_order(arr, dt, cutoff_hz=cutoff_hz)
        return butterworth_lowpass(arr, dt, cutoff_hz=cutoff_hz, order=order)
    if window is None:
        msg = f"filter kind {kind!r} requires a window (in samples)"
        raise ValueError(msg)
    if kind == "moving_average":
        return moving_average(arr, window=window)
    return savitzky_golay(arr, window=window, polyorder=polyorder)


def lowpass_first_order(values: ArrayLike, dt: float, *, cutoff_hz: float) -> NDArray[np.float64]:
    """Zero-phase one-pole (RC) low-pass filter.

    Applies the exponential recursion ``y[i] = y[i-1] + a (x[i] - y[i-1])`` forward and
    backward, with ``a = dt / (RC + dt)`` and ``RC = 1 / (2 pi cutoff_hz)``. Seeded at
    the boundary sample so a constant input is preserved.

    Parameters
    ----------
    values : ArrayLike
        Series of shape ``(N,)`` or ``(N, k)``.
    dt : float
        Sample period (s).
    cutoff_hz : float
        Cutoff frequency (Hz).

    Returns
    -------
    NDArray[np.float64]
        The smoothed series, same shape as ``values``.
    """
    rc = 1.0 / (2.0 * np.pi * cutoff_hz)
    alpha = dt / (rc + dt)

    def pass1(x: NDArray[np.float64]) -> NDArray[np.float64]:
        y = np.empty_like(x)
        y[0] = x[0]
        for i in range(1, x.shape[0]):
            y[i] = y[i - 1] + alpha * (x[i] - y[i - 1])
        return y

    def both(x: NDArray[np.float64]) -> NDArray[np.float64]:
        return pass1(pass1(x)[::-1])[::-1]

    return _apply_per_column(both, values)


def butterworth_lowpass(
    values: ArrayLike,
    dt: float,
    *,
    cutoff_hz: float,
    order: int = 2,
) -> NDArray[np.float64]:
    """Zero-phase Butterworth low-pass filter of the given order.

    Designs the digital filter from the analog Butterworth poles (bilinear transform
    with frequency pre-warping) and applies it forward-and-backward (``filtfilt``-style)
    with reflected edge padding and steady-state initial conditions.

    Parameters
    ----------
    values : ArrayLike
        Series of shape ``(N,)`` or ``(N, k)``.
    dt : float
        Sample period (s).
    cutoff_hz : float
        Cutoff frequency (Hz); must be below the Nyquist ``1 / (2 dt)``.
    order : int, optional
        Filter order (``>= 1``); keep small (``<= 4``).

    Returns
    -------
    NDArray[np.float64]
        The smoothed series, same shape as ``values``.

    Raises
    ------
    ValueError
        If ``order < 1`` or ``cutoff_hz`` is not below the Nyquist frequency.
    """
    if order < 1:
        msg = f"order must be >= 1, got {order}"
        raise ValueError(msg)
    nyquist = 0.5 / dt
    if not 0.0 < cutoff_hz < nyquist:
        msg = f"cutoff_hz must be in (0, {nyquist}) (the Nyquist frequency), got {cutoff_hz}"
        raise ValueError(msg)
    b, a = _butter_lowpass_coeffs(cutoff_hz, dt, order)
    return _apply_per_column(lambda x: _filtfilt(b, a, x), values)


def moving_average(values: ArrayLike, *, window: int) -> NDArray[np.float64]:
    """Centered (zero-phase) moving-average filter over an odd ``window`` of samples.

    Each output is the mean of the ``window`` samples centered on it; a symmetric window
    introduces no phase shift and reproduces constants and linear trends in the interior.
    The signal is reflected at the boundaries so the window stays full near the edges.

    Parameters
    ----------
    values : ArrayLike
        Series of shape ``(N,)`` or ``(N, k)``.
    window : int
        Window length in samples; must be odd and positive.

    Returns
    -------
    NDArray[np.float64]
        The smoothed series, same shape as ``values``.

    Raises
    ------
    ValueError
        If ``window`` is not a positive odd integer.
    """
    half = _validate_window(window, polyorder=None, n=np.asarray(values).shape[0])
    kernel = np.full(2 * half + 1, 1.0 / (2 * half + 1))
    return _apply_per_column(lambda x: _correlate_centered(x, kernel, half), values)


def savitzky_golay(values: ArrayLike, *, window: int, polyorder: int = 2) -> NDArray[np.float64]:
    """Zero-phase Savitzky-Golay filter: a least-squares polynomial fit over a sliding window.

    Each output is the value at the center of a degree-``polyorder`` polynomial fit to the
    ``window`` samples around it. The symmetric kernel is phase-free and reproduces
    polynomials up to ``polyorder`` exactly in the interior, so it smooths noise while
    preserving peaks better than a plain average. The signal is reflected at the boundaries.

    Parameters
    ----------
    values : ArrayLike
        Series of shape ``(N,)`` or ``(N, k)``.
    window : int
        Window length in samples; must be odd and greater than ``polyorder``.
    polyorder : int, optional
        Polynomial order fit in each window.

    Returns
    -------
    NDArray[np.float64]
        The smoothed series, same shape as ``values``.

    Raises
    ------
    ValueError
        If ``window`` is not odd or not greater than ``polyorder``.
    """
    half = _validate_window(window, polyorder=polyorder, n=np.asarray(values).shape[0])
    offsets = np.arange(-half, half + 1, dtype=np.float64)
    design = np.vander(offsets, polyorder + 1, increasing=True)  # columns z^0 .. z^polyorder
    kernel = np.linalg.pinv(design)[0]  # value at the window center; a symmetric kernel
    return _apply_per_column(lambda x: _correlate_centered(x, kernel, half), values)


def _validate_window(window: int, *, polyorder: int | None, n: int) -> int:
    """Validate an odd window (optionally above ``polyorder``) and return its half-width."""
    if window < 1 or window % 2 == 0:
        msg = f"window must be a positive odd number of samples, got {window}"
        raise ValueError(msg)
    if polyorder is not None and window <= polyorder:
        msg = f"window ({window}) must be greater than polyorder ({polyorder})"
        raise ValueError(msg)
    return min(window, n if n % 2 == 1 else n - 1) // 2  # clamp to the series length


def _correlate_centered(x: NDArray[np.float64], kernel: NDArray[np.float64], half: int) -> NDArray[np.float64]:
    """Correlate a 1-D signal with a centered, symmetric kernel using reflected edges."""
    if half == 0:
        return x.astype(np.float64)
    pre = x[half:0:-1]
    post = x[-2 : -half - 2 : -1]
    extended = np.concatenate([pre, x, post])
    return np.convolve(extended, kernel, mode="valid")  # symmetric kernel: correlation == convolution


def _butter_lowpass_coeffs(cutoff_hz: float, dt: float, order: int) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Digital ``(b, a)`` coefficients of an order-``n`` Butterworth low-pass filter."""
    # Pre-warp the digital cutoff to the analog domain for the bilinear transform.
    warped = (2.0 / dt) * np.tan(np.pi * cutoff_hz * dt)
    # Analog Butterworth poles: equally spaced on the left half of a circle of radius `warped`.
    k = np.arange(order)
    angles = np.pi * (2.0 * k + 1.0) / (2.0 * order) + np.pi / 2.0
    poles_analog = warped * np.exp(1j * angles)
    # Bilinear transform z = (1 + s dt/2)/(1 - s dt/2); all analog zeros (at infinity) map to z = -1.
    fs2 = 2.0 / dt
    poles_digital = (1.0 + poles_analog / fs2) / (1.0 - poles_analog / fs2)
    a = np.poly(poles_digital).real.astype(np.float64)
    b = np.poly(-np.ones(order)).real.astype(np.float64)  # `order` zeros at z = -1
    b *= np.sum(a) / np.sum(b)  # normalize to unity DC gain (H(z=1) = 1)
    return b, a


def _apply_per_column(
    filt1d: Callable[[NDArray[np.float64]], NDArray[np.float64]],
    values: ArrayLike,
) -> NDArray[np.float64]:
    """Apply a 1-D filter to each column of a ``(N,)`` or ``(N, k)`` series."""
    arr = np.asarray(values, dtype=np.float64)
    if arr.ndim == 1:
        return filt1d(arr)
    return np.stack([filt1d(arr[:, j]) for j in range(arr.shape[1])], axis=1)


def _filtfilt(b: NDArray[np.float64], a: NDArray[np.float64], x: NDArray[np.float64]) -> NDArray[np.float64]:
    """Zero-phase IIR filtering of a 1-D signal with reflected padding (``filtfilt``)."""
    ntaps = max(a.shape[0], b.shape[0])
    padlen = min(3 * (ntaps - 1), x.shape[0] - 1)
    if padlen > 0:
        pre = 2.0 * x[0] - x[padlen:0:-1]
        post = 2.0 * x[-1] - x[-2 : -(padlen + 2) : -1]
        ext = np.concatenate([pre, x, post])
    else:
        ext = x
    zi = _lfilter_zi(b, a)
    forward = _lfilter(b, a, ext, zi * ext[0])
    backward = _lfilter(b, a, forward[::-1], zi * forward[-1])[::-1]
    return backward[padlen : padlen + x.shape[0]]


def _lfilter(
    b: NDArray[np.float64],
    a: NDArray[np.float64],
    x: NDArray[np.float64],
    zi: NDArray[np.float64],
) -> NDArray[np.float64]:
    """One-pass IIR filter (transposed direct form II) with initial state ``zi``."""
    b, a = _pad_coeffs(b, a)
    n = a.shape[0]
    z = zi.astype(np.float64).copy()
    y = np.empty_like(x)
    for i in range(x.shape[0]):
        xi = x[i]
        yi = b[0] * xi + (z[0] if n > 1 else 0.0)
        for k in range(n - 1):
            upper = z[k + 1] if k < n - 2 else 0.0
            z[k] = b[k + 1] * xi + upper - a[k + 1] * yi
        y[i] = yi
    return y


def _lfilter_zi(b: NDArray[np.float64], a: NDArray[np.float64]) -> NDArray[np.float64]:
    """Steady-state initial filter state for a unit-step input (so DC is preserved)."""
    b, a = _pad_coeffs(b, a)
    b = b / a[0]
    a = a / a[0]
    n = a.shape[0]
    if n == 1:
        return np.zeros(0, dtype=np.float64)
    i_minus_a = np.eye(n - 1) - _companion(a).T
    rhs = b[1:] - a[1:] * b[0]
    return np.linalg.solve(i_minus_a, rhs)


def _companion(a: NDArray[np.float64]) -> NDArray[np.float64]:
    """Companion matrix of a monic-normalized coefficient vector ``a``."""
    n = a.shape[0] - 1
    c = np.zeros((n, n), dtype=np.float64)
    c[0, :] = -a[1:] / a[0]
    if n > 1:
        c[1:, :-1] = np.eye(n - 1)
    return c


def _pad_coeffs(b: NDArray[np.float64], a: NDArray[np.float64]) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Right-pad ``b`` and ``a`` with zeros to a common length."""
    n = max(a.shape[0], b.shape[0])
    b_p = np.concatenate([b, np.zeros(n - b.shape[0])]) if b.shape[0] < n else b
    a_p = np.concatenate([a, np.zeros(n - a.shape[0])]) if a.shape[0] < n else a
    return b_p, a_p
