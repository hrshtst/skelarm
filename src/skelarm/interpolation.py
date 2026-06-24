"""From-scratch interpolation for resampling reference trajectories.

A recorded or generated reference trajectory is a finite series of samples
``(t_i, y_i)`` that usually does not line up with the controller's time grid, so it
must be *resampled*. This module implements three schemes by hand (no SciPy):

- ``linear`` — piecewise-linear; cheap and shape-preserving, but only ``C^0``.
- ``cubic_spline`` — the natural cubic spline; ``C^2`` with analytic first and second
  derivatives, the right default for deriving smooth ``q̇`` / ``q̈`` references.
- ``lagrange`` — the barycentric form of the global Lagrange polynomial; exact for
  polynomial data but oscillatory for many nodes (Runge), so reserve it for short
  series.

The series may be a 1-D signal of shape ``(N,)`` or a multi-column signal of shape
``(N, k)`` (e.g. tip ``(x, y)`` or per-joint angles), interpolated column-wise.

See ``docs/reference/09_trajectory_filtering.md`` for the theory.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from numpy.typing import ArrayLike, NDArray

INTERPOLATORS = ("linear", "cubic_spline", "lagrange")
"""The supported interpolation method names (the ``method`` argument)."""


def interpolate(
    times: ArrayLike,
    values: ArrayLike,
    query: ArrayLike,
    *,
    method: str = "cubic_spline",
) -> NDArray[np.float64]:
    """Resample a sampled series onto new query times.

    Parameters
    ----------
    times : ArrayLike
        Strictly increasing sample times, shape ``(N,)``.
    values : ArrayLike
        Sample values, shape ``(N,)`` or ``(N, k)``.
    query : ArrayLike
        Times to evaluate at, a scalar or shape ``(M,)``.
    method : str, optional
        One of :data:`INTERPOLATORS` (default ``"cubic_spline"``).

    Returns
    -------
    NDArray[np.float64]
        The interpolated values: shape ``(M,)`` or ``()`` for a 1-D series, or
        ``(M, k)`` / ``(k,)`` for a multi-column series.

    Raises
    ------
    ValueError
        If ``method`` is unknown or the shapes are inconsistent.
    """
    value, _, _ = _evaluate(times, values, query, method=method, order=0)
    return value


def resample_with_derivatives(
    times: ArrayLike,
    values: ArrayLike,
    query: ArrayLike,
    *,
    method: str = "cubic_spline",
) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
    """Resample a series and return its value, first, and second time derivatives.

    For ``cubic_spline`` the derivatives are analytic; for ``linear`` and ``lagrange``
    they fall back to finite differences of the resampled value on the query grid (so
    pass a reasonably dense, near-uniform ``query`` for those methods).

    Parameters
    ----------
    times, values, query, method
        As in :func:`interpolate`.

    Returns
    -------
    tuple[NDArray, NDArray, NDArray]
        ``(value, d_dt, d2_dt2)``, each with the same shape as :func:`interpolate`
        would return.
    """
    return _evaluate(times, values, query, method=method, order=2)


def _evaluate(
    times: ArrayLike,
    values: ArrayLike,
    query: ArrayLike,
    *,
    method: str,
    order: int,
) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
    """Resample and (when ``order >= 2``) also return first/second derivatives."""
    if method not in INTERPOLATORS:
        msg = f"unknown interpolation method {method!r}; choose from {INTERPOLATORS}"
        raise ValueError(msg)
    t = np.asarray(times, dtype=np.float64)
    y2d, was_1d = _as_columns(values, t.shape[0])
    q = np.atleast_1d(np.asarray(query, dtype=np.float64))
    q_was_scalar = np.ndim(query) == 0

    if method == "cubic_spline":
        value, d1, d2 = _cubic_spline_eval(t, y2d, q)
    else:
        value = _linear_eval(t, y2d, q) if method == "linear" else _lagrange_eval(t, y2d, q)
        d1, d2 = _finite_diff(value, q) if order >= 1 else (value, value)

    return tuple(_restore_shape(arr, was_1d=was_1d, scalar=q_was_scalar) for arr in (value, d1, d2))  # type: ignore[return-value]


def _as_columns(values: ArrayLike, n: int) -> tuple[NDArray[np.float64], bool]:
    """Coerce ``values`` to a 2-D ``(N, k)`` array, reporting whether it was 1-D."""
    arr = np.asarray(values, dtype=np.float64)
    if arr.ndim == 1:
        if arr.shape[0] != n:
            msg = f"values length {arr.shape[0]} does not match times length {n}"
            raise ValueError(msg)
        return arr.reshape(n, 1), True
    if arr.ndim == 2:  # noqa: PLR2004 — a series is 1-D (signal) or 2-D (columns)
        if arr.shape[0] != n:
            msg = f"values rows {arr.shape[0]} do not match times length {n}"
            raise ValueError(msg)
        return arr, False
    msg = f"values must be 1-D or 2-D, got shape {arr.shape}"
    raise ValueError(msg)


def _restore_shape(arr: NDArray[np.float64], *, was_1d: bool, scalar: bool) -> NDArray[np.float64]:
    """Undo the column/scalar coercion for the returned arrays."""
    out = arr[:, 0] if was_1d else arr
    return out[0] if scalar else out


def _linear_eval(t: NDArray[np.float64], y: NDArray[np.float64], q: NDArray[np.float64]) -> NDArray[np.float64]:
    """Piecewise-linear interpolation per column (held at the endpoints)."""
    return np.stack([np.interp(q, t, y[:, j]) for j in range(y.shape[1])], axis=1)


def _lagrange_eval(t: NDArray[np.float64], y: NDArray[np.float64], q: NDArray[np.float64]) -> NDArray[np.float64]:
    """Barycentric Lagrange interpolation per column."""
    diff = t[:, None] - t[None, :]
    np.fill_diagonal(diff, 1.0)
    weights = 1.0 / np.prod(diff, axis=1)  # barycentric weights, shape (n,)

    delta = q[:, None] - t[None, :]  # (M, n)
    exact = np.isclose(delta, 0.0)
    safe = np.where(exact, 1.0, delta)
    coeff = weights[None, :] / safe  # (M, n)
    out = (coeff @ y) / coeff.sum(axis=1)[:, None]  # (M, k)

    # Rows that hit a node exactly take that node's value directly (avoids 0/0).
    hit_rows, hit_cols = np.where(exact)
    out[hit_rows] = y[hit_cols]
    return out


def _cubic_spline_eval(
    t: NDArray[np.float64],
    y: NDArray[np.float64],
    q: NDArray[np.float64],
) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
    """Evaluate the natural cubic spline and its first two derivatives per column."""
    n = t.shape[0]
    h = np.diff(t)  # (n-1,)
    seg = np.clip(np.searchsorted(t, q, side="right") - 1, 0, n - 2)  # segment per query
    hs = h[seg]
    a = (t[seg + 1] - q) / hs  # left weight
    b = (q - t[seg]) / hs  # right weight

    value = np.empty((q.shape[0], y.shape[1]), dtype=np.float64)
    d1 = np.empty_like(value)
    d2 = np.empty_like(value)
    for j in range(y.shape[1]):
        m = _natural_spline_moments(t, y[:, j], h)  # second derivatives at the nodes
        mi, mj = m[seg], m[seg + 1]
        yi, yj = y[seg, j], y[seg + 1, j]
        value[:, j] = a * yi + b * yj + ((a**3 - a) * mi + (b**3 - b) * mj) * hs**2 / 6.0
        d1[:, j] = (yj - yi) / hs - (3.0 * a**2 - 1.0) / 6.0 * hs * mi + (3.0 * b**2 - 1.0) / 6.0 * hs * mj
        d2[:, j] = a * mi + b * mj
    return value, d1, d2


def _natural_spline_moments(
    t: NDArray[np.float64],
    y: NDArray[np.float64],
    h: NDArray[np.float64],
) -> NDArray[np.float64]:
    """Solve for the node second derivatives of a natural cubic spline (Thomas algorithm)."""
    n = t.shape[0]
    m = np.zeros(n, dtype=np.float64)
    if n < 3:  # noqa: PLR2004 — with <3 nodes the natural spline is just the linear segment
        return m
    # Tridiagonal system A m = d for the interior moments m[1..n-2], with m[0]=m[n-1]=0.
    lower = h[:-1]
    upper = h[1:]
    diag = 2.0 * (h[:-1] + h[1:])
    rhs = 6.0 * ((y[2:] - y[1:-1]) / h[1:] - (y[1:-1] - y[:-2]) / h[:-1])

    # Thomas forward sweep / back substitution over the (n-2) interior unknowns.
    cp = np.empty(n - 2, dtype=np.float64)
    dp = np.empty(n - 2, dtype=np.float64)
    cp[0] = upper[0] / diag[0]
    dp[0] = rhs[0] / diag[0]
    for i in range(1, n - 2):
        denom = diag[i] - lower[i] * cp[i - 1]
        cp[i] = upper[i] / denom
        dp[i] = (rhs[i] - lower[i] * dp[i - 1]) / denom
    m[n - 2] = dp[-1]
    for i in range(n - 4, -1, -1):
        m[i + 1] = dp[i] - cp[i] * m[i + 2]
    return m


def _finite_diff(
    value: NDArray[np.float64],
    q: NDArray[np.float64],
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """First and second derivatives of a resampled signal by finite differences on the query grid."""
    if q.shape[0] < 2:  # noqa: PLR2004 — a single query point has no defined derivative
        zeros = np.zeros_like(value)
        return zeros, zeros
    d1 = np.gradient(value, q, axis=0)
    d2 = np.gradient(d1, q, axis=0)
    return d1, d2
