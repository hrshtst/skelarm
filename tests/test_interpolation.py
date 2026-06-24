"""Tests for the from-scratch interpolation utilities (src/skelarm/interpolation.py)."""

from __future__ import annotations

import numpy as np
import pytest

from skelarm.interpolation import INTERPOLATORS, interpolate, resample_with_derivatives


@pytest.mark.parametrize("method", INTERPOLATORS)
def test_interpolation_passes_through_nodes(method: str) -> None:
    """Every method reproduces the sample values exactly at the node times."""
    t = np.linspace(0.0, 1.0, 6)
    y = np.sin(2.0 * t)
    out = interpolate(t, y, t, method=method)
    assert out == pytest.approx(y, abs=1e-9)


@pytest.mark.parametrize("method", INTERPOLATORS)
def test_interpolation_multicolumn_shape(method: str) -> None:
    """A multi-column series interpolates column-wise, preserving the query length."""
    t = np.linspace(0.0, 1.0, 5)
    y = np.stack([t, t**2], axis=1)  # shape (5, 2)
    query = np.linspace(0.0, 1.0, 11)
    out = interpolate(t, y, query, method=method)
    assert out.shape == (11, 2)


@pytest.mark.parametrize("method", INTERPOLATORS)
def test_interpolation_scalar_query(method: str) -> None:
    """A scalar query yields a scalar (1-D series) result."""
    t = np.linspace(0.0, 1.0, 5)
    y = 3.0 * t
    out = interpolate(t, y, 0.5, method=method)
    assert float(out) == pytest.approx(1.5, abs=1e-9)


def test_linear_is_exact_on_a_line() -> None:
    """Linear interpolation is exact on an affine function."""
    t = np.array([0.0, 1.0, 2.0, 3.0])
    y = 2.0 * t + 1.0
    query = np.array([0.5, 1.5, 2.5])
    out = interpolate(t, y, query, method="linear")
    assert out == pytest.approx(2.0 * query + 1.0)


def test_lagrange_is_exact_on_a_cubic() -> None:
    """Degree-3 barycentric Lagrange through 4 nodes reproduces a cubic everywhere."""
    t = np.array([0.0, 1.0, 2.0, 3.0])
    y = t**3 - 2.0 * t + 1.0
    query = np.linspace(0.0, 3.0, 7)
    out = interpolate(t, y, query, method="lagrange")
    assert out == pytest.approx(query**3 - 2.0 * query + 1.0, abs=1e-9)


def test_cubic_spline_is_smooth_and_accurate() -> None:
    """A natural cubic spline approximates a sine closely between nodes."""
    t = np.linspace(0.0, 2.0 * np.pi, 25)
    query = np.linspace(0.0, 2.0 * np.pi, 101)
    out = interpolate(t, np.sin(t), query, method="cubic_spline")
    assert out == pytest.approx(np.sin(query), abs=2e-3)


def test_cubic_spline_derivatives_match_analytic() -> None:
    """resample_with_derivatives returns analytic 1st/2nd derivatives for the spline."""
    t = np.linspace(0.0, 2.0 * np.pi, 60)
    query = np.linspace(0.2, 2.0 * np.pi - 0.2, 50)
    val, d1, d2 = resample_with_derivatives(t, np.sin(t), query, method="cubic_spline")
    assert val == pytest.approx(np.sin(query), abs=1e-3)
    assert d1 == pytest.approx(np.cos(query), abs=1e-2)
    assert d2 == pytest.approx(-np.sin(query), abs=5e-2)


def test_resample_with_derivatives_multicolumn() -> None:
    """Derivatives are returned per column with the query length."""
    t = np.linspace(0.0, 1.0, 30)
    y = np.stack([t**2, np.sin(t)], axis=1)
    query = np.linspace(0.0, 1.0, 15)
    val, d1, d2 = resample_with_derivatives(t, y, query, method="cubic_spline")
    assert val.shape == d1.shape == d2.shape == (15, 2)
    assert d1[:, 0] == pytest.approx(2.0 * query, abs=2e-2)  # d/dt of t^2


def test_unknown_method_raises() -> None:
    """An unknown interpolation method is reported."""
    with pytest.raises(ValueError, match="unknown interpolation"):
        interpolate([0.0, 1.0], [0.0, 1.0], [0.5], method="bogus")
