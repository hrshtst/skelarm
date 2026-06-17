"""Tests for the PyQt6 viewer widgets."""

from __future__ import annotations

import math
import os
import warnings

import numpy as np
import pytest

# Run Qt without a display so the test works headless (CI and local).
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from skelarm.skeleton import LinkProp, Skeleton

pytestmark = pytest.mark.integration


@pytest.fixture(scope="module")
def qapp():  # noqa: ANN201
    """Provide a single QApplication instance for the GUI tests."""
    from PyQt6.QtWidgets import QApplication

    app = QApplication.instance() or QApplication([])
    return app


def test_sliders_respect_joint_limits(qapp) -> None:  # noqa: ANN001, ARG001
    """Each joint slider's range should come from the link's qmin/qmax, not a fixed +/-180."""
    from skelarm.canvas import SkelarmViewer

    link_props = [
        LinkProp(length=1.0, m=1.0, i=0.1, rgx=0.5, rgy=0.0, qmin=-np.pi / 2, qmax=np.pi / 2),
        LinkProp(length=1.0, m=1.0, i=0.1, rgx=0.5, rgy=0.0, qmin=-np.pi / 4, qmax=np.pi / 4),
    ]
    viewer = SkelarmViewer(Skeleton(link_props))

    assert (viewer.sliders[0].minimum(), viewer.sliders[0].maximum()) == (-90, 90)
    assert (viewer.sliders[1].minimum(), viewer.sliders[1].maximum()) == (-45, 45)


def test_slider_range_rounds_inward_to_stay_within_limits(qapp) -> None:  # noqa: ANN001, ARG001
    """Fractional limits must round inward so the slider can't exceed the enforced range."""
    from skelarm.canvas import SkelarmViewer

    qmin = np.deg2rad(-44.6)
    qmax = np.deg2rad(89.6)
    link_props = [LinkProp(length=1.0, m=1.0, i=0.1, rgx=0.5, rgy=0.0, qmin=qmin, qmax=qmax)]
    viewer = SkelarmViewer(Skeleton(link_props))
    slider = viewer.sliders[0]

    # Tightest integer degrees strictly inside the limit range.
    assert (slider.minimum(), slider.maximum()) == (-44, 89)
    # Converting the extremes back to radians stays within the enforced limits.
    assert math.radians(slider.minimum()) >= qmin
    assert math.radians(slider.maximum()) <= qmax


def test_moving_slider_to_limit_does_not_trigger_clamp_warning(qapp) -> None:  # noqa: ANN001, ARG001
    """Driving a slider to its extreme must not produce an out-of-limit clamp warning."""
    from skelarm.canvas import SkelarmViewer

    qmax = np.deg2rad(89.6)
    link_props = [LinkProp(length=1.0, m=1.0, i=0.1, rgx=0.5, rgy=0.0, qmin=-qmax, qmax=qmax)]
    viewer = SkelarmViewer(Skeleton(link_props))
    slider = viewer.sliders[0]

    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        slider.setValue(slider.maximum())  # drag to the top

    assert not [w for w in caught if "clamped" in str(w.message)]
    assert viewer.skeleton.q[0] == pytest.approx(math.radians(slider.maximum()))
    assert viewer.skeleton.q[0] <= qmax
