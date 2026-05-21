"""Tests for the PyQt6 viewer widgets."""

from __future__ import annotations

import os

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
