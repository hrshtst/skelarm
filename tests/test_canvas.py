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


def test_control_panel_width_is_fixed(qapp) -> None:  # noqa: ANN001, ARG001
    """The joint-controls panel keeps a constant width, independent of its content."""
    from skelarm.canvas import _PANEL_WIDTH_PX, SkelarmViewer

    link_props = [LinkProp(length=1.0, m=1.0, i=0.1, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi)]
    viewer = SkelarmViewer(Skeleton(link_props))
    panel = viewer.controls_panel
    assert panel.minimumWidth() == panel.maximumWidth() == _PANEL_WIDTH_PX


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


def test_moving_one_slider_leaves_other_joints_unchanged(qapp) -> None:  # noqa: ANN001, ARG001
    """Adjusting one joint's slider must not snap the other joints to whole degrees."""
    from skelarm.canvas import SkelarmViewer

    link_props = [
        LinkProp(length=1.0, m=1.0, i=0.1, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi),
        LinkProp(length=1.0, m=1.0, i=0.1, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi),
    ]
    skeleton = Skeleton(link_props)
    skeleton.q = np.array([0.2, 0.3])  # joint 2 carries sub-degree precision
    viewer = SkelarmViewer(skeleton)

    viewer.sliders[0].setValue(40)  # move only joint 1

    assert viewer.skeleton.q[0] == pytest.approx(math.radians(40))
    assert viewer.skeleton.q[1] == pytest.approx(0.3)  # joint 2 left exactly as it was


def test_refresh_from_skeleton_syncs_sliders_and_labels(qapp) -> None:  # noqa: ANN001, ARG001
    """Externally-driven joint angles are reflected in the sliders and labels."""
    from skelarm.canvas import SkelarmViewer

    link_props = [
        LinkProp(length=1.0, m=1.0, i=0.1, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi),
        LinkProp(length=1.0, m=1.0, i=0.1, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi),
    ]
    viewer = SkelarmViewer(Skeleton(link_props))

    # Simulate the model being driven outside the GUI (e.g. by a controller).
    viewer.skeleton.q = np.array([np.radians(30.0), np.radians(-60.0)])
    viewer.refresh_from_skeleton()

    assert (viewer.sliders[0].value(), viewer.sliders[1].value()) == (30, -60)
    assert (viewer.angle_labels[0].text(), viewer.angle_labels[1].text()) == ("30°", "-60°")


def test_refresh_from_skeleton_does_not_feed_back_into_state(qapp) -> None:  # noqa: ANN001, ARG001
    """Refreshing must not let the rounded slider values overwrite the exact joint state."""
    from skelarm.canvas import SkelarmViewer

    link_props = [LinkProp(length=1.0, m=1.0, i=0.1, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi)]
    viewer = SkelarmViewer(Skeleton(link_props))

    set_deg = 30.4  # sub-degree precision
    viewer.skeleton.q = np.array([np.radians(set_deg)])
    exact = viewer.skeleton.q.copy()
    viewer.refresh_from_skeleton()

    assert viewer.skeleton.q == pytest.approx(exact)  # blocked signals, no round-trip clobber
    assert viewer.sliders[0].value() == round(set_deg)


def test_fit_scale_fits_arm_within_widget() -> None:
    """The fitted scale keeps a fully-extended arm inside the widget, up to a margin."""
    from skelarm.canvas import _MARGIN_PX, _fit_scale

    width = 400
    reach = 2.0
    scale = _fit_scale(reach, width, width)

    half = width / 2
    assert reach * scale <= half  # fully-extended arm fits within half the smaller dimension
    assert reach * scale == pytest.approx(half - _MARGIN_PX)  # tight to the margin


def test_fit_scale_falls_back_for_degenerate_input() -> None:
    """A non-positive reach or widget size falls back to the default scale."""
    from skelarm.canvas import _DEFAULT_SCALE, _fit_scale

    assert _fit_scale(0.0, 400, 400) == _DEFAULT_SCALE  # arm has no reach
    assert _fit_scale(2.0, 0, 0) == _DEFAULT_SCALE  # widget not laid out yet


def test_initial_label_shows_rounded_angle_matching_slider(qapp) -> None:  # noqa: ANN001, ARG001
    """The initial joint label is the rounded angle and matches the slider value."""
    from skelarm.canvas import SkelarmViewer

    link_props = [LinkProp(length=1.0, m=1.0, i=0.1, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi)]
    skeleton = Skeleton(link_props)
    skeleton.q = np.array([np.radians(44.7)])  # rounds up to 45, not truncated to 44
    viewer = SkelarmViewer(skeleton)

    assert viewer.sliders[0].value() == round(44.7)
    assert viewer.angle_labels[0].text() == f"{viewer.sliders[0].value()}°"


# === interactive inverse kinematics ===


def _two_link_viewer():  # noqa: ANN202
    """A two-link viewer seeded at a non-singular pose for IK interaction tests."""
    from skelarm.canvas import SkelarmViewer

    link_props = [
        LinkProp(length=1.0, m=1.0, i=0.1, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi),
        LinkProp(length=1.0, m=1.0, i=0.1, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi),
    ]
    skeleton = Skeleton(link_props)
    skeleton.q = np.array([0.3, 0.3])  # bent (non-singular) seed
    return SkelarmViewer(skeleton)


def test_solve_to_world_moves_endpoint(qapp) -> None:  # noqa: ANN001, ARG001
    """solve_to_world drives the endpoint to a reachable world point."""
    viewer = _two_link_viewer()

    viewer.canvas.solve_to_world(0.5, 1.2)

    tip = viewer.canvas.skeleton.links[-1]
    assert np.array([tip.xe, tip.ye]) == pytest.approx(np.array([0.5, 1.2]), abs=1e-4)


def test_solve_to_world_updates_sliders(qapp) -> None:  # noqa: ANN001, ARG001
    """An IK solve refreshes the joint sliders to match the new pose."""
    viewer = _two_link_viewer()

    viewer.canvas.solve_to_world(0.5, 1.2)

    for slider, angle in zip(viewer.sliders, viewer.canvas.skeleton.q, strict=True):
        assert slider.value() == round(math.degrees(angle))


def test_solve_to_world_records_ik_result(qapp) -> None:  # noqa: ANN001, ARG001
    """A reachable click records a successful IKResult on the canvas."""
    viewer = _two_link_viewer()

    viewer.canvas.solve_to_world(0.5, 1.2)

    result = viewer.canvas.last_ik_result
    assert result is not None
    assert result.success
    assert result.status == "converged"


def test_unreachable_click_records_stalled_result(qapp) -> None:  # noqa: ANN001, ARG001
    """An out-of-reach click records an unsuccessful, stalled IKResult."""
    viewer = _two_link_viewer()

    viewer.canvas.solve_to_world(5.0, 0.0)  # beyond the reach of 2.0

    result = viewer.canvas.last_ik_result
    assert result is not None
    assert not result.success
    assert result.status == "stalled"


def test_manual_pose_clears_ik_status(qapp) -> None:  # noqa: ANN001, ARG001
    """Posing a joint by hand clears the stale IK target/result."""
    viewer = _two_link_viewer()
    viewer.canvas.solve_to_world(0.5, 1.2)
    assert viewer.canvas.last_ik_result is not None

    slider = viewer.sliders[0]
    slider.setValue(slider.value() + 10)  # manual forward-kinematics pose change

    assert viewer.canvas.last_ik_result is None


def test_left_click_solves_ik_at_clicked_point(qapp) -> None:  # noqa: ANN001, ARG001
    """A left click maps the pixel to a world point and solves IK to reach it."""
    from PyQt6.QtCore import QEvent, QPointF, Qt
    from PyQt6.QtGui import QMouseEvent

    viewer = _two_link_viewer()
    canvas = viewer.canvas
    canvas.resize(400, 400)

    # Build the pixel that maps back to world (0.5, 1.2) under the canvas mapping.
    target = (0.5, 1.2)
    px = canvas.width() / 2 + target[0] * canvas.scale_factor
    py = canvas.height() / 2 - target[1] * canvas.scale_factor
    event = QMouseEvent(
        QEvent.Type.MouseButtonPress,
        QPointF(px, py),
        Qt.MouseButton.LeftButton,
        Qt.MouseButton.LeftButton,
        Qt.KeyboardModifier.NoModifier,
    )
    canvas.mousePressEvent(event)

    tip = canvas.skeleton.links[-1]
    assert np.array([tip.xe, tip.ye]) == pytest.approx(np.array(target), abs=1e-3)


def test_left_drag_solves_ik(qapp) -> None:  # noqa: ANN001, ARG001
    """Moving the mouse with the left button held solves IK toward the cursor."""
    from PyQt6.QtCore import QEvent, QPointF, Qt
    from PyQt6.QtGui import QMouseEvent

    viewer = _two_link_viewer()
    canvas = viewer.canvas
    canvas.resize(400, 400)

    target = (0.8, 0.9)
    px = canvas.width() / 2 + target[0] * canvas.scale_factor
    py = canvas.height() / 2 - target[1] * canvas.scale_factor
    event = QMouseEvent(
        QEvent.Type.MouseMove,
        QPointF(px, py),
        Qt.MouseButton.NoButton,
        Qt.MouseButton.LeftButton,  # left button held during the move
        Qt.KeyboardModifier.NoModifier,
    )
    canvas.mouseMoveEvent(event)

    tip = canvas.skeleton.links[-1]
    assert np.array([tip.xe, tip.ye]) == pytest.approx(np.array(target), abs=1e-3)
