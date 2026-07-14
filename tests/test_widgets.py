"""Tests for the shared GUI widgets (skelarm.widgets)."""

from __future__ import annotations

import os

import pytest

# Run Qt without a display so the test works headless (CI and local).
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

pytestmark = pytest.mark.integration


@pytest.fixture(scope="module")
def qapp():  # noqa: ANN201
    """Provide a single QApplication instance for the GUI tests."""
    from PyQt6.QtWidgets import QApplication

    return QApplication.instance() or QApplication([])


def test_make_icon_renders_headless(qapp) -> None:  # noqa: ANN001, ARG001
    """QtAwesome icons resolve and rasterize under the offscreen platform."""
    from PyQt6.QtCore import QSize

    from skelarm.widgets import make_icon

    icon = make_icon("mdi6.play")
    assert not icon.isNull()
    assert not icon.pixmap(QSize(24, 24)).isNull()


def test_transport_bar_defaults(qapp) -> None:  # noqa: ANN001, ARG001
    """A default bar starts stopped: play unchecked, tooltip 'Play', step enabled."""
    from skelarm.widgets import TransportBar

    bar = TransportBar()
    assert not bar.play_button.isChecked()
    assert bar.play_button.toolTip() == "Play"
    assert bar.step_button.isEnabled()
    assert bar.reset_button.isEnabled()


def test_transport_bar_playing_start(qapp) -> None:  # noqa: ANN001, ARG001
    """``playing=True`` starts checked with the pause tooltip and step disabled."""
    from skelarm.widgets import TransportBar

    bar = TransportBar(playing=True)
    assert bar.play_button.isChecked()
    assert bar.play_button.toolTip() == "Pause"
    assert not bar.step_button.isEnabled()


def test_play_click_toggles_state_and_emits(qapp) -> None:  # noqa: ANN001, ARG001
    """Clicking play flips checked/tooltip/step-enablement and emits ``toggled``."""
    from skelarm.widgets import TransportBar

    bar = TransportBar()
    emitted: list[bool] = []
    bar.play_button.toggled.connect(emitted.append)

    bar.play_button.click()
    assert emitted == [True]
    assert bar.play_button.isChecked()
    assert bar.play_button.toolTip() == "Pause"
    assert not bar.step_button.isEnabled()

    bar.play_button.click()
    assert emitted == [True, False]
    assert bar.play_button.toolTip() == "Play"
    assert bar.step_button.isEnabled()


def test_set_playing_syncs_without_emitting(qapp) -> None:  # noqa: ANN001, ARG001
    """``set_playing`` updates checked/tooltip/step without re-emitting ``toggled``."""
    from skelarm.widgets import TransportBar

    bar = TransportBar()
    emitted: list[bool] = []
    bar.play_button.toggled.connect(emitted.append)

    bar.set_playing(True)
    assert emitted == []
    assert bar.play_button.isChecked()
    assert bar.play_button.toolTip() == "Pause"
    assert not bar.step_button.isEnabled()

    bar.set_playing(False)
    assert emitted == []
    assert not bar.play_button.isChecked()
    assert bar.play_button.toolTip() == "Play"
    assert bar.step_button.isEnabled()


def test_custom_labels(qapp) -> None:  # noqa: ANN001, ARG001
    """Custom labels flow into tooltips and accessible names."""
    from skelarm.widgets import TransportBar

    bar = TransportBar(play_label="Resume", step_label="Next frame", reset_label="Back to start")
    assert bar.play_button.toolTip() == "Resume"
    assert bar.step_button.toolTip() == "Next frame"
    assert bar.reset_button.toolTip() == "Back to start"
    assert bar.step_button.accessibleName() == "Next frame"
    assert bar.reset_button.accessibleName() == "Back to start"


def test_buttons_are_icon_only_tool_buttons(qapp) -> None:  # noqa: ANN001, ARG001
    """The transport buttons are icon-only QToolButtons with icons and accessible names."""
    from PyQt6.QtWidgets import QToolButton

    from skelarm.widgets import TransportBar

    bar = TransportBar()
    for button in (bar.play_button, bar.step_button, bar.reset_button):
        assert isinstance(button, QToolButton)
        assert button.text() == ""
        assert not button.icon().isNull()
        assert button.accessibleName() != ""
    assert bar.play_button.isCheckable()
    assert not bar.step_button.isCheckable()
    assert not bar.reset_button.isCheckable()
