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
    assert bar.play_button.toolTip() == "Play (Space)"
    assert bar.step_button.isEnabled()
    assert bar.reset_button.isEnabled()


def test_transport_bar_playing_start(qapp) -> None:  # noqa: ANN001, ARG001
    """``playing=True`` starts checked with the pause tooltip and step disabled."""
    from skelarm.widgets import TransportBar

    bar = TransportBar(playing=True)
    assert bar.play_button.isChecked()
    assert bar.play_button.toolTip() == "Pause (Space)"
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
    assert bar.play_button.toolTip() == "Pause (Space)"
    assert not bar.step_button.isEnabled()

    bar.play_button.click()
    assert emitted == [True, False]
    assert bar.play_button.toolTip() == "Play (Space)"
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
    assert bar.play_button.toolTip() == "Pause (Space)"
    assert not bar.step_button.isEnabled()

    bar.set_playing(False)
    assert emitted == []
    assert not bar.play_button.isChecked()
    assert bar.play_button.toolTip() == "Play (Space)"
    assert bar.step_button.isEnabled()


def test_custom_labels(qapp) -> None:  # noqa: ANN001, ARG001
    """Custom labels flow into tooltips and accessible names."""
    from skelarm.widgets import TransportBar

    bar = TransportBar(play_label="Resume", step_label="Next frame", reset_label="Back to start")
    assert bar.play_button.toolTip() == "Resume (Space)"
    assert bar.step_button.toolTip() == "Next frame (→/F)"
    assert bar.reset_button.toolTip() == "Back to start (R)"
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


def _activate(window) -> None:  # noqa: ANN001
    """Make ``window`` the active window so WindowShortcut QShortcuts fire offscreen."""
    from PyQt6.QtWidgets import QApplication

    window.show()
    window.activateWindow()
    QApplication.processEvents()


def test_shortcuts_registered_per_action(qapp) -> None:  # noqa: ANN001, ARG001
    """The bar registers window-wide shortcuts for each action, one QShortcut per key."""
    from skelarm.widgets import TransportBar

    bar = TransportBar()
    keys = {action: [s.key().toString() for s in shortcuts] for action, shortcuts in bar.shortcuts.items()}
    assert keys == {"play": ["Space"], "step": ["Right", "F"], "reset": ["R"]}
    for shortcuts in bar.shortcuts.values():
        for shortcut in shortcuts:
            assert shortcut.parent() is bar


def test_back_button_absent_by_default(qapp) -> None:  # noqa: ANN001, ARG001
    """Without a back_label there is no back button and no Left/B binding."""
    from skelarm.widgets import TransportBar

    bar = TransportBar()
    assert bar.back_button is None
    assert "back" not in bar.shortcuts


def test_back_button_built_when_labelled(qapp) -> None:  # noqa: ANN001, ARG001
    """A back_label adds an icon-only back button between play and step, bound to Left/B."""
    from PyQt6.QtWidgets import QToolButton

    from skelarm.widgets import TransportBar

    bar = TransportBar(back_label="Previous frame")
    assert isinstance(bar.back_button, QToolButton)
    assert not bar.back_button.icon().isNull()
    assert not bar.back_button.isCheckable()
    assert bar.back_button.toolTip() == "Previous frame (←/B)"
    assert bar.back_button.accessibleName() == "Previous frame"

    layout = bar.layout()
    assert layout is not None
    assert layout.indexOf(bar.play_button) < layout.indexOf(bar.back_button) < layout.indexOf(bar.step_button)
    assert [s.key().toString() for s in bar.shortcuts["back"]] == ["Left", "B"]


def test_back_button_disabled_while_playing(qapp) -> None:  # noqa: ANN001, ARG001
    """The back button follows the step button's enablement: paused only."""
    from skelarm.widgets import TransportBar

    bar = TransportBar(back_label="Previous frame")
    bar.set_playing(True)
    assert bar.back_button is not None
    assert not bar.back_button.isEnabled()
    assert not bar.step_button.isEnabled()
    bar.set_playing(False)
    assert bar.back_button.isEnabled()
    assert bar.step_button.isEnabled()


def test_keys_drive_buttons_end_to_end(qapp) -> None:  # noqa: ANN001, ARG001
    """Real key presses drive the buttons once the bar's window is active."""
    from PyQt6.QtCore import Qt
    from PyQt6.QtTest import QTest

    from skelarm.widgets import TransportBar

    bar = TransportBar()
    _activate(bar)

    QTest.keyClick(bar, Qt.Key.Key_Space)  # type: ignore[call-overload]  # PyQt6 stubs type QTest methods as bound
    assert bar.play_button.isChecked()  # Space toggled play

    stepped: list[bool] = []
    bar.step_button.clicked.connect(stepped.append)
    QTest.keyClick(bar, Qt.Key.Key_F)  # type: ignore[call-overload]  # PyQt6 stubs type QTest methods as bound
    assert stepped == []  # step disabled while playing -> click() is a no-op

    bar.set_playing(False)
    QTest.keyClick(bar, Qt.Key.Key_Right)  # type: ignore[call-overload]  # PyQt6 stubs type QTest methods as bound
    assert stepped == [False]  # Right stepped once while paused

    reset_fired: list[bool] = []
    bar.reset_button.clicked.connect(reset_fired.append)
    QTest.keyClick(bar, Qt.Key.Key_R)  # type: ignore[call-overload]  # PyQt6 stubs type QTest methods as bound
    assert reset_fired == [False]
