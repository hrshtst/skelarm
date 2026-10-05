# Copyright (C) 2025-2026 Hiroshi Atsuta <atsuta@ieee.org>
# SPDX-License-Identifier: GPL-3.0-only

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


# === keyboard shortcuts past a focused number box ===


def _bar_and_box(box_class: type) -> tuple:
    """An active window with a transport bar and a number box of ``box_class``, the box focused."""
    from PyQt6.QtWidgets import QApplication, QVBoxLayout, QWidget

    from skelarm.widgets import TransportBar

    window = QWidget()
    layout = QVBoxLayout(window)
    bar = TransportBar()
    box = box_class()
    layout.addWidget(bar)
    layout.addWidget(box)
    _activate(window)
    box.setFocus()
    QApplication.processEvents()
    return window, bar, box


def test_shortcut_friendly_spin_box_is_exported(qapp) -> None:  # noqa: ANN001, ARG001
    """The shortcut-friendly number box is available at the package top level, and the speed box is one."""
    import skelarm
    from skelarm.widgets import ShortcutFriendlySpinBox, SpeedSpinBox

    assert skelarm.ShortcutFriendlySpinBox is ShortcutFriendlySpinBox
    assert "ShortcutFriendlySpinBox" in skelarm.__all__
    assert issubclass(SpeedSpinBox, ShortcutFriendlySpinBox)


def test_a_plain_spin_box_swallows_the_shortcuts(qapp) -> None:  # noqa: ANN001, ARG001
    """The bug this guards against: a focused QDoubleSpinBox keeps Space from the window's shortcuts."""
    from PyQt6.QtCore import Qt
    from PyQt6.QtTest import QTest
    from PyQt6.QtWidgets import QDoubleSpinBox

    _, bar, box = _bar_and_box(QDoubleSpinBox)

    QTest.keyClick(box, Qt.Key.Key_Space)  # type: ignore[call-overload]  # PyQt6 stubs type QTest methods as bound
    assert not bar.play_button.isChecked()


def test_a_focused_number_box_leaves_space_and_letters_to_the_shortcuts(qapp) -> None:  # noqa: ANN001, ARG001
    """Space and letter keys reach the window's shortcuts while the box keeps its focus."""
    from PyQt6.QtCore import Qt
    from PyQt6.QtTest import QTest

    from skelarm.widgets import ShortcutFriendlySpinBox

    _, bar, box = _bar_and_box(ShortcutFriendlySpinBox)
    reset_fired: list[bool] = []
    bar.reset_button.clicked.connect(reset_fired.append)

    QTest.keyClick(box, Qt.Key.Key_Space)  # type: ignore[call-overload]  # PyQt6 stubs type QTest methods as bound
    assert bar.play_button.isChecked()
    QTest.keyClick(box, Qt.Key.Key_R)  # type: ignore[call-overload]  # PyQt6 stubs type QTest methods as bound
    assert reset_fired == [False]
    assert box.hasFocus()


def test_a_focused_number_box_still_takes_digits_and_cursor_keys(qapp) -> None:  # noqa: ANN001, ARG001
    """Typing a number and moving the cursor still edit the box rather than firing shortcuts."""
    from PyQt6.QtCore import Qt
    from PyQt6.QtTest import QTest

    from skelarm.widgets import ShortcutFriendlySpinBox

    _, bar, box = _bar_and_box(ShortcutFriendlySpinBox)
    stepped: list[bool] = []
    bar.step_button.clicked.connect(stepped.append)
    line_edit = box.lineEdit()
    assert line_edit is not None

    line_edit.selectAll()
    QTest.keyClicks(box, "2.5")  # type: ignore[call-arg, arg-type]  # PyQt6 stubs type QTest methods as bound
    assert box.value() == pytest.approx(2.5)
    QTest.keyClick(box, Qt.Key.Key_Home)  # type: ignore[call-overload]  # PyQt6 stubs type QTest methods as bound
    QTest.keyClick(box, Qt.Key.Key_Right)  # type: ignore[call-overload]  # PyQt6 stubs type QTest methods as bound
    assert line_edit.cursorPosition() == 1
    assert stepped == []  # Right moved the cursor, not the timeline


def test_enter_and_escape_hand_the_focus_back_to_the_window(qapp) -> None:  # noqa: ANN001, ARG001
    """After Enter or Escape, the box lets go of the focus, so every shortcut works again."""
    from PyQt6.QtCore import Qt
    from PyQt6.QtTest import QTest
    from PyQt6.QtWidgets import QApplication

    from skelarm.widgets import ShortcutFriendlySpinBox

    window, bar, box = _bar_and_box(ShortcutFriendlySpinBox)
    stepped: list[bool] = []
    bar.step_button.clicked.connect(stepped.append)

    for key in (Qt.Key.Key_Return, Qt.Key.Key_Enter, Qt.Key.Key_Escape):
        box.setFocus()
        QApplication.processEvents()
        QTest.keyClick(box, key)  # type: ignore[call-overload]  # PyQt6 stubs type QTest methods as bound
        assert not box.hasFocus()
    QTest.keyClick(window, Qt.Key.Key_Right)  # type: ignore[call-overload]  # PyQt6 stubs type QTest methods as bound
    assert stepped == [False]


# === playback clock and speed spin box ===


def test_playback_clock_and_speed_spin_box_are_exported(qapp) -> None:  # noqa: ANN001, ARG001
    """Both live next to TransportBar at the package top level."""
    import skelarm
    from skelarm.widgets import PlaybackClock, SpeedSpinBox

    assert skelarm.PlaybackClock is PlaybackClock
    assert skelarm.SpeedSpinBox is SpeedSpinBox
    assert {"PlaybackClock", "SpeedSpinBox"} <= set(skelarm.__all__)


def test_playback_clock_defaults(qapp) -> None:  # noqa: ANN001, ARG001
    """A default clock is stopped, ticks every 20 ms, and runs at speed 1."""
    from skelarm.widgets import PlaybackClock

    clock = PlaybackClock()
    assert clock.period_ms == 20  # noqa: PLR2004
    assert clock.speed == 1.0
    assert clock.is_running is False


def test_playback_clock_ticks_its_period_times_the_speed(qapp) -> None:  # noqa: ANN001, ARG001
    """Each tick reports the timeline seconds to advance: the period scaled by the speed it has at that tick."""
    from PyQt6.QtTest import QSignalSpy

    from skelarm.widgets import PlaybackClock

    clock = PlaybackClock(period_ms=5, speed=2.5)
    ticks = QSignalSpy(clock.ticked)
    clock.start()
    assert ticks.wait(1000)
    assert ticks[0][0] == pytest.approx(0.005 * 2.5)

    clock.speed = 0.5  # a running clock picks up a new speed on its next tick
    later = QSignalSpy(clock.ticked)
    assert later.wait(1000)
    assert later[0][0] == pytest.approx(0.005 * 0.5)
    clock.stop()


def test_playback_clock_starts_and_stops(qapp) -> None:  # noqa: ANN001, ARG001
    """``start`` runs the clock and ``stop`` halts it; a stopped clock does not tick."""
    from PyQt6.QtTest import QSignalSpy

    from skelarm.widgets import PlaybackClock

    clock = PlaybackClock(period_ms=5)
    clock.start()
    assert clock.is_running is True
    clock.stop()
    assert clock.is_running is False
    ticks = QSignalSpy(clock.ticked)
    assert not ticks.wait(50)


def test_playback_clock_keeps_speed_unclamped(qapp) -> None:  # noqa: ANN001, ARG001
    """The speed is stored as a float as given; only the spin box limits what a user can pick."""
    from skelarm.widgets import PlaybackClock

    clock = PlaybackClock(speed=20)
    assert clock.speed == 20.0  # noqa: PLR2004
    assert isinstance(clock.speed, float)
    clock.speed = 3
    assert isinstance(clock.speed, float)


def test_speed_spin_box_is_preconfigured(qapp) -> None:  # noqa: ANN001, ARG001
    """The spin box offers 0.1x to 10x in steps of 0.1, shown with two decimals."""
    from PyQt6.QtWidgets import QDoubleSpinBox

    from skelarm.widgets import SpeedSpinBox

    spin = SpeedSpinBox()
    assert isinstance(spin, QDoubleSpinBox)
    assert spin.decimals() == 2  # noqa: PLR2004
    assert (spin.minimum(), spin.maximum()) == (pytest.approx(0.1), pytest.approx(10.0))
    assert spin.singleStep() == pytest.approx(0.1)
    assert spin.value() == pytest.approx(1.0)


def test_speed_spin_box_starts_at_the_given_speed_and_steps(qapp) -> None:  # noqa: ANN001, ARG001
    """The initial speed is shown (clamped to the range), and a step moves it by 0.1."""
    from skelarm.widgets import SpeedSpinBox

    spin = SpeedSpinBox(speed=2.0)
    assert spin.value() == pytest.approx(2.0)
    spin.stepUp()
    assert spin.value() == pytest.approx(2.1)
    spin.stepDown()
    spin.stepDown()
    assert spin.value() == pytest.approx(1.9)
    assert SpeedSpinBox(speed=20.0).value() == pytest.approx(10.0)
    assert SpeedSpinBox(speed=0.0).value() == pytest.approx(0.1)
