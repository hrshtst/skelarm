# Copyright (C) 2025-2026 Hiroshi Atsuta <atsuta@ieee.org>
# SPDX-License-Identifier: GPL-3.0-only

"""Shared PyQt6 widgets for the skelarm GUI tools: QtAwesome icons, the transport bar, and the playback clock.

The tools bind their keys (Space, R, Q, ...) as window-wide shortcuts. A focused
text field normally claims every key it could type, which silences those
shortcuts until the user clicks another widget that takes the focus; number
inputs should therefore be :class:`ShortcutFriendlySpinBox`.
"""

from __future__ import annotations

from PyQt6.QtCore import QEvent, QObject, QSignalBlocker, QSize, Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QIcon, QKeyEvent, QKeySequence, QShortcut
from PyQt6.QtWidgets import QDoubleSpinBox, QHBoxLayout, QToolButton, QWidget

_PLAYBACK_PERIOD_MS = 20  # default playback/render tick
_SPEED_DECIMALS = 2
_SPEED_RANGE = (0.1, 10.0)  # what a user can pick in the speed spin box
_SPEED_STEP = 0.1
_ICON_SIZE_PX = 28
_PLAY_ICON = "mdi6.play"
_PAUSE_ICON = "mdi6.pause"
_STEP_ICON = "mdi6.step-forward"
_BACK_ICON = "mdi6.step-backward"
_RESET_ICON = "mdi6.restart"
_PLAY_KEYS = ("Space",)
_STEP_KEYS = ("Right", "F")
_BACK_KEYS = ("Left", "B")
_RESET_KEYS = ("R",)
_KEY_GLYPHS = {"Right": "→", "Left": "←"}
# The keys a number box needs for editing; it leaves every other key to the window's shortcuts.
_NUMBER_EDITING_KEYS = frozenset(
    {getattr(Qt.Key, f"Key_{digit}") for digit in range(10)}
    | {
        Qt.Key.Key_Period,
        Qt.Key.Key_Comma,
        Qt.Key.Key_Minus,
        Qt.Key.Key_Plus,
        Qt.Key.Key_Backspace,
        Qt.Key.Key_Delete,
        Qt.Key.Key_Left,
        Qt.Key.Key_Right,
        Qt.Key.Key_Home,
        Qt.Key.Key_End,
        Qt.Key.Key_Up,
        Qt.Key.Key_Down,
        Qt.Key.Key_PageUp,
        Qt.Key.Key_PageDown,
        Qt.Key.Key_Return,
        Qt.Key.Key_Enter,
        Qt.Key.Key_Escape,
    }
)
_DONE_KEYS = (Qt.Key.Key_Return, Qt.Key.Key_Enter, Qt.Key.Key_Escape)
_COMMAND_MODIFIERS = (
    Qt.KeyboardModifier.ControlModifier | Qt.KeyboardModifier.AltModifier | Qt.KeyboardModifier.MetaModifier
)


def _hinted(label: str, keys: tuple[str, ...]) -> str:
    """Append a keyboard-shortcut hint to a tooltip label, e.g. ``"Play (Space)"``."""
    return f"{label} ({'/'.join(_KEY_GLYPHS.get(key, key) for key in keys)})"


def bind_quit_key(window: QWidget) -> QShortcut:
    """Bind ``Q`` to close ``window``, returning the created shortcut.

    Closing goes through the normal ``close()`` path, so ``closeEvent``
    handlers (e.g. the trajectory recorder's unsaved-take warning) still run.
    """
    shortcut = QShortcut(QKeySequence("Q"), window)
    shortcut.activated.connect(window.close)
    return shortcut


def make_icon(name: str) -> QIcon:
    """Return a QtAwesome icon by name.

    Parameters
    ----------
    name : str
        Icon name including its font prefix, e.g. ``"mdi6.play"``
        (Material Design Icons 6). Unknown names raise from qtawesome.

    Returns
    -------
    QIcon
        A font-rendered icon that works on every platform, including
        the offscreen one used in headless tests.
    """
    # qtawesome loads its icon fonts through Qt, so a QApplication must
    # already exist; keep the import lazy and never build icons at module level.
    import qtawesome

    icon: QIcon = qtawesome.icon(name)
    return icon


class TransportBar(QWidget):
    """A media-player-style horizontal row of icon-only playback buttons.

    The bar owns up to four :class:`~PyQt6.QtWidgets.QToolButton` widgets:

    - ``play_button`` — checkable; ``checked`` means *playing*. Its icon and
      tooltip swap between play and pause, and it drives the enablement of
      ``step_button`` and ``back_button`` (stepping is only meaningful while
      paused).
    - ``back_button`` — step one tick/frame backward while paused. Only built
      when ``back_label`` is given (the replay player); otherwise ``None``.
    - ``step_button`` — advance one tick/frame while paused.
    - ``reset_button`` — return to the initial state.

    Wire behavior by connecting to ``play_button.toggled``,
    ``back_button.clicked``, ``step_button.clicked``, and
    ``reset_button.clicked``.

    Each button is also bound to window-wide keyboard shortcuts, advertised in
    its tooltip: ``Space`` play/pause, ``→``/``F`` step, ``←``/``B`` back (when
    present), ``R`` reset. The :class:`~PyQt6.QtGui.QShortcut` objects live in
    :attr:`shortcuts`, keyed by action name (``"play"``, ``"step"``,
    ``"reset"``, and ``"back"`` when built). A key must be bound at most once
    per window — Qt resolves duplicates as ambiguous and then activates
    *neither* binding, silently.

    Parameters
    ----------
    parent : QWidget, optional
        Parent widget.
    playing : bool, optional
        Initial toggle state (``True`` = playing).
    play_label : str, optional
        Tooltip shown while paused (the action the click performs).
    pause_label : str, optional
        Tooltip shown while playing.
    step_label : str, optional
        Tooltip and accessible name of the step button.
    reset_label : str, optional
        Tooltip and accessible name of the reset button.
    back_label : str, optional
        Tooltip and accessible name of the backward-step button; omit (the
        default) to build the bar without one.
    """

    def __init__(
        self,
        parent: QWidget | None = None,
        *,
        playing: bool = False,
        play_label: str = "Play",
        pause_label: str = "Pause",
        step_label: str = "Step",
        reset_label: str = "Reset",
        back_label: str | None = None,
    ) -> None:
        """Build the bar and set the initial toggle state."""
        super().__init__(parent)
        self._play_label = play_label
        self._pause_label = pause_label

        self.play_button = self._make_button(_PLAY_ICON, play_label, keys=_PLAY_KEYS, checkable=True)
        self.play_button.setAccessibleName("Play/Pause")
        self.back_button: QToolButton | None = (
            None if back_label is None else self._make_button(_BACK_ICON, back_label, keys=_BACK_KEYS)
        )
        self.step_button = self._make_button(_STEP_ICON, step_label, keys=_STEP_KEYS)
        self.reset_button = self._make_button(_RESET_ICON, reset_label, keys=_RESET_KEYS)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)
        layout.addWidget(self.play_button)
        if self.back_button is not None:
            layout.addWidget(self.back_button)
        layout.addWidget(self.step_button)
        layout.addWidget(self.reset_button)
        layout.addStretch()

        # One QShortcut per key, each clicking its button; clicking a disabled
        # button is a no-op, so step/back shortcuts are inert while playing.
        self.shortcuts: dict[str, tuple[QShortcut, ...]] = {
            "play": self._bind_keys(_PLAY_KEYS, self.play_button),
            "step": self._bind_keys(_STEP_KEYS, self.step_button),
            "reset": self._bind_keys(_RESET_KEYS, self.reset_button),
        }
        if self.back_button is not None:
            self.shortcuts["back"] = self._bind_keys(_BACK_KEYS, self.back_button)

        self.play_button.toggled.connect(self._sync_play_state)
        self.play_button.setChecked(playing)
        self._sync_play_state()  # setChecked(False) on a fresh button does not emit

    def _make_button(
        self, icon_name: str, label: str, *, keys: tuple[str, ...] | None = None, checkable: bool = False
    ) -> QToolButton:
        """Build one icon-only tool button with a shortcut-hinted tooltip and accessible name."""
        button = QToolButton(self)
        button.setAutoRaise(True)
        button.setIconSize(QSize(_ICON_SIZE_PX, _ICON_SIZE_PX))
        button.setIcon(make_icon(icon_name))
        button.setToolTip(_hinted(label, keys) if keys else label)
        button.setAccessibleName(label)
        button.setCheckable(checkable)
        return button

    def _bind_keys(self, keys: tuple[str, ...], button: QToolButton) -> tuple[QShortcut, ...]:
        """Bind window-wide shortcuts that click ``button`` (a no-op while it is disabled)."""
        shortcuts: list[QShortcut] = []
        for key in keys:
            shortcut = QShortcut(QKeySequence(key), self)  # WindowShortcut context: fires anywhere in the window
            shortcut.activated.connect(button.click)
            shortcuts.append(shortcut)
        return tuple(shortcuts)

    def set_playing(self, playing: bool) -> None:  # noqa: FBT001
        """Sync the toggle to ``playing`` without emitting ``toggled``.

        Use this for programmatic state changes (e.g. auto-pause at the end of
        a replay) so the connected pause/resume handlers do not fire again.
        """
        with QSignalBlocker(self.play_button):
            self.play_button.setChecked(playing)
        # The blocker also suppressed our own toggled hook, so re-sync explicitly.
        self._sync_play_state()

    def _sync_play_state(self) -> None:
        """Match icon, tooltip, and step/back enablement to the toggle state."""
        playing = self.play_button.isChecked()
        self.play_button.setIcon(make_icon(_PAUSE_ICON if playing else _PLAY_ICON))
        self.play_button.setToolTip(_hinted(self._pause_label if playing else self._play_label, _PLAY_KEYS))
        self.step_button.setEnabled(not playing)
        if self.back_button is not None:
            self.back_button.setEnabled(not playing)


class PlaybackClock(QObject):
    """A real-time playback clock that ticks a timeline forward at an adjustable speed.

    The clock owns a :class:`~PyQt6.QtCore.QTimer` firing every ``period_ms``
    milliseconds while it runs. Each tick emits :attr:`ticked` with the timeline
    seconds to advance: the period times the :attr:`speed` at that tick, so a speed
    change takes effect on the next tick. The period is nominal; timer jitter is not
    measured. The clock holds no timeline position of its own: connect :attr:`ticked`
    to whatever advances the timeline, and pair :meth:`start` / :meth:`stop` with
    :meth:`TransportBar.set_playing` to keep a play button in step.

    Parameters
    ----------
    parent : QObject, optional
        Parent object, which owns the clock.
    period_ms : int, optional
        Real time between ticks, in milliseconds (default: 20).
    speed : float, optional
        Initial speed: timeline seconds per real second (default: 1).
    """

    ticked = pyqtSignal(float)
    """Emitted on each tick with the timeline seconds to advance (``period_ms / 1000 * speed``)."""

    def __init__(
        self, parent: QObject | None = None, *, period_ms: int = _PLAYBACK_PERIOD_MS, speed: float = 1.0
    ) -> None:
        """Build a stopped clock."""
        super().__init__(parent)
        self._period_ms = period_ms
        self._speed = float(speed)
        self._timer = QTimer(self)
        self._timer.setInterval(period_ms)
        self._timer.timeout.connect(self._on_timeout)

    @property
    def period_ms(self) -> int:
        """Real time between ticks, in milliseconds."""
        return self._period_ms

    @property
    def speed(self) -> float:
        """Timeline seconds per real second; any value is kept as given (no clamping)."""
        return self._speed

    @speed.setter
    def speed(self, value: float) -> None:
        self._speed = float(value)

    @property
    def is_running(self) -> bool:
        """Whether the clock is ticking."""
        return self._timer.isActive()

    def start(self) -> None:
        """Start ticking; a running clock restarts its period."""
        self._timer.start()

    def stop(self) -> None:
        """Stop ticking."""
        self._timer.stop()

    def _on_timeout(self) -> None:
        """Report one tick's worth of timeline seconds."""
        self.ticked.emit(self._period_ms / 1000.0 * self._speed)


class ShortcutFriendlySpinBox(QDoubleSpinBox):
    """A number box that leaves the window's keyboard shortcuts working.

    A plain :class:`~PyQt6.QtWidgets.QDoubleSpinBox` claims every key it could
    type while it has the focus, and keeps the focus until the user clicks another
    widget that takes it, so shortcuts such as Space (play) or R (reset) stop
    working after the user edits a value. This box claims only the keys that edit
    a number: the digits, the decimal point, the signs, the cursor and deletion
    keys, and the keys that step the value; Space, letters, and other keys reach
    the shortcuts even while it has the focus. Enter or Escape commits the value
    and hands the focus back to the window, so the keys it does claim (such as
    Right or Home) work as shortcuts again. Key combinations with Ctrl, Alt, or
    Meta behave as in any text field.
    """

    def event(self, event: QEvent | None) -> bool:
        """Decline the keys a number does not need, so they reach the window's shortcuts."""
        if event is not None and event.type() == QEvent.Type.ShortcutOverride and isinstance(event, QKeyEvent):
            commanded = bool(event.modifiers() & _COMMAND_MODIFIERS)
            if not commanded and Qt.Key(event.key()) not in _NUMBER_EDITING_KEYS:
                event.ignore()
                return False
        return super().event(event)

    def keyPressEvent(self, e: QKeyEvent | None) -> None:  # noqa: N802
        """Commit on Enter or Escape and give the focus back to the window."""
        super().keyPressEvent(e)
        if e is not None and Qt.Key(e.key()) in _DONE_KEYS:
            self.interpretText()
            self.clearFocus()


class SpeedSpinBox(ShortcutFriendlySpinBox):
    """A spin box for a playback speed multiplier, as the replay player shows it.

    It offers 0.1x to 10x in steps of 0.1, with two decimals. Handle its
    ``valueChanged`` signal by setting a :class:`PlaybackClock`'s
    :attr:`~PlaybackClock.speed`.

    Parameters
    ----------
    parent : QWidget, optional
        Parent widget.
    speed : float, optional
        Initial value, clamped to the range (default: 1). It is set before any
        signal is connected, so it emits nothing.
    """

    def __init__(self, parent: QWidget | None = None, *, speed: float = 1.0) -> None:
        """Configure the range, step, and decimals, and show ``speed``."""
        super().__init__(parent)
        self.setDecimals(_SPEED_DECIMALS)
        self.setRange(*_SPEED_RANGE)
        self.setSingleStep(_SPEED_STEP)
        self.setValue(speed)
