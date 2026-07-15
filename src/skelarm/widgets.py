"""Shared PyQt6 widgets for the skelarm GUI tools: QtAwesome icons and the transport bar."""

from __future__ import annotations

from PyQt6.QtCore import QSignalBlocker, QSize
from PyQt6.QtGui import QIcon, QKeySequence, QShortcut
from PyQt6.QtWidgets import QHBoxLayout, QToolButton, QWidget

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


def _hinted(label: str, keys: tuple[str, ...]) -> str:
    """Append a keyboard-shortcut hint to a tooltip label, e.g. ``"Play (Space)"``."""
    return f"{label} ({'/'.join(_KEY_GLYPHS.get(key, key) for key in keys)})"


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
