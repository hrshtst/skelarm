"""Shared PyQt6 widgets for the skelarm GUI tools: QtAwesome icons and the transport bar."""

from __future__ import annotations

from PyQt6.QtCore import QSignalBlocker, QSize
from PyQt6.QtGui import QIcon
from PyQt6.QtWidgets import QHBoxLayout, QToolButton, QWidget

_ICON_SIZE_PX = 28
_PLAY_ICON = "mdi6.play"
_PAUSE_ICON = "mdi6.pause"
_STEP_ICON = "mdi6.step-forward"
_RESET_ICON = "mdi6.restart"


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

    The bar owns three :class:`~PyQt6.QtWidgets.QToolButton` widgets:

    - ``play_button`` — checkable; ``checked`` means *playing*. Its icon and
      tooltip swap between play and pause, and it drives ``step_button``'s
      enablement (stepping is only meaningful while paused).
    - ``step_button`` — advance one tick/frame while paused.
    - ``reset_button`` — return to the initial state.

    Wire behavior by connecting to ``play_button.toggled``,
    ``step_button.clicked``, and ``reset_button.clicked``.

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
    ) -> None:
        """Build the bar and set the initial toggle state."""
        super().__init__(parent)
        self._play_label = play_label
        self._pause_label = pause_label

        self.play_button = self._make_button(_PLAY_ICON, play_label, checkable=True)
        self.play_button.setAccessibleName("Play/Pause")
        self.step_button = self._make_button(_STEP_ICON, step_label)
        self.reset_button = self._make_button(_RESET_ICON, reset_label)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)
        layout.addWidget(self.play_button)
        layout.addWidget(self.step_button)
        layout.addWidget(self.reset_button)
        layout.addStretch()

        self.play_button.toggled.connect(self._sync_play_state)
        self.play_button.setChecked(playing)
        self._sync_play_state()  # setChecked(False) on a fresh button does not emit

    def _make_button(self, icon_name: str, label: str, *, checkable: bool = False) -> QToolButton:
        """Build one icon-only tool button with tooltip and accessible name."""
        button = QToolButton(self)
        button.setAutoRaise(True)
        button.setIconSize(QSize(_ICON_SIZE_PX, _ICON_SIZE_PX))
        button.setIcon(make_icon(icon_name))
        button.setToolTip(label)
        button.setAccessibleName(label)
        button.setCheckable(checkable)
        return button

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
        """Match icon, tooltip, and step enablement to the toggle state."""
        playing = self.play_button.isChecked()
        self.play_button.setIcon(make_icon(_PAUSE_ICON if playing else _PLAY_ICON))
        self.play_button.setToolTip(self._pause_label if playing else self._play_label)
        self.step_button.setEnabled(not playing)
