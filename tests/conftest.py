# Copyright (C) 2025-2026 Hiroshi Atsuta <atsuta@ieee.org>
# SPDX-License-Identifier: GPL-3.0-only

"""Shared pytest fixtures for the test suite."""

from __future__ import annotations

import sys
from typing import TYPE_CHECKING

import pytest

if TYPE_CHECKING:
    from collections.abc import Iterator


@pytest.fixture(autouse=True)
def delete_shown_qt_windows() -> Iterator[None]:
    """Delete every top-level Qt window a test leaves on screen, right after the test.

    A window that a test shows and then drops stays on screen until the garbage
    collector reaches it, and that can happen in the middle of painting it while a
    later test processes events: Qt then destroys a widget that is being painted and
    the whole run crashes ("Cannot destroy paint device that is being painted", a
    segmentation fault or an abort, depending on the test order). Deleting the shown
    windows here, outside any event processing, removes that hazard.

    ``sip.delete`` destroys a window without a ``closeEvent``, so a tool that asks
    before closing (the trajectory recorder with an unsaved take) cannot block the
    run with a dialog. Hidden windows are never painted and are left alone, and the
    fixture does nothing in a test that never imported PyQt6.
    """
    yield
    if "PyQt6.QtWidgets" not in sys.modules:
        return
    from PyQt6 import sip
    from PyQt6.QtWidgets import QApplication

    if QApplication.instance() is None:
        return
    # Ask Qt afresh after each deletion: deleting a window also destroys the windows it
    # owns (a floating dock, say), which a list taken up front would still hold.
    while shown := [widget for widget in QApplication.topLevelWidgets() if widget.isVisible()]:
        sip.delete(shown[0])
