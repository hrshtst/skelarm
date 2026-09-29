# Copyright (C) 2025-2026 Hiroshi Atsuta <atsuta@ieee.org>
# SPDX-License-Identifier: GPL-3.0-only

"""Tests for the documentation asset generator's interactive checklist (tools/generate_assets.py)."""

from __future__ import annotations

import os
import re

import pytest

# Importing the tool pulls in PyQt6; run headless.
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from tools.generate_assets import _print_interactive_checklist


def test_teaching_step_removes_the_committed_log_before_recording(capsys: pytest.CaptureFixture[str]) -> None:
    """The recorder never overwrites a file, so the step clears the committed teach log before recording to it."""
    _print_interactive_checklist()
    checklist = capsys.readouterr().out
    step = checklist[checklist.index("Teaching a trajectory") : checklist.index("FK/IK posing")]
    output = re.search(r"--output (\S+)", step)
    assert output is not None
    removal = f"rm -f {output.group(1)}"
    assert removal in step
    assert step.index(removal) < step.index("trajectory_recorder.py")
