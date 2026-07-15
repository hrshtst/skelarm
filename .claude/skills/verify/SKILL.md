---
name: verify
description: How to drive the PyQt6 GUI tools end-to-end headlessly and capture screenshot evidence for verification.
---

# Verifying skelarm GUI changes

The GUI tools (`tools/*.py`) run under `QT_QPA_PLATFORM=offscreen` and can be driven
through their real `main()` entry points, with pixel screenshots via `QWidget.grab()`.

## Recipe

Each tool's `main()` creates its own `QApplication` and calls `app.exec()`. Wrap
`QApplication.exec` with a version that schedules a `QTimer.singleShot` driver, then
execute the real script with `runpy`:

```python
import os, runpy, sys
os.environ["QT_QPA_PLATFORM"] = "offscreen"
sys.argv = ["dynamics_simulator.py", "examples/four_dof_robot.toml", "--no-plot"]
from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import QApplication

orig_exec = QApplication.exec

def drive():
    app = QApplication.instance()
    w = next(x for x in app.topLevelWidgets() if x.__class__.__name__ == "DynamicsSimulator")
    w.grab().save("/tmp/shot.png")   # pixel evidence; icons render offscreen (qtawesome is font-based)
    w.pause_button.click()           # buttons respond to .click() without a display
    app.quit()

QApplication.exec = lambda *a, **k: (QTimer.singleShot(400, drive), orig_exec())[1]
runpy.run_path("tools/dynamics_simulator.py", run_name="__main__")
```

Run with `uv run python <driver>.py` from the repo root (tools import `tools._scenario_cli`).

## Gotchas

- Pass `--no-plot` to the dynamics simulator or it opens a matplotlib window after `exec()`.
- Avoid clicking **Export…** in a driver — it opens a modal `QFileDialog` and hangs.
- To exercise the player's auto-pause-at-end, raise `speed_spin` (e.g. `setValue(10.0)`)
  before playing, or a 10 s log won't finish within the driver's wait.
- Produce a replayable log via the real CLI:
  `QT_QPA_PLATFORM=offscreen uv run python tools/reaching_simulator.py examples/reach.toml --save /tmp/reach.sklog.npz`.
- `This plugin does not support propagateSizeHints()` on stderr is offscreen-platform noise; ignore it.

## Flows worth driving

- Transport bar (all simulators): windows launch paused (`--run` starts immediately);
  the play toggle starts/stops the loop, step-while-paused advances `time` by 0.02 s,
  reset zeroes the clock, programmatic `pause()`/`resume()` syncs the toggle.
- Player: play at speed, auto-pause at the last frame (toggle unchecks), frame-step,
  back-to-start (pauses at frame 0), step past the end clamps.
