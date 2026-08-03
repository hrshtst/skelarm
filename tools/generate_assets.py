# Copyright (C) 2025-2026 Hiroshi Atsuta <atsuta@ieee.org>
# SPDX-License-Identifier: GPL-3.0-only

"""Generate the reproducible documentation assets in ``docs/assets``.

Runs entirely headless (offscreen Qt, Agg matplotlib) and is deterministic, so
the committed assets can be regenerated at any time:

- **Animations** — each demo scenario is run headlessly with ``run_scenario``,
  saved as a replayable ``.sklog.npz`` next to its GIF, and rendered with the
  player's export pipeline (task overlays included).
- **Figures** — the plotting examples are executed with ``plt.show`` redirected
  to ``savefig``.
- **Screenshots** — the GUI tools are instantiated offscreen and grabbed as
  PNGs.

Interactive demonstrations (mouse dragging, live target switching, trajectory
teaching) cannot be scripted; the checklist printed at the end lists the exact
commands and actions to capture them. Once a taught ``teach.sklog.npz`` exists
in ``docs/assets``, re-running this script also generates the
trajectory-tracking animation from it.

Usage::

    uv run python tools/generate_assets.py
"""

from __future__ import annotations

import os
import runpy
import subprocess
import sys
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")  # render Qt without a display
os.environ.setdefault("MPLBACKEND", "Agg")  # render matplotlib without a display

from PyQt6.QtWidgets import QApplication, QWidget

from skelarm import StateLog

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # allow `tools.` imports when run as a script
from tools._scenario_cli import ScenarioSimulator, build_scenario, save_scenario_run
from tools.dynamics_simulator import DynamicsSimulator
from tools.kinematics_inspector import KinematicsInspector
from tools.player import PlaybackWindow

_REPO = Path(__file__).resolve().parents[1]
_EXAMPLES = _REPO / "examples"
_ASSETS = _REPO / "docs" / "assets"

_GIF_FPS = 20.0  # output frame rate; modest to keep the committed GIFs small
_GIF_SIZE_PX = 480  # square frame size; modest to keep the committed GIFs small

# (asset stem, scenario config, playback speed) — speed > 1 compresses long runs.
_ANIMATIONS: tuple[tuple[str, Path, float], ...] = (
    ("reach", _EXAMPLES / "reach.toml", 1.0),
    ("mpc_reach", _EXAMPLES / "mpc.toml", 1.0),
    ("periodic_curve", _EXAMPLES / "periodic_curve.toml", 2.0),
    ("multi_target", _EXAMPLES / "multi_target.toml", 1.0),
)

# Plotting examples rendered to PNG (each calls plt.show() exactly once).
_FIGURES: tuple[str, ...] = ("basic_plotting", "inverse_kinematics", "reaching", "periodic_curve")

_TRACK_CONFIG_TABLES = """
[task]
type = "joint_trajectory_tracking"
file = "teach.sklog.npz"
filter = { kind = "butterworth", cutoff_hz = 8.0, order = 4 }
interpolator = "cubic_spline"

[simulator]
dt = 0.002

[controller]
type = "computed_torque"
kp = 200.0
kd = 30.0
"""


def _export_gif(log_path: Path, gif_path: Path, *, speed: float) -> None:
    """Render a saved log to a GIF via the player's headless export pipeline."""
    window = PlaybackWindow(StateLog.load(log_path), speed=speed)
    frames = window.export(gif_path, fps=_GIF_FPS, size=_GIF_SIZE_PX)
    window.close()
    print(f"wrote {frames} frames to {gif_path}")


def _record_animations() -> None:
    """Run each demo scenario headlessly; save its replayable log and its GIF."""
    for stem, config, speed in _ANIMATIONS:
        log_path = save_scenario_run(build_scenario(config), _ASSETS / f"{stem}.sklog.npz")
        _export_gif(log_path, _ASSETS / f"{stem}.gif", speed=speed)


def _record_tracking_animation() -> None:
    """Generate the trajectory-tracking assets when a taught reference exists."""
    teach_log = _ASSETS / "teach.sklog.npz"
    if not teach_log.exists():
        print(f"skipping tracking animation: no {teach_log} (teach one first, see the checklist below)")
        return
    track_config = _ASSETS / "track.toml"
    robot_tables = (_EXAMPLES / "four_dof_robot.toml").read_text(encoding="utf-8")
    track_config.write_text(robot_tables + _TRACK_CONFIG_TABLES, encoding="utf-8")
    log_path = save_scenario_run(build_scenario(track_config), _ASSETS / "trajectory_tracking.sklog.npz")
    _export_gif(log_path, _ASSETS / "trajectory_tracking.gif", speed=1.0)


def _render_figures() -> None:
    """Execute the plotting examples with ``plt.show`` redirected to ``savefig``."""
    import matplotlib.pyplot as plt

    original_show = plt.show
    target = _ASSETS / "placeholder.png"

    def _save(*_args: object, **_kwargs: object) -> None:
        # bbox_inches="tight" retriggers text layout and can crash FreeType offscreen.
        plt.gcf().savefig(target, dpi=150)
        plt.close("all")
        print(f"wrote {target}")

    plt.show = _save  # type: ignore[assignment]
    try:
        for stem in _FIGURES:
            target = _ASSETS / f"{stem}.png"
            runpy.run_path(str(_EXAMPLES / f"{stem}.py"), run_name="__main__")
    finally:
        plt.show = original_show


def _grab(window: QWidget, png_path: Path) -> None:
    """Render a window offscreen and save its pixels."""
    window.show()
    QApplication.processEvents()
    window.grab().save(str(png_path))
    window.close()
    print(f"wrote {png_path}")


def _grab_screenshots() -> None:
    """Screenshot the GUI tools' windows offscreen."""
    from skelarm import Skeleton

    _grab(
        KinematicsInspector(Skeleton.from_toml(_EXAMPLES / "four_dof_robot.toml")), _ASSETS / "kinematics_inspector.png"
    )
    _grab(DynamicsSimulator(Skeleton.from_toml(_EXAMPLES / "four_dof_robot.toml")), _ASSETS / "dynamics_simulator.png")
    _grab(ScenarioSimulator(build_scenario(_EXAMPLES / "reach.toml")), _ASSETS / "reaching_simulator.png")
    _grab(PlaybackWindow(StateLog.load(_ASSETS / "reach.sklog.npz")), _ASSETS / "player.png")


def _print_interactive_checklist() -> None:
    """List the demonstrations that need a human and a screen recorder."""
    print(
        """
All reproducible assets are in docs/assets/. The following demonstrations need a
human and a screen recorder (e.g. Peek or OBS for GIFs) — capture each window
region, then place the recording in docs/assets/:

1. FK/IK posing            uv run python tools/kinematics_inspector.py examples/four_dof_robot.toml --show-com
   Do: drag a joint slider through its range, then click-drag the tip around the
   canvas so the IK solution follows the cursor.

2. Drag-to-perturb         uv run python tools/dynamics_simulator.py examples/four_dof_robot.toml --run
   Do: press and drag the tip; release and watch the arm swing freely (the red
   arrow is the applied force). Raise the friction spin box mid-run to damp it.

3. Disturbing a reach      uv run python tools/reaching_simulator.py examples/reach.toml --run
   Do: let the arm reach the purple target, then drag the tip away and release —
   the controller pulls it back.

4. Live target switching   uv run python tools/multi_target_simulator.py examples/multi_target.toml --run
   Do: while it runs, press 1, 2, … to switch the active target and watch the
   arm retarget.

5. Teaching a trajectory   uv run python tools/trajectory_recorder.py \\
                               examples/four_dof_robot.toml --output docs/assets/teach.sklog.npz
   Do: grab the tip and draw a smooth shape (recording starts on the first
   grab), then press F to finish. Afterwards RE-RUN THIS SCRIPT: it will find
   docs/assets/teach.sklog.npz and generate the trajectory-tracking animation
   from it automatically.
"""
    )


def main() -> None:
    """Generate every reproducible asset, then print the interactive checklist."""
    _ASSETS.mkdir(parents=True, exist_ok=True)
    if "--figures-only" in sys.argv:
        _render_figures()
        return
    # Matplotlib's and Qt's FreeType usage conflict in one process (raster-overflow
    # crashes), so the figure stage runs isolated in a subprocess without Qt.
    subprocess.run([sys.executable, str(Path(__file__).resolve()), "--figures-only"], check=True)  # noqa: S603
    app = QApplication.instance() or QApplication([])
    _record_animations()
    _record_tracking_animation()
    _grab_screenshots()
    _print_interactive_checklist()
    del app


if __name__ == "__main__":
    main()
