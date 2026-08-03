# Copyright (C) 2025-2026 Hiroshi Atsuta <atsuta@ieee.org>
# SPDX-License-Identifier: GPL-3.0-only

"""Generate the reproducible documentation assets in ``docs/assets``.

Runs entirely headless (offscreen Qt, Agg matplotlib) and is deterministic, so
the committed assets can be regenerated at any time:

- **Animations** — each demo scenario is run headlessly with ``run_scenario``,
  saved as a replayable ``.sklog.npz`` next to its GIF, and rendered with the
  player's export pipeline (task overlays and the side panel included).
- **Figures** — the plotting examples are executed with ``plt.show`` redirected
  to ``savefig``.
- **Screenshots** — the GUI tools are instantiated offscreen and grabbed as
  PNGs.

Interactive demonstrations (mouse dragging, live target switching, trajectory
teaching) cannot be scripted; the checklist printed at the end lists the exact
commands, actions, and file names to capture them. Most only need the GUI's own
**Record** / **Export…** — save the log under the suggested name in
``docs/assets`` and re-run this script: it renders a panel GIF from every
capture log it finds (the replay draws the recorded drag force, friction, and
target switches), and generates the trajectory-tracking animation once a taught
``teach.sklog.npz`` exists.

Usage::

    uv run python tools/generate_assets.py
"""

from __future__ import annotations

import os
import runpy
import subprocess
import sys
import tomllib
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")  # render Qt without a display
os.environ.setdefault("MPLBACKEND", "Agg")  # render matplotlib without a display

from PyQt6.QtWidgets import QApplication, QWidget

from skelarm import StateLog
from skelarm.recording import dump_toml

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

# Interactive capture logs (exported from a GUI's Record / Export…, see the checklist);
# each present <stem>.sklog.npz is rendered to <stem>.gif with the side panel.
_INTERACTIVE_CAPTURES: tuple[str, ...] = ("dynamics_drag", "reach_disturb", "multi_target_switch")

# The teaching scenario: a 4-DOF arm whose [task] target is drawn while teaching,
# giving the demonstration a goal; the tracking animation keeps the same marker.
_TEACH_CONFIG = _EXAMPLES / "reach_four_dof_robot.toml"


def _export_gif(log_path: Path, gif_path: Path, *, speed: float = 1.0) -> None:
    """Render a saved log to a side-panel GIF via the player's headless export pipeline."""
    window = PlaybackWindow(StateLog.load(log_path), speed=speed)
    frames = window.export(gif_path, fps=_GIF_FPS, size=_GIF_SIZE_PX, panel=True)
    window.close()
    print(f"wrote {frames} frames to {gif_path}")


def _record_animations() -> None:
    """Run each demo scenario headlessly; save its replayable log and its GIF."""
    for stem, config, speed in _ANIMATIONS:
        log_path = save_scenario_run(build_scenario(config), _ASSETS / f"{stem}.sklog.npz")
        _export_gif(log_path, _ASSETS / f"{stem}.gif", speed=speed)


def _export_interactive_captures() -> None:
    """Render a panel GIF from every interactive capture log present in docs/assets."""
    for stem in _INTERACTIVE_CAPTURES:
        log_path = _ASSETS / f"{stem}.sklog.npz"
        if not log_path.exists():
            print(f"skipping {stem}.gif: no {log_path} (capture one, see the checklist below)")
            continue
        _export_gif(log_path, _ASSETS / f"{stem}.gif")


def _record_tracking_animation() -> None:
    """Generate the trajectory-tracking assets when a taught reference exists.

    The track scenario reuses the teaching config's robot, start pose, and target
    marker, swapping the task for ``joint_trajectory_tracking`` of the taught log.
    """
    teach_log = _ASSETS / "teach.sklog.npz"
    if not teach_log.exists():
        print(f"skipping tracking animation: no {teach_log} (teach one first, see the checklist below)")
        return
    config = tomllib.loads(_TEACH_CONFIG.read_text(encoding="utf-8"))
    task_table: dict[str, object] = {
        "type": "joint_trajectory_tracking",
        "file": str(teach_log.resolve()),  # the reference path resolves against the cwd, so keep it absolute
        "filter": {"kind": "butterworth", "cutoff_hz": 8.0, "order": 4},
        "interpolator": "cubic_spline",
        # duration omitted: defaults to the taught reference's length
    }
    target = config.get("task", {}).get("target")
    if target is not None:
        task_table["target"] = target  # keep the taught scenario's goal marker in the replay
    config["task"] = task_table
    config["simulator"] = {"dt": 0.002}
    config["controller"] = {"type": "computed_torque", "kp": 200.0, "kd": 30.0}
    track_config = _ASSETS / "track.toml"  # regenerable intermediate (gitignored)
    track_config.write_text(dump_toml(config).strip() + "\n", encoding="utf-8")
    log_path = save_scenario_run(build_scenario(track_config), _ASSETS / "trajectory_tracking.sklog.npz")
    _export_gif(log_path, _ASSETS / "trajectory_tracking.gif")


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
    """List the demonstrations that need a human, with the file names this script expects."""
    print(
        """
All reproducible assets are in docs/assets/. The remaining demonstrations need a
human. Most only need the GUI's own recording: perform the actions, press
Export…, save the log under the name given below, then RE-RUN THIS SCRIPT — it
renders a side-panel GIF from each capture log it finds (the replay draws the
recorded drag force, friction, and target switches; your cursor is not shown).

1. Drag-to-perturb         uv run python tools/dynamics_simulator.py examples/four_dof_robot.toml --run
   Do: press and drag the tip; release and watch the arm swing freely. Raise the
   friction spin box mid-run to damp it. Export… the log as
   -> docs/assets/dynamics_drag.sklog.npz        (becomes dynamics_drag.gif)

2. Disturbing a reach      uv run python tools/reaching_simulator.py examples/reach.toml --run
   Do: let the arm reach the purple target, then drag the tip away and release —
   the controller pulls it back. Export… the log as
   -> docs/assets/reach_disturb.sklog.npz        (becomes reach_disturb.gif)

3. Live target switching   uv run python tools/multi_target_simulator.py examples/multi_target.toml --run
   Do: while it runs, press 1, 2, … to switch the active target and watch the
   arm retarget. Export… the log as
   -> docs/assets/multi_target_switch.sklog.npz  (becomes multi_target_switch.gif)

4. Teaching a trajectory   uv run python tools/trajectory_recorder.py \\
                               examples/reach_four_dof_robot.toml --output docs/assets/teach.sklog.npz
   Do: the purple target is the goal — grab the tip and demonstrate a smooth
   motion toward it (recording starts on the first grab), then press F to
   finish; the log saves itself. Re-running this script then generates
   trajectory_tracking.sklog.npz / trajectory_tracking.gif on the same robot,
   with the target marker kept in the replay.

5. FK/IK posing            uv run python tools/kinematics_inspector.py examples/four_dof_robot.toml --show-com
   Do: this one is a SCREEN RECORDING (the inspector poses kinematically and
   records nothing; the cursor is the demo). Drag a joint slider through its
   range, then click-drag the tip so the IK solution follows the cursor.
   Capture the window with e.g. Peek or OBS and save it as
   -> docs/assets/kinematics_posing_demo.gif
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
    _export_interactive_captures()
    _record_tracking_animation()
    _grab_screenshots()
    _print_interactive_checklist()
    del app


if __name__ == "__main__":
    main()
