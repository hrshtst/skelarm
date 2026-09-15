# Copyright (C) 2025-2026 Hiroshi Atsuta <atsuta@ieee.org>
# SPDX-License-Identifier: GPL-3.0-only

"""Generate the reproducible documentation assets in ``docs/assets``.

Runs entirely headless (offscreen Qt, Agg matplotlib) and is repeatable, so the
committed assets can be regenerated at any time (logs embed fresh creation
timestamps, and pixel output can vary across font/Qt versions):

- **Animations** — each demo scenario is run headlessly with ``run_scenario``,
  saved as a replayable ``.sklog.npz``, and rendered with the player's export
  pipeline (task overlays and the side panel included) to an **MP4** — used by
  the MkDocs pages via a ``<video>`` tag (smaller, smoother, scrubbable) — plus
  a **GIF** for the few stems the README embeds inline (GitHub only plays GIFs
  inline; see ``_README_GIFS``).
- **Figures** — the plotting examples are executed with ``plt.show`` redirected
  to ``savefig``.
- **Screenshots** — the GUI tools are instantiated offscreen and grabbed as
  PNGs.
- **Screen recordings** — every hand-captured ``.mp4`` dropped into
  ``docs/assets`` (any stem this script does not generate itself) gets a README
  GIF derived from it with ffmpeg's palette pipeline.

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

from skelarm import StateLog, scenario_from_config

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # allow `tools.` imports when run as a script
from tools._scenario_cli import ScenarioSimulator, build_scenario, save_scenario_run
from tools.dynamics_simulator import DynamicsSimulator
from tools.kinematics_inspector import KinematicsInspector
from tools.player import PlaybackWindow

_REPO = Path(__file__).resolve().parents[1]
_EXAMPLES = _REPO / "examples"
_ASSETS = _REPO / "docs" / "assets"

_GIF_FPS = 20.0  # GIF frame rate; modest to keep the committed GIFs small
_MP4_FPS = 30.0  # MP4 affords a smoother rate at little size cost
_FRAME_SIZE_PX = 480  # square canvas size; modest to keep the committed files small

# (asset stem, scenario config, playback speed) — speed > 1 compresses long runs.
_ANIMATIONS: tuple[tuple[str, Path, float], ...] = (
    ("reach", _EXAMPLES / "reach.toml", 1.0),
    ("mpc_reach", _EXAMPLES / "mpc.toml", 1.0),
    ("periodic_curve", _EXAMPLES / "periodic_curve.toml", 2.0),
    ("multi_target", _EXAMPLES / "multi_target.toml", 1.0),
)

# Plotting examples rendered to PNG (each calls plt.show() exactly once).
_FIGURES: tuple[str, ...] = (
    "basic_plotting",
    "inverse_kinematics",
    "reaching",
    "periodic_curve",
    "filtering_demo",
    "interpolation_demo",
)

# Interactive capture logs (exported from a GUI's Record / Export…, see the checklist);
# each present <stem>.sklog.npz is rendered to <stem>.gif with the side panel.
_INTERACTIVE_CAPTURES: tuple[str, ...] = ("dynamics_drag", "reach_disturb", "multi_target_switch")

# The teaching scenario: a 4-DOF arm whose [task] target is drawn while teaching,
# giving the demonstration a goal; the tracking animation keeps the same marker.
_TEACH_CONFIG = _EXAMPLES / "reach_four_dof_robot.toml"

# Stems the README embeds as inline GIFs; everything else is MP4-only (the docs
# pages use <video> tags), keeping multi-megabyte unused GIFs out of the repo.
_README_GIFS = frozenset({"reach", "reach_disturb", "teach_mouse"})


def _export_animation(log_path: Path, stem: str, *, speed: float = 1.0) -> None:
    """Render a saved log to a side-panel MP4 (docs pages) and, for README stems, a GIF."""
    window = PlaybackWindow(StateLog.load(log_path), speed=speed)
    formats = [(".mp4", _MP4_FPS)] + ([(".gif", _GIF_FPS)] if stem in _README_GIFS else [])
    for suffix, fps in formats:
        out = _ASSETS / f"{stem}{suffix}"
        frames = window.export(out, fps=fps, size=_FRAME_SIZE_PX, panel=True)
        print(f"wrote {frames} frames to {out}")
    window.close()


def _record_animations() -> None:
    """Run each demo scenario headlessly; save its replayable log and its GIF/MP4 pair."""
    for stem, config, speed in _ANIMATIONS:
        log_path = save_scenario_run(build_scenario(config), _ASSETS / f"{stem}.sklog.npz")
        _export_animation(log_path, stem, speed=speed)


def _export_interactive_captures() -> None:
    """Render a panel GIF/MP4 pair from every interactive capture log present in docs/assets."""
    for stem in _INTERACTIVE_CAPTURES:
        log_path = _ASSETS / f"{stem}.sklog.npz"
        if not log_path.exists():
            print(f"skipping {stem}.gif/.mp4: no {log_path} (capture one, see the checklist below)")
            continue
        _export_animation(log_path, stem)


def _convert_screen_recordings() -> None:
    """Derive a README GIF from the hand-captured screen recordings the README embeds."""
    import imageio_ffmpeg

    generated = {stem for stem, _, _ in _ANIMATIONS} | set(_INTERACTIVE_CAPTURES) | {"trajectory_tracking"}
    ffmpeg = imageio_ffmpeg.get_ffmpeg_exe()
    filters = "fps=12,scale=640:-1:flags=lanczos,split[s0][s1];[s0]palettegen[p];[s1][p]paletteuse"
    for mp4 in sorted(_ASSETS.glob("*.mp4")):
        if mp4.stem in generated or mp4.stem not in _README_GIFS:
            continue  # docs pages embed the MP4 directly; only README stems need a GIF
        gif = mp4.with_suffix(".gif")
        subprocess.run(  # noqa: S603
            [ffmpeg, "-y", "-loglevel", "error", "-i", str(mp4), "-vf", filters, "-loop", "0", str(gif)],
            check=True,
        )
        print(f"wrote {gif} ({gif.stat().st_size // 1024} KiB) from {mp4.name}")


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
        # Repo-relative provenance (main() pins the cwd to the repo root); the built
        # scenario inlines the reference samples, so the log stays self-contained
        # and no machine-local path is embedded anywhere.
        "file": "docs/assets/teach.sklog.npz",
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
    log_path = save_scenario_run(scenario_from_config(config), _ASSETS / "trajectory_tracking.sklog.npz")
    _export_animation(log_path, "trajectory_tracking")


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
   -> docs/assets/dynamics_drag.sklog.npz        (becomes dynamics_drag.gif + .mp4)

2. Disturbing a reach      uv run python tools/reaching_simulator.py examples/reach.toml --run
   Do: let the arm reach the purple target, then drag the tip away and release —
   the controller pulls it back. Export… the log as
   -> docs/assets/reach_disturb.sklog.npz        (becomes reach_disturb.gif + .mp4)

3. Live target switching   uv run python tools/multi_target_simulator.py examples/multi_target.toml --run
   Do: while it runs, press 1, 2, … to switch the active target and watch the
   arm retarget. Export… the log as
   -> docs/assets/multi_target_switch.sklog.npz  (becomes multi_target_switch.gif + .mp4)

4. Teaching a trajectory   uv run python tools/trajectory_recorder.py \\
                               examples/reach_four_dof_robot.toml --output docs/assets/teach.sklog.npz
   Do: the purple target is the goal — press Space to start the take, grab
   the tip and demonstrate a smooth motion toward it, then press S to save
   and Q to close. Re-running this script then generates
   trajectory_tracking.sklog.npz / trajectory_tracking.gif on the same robot,
   with the target marker kept in the replay.

5. FK/IK posing            uv run python tools/kinematics_inspector.py examples/four_dof_robot.toml --show-com
   Do: this one is a SCREEN RECORDING (the inspector poses kinematically and
   records nothing; the cursor is the demo). Drag a joint slider through its
   range, then click-drag the tip so the IK solution follows the cursor.
   Capture the window (e.g. Kooha/OBS) as MP4 and save it as
   -> docs/assets/kinematics_posing_demo.mp4
   The docs embed hand-captured MP4s directly; a README GIF is derived only for
   the stems listed in _README_GIFS.
"""
    )


def main() -> None:
    """Generate every reproducible asset, then print the interactive checklist."""
    os.chdir(_REPO)  # embedded reference paths are repo-relative; resolve them regardless of caller cwd
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
    _convert_screen_recordings()
    _grab_screenshots()
    _print_interactive_checklist()
    del app


if __name__ == "__main__":
    main()
