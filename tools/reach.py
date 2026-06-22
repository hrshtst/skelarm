"""
Run a reaching scenario from a TOML config and export the recorded run.

Loads a combined config (``[skeleton]`` / ``[initial]`` / ``[task]`` /
``[controller]``; see :func:`skelarm.scenario.load_scenario`), simulates the reach
with the fixed-step control loop, and writes a ``.sklog.npz`` state log that can be
replayed and analyzed with ``tools/replay.py``.

The ``[initial]`` / ``[task]`` / ``[controller]`` sections can be overridden from
separate files on the command line, so the same base robot can be compared across
controllers (``--controller``) or tasks (``--task``) without editing the base file.

Usage::

    uv run python tools/reach.py examples/reach.toml
    uv run python tools/reach.py examples/reach.toml --output run.sklog.npz
    uv run python tools/reach.py examples/reach.toml --controller pd.toml      # same task, other controller
    uv run python tools/reach.py examples/reach.toml --task far.toml           # same controller, other task
    uv run python tools/reach.py examples/reach.toml --initial pose.toml --pose 20,45
    uv run python tools/replay.py run.sklog.npz   # then replay/analyze the result
"""

from __future__ import annotations

import argparse
import tomllib
from pathlib import Path
from typing import Any

import numpy as np

from skelarm.scenario import run_scenario, scenario_from_config


def build_parser() -> argparse.ArgumentParser:
    """Build the command-line argument parser for the tool."""
    parser = argparse.ArgumentParser(description="Run a skelarm reaching scenario and export the recorded run.")
    parser.add_argument("config", type=Path, help="path to a combined scenario TOML config")
    parser.add_argument(
        "--output", type=Path, default=None, help="output .sklog.npz path (default: <config>.sklog.npz)"
    )
    parser.add_argument("--duration", type=float, default=None, help="override the task's simulated duration (s)")
    parser.add_argument(
        "--initial", type=Path, default=None, help="TOML file whose [initial] table overrides the base initial pose"
    )
    parser.add_argument(
        "--pose", default=None, help="initial joint angles in degrees, e.g. 20,45,60,30 (overrides --initial)"
    )
    parser.add_argument("--task", type=Path, default=None, help="TOML file whose [task] table overrides the base task")
    parser.add_argument(
        "--controller",
        type=Path,
        default=None,
        help="TOML file whose [controller] table overrides the base controller",
    )
    return parser


def _read_toml(path: Path) -> dict[str, Any]:
    """Parse a TOML file into a dict."""
    with path.open("rb") as f:
        return tomllib.load(f)


def _read_section(path: Path, section: str) -> dict[str, Any]:
    """Read a single ``[section]`` table from an override TOML file."""
    if not path.exists():
        msg = f"override file not found: {path}"
        raise FileNotFoundError(msg)
    data = _read_toml(path)
    if section not in data:
        msg = f"no [{section}] section in {path}"
        raise ValueError(msg)
    return dict(data[section])


def _apply_overrides(
    config: dict[str, Any],
    *,
    initial: Path | None,
    pose: str | None,
    task: Path | None,
    controller: Path | None,
) -> None:
    """Overwrite the config's ``[initial]`` / ``[task]`` / ``[controller]`` tables in place.

    ``--initial`` replaces the whole ``[initial]`` table; ``--pose`` then overrides
    just its ``q`` (degrees), matching the kinematics/dynamics tools.
    """
    if task is not None:
        config["task"] = _read_section(task, "task")
    if controller is not None:
        config["controller"] = _read_section(controller, "controller")
    if initial is not None:
        config["initial"] = _read_section(initial, "initial")
    if pose is not None:
        config.setdefault("initial", {})["q"] = [float(value) for value in pose.split(",")]


def run_reach(
    config: str | Path,
    *,
    output: str | Path | None = None,
    duration: float | None = None,
    initial: str | Path | None = None,
    pose: str | None = None,
    task: str | Path | None = None,
    controller: str | Path | None = None,
) -> Path:
    """Load a scenario (with optional overrides), simulate the reach, and save the log.

    Parameters
    ----------
    config : str | Path
        Path to the combined scenario TOML config.
    output : str | Path | None, optional
        Output ``.sklog.npz`` path; defaults to the config path with that suffix.
    duration : float | None, optional
        Override the task's simulated duration (seconds).
    initial : str | Path | None, optional
        TOML file whose ``[initial]`` table overrides the base initial pose.
    pose : str | None, optional
        Comma-separated joint angles in degrees overriding ``q`` (after ``initial``).
    task : str | Path | None, optional
        TOML file whose ``[task]`` table overrides the base task.
    controller : str | Path | None, optional
        TOML file whose ``[controller]`` table overrides the base controller.

    Returns
    -------
    Path
        The path the state log was written to.
    """
    config = Path(config)
    data = _read_toml(config)
    _apply_overrides(
        data,
        initial=None if initial is None else Path(initial),
        pose=pose,
        task=None if task is None else Path(task),
        controller=None if controller is None else Path(controller),
    )
    scenario = scenario_from_config(data)
    log = run_scenario(scenario, duration=duration)

    out_path = Path(output) if output is not None else config.with_suffix(".sklog.npz")
    log.save(out_path)

    tip = scenario.skeleton.clone()
    tip.q = log.channel("q")[-1]
    endpoint = np.array([tip.links[-1].xe, tip.links[-1].ye])
    error = float(np.linalg.norm(endpoint - scenario.task.target))
    print(f"reach: {type(scenario.controller).__name__} -> tip ({endpoint[0]:.3f}, {endpoint[1]:.3f}) m")
    print(f"target ({scenario.task.target[0]:.3f}, {scenario.task.target[1]:.3f}) m, error {error * 1000:.2f} mm")
    print(f"wrote {len(log)} frames to {out_path}")
    return out_path


def main() -> None:
    """Parse arguments, run the reach, and export the state log."""
    parser = build_parser()
    args = parser.parse_args()
    if not args.config.exists():
        parser.error(f"config file not found: {args.config}")
    try:
        run_reach(
            args.config,
            output=args.output,
            duration=args.duration,
            initial=args.initial,
            pose=args.pose,
            task=args.task,
            controller=args.controller,
        )
    except (FileNotFoundError, ValueError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    main()
