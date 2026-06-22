"""
Run a reaching scenario from a TOML config and export the recorded run.

Loads a combined config (``[skeleton]`` / ``[initial]`` / ``[task]`` /
``[controller]``; see :func:`skelarm.scenario.load_scenario`), simulates the reach
with the fixed-step control loop, and writes a ``.sklog.npz`` state log that can be
replayed and analyzed with ``tools/replay.py``.

Usage::

    uv run python tools/reach.py examples/reach.toml
    uv run python tools/reach.py examples/reach.toml --output run.sklog.npz
    uv run python tools/replay.py run.sklog.npz   # then replay/analyze the result
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from skelarm.scenario import load_scenario, run_scenario


def build_parser() -> argparse.ArgumentParser:
    """Build the command-line argument parser for the tool."""
    parser = argparse.ArgumentParser(description="Run a skelarm reaching scenario and export the recorded run.")
    parser.add_argument("config", type=Path, help="path to a combined scenario TOML config")
    parser.add_argument(
        "--output", type=Path, default=None, help="output .sklog.npz path (default: <config>.sklog.npz)"
    )
    parser.add_argument("--duration", type=float, default=None, help="override the task's simulated duration (s)")
    return parser


def run_reach(config: str | Path, *, output: str | Path | None = None, duration: float | None = None) -> Path:
    """Load a scenario, simulate the reach, save the state log, and return its path.

    Parameters
    ----------
    config : str | Path
        Path to the combined scenario TOML config.
    output : str | Path | None, optional
        Output ``.sklog.npz`` path; defaults to the config path with that suffix.
    duration : float | None, optional
        Override the task's simulated duration (seconds).

    Returns
    -------
    Path
        The path the state log was written to.
    """
    config = Path(config)
    scenario = load_scenario(config)
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
        run_reach(args.config, output=args.output, duration=args.duration)
    except (FileNotFoundError, ValueError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    main()
