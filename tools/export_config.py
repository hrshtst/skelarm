"""
Export the embedded scenario config from a skelarm state log.

Load a ``.sklog.npz`` recording produced by ``run_scenario`` / ``tools/reaching_simulator.py``
and write its embedded scenario config — the editable ``[skeleton]`` / ``[initial]``
/ ``[task]`` / ``[controller]`` tables — to a TOML file. Edit a value and re-run it
with ``tools/reaching_simulator.py`` to compare; an unedited re-run reproduces the original
exactly for the deterministic controllers.

Usage::

    uv run python tools/export_config.py run.sklog.npz
    uv run python tools/export_config.py run.sklog.npz --output edited.toml
    uv run python tools/reaching_simulator.py edited.toml   # re-run the (edited) scenario
"""

from __future__ import annotations

import argparse
from pathlib import Path

from skelarm import export_scenario_toml
from skelarm.recording import StateLog

_LOG_SUFFIX = ".sklog.npz"


def build_parser() -> argparse.ArgumentParser:
    """Build the command-line argument parser for the tool."""
    parser = argparse.ArgumentParser(description="Export the scenario config embedded in a skelarm state log.")
    parser.add_argument("logfile", type=Path, help="path to a .sklog.npz state log")
    parser.add_argument(
        "--output",
        "-o",
        type=Path,
        default=None,
        help="output .toml path (default: the log path with a .toml suffix)",
    )
    return parser


def _default_output(logfile: Path) -> Path:
    """The log path with its ``.sklog.npz`` (or other) suffix replaced by ``.toml``."""
    name = logfile.name
    stem = name[: -len(_LOG_SUFFIX)] if name.endswith(_LOG_SUFFIX) else logfile.stem
    return logfile.with_name(stem + ".toml")


def export_config(logfile: str | Path, *, output: str | Path | None = None) -> Path:
    """Load a state log and write its embedded scenario config to a TOML file.

    Parameters
    ----------
    logfile : str | Path
        Path to a ``.sklog.npz`` state log produced by :func:`skelarm.run_scenario`.
    output : str | Path | None, optional
        Destination ``.toml`` path; defaults to ``logfile`` with a ``.toml`` suffix.

    Returns
    -------
    Path
        The path the config was written to.

    Raises
    ------
    ValueError
        If the log carries no embedded scenario config.
    """
    logfile = Path(logfile)
    log = StateLog.load(logfile)
    out_path = Path(output) if output is not None else _default_output(logfile)
    export_scenario_toml(log, out_path)
    return out_path


def main() -> None:
    """Parse arguments, load the log, and export its scenario config."""
    parser = build_parser()
    args = parser.parse_args()
    if not args.logfile.exists():
        parser.error(f"log file not found: {args.logfile}")
    try:
        out_path = export_config(args.logfile, output=args.output)
    except (OSError, ValueError, KeyError) as exc:
        parser.error(str(exc))
    print(f"wrote scenario config to {out_path}")


if __name__ == "__main__":
    main()
