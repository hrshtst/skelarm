"""Tests for the config-export tool (tools/export_config.py)."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from skelarm.recording import StateLog
from skelarm.scenario import load_scenario, run_scenario
from skelarm.skeleton import LinkProp, Skeleton
from tools.export_config import build_parser, export_config

_SCENARIO_TOML = (
    "[skeleton]\n"
    "[[skeleton.link]]\nlength = 1.0\nmass = 1.0\ninertia = 0.1\ncom = [0.5, 0.0]\nlimits = [-180.0, 180.0]\n"
    "[[skeleton.link]]\nlength = 0.8\nmass = 0.8\ninertia = 0.05\ncom = [0.4, 0.0]\nlimits = [-180.0, 180.0]\n"
    "[initial]\nq = [34.4, 57.3]\n"
    "[task]\ntarget = [0.55, 1.21]\nduration = 0.03\ndt = 0.01\n"
    '[controller]\ntype = "computed_torque"\nkp = 200.0\nkd = 30.0\n'
)


def _write_log(tmp_path: Path) -> Path:
    """Run a tiny scenario and save it as a .sklog.npz; return the log path."""
    config = tmp_path / "reach.toml"
    config.write_text(_SCENARIO_TOML, encoding="utf-8")
    log_path = tmp_path / "run.sklog.npz"
    run_scenario(load_scenario(config)).save(log_path)
    return log_path


def test_parser_requires_logfile() -> None:
    """The logfile argument is required."""
    with pytest.raises(SystemExit):
        build_parser().parse_args([])


def test_export_config_writes_specified_filename(tmp_path: Path) -> None:
    """Exporting to a specified output writes an editable, loadable scenario config."""
    log_path = _write_log(tmp_path)
    output = tmp_path / "edited.toml"

    written = export_config(log_path, output=output)

    assert written == output
    assert type(load_scenario(output).controller).__name__ == "ComputedTorque"


def test_export_config_defaults_output_beside_log(tmp_path: Path) -> None:
    """Without an output the config is written beside the log with a .toml suffix."""
    log_path = _write_log(tmp_path)

    written = export_config(log_path)

    assert written == tmp_path / "run.toml"
    assert written.exists()


def test_export_config_rejects_log_without_metadata(tmp_path: Path) -> None:
    """A log with no embedded scenario config is rejected."""
    skeleton = Skeleton([LinkProp(length=1.0, m=1.0, i=0.1, rgx=0.5, rgy=0.0, qmin=-np.pi, qmax=np.pi)])
    log = StateLog(skeleton)
    log.record(0.0, q=[0.0])
    bare = tmp_path / "bare.sklog.npz"
    log.save(bare)

    with pytest.raises(ValueError, match="scenario config"):
        export_config(bare)


def test_runs_as_a_standalone_script() -> None:
    """Running the file directly (script mode) must resolve all of its imports."""
    script = Path(__file__).resolve().parents[1] / "tools" / "export_config.py"
    result = subprocess.run(  # noqa: S603  # trusted: our own interpreter and script path
        [sys.executable, str(script), "--help"],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    assert "config" in result.stdout.lower()
