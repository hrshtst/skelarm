"""Tests for the StateLog time-series recorder and its file format."""

from __future__ import annotations

import tomllib

import numpy as np
import pytest

from skelarm.recording import StateLog, dump_toml
from skelarm.skeleton import LinkProp, Skeleton


def _skeleton() -> Skeleton:
    """Build a simple two-link arm."""
    link_props = [
        LinkProp(length=0.3, m=1.0, i=0.01, rgx=0.15, rgy=0.0, qmin=-np.pi, qmax=np.pi),
        LinkProp(length=0.2, m=0.5, i=0.005, rgx=0.1, rgy=0.0, qmin=-np.pi / 2, qmax=np.pi / 2),
    ]
    return Skeleton(link_props, base_length=0.05)


def test_record_accumulates_channels() -> None:
    """Recording rows exposes time and stacked per-channel arrays."""
    log = StateLog(_skeleton(), producer="test")
    log.record(0.0, q=[0.1, 0.2], dq=[0.0, 0.0])
    log.record(0.1, q=[0.15, 0.25], dq=[0.5, 0.5])

    expected_frames = 2
    assert len(log) == expected_frames
    assert log.channel_names == ["q", "dq"]
    assert log.times == pytest.approx([0.0, 0.1])
    assert log.channel("q").shape == (2, 2)
    assert log.channel("q")[1] == pytest.approx([0.15, 0.25])


def test_scalar_channel_is_one_dimensional() -> None:
    """A scalar channel recorded each step stacks to a 1-D array."""
    log = StateLog(_skeleton())
    log.record(0.0, energy=0.0)
    log.record(0.1, energy=0.5)
    assert log.channel("energy").shape == (2,)
    assert log.channel("energy") == pytest.approx([0.0, 0.5])


def test_record_rejects_inconsistent_channel_set() -> None:
    """The channel set is fixed by the first record."""
    log = StateLog()
    log.record(0.0, q=[0.1, 0.2])
    with pytest.raises(ValueError, match="channel"):
        log.record(0.1, q=[0.1, 0.2], extra=[1.0])


def test_record_rejects_inconsistent_channel_width() -> None:
    """A channel's width must stay constant across rows."""
    log = StateLog()
    log.record(0.0, q=[0.1, 0.2])
    with pytest.raises(ValueError, match="shape"):
        log.record(0.1, q=[0.1, 0.2, 0.3])


def test_record_skeleton_captures_state() -> None:
    """record_skeleton snapshots q, dq, and tau from the skeleton plus extras."""
    skeleton = _skeleton()
    skeleton.q = np.array([0.2, 0.3])
    log = StateLog(skeleton)
    log.record_skeleton(skeleton, time=0.0, target=[0.4, 0.1])

    assert set(log.channel_names) == {"q", "dq", "tau", "target"}
    assert log.channel("q")[0] == pytest.approx([0.2, 0.3])
    assert log.channel("target")[0] == pytest.approx([0.4, 0.1])


def test_save_load_round_trips(tmp_path) -> None:  # noqa: ANN001
    """Saving then loading preserves channels, metadata, and the robot."""
    skeleton = _skeleton()
    log = StateLog(
        skeleton,
        producer="dynamics_simulator",
        channel_meta={"q": {"unit": "rad", "label": "joint angle", "columns": ["q1", "q2"]}},
    )
    log.record(0.0, q=[0.1, 0.2], dq=[0.0, 0.0], energy=0.0)
    log.record(0.1, q=[0.2, 0.3], dq=[1.0, 1.0], energy=0.5)

    path = tmp_path / "run.sklog.npz"
    log.save(path)
    loaded = StateLog.load(path)

    assert loaded.producer == "dynamics_simulator"
    assert loaded.times == pytest.approx(log.times)
    assert loaded.channel("q") == pytest.approx(log.channel("q"))
    assert loaded.channel("energy") == pytest.approx([0.0, 0.5])
    assert loaded.channel_meta["q"]["unit"] == "rad"
    assert loaded.channel_meta["q"]["columns"] == ["q1", "q2"]

    restored = loaded.build_skeleton()
    assert restored.num_joints == skeleton.num_joints
    for original, copy in zip(skeleton.links[1:], restored.links[1:], strict=True):
        assert copy.prop == original.prop


def test_export_toml_is_human_readable_and_parses(tmp_path) -> None:  # noqa: ANN001
    """The TOML export embeds metadata and the data arrays, and parses back."""
    log = StateLog(_skeleton(), producer="dyn")
    log.record(0.0, q=[0.1, 0.2])
    log.record(0.1, q=[0.2, 0.3])

    path = tmp_path / "run.toml"
    log.export_toml(path)
    data = tomllib.loads(path.read_text(encoding="utf-8"))

    assert data["producer"] == "dyn"
    assert data["skeleton"]["base_length"] == pytest.approx(0.05)
    assert data["data"]["time"] == pytest.approx([0.0, 0.1])
    assert np.array(data["data"]["q"]) == pytest.approx(np.array([[0.1, 0.2], [0.2, 0.3]]))


def test_dump_toml_round_trips_via_tomllib() -> None:
    """The minimal TOML writer round-trips scalars, arrays, tables, and tables-arrays."""
    source = {
        "schema_version": 1,
        "name": "hello",
        "ratio": 3.5,
        "flag": True,
        "skeleton": {"base_length": 0.05, "links": [{"length": 0.3, "qmin": -3.0}, {"length": 0.2, "qmin": -1.5}]},
        "channels": {"q": {"unit": "rad", "columns": ["q1", "q2"]}},
        "data": {"q": [[0.1, 0.2], [0.3, 0.4]]},
    }
    assert tomllib.loads(dump_toml(source)) == source
