"""Time-series logging of robot states with TOML-described ``.sklog.npz`` files.

A :class:`StateLog` records a sequence of frames, each a timestamp plus a set of
named channels (``q``, ``dq``, ``tau`` and any extra signals a producer wants to
keep, e.g. a controller's target or tracking error). The container is
schema-agnostic: it stores whatever channels it is given, so new signals need no
format change.

The canonical on-disk format is a numpy ``.npz`` archive holding one array per
channel plus a ``__meta__`` member with TOML metadata (schema version, producer,
the embedded robot geometry, per-channel units/labels, and any free-form ``extra``
metadata such as the scenario config that produced the run). A standalone
human-readable ``.toml`` export is also available for small logs.
"""

from __future__ import annotations

import re
import tomllib
from datetime import UTC, datetime
from typing import TYPE_CHECKING, Any

import numpy as np

from skelarm.skeleton import Skeleton

if TYPE_CHECKING:
    from collections.abc import Mapping
    from pathlib import Path

    from numpy.typing import ArrayLike, NDArray

_SCHEMA_VERSION = 1
_META_KEY = "__meta__"  # npz member holding the TOML metadata string
_TIME = "time"
_BARE_KEY = re.compile(r"^[A-Za-z0-9_-]+$")


class StateLog:
    """A time series of robot states, savable to and loadable from a file.

    Parameters
    ----------
    skeleton : Skeleton, optional
        The robot whose geometry is embedded in the log so it can be replayed or
        analyzed without the original config. Required for :meth:`build_skeleton`.
    producer : str, optional
        A free-form name of whatever generated the log (e.g. ``"dynamics_simulator"``).
    channel_meta : Mapping[str, Mapping[str, Any]], optional
        Optional per-channel descriptors (``unit``, ``label``, ``columns``) used to
        document the physical meaning of each channel and label analysis plots.
    extra : Mapping[str, Any], optional
        Optional extra metadata stored under an ``[extra]`` table.
    """

    def __init__(
        self,
        skeleton: Skeleton | None = None,
        *,
        producer: str = "",
        channel_meta: Mapping[str, Mapping[str, Any]] | None = None,
        extra: Mapping[str, Any] | None = None,
    ) -> None:
        """Initialize an empty log."""
        self._skeleton_dict: dict[str, Any] | None = skeleton.to_dict() if skeleton is not None else None
        self._producer = producer
        self._channel_meta: dict[str, dict[str, Any]] = {k: dict(v) for k, v in (channel_meta or {}).items()}
        self._extra: dict[str, Any] = dict(extra or {})
        self._created_at = datetime.now(UTC).isoformat()
        self._records: list[dict[str, NDArray[np.float64]]] = []
        self._channel_order: list[str] | None = None

    def record(self, time: float, **channels: ArrayLike) -> None:
        """Append one frame at ``time`` with the given named channels.

        The set of channel names and each channel's width are fixed by the first
        call; later calls must match.

        Parameters
        ----------
        time : float
            The frame timestamp.
        **channels : ArrayLike
            Named channel values for this frame (scalars or 1-D arrays).

        Raises
        ------
        ValueError
            If the channel names or a channel's shape differ from the first frame.
        """
        values = {name: np.asarray(value, dtype=np.float64) for name, value in channels.items()}
        if self._channel_order is None:
            self._channel_order = list(values)
        elif set(values) != set(self._channel_order):
            msg = f"channel set {sorted(values)} does not match the log's channels {sorted(self._channel_order)}"
            raise ValueError(msg)
        else:
            first = self._records[0]
            for name in self._channel_order:
                if values[name].shape != first[name].shape:
                    msg = f"channel {name!r} has shape {values[name].shape}, expected {first[name].shape}"
                    raise ValueError(msg)
        row = {_TIME: np.asarray(time, dtype=np.float64)}
        for name in self._channel_order:
            row[name] = values[name]
        self._records.append(row)

    def record_skeleton(self, skeleton: Skeleton, time: float, **extra: ArrayLike) -> None:
        """Record the canonical robot state (``q``, ``dq``, ``tau``) plus extra channels."""
        self.record(time, q=skeleton.q, dq=skeleton.dq, tau=skeleton.tau, **extra)

    def __len__(self) -> int:
        """Return the number of recorded frames."""
        return len(self._records)

    @property
    def producer(self) -> str:
        """The name of whatever produced the log."""
        return self._producer

    @property
    def created_at(self) -> str:
        """ISO-8601 timestamp of when the log was created."""
        return self._created_at

    @property
    def channel_meta(self) -> dict[str, dict[str, Any]]:
        """Per-channel descriptors (``unit`` / ``label`` / ``columns``)."""
        return self._channel_meta

    @property
    def extra(self) -> dict[str, Any]:
        """Free-form extra metadata stored under the ``[extra]`` table."""
        return self._extra

    @property
    def channel_names(self) -> list[str]:
        """The recorded channel names (excluding ``time``), in record order."""
        return list(self._channel_order) if self._channel_order is not None else []

    @property
    def times(self) -> NDArray[np.float64]:
        """The frame timestamps as a 1-D array."""
        return np.array([record[_TIME] for record in self._records], dtype=np.float64)

    def channel(self, name: str) -> NDArray[np.float64]:
        """Return channel ``name`` stacked over frames (shape ``(N,)`` or ``(N, k)``)."""
        if name == _TIME:
            return self.times
        if self._channel_order is None or name not in self._channel_order:
            msg = f"unknown channel {name!r}; available: {self.channel_names}"
            raise KeyError(msg)
        return np.stack([record[name] for record in self._records])

    def to_arrays(self) -> dict[str, NDArray[np.float64]]:
        """Return every channel (including ``time``) as a mapping of stacked arrays."""
        arrays = {_TIME: self.times}
        for name in self.channel_names:
            arrays[name] = self.channel(name)
        return arrays

    def build_skeleton(self) -> Skeleton:
        """Reconstruct the robot embedded in the log.

        Raises
        ------
        ValueError
            If the log carries no embedded robot geometry.
        """
        if self._skeleton_dict is None:
            msg = "this log has no embedded skeleton"
            raise ValueError(msg)
        return Skeleton.from_dict(self._skeleton_dict)

    def save(self, path: str | Path) -> None:
        """Save the log as a ``.sklog.npz`` archive (channel arrays + TOML metadata)."""
        meta_toml = _dump_toml(self._meta_dict())
        # dict[str, Any] so the meta string array and the float arrays can share one
        # ``**`` spread without tripping savez_compressed's typed ``allow_pickle`` kwarg.
        payload: dict[str, Any] = {**self.to_arrays(), _META_KEY: np.array(meta_toml)}
        np.savez_compressed(path, **payload)

    def export_toml(self, path: str | Path) -> None:
        """Write a standalone, human-readable TOML file (metadata + data arrays).

        Practical for small logs; for long runs prefer :meth:`save` (``.npz``).
        """
        document = self._meta_dict()
        document["data"] = {name: array.tolist() for name, array in self.to_arrays().items()}
        from pathlib import Path as _Path

        _Path(path).write_text(_dump_toml(document), encoding="utf-8")

    @classmethod
    def load(cls, path: str | Path) -> StateLog:
        """Load a log previously written by :meth:`save`."""
        with np.load(path, allow_pickle=False) as archive:
            meta = tomllib.loads(str(archive[_META_KEY]))
            arrays = {name: archive[name] for name in archive.files if name != _META_KEY}
        log = cls()
        log._populate(meta, arrays)
        return log

    def _populate(self, meta: Mapping[str, Any], arrays: dict[str, NDArray[np.float64]]) -> None:
        """Fill this (freshly constructed) log from loaded metadata and channel arrays."""
        self._skeleton_dict = meta.get("skeleton")
        self._producer = meta.get("producer", "")
        self._channel_meta = meta.get("channels", {})
        self._extra = meta.get("extra", {})
        self._created_at = meta.get("created_at", "")

        columns = {name: array for name, array in arrays.items() if name != _TIME}
        times = arrays[_TIME]
        self._channel_order = list(columns)
        self._records = [
            {_TIME: np.asarray(times[i], dtype=np.float64), **{name: columns[name][i] for name in columns}}
            for i in range(len(times))
        ]

    def _meta_dict(self) -> dict[str, Any]:
        """Assemble the TOML-serializable metadata document."""
        meta: dict[str, Any] = {
            "schema_version": _SCHEMA_VERSION,
            "created_at": self._created_at,
            "producer": self._producer,
        }
        if self._skeleton_dict is not None:
            meta["skeleton"] = self._skeleton_dict
        if self._channel_meta:
            meta["channels"] = self._channel_meta
        if self._extra:
            meta["extra"] = self._extra
        return meta


def _dump_toml(data: Mapping[str, Any]) -> str:
    """Serialize a constrained nested mapping to TOML text.

    Supports scalars (``bool``, ``int``, ``float``, ``str``), (possibly nested)
    lists of scalars, sub-tables (``dict``), and arrays of tables (``list`` of
    ``dict``). This is enough for :class:`StateLog` metadata and exports; stdlib
    ``tomllib`` reads the result back.
    """
    lines: list[str] = []
    _emit_table(data, [], lines)
    return "\n".join(lines) + "\n"


def _emit_table(table: Mapping[str, Any], path: list[str], lines: list[str]) -> None:
    """Emit a single TOML table, then its sub-tables and arrays of tables."""
    sub_tables: list[tuple[str, Mapping[str, Any]]] = []
    table_arrays: list[tuple[str, list[Any]]] = []
    for key, value in table.items():
        if isinstance(value, dict):
            sub_tables.append((key, value))
        elif isinstance(value, list) and value and all(isinstance(item, dict) for item in value):
            table_arrays.append((key, value))
        else:
            lines.append(f"{_format_key(key)} = {_format_value(value)}")

    for key, value in sub_tables:
        new_path = [*path, key]
        lines.append("")
        lines.append(f"[{'.'.join(_format_key(part) for part in new_path)}]")
        _emit_table(value, new_path, lines)

    for key, value in table_arrays:
        new_path = [*path, key]
        for item in value:
            lines.append("")
            lines.append(f"[[{'.'.join(_format_key(part) for part in new_path)}]]")
            _emit_table(item, new_path, lines)


def _format_key(key: str) -> str:
    """Format a TOML key, quoting it only when it is not a bare key."""
    return key if _BARE_KEY.match(key) else _format_value(key)


def _format_value(value: Any) -> str:  # noqa: ANN401
    """Format a scalar or (possibly nested) list as a TOML value."""
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (int, np.integer)):
        return str(int(value))
    if isinstance(value, (float, np.floating)):
        return repr(float(value))
    if isinstance(value, str):
        return '"' + value.replace("\\", "\\\\").replace('"', '\\"').replace("\n", "\\n") + '"'
    if isinstance(value, (list, tuple, np.ndarray)):
        return "[" + ", ".join(_format_value(item) for item in value) + "]"
    msg = f"unsupported TOML value type: {type(value).__name__}"
    raise TypeError(msg)
