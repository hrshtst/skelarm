"""A robot arm simulation package."""

from __future__ import annotations

from .skeleton import Link, LinkProp, Skeleton

__all__ = ["Link", "LinkProp", "Skeleton", "hello"]


def hello() -> str:
    """Return a greeting message from skelarm."""
    return "Hello from skelarm!"
