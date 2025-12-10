"""A robot arm simulation package."""

from __future__ import annotations

from .kinematics import compute_forward_kinematics
from .skeleton import Link, LinkProp, Skeleton

__all__ = ["Link", "LinkProp", "Skeleton", "compute_forward_kinematics", "hello"]


def hello() -> str:
    """Return a greeting message from skelarm."""
    return "Hello from skelarm!"
