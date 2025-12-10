"""A robot arm simulation package."""

from __future__ import annotations

from .dynamics import compute_inverse_dynamics
from .kinematics import compute_forward_kinematics
from .plotting import draw_skeleton, plot_trajectory
from .skeleton import Link, LinkProp, Skeleton

__all__ = [
    "Link",
    "LinkProp",
    "Skeleton",
    "compute_forward_kinematics",
    "compute_inverse_dynamics",
    "draw_skeleton",
    "hello",
    "plot_trajectory",
]


def hello() -> str:
    """Return a greeting message from skelarm."""
    return "Hello from skelarm!"
