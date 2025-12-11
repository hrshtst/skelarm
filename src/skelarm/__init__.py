"""A robot arm simulation package."""

from __future__ import annotations

from .canvas import SkelarmCanvas, SkelarmViewer
from .dynamics import (
    compute_coriolis_gravity_vector,
    compute_forward_dynamics,
    compute_inverse_dynamics,
    compute_kinetic_energy,
    compute_kinetic_energy_rate,
    compute_mass_matrix,
    simulate_robot,
)
from .kinematics import compute_forward_kinematics
from .plotting import draw_skeleton, plot_trajectory
from .skeleton import Link, LinkProp, Skeleton

__all__ = [
    "Link",
    "LinkProp",
    "SkelarmCanvas",
    "SkelarmViewer",
    "Skeleton",
    "compute_coriolis_gravity_vector",
    "compute_forward_dynamics",
    "compute_forward_kinematics",
    "compute_inverse_dynamics",
    "compute_kinetic_energy",
    "compute_kinetic_energy_rate",
    "compute_mass_matrix",
    "draw_skeleton",
    "hello",
    "plot_trajectory",
    "simulate_robot",
]


def hello() -> str:
    """Return a greeting message from skelarm."""
    return "Hello from skelarm!"
