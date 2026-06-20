"""A robot arm simulation package."""

from __future__ import annotations

from .canvas import SkelarmCanvas, SkelarmViewer
from .control import (
    ComputedTorque,
    Controller,
    InverseDynamicsFeedforwardPD,
    JointPD,
    SampledJointReference,
    TrackingController,
    ik_joint_reference,
    resolved_rate_joint_reference,
    simulate_controlled,
)
from .dynamics import (
    compute_coriolis_gravity_vector,
    compute_forward_dynamics,
    compute_inverse_dynamics,
    compute_kinetic_energy,
    compute_kinetic_energy_rate,
    compute_mass_matrix,
    simulate_robot,
)
from .kinematics import (
    IKResult,
    compute_coriolis_basis,
    compute_endpoint_acceleration,
    compute_endpoint_velocity,
    compute_forward_kinematics,
    compute_inverse_kinematics,
    compute_jacobian,
)
from .plotting import draw_skeleton, plot_trajectory
from .reaching import (
    EndpointController,
    OnlineReferenceShaping,
    PositionDependentShaping,
    TimeVaryingStiffness,
    VirtualSpringDamper,
    shaping_ratio,
)
from .recording import StateLog
from .simulator import SimulatorCanvas, SkelarmSimulator
from .skeleton import Link, LinkProp, Skeleton
from .trajectory import Trajectory, evaluate_schedule

__all__ = [
    "ComputedTorque",
    "Controller",
    "EndpointController",
    "IKResult",
    "InverseDynamicsFeedforwardPD",
    "JointPD",
    "Link",
    "LinkProp",
    "OnlineReferenceShaping",
    "PositionDependentShaping",
    "SampledJointReference",
    "SimulatorCanvas",
    "SkelarmCanvas",
    "SkelarmSimulator",
    "SkelarmViewer",
    "Skeleton",
    "StateLog",
    "TimeVaryingStiffness",
    "TrackingController",
    "Trajectory",
    "VirtualSpringDamper",
    "compute_coriolis_basis",
    "compute_coriolis_gravity_vector",
    "compute_endpoint_acceleration",
    "compute_endpoint_velocity",
    "compute_forward_dynamics",
    "compute_forward_kinematics",
    "compute_inverse_dynamics",
    "compute_inverse_kinematics",
    "compute_jacobian",
    "compute_kinetic_energy",
    "compute_kinetic_energy_rate",
    "compute_mass_matrix",
    "draw_skeleton",
    "evaluate_schedule",
    "hello",
    "ik_joint_reference",
    "plot_trajectory",
    "resolved_rate_joint_reference",
    "shaping_ratio",
    "simulate_controlled",
    "simulate_robot",
]


def hello() -> str:
    """Return a greeting message from skelarm."""
    return "Hello from skelarm!"
