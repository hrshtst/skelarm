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
    integrate_with_limits,
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
from .mpc import JointSpaceMPC
from .plotting import draw_skeleton, plot_trajectory
from .reaching import (
    AdaptiveReferenceShaping,
    EndpointController,
    OnlineReferenceShaping,
    PositionDependentShaping,
    TimeVaryingStiffness,
    VirtualSpringDamper,
    adaptive_shaping_ratio,
    shaping_ratio,
)
from .recording import StateLog
from .scenario import (
    ControllerBuilder,
    Scenario,
    Task,
    build_controller,
    controller_types,
    export_scenario_toml,
    load_scenario,
    register_controller,
    register_task_type,
    rerun_log,
    run_scenario,
    scenario_from_config,
    scenario_from_log,
    task_types,
)
from .simulator import SimulatorCanvas, SkelarmSimulator
from .skeleton import Link, LinkProp, Skeleton
from .trajectory import Trajectory, evaluate_schedule

__all__ = [
    "AdaptiveReferenceShaping",
    "ComputedTorque",
    "Controller",
    "ControllerBuilder",
    "EndpointController",
    "IKResult",
    "InverseDynamicsFeedforwardPD",
    "JointPD",
    "JointSpaceMPC",
    "Link",
    "LinkProp",
    "OnlineReferenceShaping",
    "PositionDependentShaping",
    "SampledJointReference",
    "Scenario",
    "SimulatorCanvas",
    "SkelarmCanvas",
    "SkelarmSimulator",
    "SkelarmViewer",
    "Skeleton",
    "StateLog",
    "Task",
    "TimeVaryingStiffness",
    "TrackingController",
    "Trajectory",
    "VirtualSpringDamper",
    "adaptive_shaping_ratio",
    "build_controller",
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
    "controller_types",
    "draw_skeleton",
    "evaluate_schedule",
    "export_scenario_toml",
    "hello",
    "ik_joint_reference",
    "integrate_with_limits",
    "load_scenario",
    "plot_trajectory",
    "register_controller",
    "register_task_type",
    "rerun_log",
    "resolved_rate_joint_reference",
    "run_scenario",
    "scenario_from_config",
    "scenario_from_log",
    "shaping_ratio",
    "simulate_controlled",
    "simulate_robot",
    "task_types",
]


def hello() -> str:
    """Return a greeting message from skelarm."""
    return "Hello from skelarm!"
