"""Defines the Link and Skeleton classes for the skelarm robot arm simulator."""

from __future__ import annotations

import tomllib
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any, cast

import numpy as np

# Runtime-safe despite the apparent cycle: kinematics imports Skeleton only
# under TYPE_CHECKING.
from skelarm.kinematics import compute_forward_kinematics

if TYPE_CHECKING:
    from numpy.typing import NDArray


@dataclass
class LinkProp:
    """Properties of a single robot arm link."""

    length: float
    m: float
    i: float
    rgx: float
    rgy: float
    qmin: float  # Radians
    qmax: float  # Radians


class Link:
    """Represents a single link of the robot arm."""

    def __init__(self, properties: LinkProp | dict[str, Any]) -> None:
        """Initialize a Link object from LinkProp or a dictionary.

        Parameters
        ----------
        properties : LinkProp | dict[str, Any]
            Either a LinkProp object or a dictionary containing link properties
            ('length', 'm', 'i', 'rgx', 'rgy', 'qmin', 'qmax').
        """
        if isinstance(properties, LinkProp):
            self.prop = properties
        elif isinstance(properties, dict):
            # Copy first so the caller's dict is not mutated, then accept 'l' as
            # an alias for 'length'.
            props = dict(properties)
            if "l" in props:
                props["length"] = props.pop("l")
            self.prop = LinkProp(**props)
        else:
            error_msg = "Properties must be a LinkProp object or a dictionary."
            raise TypeError(error_msg)

        # State variables, initialized to zeros or defaults
        self.q: float = 0.0  # Joint angle
        self.dq: float = 0.0  # Joint angular velocity
        self.ddq: float = 0.0  # Joint angular acceleration
        self.q_absolute: float = 0.0  # Absolute angle of the link (used in RNE)

        # Joint-origin (link start) position, plus tip (end-effector) velocity
        # and acceleration in the base frame. The tip position is xe/ye below.
        self.x: float = 0.0  # joint-origin x
        self.y: float = 0.0  # joint-origin y
        self.vx: float = 0.0  # tip velocity x
        self.vy: float = 0.0  # tip velocity y
        self.ax: float = 0.0  # tip acceleration x
        self.ay: float = 0.0  # tip acceleration y

        # Center of mass position and velocity (global)
        self.xg: float = 0.0
        self.yg: float = 0.0
        self.agx: float = 0.0
        self.agy: float = 0.0

        # Endpoint Jacobian column and centripetal/Coriolis acceleration basis
        # for this joint (filled by the differential-kinematics helpers).
        self.jx: float = 0.0  # x-component of this joint's Jacobian column
        self.jy: float = 0.0  # y-component of this joint's Jacobian column
        self.hx: float = 0.0  # x-component of the centripetal/Coriolis basis
        self.hy: float = 0.0  # y-component of the centripetal/Coriolis basis

        # Joint forces/torques
        self.fx: float = 0.0  # Force applied to link x
        self.fy: float = 0.0  # Force applied to link y
        self.tau: float = 0.0  # Torque at joint

        # External forces/torques
        self.fex: float = 0.0  # External force x
        self.fey: float = 0.0  # External force y
        self.rex: float = 0.0  # External force x application point in the link frame
        self.rey: float = 0.0  # External force y application point in the link frame

        # End-effector position (global)
        self.xe: float = 0.0
        self.ye: float = 0.0

        # --- Attributes for Inverse Dynamics (RNE) ---
        self.w: float = 0.0  # Angular velocity of link frame
        self.dw: float = 0.0  # Angular acceleration of link frame

        # Linear velocity/acceleration of link origin (joint)
        self.v: NDArray[np.float64] = cast("NDArray[np.float64]", np.zeros(2))
        self.dv: NDArray[np.float64] = cast("NDArray[np.float64]", np.zeros(2))

        # Linear velocity/acceleration of center of mass
        self.vc: NDArray[np.float64] = cast("NDArray[np.float64]", np.zeros(2))
        self.dvc: NDArray[np.float64] = cast("NDArray[np.float64]", np.zeros(2))

        # Forces and moments
        self.f: NDArray[np.float64] = cast("NDArray[np.float64]", np.zeros(2))  # Force from parent link
        self.n: float = 0.0  # Moment exerted by parent link on current link


class Skeleton:
    """Represents the entire robot arm (skeleton)."""

    def __init__(
        self,
        link_props: Sequence[LinkProp | dict[str, Any]],
        base_length: float = 0.0,
    ) -> None:
        """Initialize the Skeleton with a list of movable-link properties.

        Following the reference notes, the arm is built around a fixed *base*
        (zeroth) link of length ``base_length``. It is stored as ``links[0]`` and
        carries the first joint at ``(base_length, 0)``; the actuated links given
        in ``link_props`` follow as ``links[1:]``. A "2-link arm" therefore holds
        three links in total: the base plus two movable links.

        Forward kinematics is computed once at the end of construction, so the
        derived link states (positions, COM, ...) start out consistent with the
        initial pose.

        Parameters
        ----------
        link_props : Sequence[LinkProp | dict[str, Any]]
            A list of LinkProp objects or dictionaries, one for each *movable*
            link.
        base_length : float, optional
            Length of the fixed base link, i.e. the offset from the origin to the
            first joint. Defaults to 0.0 (first joint at the origin).
        """
        base_prop = LinkProp(length=base_length, m=0.0, i=0.0, rgx=0.0, rgy=0.0, qmin=0.0, qmax=0.0)
        self.links: list[Link] = [Link(base_prop), *(Link(prop) for prop in link_props)]
        # Total number of links including the fixed base (matches the reference
        # ``num``); the actuated degrees of freedom are the movable links only.
        self.num_links: int = len(self.links)
        self.num_joints: int = self.num_links - 1
        # Make the derived link states (positions, COM, ...) consistent with the
        # initial pose right away, so a fresh skeleton can be drawn or queried
        # without an explicit forward-kinematics call.
        compute_forward_kinematics(self)

    @property
    def base_length(self) -> float:
        """Length of the fixed base (zeroth) link."""
        return self.links[0].prop.length

    @classmethod
    def from_toml(cls, file_path: str | Path) -> Skeleton:
        """Create a Skeleton from a TOML configuration file.

        Parameters
        ----------
        file_path : str | Path
            Path to the TOML file.

        Returns
        -------
        Skeleton
            A new Skeleton instance. Each link's initial joint angle is taken
            from its optional ``q0`` key (in degrees, like the limits) and
            defaults to zero; the link positions are already consistent with
            that pose.
        """
        path = Path(file_path)
        with path.open("rb") as f:
            data = tomllib.load(f)

        # Optional fixed base link length (offset from the origin to joint 1).
        base_length = float(data.get("base_length", 0.0))

        link_props = []
        initial_angles = []
        for link_data in data.get("link", []):
            # Extract properties from TOML data
            length = link_data["length"]
            mass = link_data["mass"]
            inertia = link_data["inertia"]

            # Allow 'com' as [x, y] or 'rgx'/'rgy' keys
            if "com" in link_data:
                rgx, rgy = link_data["com"]
            else:
                rgx = link_data.get("rgx", 0.0)
                rgy = link_data.get("rgy", 0.0)

            # Allow 'limits' as [min, max] or 'qmin'/'qmax' keys
            if "limits" in link_data:
                qmin_deg, qmax_deg = link_data["limits"]
            else:
                qmin_deg = link_data.get("qmin", -180.0)
                qmax_deg = link_data.get("qmax", 180.0)

            # Convert limits from degrees (config) to radians (internal)
            qmin = np.deg2rad(qmin_deg)
            qmax = np.deg2rad(qmax_deg)

            # Optional initial joint angle, in degrees like the limits.
            initial_angles.append(np.deg2rad(link_data.get("q0", 0.0)))

            link_props.append(
                LinkProp(
                    length=length,
                    m=mass,
                    i=inertia,
                    rgx=rgx,
                    rgy=rgy,
                    qmin=qmin,
                    qmax=qmax,
                )
            )

        skeleton = cls(link_props, base_length=base_length)
        skeleton.q = np.array(initial_angles, dtype=np.float64)
        return skeleton

    @property
    def q(self) -> NDArray[np.float64]:
        """Return current joint angles (one per movable link)."""
        return np.array([link.q for link in self.links[1:]], dtype=np.float64)

    @q.setter
    def q(self, q_values: NDArray[np.float64]) -> None:
        """Set joint angles and refresh the derived link states."""
        if len(q_values) != self.num_joints:
            error_msg = f"Expected {self.num_joints} joint angles, but got {len(q_values)}"
            raise ValueError(error_msg)
        for link, value in zip(self.links[1:], q_values, strict=True):
            link.q = value
        compute_forward_kinematics(self)

    @property
    def dq(self) -> NDArray[np.float64]:
        """Return current joint angular velocities (one per movable link)."""
        return np.array([link.dq for link in self.links[1:]], dtype=np.float64)

    @dq.setter
    def dq(self, dq_values: NDArray[np.float64]) -> None:
        """Set joint angular velocities and refresh the derived link states."""
        if len(dq_values) != self.num_joints:
            error_msg = f"Expected {self.num_joints} joint angular velocities, but got {len(dq_values)}"
            raise ValueError(error_msg)
        for link, value in zip(self.links[1:], dq_values, strict=True):
            link.dq = value
        compute_forward_kinematics(self)

    @property
    def ddq(self) -> NDArray[np.float64]:
        """Return current joint angular accelerations (one per movable link)."""
        return np.array([link.ddq for link in self.links[1:]], dtype=np.float64)

    @ddq.setter
    def ddq(self, ddq_values: NDArray[np.float64]) -> None:
        """Set joint angular accelerations and refresh the derived link states."""
        if len(ddq_values) != self.num_joints:
            error_msg = f"Expected {self.num_joints} joint angular accelerations, but got {len(ddq_values)}"
            raise ValueError(error_msg)
        for link, value in zip(self.links[1:], ddq_values, strict=True):
            link.ddq = value
        compute_forward_kinematics(self)

    def set_state(
        self,
        q: NDArray[np.float64] | None = None,
        dq: NDArray[np.float64] | None = None,
        ddq: NDArray[np.float64] | None = None,
    ) -> None:
        """Set joint angles, velocities, and accelerations in one call.

        Unlike assigning ``q``, ``dq``, and ``ddq`` separately (each of which
        re-runs forward kinematics), this writes all provided values first and
        refreshes the derived link states with a single forward-kinematics
        pass. Arguments left as ``None`` keep their current values.

        Parameters
        ----------
        q : NDArray[np.float64] | None, optional
            Joint angles, one per movable link.
        dq : NDArray[np.float64] | None, optional
            Joint angular velocities, one per movable link.
        ddq : NDArray[np.float64] | None, optional
            Joint angular accelerations, one per movable link.

        Raises
        ------
        ValueError
            If any provided array does not hold one value per movable link.
            The skeleton is left unmodified in that case.
        """
        # Validate everything before writing anything, so a bad argument
        # cannot leave the skeleton partially updated.
        for name, values in (("q", q), ("dq", dq), ("ddq", ddq)):
            if values is not None and len(values) != self.num_joints:
                error_msg = f"Expected {self.num_joints} values for {name}, but got {len(values)}"
                raise ValueError(error_msg)

        if q is not None:
            for link, value in zip(self.links[1:], q, strict=True):
                link.q = value
        if dq is not None:
            for link, value in zip(self.links[1:], dq, strict=True):
                link.dq = value
        if ddq is not None:
            for link, value in zip(self.links[1:], ddq, strict=True):
                link.ddq = value
        compute_forward_kinematics(self)

    @property
    def tau(self) -> NDArray[np.float64]:
        """Return current joint torques (one per movable link)."""
        return np.array([link.tau for link in self.links[1:]], dtype=np.float64)

    @tau.setter
    def tau(self, tau_values: NDArray[np.float64]) -> None:
        """Set joint torques."""
        if len(tau_values) != self.num_joints:
            error_msg = f"Expected {self.num_joints} joint torques, but got {len(tau_values)}"
            raise ValueError(error_msg)
        for link, value in zip(self.links[1:], tau_values, strict=True):
            link.tau = value
