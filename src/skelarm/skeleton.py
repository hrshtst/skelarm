"""Defines the Link and Skeleton classes for the skelarm robot arm simulator."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    import numpy as np  # Import numpy only for type checking
    from numpy.typing import NDArray


@dataclass
class LinkProp:
    """Properties of a single robot arm link."""

    length: float
    m: float
    i: float
    rgx: float
    rgy: float
    qmin: float
    qmax: float


class Link:
    """Represents a single link of the robot arm."""

    def __init__(self, properties: LinkProp | dict[str, Any]) -> None:
        """
        Initialize a Link object from LinkProp or a dictionary.

        :param properties: Either a LinkProp object or a dictionary containing
                           link properties ('length', 'm', 'i', 'rgx', 'rgy', 'qmin', 'qmax').
        """
        if isinstance(properties, LinkProp):
            self.prop = properties
        elif isinstance(properties, dict):
            # Ensure 'l' is handled if passed in dict, convert to 'length'
            if "l" in properties:
                properties["length"] = properties.pop("l")
            self.prop = LinkProp(**properties)
        else:
            error_msg = "Properties must be a LinkProp object or a dictionary."
            raise TypeError(error_msg)

        # State variables, initialized to zeros or defaults
        self.q: float = 0.0  # Joint angle
        self.dq: float = 0.0  # Joint angular velocity
        self.ddq: float = 0.0  # Joint angular acceleration

        # End-effector position and velocity (local to link)
        self.x: float = 0.0
        self.y: float = 0.0
        self.vx: float = 0.0
        self.vy: float = 0.0
        self.ax: float = 0.0
        self.ay: float = 0.0

        # Center of mass position and velocity (global)
        self.xg: float = 0.0
        self.yg: float = 0.0
        self.agx: float = 0.0
        self.agy: float = 0.0

        # Joint forces/torques
        self.jx: float = 0.0  # Force at joint x
        self.jy: float = 0.0  # Force at joint y
        self.hx: float = 0.0  # Moment at joint x
        self.hy: float = 0.0  # Moment at joint y
        self.fx: float = 0.0  # Force applied to link x
        self.fy: float = 0.0  # Force applied to link y
        self.tau: float = 0.0  # Torque at joint

        # External forces/torques
        self.fex: float = 0.0  # External force x
        self.fey: float = 0.0  # External force y
        self.rex: float = 0.0  # External force x application point
        self.rey: float = 0.0  # External force y application point

        # End-effector position (global)
        self.xe: float = 0.0
        self.ye: float = 0.0


class Skeleton:
    """Represents the entire robot arm (skeleton)."""

    def __init__(self, link_props: list[LinkProp | dict[str, Any]]) -> None:
        """
        Initialize the Skeleton with a list of link properties.

        :param link_props: A list of LinkProp objects or dictionaries, one for each link.
        """
        self.links: list[Link] = [Link(prop) for prop in link_props]
        self.num_links: int = len(self.links)

    @property
    def q(self) -> NDArray[np.float64]:
        """Return current joint angles."""
        import numpy as np  # Import at runtime for actual usage

        return np.array([link.q for link in self.links], dtype=np.float64)

    @q.setter
    def q(self, q_values: NDArray[np.float64]) -> None:
        """Set joint angles."""
        if len(q_values) != self.num_links:
            error_msg = f"Expected {self.num_links} joint angles, but got {len(q_values)}"
            raise ValueError(error_msg)
        for i, link in enumerate(self.links):
            link.q = q_values[i]

    @property
    def dq(self) -> NDArray[np.float64]:
        """Return current joint angular velocities."""
        import numpy as np  # Import at runtime for actual usage

        return np.array([link.dq for link in self.links], dtype=np.float64)

    @dq.setter
    def dq(self, dq_values: NDArray[np.float64]) -> None:
        """Set joint angular velocities."""
        if len(dq_values) != self.num_links:
            error_msg = f"Expected {self.num_links} joint angular velocities, but got {len(dq_values)}"
            raise ValueError(error_msg)
        for i, link in enumerate(self.links):
            link.dq = dq_values[i]

    @property
    def ddq(self) -> NDArray[np.float64]:
        """Return current joint angular accelerations."""
        import numpy as np  # Import at runtime for actual usage

        return np.array([link.ddq for link in self.links], dtype=np.float64)

    @ddq.setter
    def ddq(self, ddq_values: NDArray[np.float64]) -> None:
        """Set joint angular accelerations."""
        if len(ddq_values) != self.num_links:
            error_msg = f"Expected {self.num_links} joint angular accelerations, but got {len(ddq_values)}"
            raise ValueError(error_msg)
        for i, link in enumerate(self.links):
            link.ddq = ddq_values[i]
