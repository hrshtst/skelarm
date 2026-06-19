"""Real-time simulator widgets: drag the tip to apply an external force."""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import numpy as np
from PyQt6.QtCore import QPointF, Qt, QTimer
from PyQt6.QtGui import QColor, QMouseEvent, QPainter, QPaintEvent, QPen
from PyQt6.QtWidgets import (
    QCheckBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QSlider,
    QVBoxLayout,
    QWidget,
)

from skelarm.canvas import SkelarmCanvas
from skelarm.dynamics import compute_forward_dynamics
from skelarm.kinematics import compute_forward_kinematics, compute_jacobian

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from skelarm.skeleton import Skeleton

_TIMER_MS = 20  # GUI/render period in milliseconds
_SUBSTEPS = 4  # physics steps per render tick (integration stability)
_DEFAULT_STIFFNESS = 5.0  # N/m: external tip force = stiffness * (cursor - tip)
_ARROW_COLOR = QColor(220, 0, 0)  # red
_ARROW_HEAD_PX = 10.0


class SimulatorCanvas(SkelarmCanvas):
    """Canvas that applies a tip force on left-drag and draws it as a red arrow.

    Unlike the base canvas, the mouse does not solve inverse kinematics; the drag
    point defines an external force the simulator reads via :meth:`external_force`.
    """

    def __init__(self, skeleton: Skeleton) -> None:
        """Initialize the simulator canvas."""
        super().__init__(skeleton)
        self._drag_world: tuple[float, float] | None = None

    def external_force(self, stiffness: float) -> NDArray[np.float64]:
        """Return the spring force pulling the tip toward the drag point.

        Parameters
        ----------
        stiffness : float
            Force per meter of tip-to-cursor distance.

        Returns
        -------
        NDArray[np.float64]
            The force ``stiffness * (cursor - tip)``, or zeros if not dragging.
        """
        if self._drag_world is None:
            return np.zeros(2, dtype=np.float64)
        tip = self.skeleton.links[-1]
        return stiffness * (np.array(self._drag_world, dtype=np.float64) - np.array([tip.xe, tip.ye]))

    def mousePressEvent(self, a0: QMouseEvent | None) -> None:  # noqa: N802
        """Begin applying a force toward the cursor on left press."""
        if a0 is not None and a0.button() == Qt.MouseButton.LeftButton:
            self._drag_world = self._world_from_screen(a0.position())
            self.update()

    def mouseMoveEvent(self, a0: QMouseEvent | None) -> None:  # noqa: N802
        """Update the force target while dragging with the left button held."""
        if a0 is not None and a0.buttons() & Qt.MouseButton.LeftButton:
            self._drag_world = self._world_from_screen(a0.position())
            self.update()

    def mouseReleaseEvent(self, a0: QMouseEvent | None) -> None:  # noqa: ARG002, N802
        """Stop applying the force on release."""
        self._drag_world = None
        self.update()

    def paintEvent(self, a0: QPaintEvent | None) -> None:  # noqa: N802
        """Draw the arm, then the red force arrow from the tip to the cursor."""
        super().paintEvent(a0)
        if self._drag_world is None:
            return
        center_x = self.width() / 2
        center_y = self.height() / 2
        tip = self.skeleton.links[-1]
        start = self._world_to_screen(tip.xe, tip.ye, center_x, center_y)
        end = self._world_to_screen(self._drag_world[0], self._drag_world[1], center_x, center_y)

        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        pen = QPen(_ARROW_COLOR)
        pen.setWidth(2)
        painter.setPen(pen)
        painter.drawLine(start, end)
        angle = math.atan2(end.y() - start.y(), end.x() - start.x())
        for offset in (math.radians(150.0), math.radians(-150.0)):
            head_x = end.x() + _ARROW_HEAD_PX * math.cos(angle + offset)
            head_y = end.y() + _ARROW_HEAD_PX * math.sin(angle + offset)
            painter.drawLine(end, QPointF(head_x, head_y))


class SkelarmSimulator(QMainWindow):
    """A real-time dynamics viewer with interactive tip forces.

    The arm runs under zero-torque control; press and drag the left mouse button
    in the canvas to apply an external force at the tip (shown as a red arrow).
    The elapsed time is displayed prominently, the joint sliders are read-only and
    show the simulated angles, and a checkbox toggles the link centers of mass.
    Joint limits act as hard stops.
    """

    def __init__(self, skeleton: Skeleton, *, stiffness: float = _DEFAULT_STIFFNESS) -> None:
        """Build the simulator window for the given skeleton.

        Parameters
        ----------
        skeleton : Skeleton
            The arm to simulate (its initial ``q`` / ``dq`` are the start state).
        stiffness : float, optional
            Force per meter of tip-to-cursor distance for the interactive drag.
        """
        super().__init__()
        self.skeleton = skeleton
        self.time = 0.0
        self._stiffness = stiffness
        self._lower = np.array([link.prop.qmin for link in skeleton.links[1:]], dtype=np.float64)
        self._upper = np.array([link.prop.qmax for link in skeleton.links[1:]], dtype=np.float64)

        self.canvas = SimulatorCanvas(skeleton)
        self.setWindowTitle("Skelarm Simulator")
        self.resize(1024, 768)

        central = QWidget()
        self.setCentralWidget(central)
        layout = QHBoxLayout(central)
        layout.addWidget(self.canvas, stretch=3)

        panel = QWidget()
        controls = QVBoxLayout(panel)
        controls.addWidget(QLabel("<b>Simulation</b>"))
        hint = QLabel("Press and drag in the canvas to pull the tip with a force.")
        hint.setWordWrap(True)
        controls.addWidget(hint)

        self.time_label = QLabel()
        time_font = self.time_label.font()
        time_font.setPointSize(time_font.pointSize() + 6)
        time_font.setBold(True)
        self.time_label.setFont(time_font)
        controls.addWidget(self.time_label)

        self.sliders: list[QSlider] = []
        self.joint_labels: list[QLabel] = []
        for link in skeleton.links[1:]:
            joint_label = QLabel()
            controls.addWidget(joint_label)
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(round(math.degrees(link.prop.qmin)), round(math.degrees(link.prop.qmax)))
            slider.setEnabled(False)  # read-only: the slider shows the simulated angle
            controls.addWidget(slider)
            self.sliders.append(slider)
            self.joint_labels.append(joint_label)

        self.com_checkbox = QCheckBox("Show center of mass")
        self.com_checkbox.toggled.connect(self._on_show_com_toggled)
        controls.addWidget(self.com_checkbox)

        controls.addStretch()
        layout.addWidget(panel, stretch=1)

        self._update_displays()
        self._timer = QTimer(self)
        self._timer.timeout.connect(self.step)
        self._timer.start(_TIMER_MS)

    def _on_show_com_toggled(self) -> None:
        """Toggle the center-of-mass overlay on the canvas."""
        self.canvas.show_com = self.com_checkbox.isChecked()
        self.canvas.update()

    def step(self) -> None:
        """Advance the dynamics by one render tick under zero-torque control + tip force."""
        dt = _TIMER_MS / 1000.0 / _SUBSTEPS
        for _ in range(_SUBSTEPS):
            # Zero control torque; the only torque is from the external tip force.
            tau = compute_jacobian(self.skeleton).T @ self.canvas.external_force(self._stiffness)
            ddq = compute_forward_dynamics(self.skeleton, tau)
            # Semi-implicit (symplectic) Euler keeps the undamped system bounded.
            dq = self.skeleton.dq + ddq * dt
            q = self.skeleton.q + dq * dt
            # Joint limits act as hard stops: clamp the angle and halt that joint.
            q_clamped = np.clip(q, self._lower, self._upper)
            dq = np.where(q_clamped != q, 0.0, dq)
            # Write link state directly to bypass the clamping (warning) setter.
            for link, q_value, dq_value in zip(self.skeleton.links[1:], q_clamped, dq, strict=True):
                link.q = float(q_value)
                link.dq = float(dq_value)
            compute_forward_kinematics(self.skeleton)
            self.time += dt
        self._update_displays()

    def _update_displays(self) -> None:
        """Refresh the time label, joint readouts, and canvas."""
        self.time_label.setText(f"t = {self.time:.2f} s")
        for i, (slider, label, angle) in enumerate(zip(self.sliders, self.joint_labels, self.skeleton.q, strict=True)):
            deg = round(math.degrees(angle))
            slider.setValue(deg)
            label.setText(f"Joint {i + 1}: {deg}°")
        self.canvas.update()
