"""Real-time simulator widgets: drag the tip to apply an external force."""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import numpy as np
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QColor, QMouseEvent, QPainter, QPaintEvent
from PyQt6.QtWidgets import (
    QCheckBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QSlider,
    QVBoxLayout,
    QWidget,
)

from skelarm.canvas import _GOAL_COLOR, SkelarmCanvas, draw_arrow
from skelarm.dynamics import integrate_with_limits
from skelarm.kinematics import compute_forward_kinematics, compute_jacobian
from skelarm.recording import StateLog

if TYPE_CHECKING:
    from collections.abc import Mapping
    from typing import Any

    from numpy.typing import ArrayLike, NDArray

    from skelarm.control import Controller
    from skelarm.skeleton import Skeleton

_TIMER_MS = 20  # GUI/render period in milliseconds
_SUBSTEPS = 4  # physics steps per render tick (integration stability)
_DEFAULT_STIFFNESS = 0.1  # N/m: external tip force = stiffness * (cursor - tip)
_ARROW_COLOR = QColor(220, 0, 0)  # red
_PANEL_WIDTH_PX = 300  # fixed side-panel width so the varying time readout can't resize it


class SimulatorCanvas(SkelarmCanvas):
    """Canvas that applies a tip force on left-drag and draws it as a red arrow.

    Unlike the base canvas, the mouse does not solve inverse kinematics; the drag
    point defines an external force the simulator reads via :meth:`external_force`.
    """

    def __init__(self, skeleton: Skeleton) -> None:
        """Initialize the simulator canvas."""
        super().__init__(skeleton)
        self._drag_world: tuple[float, float] | None = None
        self.target: NDArray[np.float64] | None = None  # optional task-space goal marker (the active one)
        self.target_color = _GOAL_COLOR  # marker color
        self.target_tolerance: float | None = None  # success radius (m); sizes the ring
        # Inactive candidate goals for multi-target tasks, drawn as faint hollow rings.
        self.secondary_targets: list[tuple[NDArray[np.float64], QColor]] = []
        # When set, a left-press only starts a drag if it is within this distance
        # (meters) of the tip ("grab near the tip"); None grabs anywhere.
        self.grab_radius: float | None = None
        # Whether to draw the red drag arrow (a force cue); off for kinematic drags.
        self.show_drag_arrow = True

    @property
    def drag_point(self) -> tuple[float, float] | None:
        """The current pointer world position ``(x, y)`` while dragging, else ``None``.

        Settable so the drag target can be driven programmatically (scripting/tests),
        bypassing the mouse and :attr:`grab_radius` gate.
        """
        return self._drag_world

    @drag_point.setter
    def drag_point(self, value: tuple[float, float] | None) -> None:
        self._drag_world = None if value is None else (float(value[0]), float(value[1]))
        self.update()

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
        """Begin a drag on left press (only near the tip when ``grab_radius`` is set)."""
        if a0 is not None and a0.button() == Qt.MouseButton.LeftButton:
            world = self._world_from_screen(a0.position())
            if self.grab_radius is not None:
                tip = self.skeleton.links[-1]
                if math.hypot(world[0] - tip.xe, world[1] - tip.ye) > self.grab_radius:
                    return
            self._drag_world = world
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
        """Draw the arm, then the task target (if any) and the drag force arrow."""
        super().paintEvent(a0)
        center_x = self.width() / 2
        center_y = self.height() / 2
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        # Inactive multi-target candidates: dashed hollow rings, no fill.
        for pos, color in self.secondary_targets:
            self._draw_target_marker(painter, pos, center_x, center_y, color=color, tolerance=None, active=False)

        # Task-space goal: a filled dot inside a hollow ring sized by the tolerance.
        if self.target is not None:
            self._draw_target_marker(
                painter,
                self.target,
                center_x,
                center_y,
                color=self.target_color,
                tolerance=self.target_tolerance,
                active=True,
            )

        # Interactive drag force: a red arrow from the tip to the cursor.
        if self._drag_world is not None and self.show_drag_arrow:
            tip = self.skeleton.links[-1]
            start = self._world_to_screen(tip.xe, tip.ye, center_x, center_y)
            end = self._world_to_screen(self._drag_world[0], self._drag_world[1], center_x, center_y)
            draw_arrow(painter, start, end, color=_ARROW_COLOR)


class SkelarmSimulator(QMainWindow):
    """A real-time dynamics viewer with interactive tip forces.

    By default the arm runs under zero-torque control; pass a ``controller`` to
    drive it (e.g. a reaching controller tracking a ``target``). Press and drag the
    left mouse button in the canvas to apply an external force at the tip (shown as
    a red arrow), which is added on top of the control torque. The elapsed time is
    displayed prominently, the joint sliders are read-only and show the simulated
    angles, and a checkbox toggles the link centers of mass. Joint limits act as
    hard stops.
    """

    def __init__(
        self,
        skeleton: Skeleton,
        *,
        controller: Controller | None = None,
        target: ArrayLike | None = None,
        target_color: str | None = None,
        target_tolerance: float | None = None,
        stiffness: float = _DEFAULT_STIFFNESS,
        friction: float = 0.0,
        enforce_limits: bool = True,
        log_extra: Mapping[str, Any] | None = None,
    ) -> None:
        """Build the simulator window for the given skeleton.

        Parameters
        ----------
        skeleton : Skeleton
            The arm to simulate (its initial ``q`` / ``dq`` are the start state).
        controller : Controller | None, optional
            Torque controller to drive the arm; ``None`` runs it under zero torque.
            Its ``update`` / ``control`` hooks are called each physics substep.
        target : ArrayLike | None, optional
            A task-space goal ``(x, y)`` drawn as a marker on the canvas.
        target_color : str | None, optional
            Marker color for ``target`` (any Qt/SVG color name); defaults to purple.
        target_tolerance : float | None, optional
            Success radius (m): when set, the target marker's ring is drawn at this
            distance in world units.
        stiffness : float, optional
            Force per meter of tip-to-cursor distance for the interactive drag.
        friction : float, optional
            Joint viscous friction coefficient (N·m·s/rad). Each joint feels a
            damping torque ``-friction * dq`` that dissipates energy; the default
            of ``0`` leaves the arm frictionless.
        enforce_limits : bool, optional
            Apply the joint limits as hard stops in the dynamics (default). When
            ``False``, the limits no longer constrain the simulation (they still
            apply to the kinematics setters and inverse kinematics).
        log_extra : Mapping[str, Any] | None, optional
            Free-form metadata embedded in the recorded log's ``[extra]`` table (e.g.
            ``{"source_config": ...}`` so a later player can reconstruct the task).
        """
        super().__init__()
        self.skeleton = skeleton
        self.time = 0.0
        self._controller = controller
        self._stiffness = stiffness
        self._friction = friction
        self._log_extra = log_extra
        # Joint limits passed to the integrator each step; None disables the hard stop.
        self._lower = (
            np.array([link.prop.qmin for link in skeleton.links[1:]], dtype=np.float64) if enforce_limits else None
        )
        self._upper = (
            np.array([link.prop.qmax for link in skeleton.links[1:]], dtype=np.float64) if enforce_limits else None
        )
        self._initial_q = skeleton.q.copy()  # restored by reset()
        self._initial_dq = skeleton.dq.copy()
        self.state_log: StateLog | None = None  # set by start_recording()
        self._recording = False

        self.canvas = SimulatorCanvas(skeleton)
        self.canvas.target = None if target is None else np.asarray(target, dtype=np.float64)
        if target_color is not None:
            self.canvas.target_color = QColor(target_color)
        self.canvas.target_tolerance = target_tolerance
        self.setWindowTitle("Skelarm Simulator")
        self.resize(1024, 768)
        if self._controller is not None:
            self._controller.reset(skeleton)

        central = QWidget()
        self.setCentralWidget(central)
        layout = QHBoxLayout(central)
        layout.addWidget(self.canvas, stretch=3)

        panel = QWidget()
        panel.setFixedWidth(_PANEL_WIDTH_PX)  # keep a constant width; the time readout won't resize it
        controls = QVBoxLayout(panel)
        self.controls_panel = panel  # exposed for sizing (fixed width) and tests
        self.controls_layout = controls  # exposed so subclasses can add_control
        controls.addWidget(QLabel("<b>Simulation</b>"))
        hint = QLabel("Press and drag in the canvas to pull the tip with a force.")
        hint.setWordWrap(True)
        controls.addWidget(hint)

        self.time_label = QLabel()
        self.time_label.setWordWrap(True)  # wrap rather than clip if the readout outgrows the fixed width
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

    def add_control(self, widget: QWidget) -> None:
        """Add an extra control widget above the trailing stretch (for subclasses)."""
        self.controls_layout.insertWidget(self.controls_layout.count() - 1, widget)

    @property
    def stiffness(self) -> float:
        """Force per meter of tip-to-cursor distance for the interactive drag."""
        return self._stiffness

    @stiffness.setter
    def stiffness(self, value: float) -> None:
        self._stiffness = float(value)

    @property
    def friction(self) -> float:
        """Joint viscous friction coefficient (N·m·s/rad); ``0`` disables dissipation."""
        return self._friction

    @friction.setter
    def friction(self, value: float) -> None:
        self._friction = float(value)

    @property
    def running(self) -> bool:
        """Whether the simulation loop is currently advancing."""
        return self._timer.isActive()

    def pause(self) -> None:
        """Stop advancing the simulation (the state is frozen until :meth:`resume`)."""
        self._timer.stop()

    def resume(self) -> None:
        """Resume advancing the simulation after a :meth:`pause`."""
        if not self._timer.isActive():
            self._timer.start(_TIMER_MS)

    @property
    def is_recording(self) -> bool:
        """Whether each step is being appended to :attr:`state_log`."""
        return self._recording

    def start_recording(self) -> None:
        """Begin a fresh state log, capturing the current frame as the first sample."""
        joints = [f"j{i + 1}" for i in range(self.skeleton.num_joints)]
        channel_meta = {
            "q": {"unit": "rad", "label": "joint angle", "columns": joints},
            "dq": {"unit": "rad/s", "label": "joint velocity", "columns": joints},
            "tau": {"unit": "N*m", "label": "applied joint torque", "columns": joints},
            "ext_force": {"unit": "N", "label": "external tip force", "columns": ["fx", "fy"]},
        }
        self.state_log = StateLog(
            self.skeleton, producer="skelarm_simulator", channel_meta=channel_meta, extra=self._log_extra
        )
        self.state_log.record(
            self.time,
            q=self.skeleton.q,
            dq=self.skeleton.dq,
            tau=np.zeros(self.skeleton.num_joints),
            ext_force=self.canvas.external_force(self._stiffness),
        )
        self._recording = True

    def stop_recording(self) -> None:
        """Stop appending frames to the state log (the log itself is kept)."""
        self._recording = False

    def reset(self) -> None:
        """Restore the initial pose and velocity, reset any controller, and zero the clock."""
        for link, q_value, dq_value in zip(self.skeleton.links[1:], self._initial_q, self._initial_dq, strict=True):
            link.q = float(q_value)
            link.dq = float(dq_value)
        compute_forward_kinematics(self.skeleton)
        self.time = 0.0
        if self._controller is not None:
            self._controller.reset(self.skeleton)
        if self._recording:
            self.start_recording()  # restart the log for the fresh run
        self._update_displays()

    def _control_torque(self, dt: float) -> NDArray[np.float64]:
        """Active control torque for the current substep (zero unless a controller is set).

        Advances any stateful controller and returns its torque, mirroring the
        ``update`` then ``control`` order of :func:`skelarm.simulate_controlled`.
        """
        if self._controller is None:
            return np.zeros(self.skeleton.num_joints, dtype=np.float64)
        self._controller.update(self.time, self.skeleton, dt)
        return self._controller.control(self.time, self.skeleton)

    def step(self) -> None:
        """Advance the dynamics by one render tick under the controller (if any) plus the tip force."""
        dt = _TIMER_MS / 1000.0 / _SUBSTEPS
        for _ in range(_SUBSTEPS):
            # Total torque = active control (zero by default) + the external tip
            # force mapped to joints - joint viscous friction (-friction * dq).
            tau = self._control_torque(dt)
            tau = tau + compute_jacobian(self.skeleton).T @ self.canvas.external_force(self._stiffness)
            tau = tau - self._friction * self.skeleton.dq
            integrate_with_limits(self.skeleton, tau, dt, self._lower, self._upper)
            self.time += dt
        if self._recording and self.state_log is not None:
            self.state_log.record(
                self.time,
                q=self.skeleton.q,
                dq=self.skeleton.dq,
                tau=tau,
                ext_force=self.canvas.external_force(self._stiffness),
            )
        self._update_displays()

    def _update_displays(self) -> None:
        """Refresh the time label, joint readouts, and canvas."""
        self.time_label.setText(f"t = {self.time:.2f} s")
        for i, (slider, label, angle) in enumerate(zip(self.sliders, self.joint_labels, self.skeleton.q, strict=True)):
            deg = round(math.degrees(angle))
            slider.setValue(deg)
            label.setText(f"Joint {i + 1}: {deg}°")
        self.canvas.update()
