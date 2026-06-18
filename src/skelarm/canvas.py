"""Provides a PyQt6 widget for visualizing the robot arm."""

from __future__ import annotations

import functools
import math
from typing import TYPE_CHECKING

from PyQt6.QtCore import QPointF, QSignalBlocker, Qt, pyqtSignal
from PyQt6.QtGui import QBrush, QColor, QMouseEvent, QPainter, QPaintEvent, QPen
from PyQt6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QWidget,
)

from skelarm.kinematics import compute_inverse_kinematics

if TYPE_CHECKING:
    from skelarm.kinematics import IKResult
    from skelarm.skeleton import Skeleton

_MARGIN_PX = 20.0  # Empty border kept between the arm and the widget edge.
_DEFAULT_SCALE = 100.0  # Pixels per metre, used before the reach/size are known.


def _fit_scale(reach: float, width: int, height: int) -> float:
    """Compute a pixels-per-metre scale that fits an arm of ``reach`` in the widget.

    The arm is drawn from the centre of the widget and can extend up to ``reach``
    metres in any direction, so the scale is set from the smaller half-dimension
    (minus a margin) to guarantee the fully-extended arm stays visible.

    Parameters
    ----------
    reach : float
        Maximum distance the arm can extend from the origin (sum of link lengths).
    width, height : int
        Current widget size in pixels.

    Returns
    -------
    float
        Scale in pixels per metre, falling back to :data:`_DEFAULT_SCALE` when the
        reach or widget size is non-positive.
    """
    half = min(width, height) / 2 - _MARGIN_PX
    if reach <= 0 or half <= 0:
        return _DEFAULT_SCALE
    return half / reach


class SkelarmCanvas(QWidget):
    """A widget to draw the robot arm skeleton."""

    # Emitted after the pose changes from an in-canvas IK solve, so the viewer can
    # refresh its sliders.
    pose_changed = pyqtSignal()

    _JOINT_RADIUS_PX = 5  # also used for the origin, so the two match in size
    _COM_RADIUS_PX = 7  # a bit larger than the joints
    _LINK_WIDTH_PX = 4
    _LINK_COLOR = QColor(0, 100, 200)  # blue (movable links)
    _BASE_LINK_COLOR = QColor(150, 150, 150)  # gray (fixed base link)
    _JOINT_COLOR = QColor(0, 170, 0)  # green
    _COM_COLOR = QColor(200, 0, 0)  # red (centers of mass)
    _TARGET_COLOR = QColor(230, 120, 0)  # orange (IK target)
    _TARGET_RADIUS_PX = 7

    def __init__(self, skeleton: Skeleton, parent: QWidget | None = None) -> None:
        """Initialize the canvas."""
        super().__init__(parent)
        self.skeleton = skeleton
        self.scale_factor = _DEFAULT_SCALE  # Pixels per metre; re-fit on each paint.
        self.show_com = False  # whether to overlay each link's center of mass
        self.ik_method = "lm_sugihara"  # solver method used by solve_to_world
        self.last_ik_result: IKResult | None = None  # outcome of the latest IK solve
        self._ik_target: tuple[float, float] | None = None  # latest IK click target
        # Set background color to white
        self.setAutoFillBackground(True)
        p = self.palette()
        p.setColor(self.backgroundRole(), QColor("white"))
        self.setPalette(p)

    def paintEvent(self, a0: QPaintEvent | None) -> None:  # noqa: N802, ARG002
        """Paint the robot arm."""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        # Auto-fit the drawing to the current widget size and the arm's reach.
        reach = sum(link.prop.length for link in self.skeleton.links)
        self.scale_factor = _fit_scale(reach, self.width(), self.height())

        # Center of the widget
        center_x = self.width() / 2
        center_y = self.height() / 2

        # Pens for the fixed base link (gray) and the movable links (blue).
        pen_base = QPen(self._BASE_LINK_COLOR)
        pen_base.setWidth(self._LINK_WIDTH_PX)
        pen_base.setCapStyle(Qt.PenCapStyle.RoundCap)
        pen_link = QPen(self._LINK_COLOR)
        pen_link.setWidth(self._LINK_WIDTH_PX)
        pen_link.setCapStyle(Qt.PenCapStyle.RoundCap)

        # We assume compute_forward_kinematics has already run, so positions are
        # current. Draw all link segments first (base gray, movable blue) so the
        # origin and joint circles below are rendered on top of the links.
        for i, link in enumerate(self.skeleton.links):
            p1 = self._world_to_screen(link.x, link.y, center_x, center_y)
            p2 = self._world_to_screen(link.xe, link.ye, center_x, center_y)
            painter.setPen(pen_base if i == 0 else pen_link)
            painter.drawLine(p1, p2)

        # Draw the origin (black) and every joint (green) on top, all the same size.
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QBrush(Qt.GlobalColor.black))
        origin = self._world_to_screen(0, 0, center_x, center_y)
        painter.drawEllipse(origin, self._JOINT_RADIUS_PX, self._JOINT_RADIUS_PX)
        painter.setBrush(QBrush(self._JOINT_COLOR))
        for link in self.skeleton.links:
            p2 = self._world_to_screen(link.xe, link.ye, center_x, center_y)
            painter.drawEllipse(p2, self._JOINT_RADIUS_PX, self._JOINT_RADIUS_PX)

        # Optionally overlay each movable link's center of mass (red, a bit larger).
        if self.show_com:
            painter.setBrush(QBrush(self._COM_COLOR))
            for link in self.skeleton.links[1:]:
                com = self._world_to_screen(link.xg, link.yg, center_x, center_y)
                painter.drawEllipse(com, self._COM_RADIUS_PX, self._COM_RADIUS_PX)

        # Show the latest IK click target: a hollow orange ring, plus a dashed line
        # to the tip so an unreachable target's residual gap is visible.
        if self._ik_target is not None:
            tip = self.skeleton.links[-1]
            tip_screen = self._world_to_screen(tip.xe, tip.ye, center_x, center_y)
            target_screen = self._world_to_screen(self._ik_target[0], self._ik_target[1], center_x, center_y)
            pen = QPen(self._TARGET_COLOR)
            pen.setStyle(Qt.PenStyle.DashLine)
            painter.setPen(pen)
            painter.setBrush(QBrush())  # hollow
            painter.drawLine(tip_screen, target_screen)
            pen.setStyle(Qt.PenStyle.SolidLine)
            painter.setPen(pen)
            painter.drawEllipse(target_screen, self._TARGET_RADIUS_PX, self._TARGET_RADIUS_PX)

    def _world_to_screen(self, wx: float, wy: float, cx: float, cy: float) -> QPointF:
        """Convert world coordinates (meters) to screen coordinates (pixels)."""
        sx = cx + wx * self.scale_factor
        # Invert Y because screen Y is down
        sy = cy - wy * self.scale_factor
        return QPointF(sx, sy)

    def update_skeleton(self) -> None:
        """Trigger a repaint."""
        self.update()

    def _world_from_screen(self, screen_pos: QPointF) -> tuple[float, float]:
        """Convert a widget pixel position to world coordinates (metres)."""
        center_x = self.width() / 2
        center_y = self.height() / 2
        # Inverse of _world_to_screen (screen Y is down).
        return (
            (screen_pos.x() - center_x) / self.scale_factor,
            (center_y - screen_pos.y()) / self.scale_factor,
        )

    def solve_to_world(self, x: float, y: float) -> None:
        """Solve inverse kinematics so the endpoint reaches world point ``(x, y)``.

        Updates the skeleton in place, emits :attr:`pose_changed`, and repaints.

        Parameters
        ----------
        x, y : float
            Target endpoint position in world coordinates (metres).
        """
        self.last_ik_result = compute_inverse_kinematics(self.skeleton, (x, y), method=self.ik_method)
        self._ik_target = (x, y)
        self.pose_changed.emit()
        self.update()

    def clear_ik_target(self) -> None:
        """Forget the last IK target and result (e.g. after manual posing)."""
        self.last_ik_result = None
        self._ik_target = None
        self.update()

    def mousePressEvent(self, a0: QMouseEvent | None) -> None:  # noqa: N802
        """Solve IK toward the point clicked with the left mouse button."""
        if a0 is not None and a0.button() == Qt.MouseButton.LeftButton:
            self.solve_to_world(*self._world_from_screen(a0.position()))

    def mouseMoveEvent(self, a0: QMouseEvent | None) -> None:  # noqa: N802
        """Continuously solve IK while dragging with the left mouse button held."""
        if a0 is not None and a0.buttons() & Qt.MouseButton.LeftButton:
            self.solve_to_world(*self._world_from_screen(a0.position()))


class SkelarmViewer(QMainWindow):
    """Main window for the Skelarm visualizer."""

    def __init__(self, skeleton: Skeleton) -> None:
        """Initialize the viewer."""
        super().__init__()
        self.skeleton = skeleton
        self._initial_q = skeleton.q.copy()  # pose to restore on reset

        self.canvas = SkelarmCanvas(skeleton)

        self.setWindowTitle("Skelarm Viewer")
        self.resize(1024, 768)

        # Main layout container
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        # Add canvas. Clicking/dragging in it solves IK; refresh the sliders to match.
        main_layout.addWidget(self.canvas, stretch=3)
        self.canvas.pose_changed.connect(self.refresh_from_skeleton)

        # Controls panel
        controls_panel = QWidget()
        controls_layout = QVBoxLayout(controls_panel)

        controls_label = QLabel("<b>Joint Controls</b>")
        controls_layout.addWidget(controls_label)
        hint_label = QLabel("Click or drag in the canvas to move the tip (IK).")
        hint_label.setWordWrap(True)
        controls_layout.addWidget(hint_label)

        self.sliders: list[QSlider] = []
        self.angle_labels: list[QLabel] = []

        # Skip the fixed base link (links[0]); only movable joints get a slider.
        for i, link in enumerate(skeleton.links[1:]):
            row_layout = QVBoxLayout()

            header_layout = QHBoxLayout()
            label = QLabel(f"Joint {i + 1}")
            value_label = QLabel()
            header_layout.addWidget(label)
            header_layout.addStretch()
            header_layout.addWidget(value_label)

            slider = QSlider(Qt.Orientation.Horizontal)
            # Range comes from the link's joint limits (radians). Round *inward*
            # (ceil the lower bound, floor the upper) so the integer-degree slider
            # can never request an angle past the enforced limits and trip a clamp
            # warning; max(...) guards a degenerate sub-degree range.
            lower_deg = math.ceil(math.degrees(link.prop.qmin))
            upper_deg = math.floor(math.degrees(link.prop.qmax))
            slider.setRange(lower_deg, max(lower_deg, upper_deg))
            # Set the initial value (Qt clamps it to the range above).
            slider.setValue(round(math.degrees(link.q)))
            slider.valueChanged.connect(functools.partial(self._on_joint_change, i))

            row_layout.addLayout(header_layout)
            row_layout.addWidget(slider)

            controls_layout.addLayout(row_layout)
            controls_layout.addSpacing(10)

            self.sliders.append(slider)
            self.angle_labels.append(value_label)

            # Label the (range-clamped) slider value so the two always agree.
            value_label.setText(f"{slider.value()}°")

        # Toggle for overlaying each link's center of mass on the canvas.
        self.com_checkbox = QCheckBox("Show center of mass")
        self.com_checkbox.toggled.connect(self._on_show_com_toggled)
        controls_layout.addWidget(self.com_checkbox)

        # IK solver method. Newton-Raphson needs a square Jacobian, so only offer it
        # for a two-joint arm.
        controls_layout.addWidget(QLabel("IK method"))
        self.method_combo = QComboBox()
        methods = ["lm_sugihara", "lm", "sr_inverse", "pseudoinverse"]
        if skeleton.num_joints == 2:  # noqa: PLR2004
            methods.append("nr")
        self.method_combo.addItems(methods)
        self.method_combo.currentTextChanged.connect(self._on_method_changed)
        controls_layout.addWidget(self.method_combo)

        # Reset the arm to the pose it was loaded with.
        self.reset_button = QPushButton("Reset pose")
        self.reset_button.clicked.connect(self._on_reset)
        controls_layout.addWidget(self.reset_button)

        # Live status: endpoint position and the latest IK result.
        self.status_label = QLabel()
        self.status_label.setWordWrap(True)
        controls_layout.addWidget(self.status_label)

        controls_layout.addStretch()
        main_layout.addWidget(controls_panel, stretch=1)
        self._update_status()

    def _on_show_com_toggled(self) -> None:
        """Toggle the center-of-mass overlay on the canvas and repaint."""
        self.canvas.show_com = self.com_checkbox.isChecked()
        self.canvas.update_skeleton()

    def _on_method_changed(self, method: str) -> None:
        """Route the selected IK method to the canvas solver."""
        self.canvas.ik_method = method

    def _on_reset(self) -> None:
        """Restore the initial pose and clear any IK target."""
        self.skeleton.q = self._initial_q.copy()
        self.canvas.clear_ik_target()
        self.refresh_from_skeleton()

    def _on_joint_change(self, joint: int, angle_deg: int) -> None:
        """Apply a single joint's slider value, leaving the other joints untouched.

        Updating only the moved joint preserves the exact angles of the others
        (the integer-degree sliders would otherwise snap them to whole degrees).

        Parameters
        ----------
        joint : int
            Index of the movable joint whose slider changed (0-based).
        angle_deg : int
            The slider's new value, in degrees.
        """
        self.angle_labels[joint].setText(f"{angle_deg}°")
        q = self.skeleton.q
        q[joint] = math.radians(angle_deg)
        # The eager q setter refreshes the forward kinematics.
        self.skeleton.q = q
        # Manual posing invalidates any prior IK target marker/result.
        self.canvas.clear_ik_target()
        self._update_status()

    def refresh_from_skeleton(self) -> None:
        """Sync the sliders, labels, and canvas to the current skeleton state.

        Call this after the joint angles change outside the GUI (for example,
        driven by a controller or simulation) so the controls reflect the model.
        Slider signals are blocked during the update so the rounded display
        values are not fed back into the skeleton.
        """
        for slider, label, angle in zip(self.sliders, self.angle_labels, self.skeleton.q, strict=True):
            with QSignalBlocker(slider):  # don't feed the rounded value back into the skeleton
                slider.setValue(round(math.degrees(angle)))
            # Read back the (range-clamped) slider value so the label always matches it.
            label.setText(f"{slider.value()}°")
        self.canvas.update_skeleton()
        self._update_status()

    def _update_status(self) -> None:
        """Refresh the status label with the endpoint position and latest IK result."""
        tip = self.skeleton.links[-1]
        text = f"Tip: ({tip.xe:.3f}, {tip.ye:.3f}) m"
        result = self.canvas.last_ik_result
        if result is not None:
            text += f"\nIK: {result.status}, residual={result.residual_norm:.3g}"
        self.status_label.setText(text)
