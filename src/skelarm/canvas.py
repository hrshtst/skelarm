"""Provides a PyQt6 widget for visualizing the robot arm."""

from __future__ import annotations

import functools
import itertools
import math
from typing import TYPE_CHECKING

from PyQt6.QtCore import QPointF, QSignalBlocker, Qt, pyqtSignal
from PyQt6.QtGui import QBrush, QColor, QMouseEvent, QPainter, QPaintEvent, QPen
from PyQt6.QtWidgets import (
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QSlider,
    QVBoxLayout,
    QWidget,
)

from skelarm.kinematics import compute_inverse_kinematics

if TYPE_CHECKING:
    import numpy as np
    from numpy.typing import NDArray

    from skelarm.kinematics import IKResult
    from skelarm.skeleton import Skeleton

_MARGIN_PX = 20.0  # Empty border kept between the arm and the widget edge.
_DEFAULT_SCALE = 100.0  # Pixels per meter, used before the reach/size are known.
_ARROW_HEAD_PX = 10.0  # Length of the arrowhead strokes, in pixels.
_PANEL_WIDTH_PX = 300  # Fixed side-panel width so its content can't resize it.

# Task-overlay marker styling, shared by the interactive simulators and the player.
_GOAL_COLOR = QColor(170, 0, 170)  # purple (task target marker)
_GOAL_DOT_PX = 4.0  # filled target-dot radius (px)
_GOAL_RING_PX = 9.0  # hollow ring radius (px) when no success tolerance is set
_REFERENCE_COLOR = QColor(120, 120, 120)  # gray (reference curve / trajectory path)


def draw_arrow(
    painter: QPainter,
    start: QPointF,
    end: QPointF,
    *,
    color: QColor,
    width: int = 2,
    head_px: float = _ARROW_HEAD_PX,
) -> None:
    """Draw a line from ``start`` to ``end`` with an arrowhead at ``end``.

    A small reusable primitive shared by the simulator's interactive force arrow
    and the replay tool's recorded-force overlay.

    Parameters
    ----------
    painter : QPainter
        An active painter to draw with.
    start, end : QPointF
        Screen-space endpoints; the arrowhead is drawn at ``end``.
    color : QColor
        Pen color for the shaft and head.
    width : int, optional
        Pen width in pixels.
    head_px : float, optional
        Length of each arrowhead stroke in pixels.
    """
    pen = QPen(color)
    pen.setWidth(width)
    painter.setPen(pen)
    painter.drawLine(start, end)
    angle = math.atan2(end.y() - start.y(), end.x() - start.x())
    for offset in (math.radians(150.0), math.radians(-150.0)):
        head_x = end.x() + head_px * math.cos(angle + offset)
        head_y = end.y() + head_px * math.sin(angle + offset)
        painter.drawLine(end, QPointF(head_x, head_y))


def _fit_scale(reach: float, width: int, height: int) -> float:
    """Compute a pixels-per-meter scale that fits an arm of ``reach`` in the widget.

    The arm is drawn from the center of the widget and can extend up to ``reach``
    meters in any direction, so the scale is set from the smaller half-dimension
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
        Scale in pixels per meter, falling back to :data:`_DEFAULT_SCALE` when the
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
    _FORCE_COLOR = QColor(220, 0, 0)  # red (external force arrow)

    def __init__(self, skeleton: Skeleton, parent: QWidget | None = None) -> None:
        """Initialize the canvas."""
        super().__init__(parent)
        self.skeleton = skeleton
        self.scale_factor = _DEFAULT_SCALE  # Pixels per meter; re-fit on each paint.
        self.show_com = False  # whether to overlay each link's center of mass
        self.ik_method = "lm_sugihara"  # solver method used by solve_to_world
        self.last_ik_result: IKResult | None = None  # outcome of the latest IK solve
        self._ik_target: tuple[float, float] | None = None  # latest IK click target
        # Optional external force (N) at the tip, drawn as an arrow of length
        # force * force_scale (meters per Newton). None hides the arrow.
        self.tip_force: NDArray[np.float64] | None = None
        self.force_scale = 1.0
        # Task overlays (set by the player / scenario simulators), each toggleable:
        # a reference polyline (curve / tracked trajectory) and target markers, where
        # each target is (position, color, success tolerance | None, active?).
        self.overlay_path: NDArray[np.float64] | None = None
        self.overlay_path_color = _REFERENCE_COLOR
        self.show_overlay_path = True
        self.overlay_targets: list[tuple[NDArray[np.float64], QColor, float | None, bool]] = []
        self.show_overlay_targets = True
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

        self._draw_task_overlays(painter, center_x, center_y)

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

        # Show an external force applied at the tip as a red arrow (force * force_scale).
        if self.tip_force is not None:
            fx = float(self.tip_force[0])
            fy = float(self.tip_force[1])
            if math.hypot(fx, fy) > 0.0:
                tip = self.skeleton.links[-1]
                start = self._world_to_screen(tip.xe, tip.ye, center_x, center_y)
                end = self._world_to_screen(
                    tip.xe + fx * self.force_scale, tip.ye + fy * self.force_scale, center_x, center_y
                )
                draw_arrow(painter, start, end, color=self._FORCE_COLOR)

    def _world_to_screen(self, wx: float, wy: float, cx: float, cy: float) -> QPointF:
        """Convert world coordinates (meters) to screen coordinates (pixels)."""
        sx = cx + wx * self.scale_factor
        # Invert Y because screen Y is down
        sy = cy - wy * self.scale_factor
        return QPointF(sx, sy)

    def _draw_target_marker(
        self,
        painter: QPainter,
        pos: NDArray[np.float64],
        cx: float,
        cy: float,
        *,
        color: QColor,
        tolerance: float | None,
        active: bool,
    ) -> None:
        """Draw a task target: a filled dot + ring when active, a dashed ring otherwise."""
        point = self._world_to_screen(float(pos[0]), float(pos[1]), cx, cy)
        if active:
            painter.setPen(QPen(color))
            painter.setBrush(QBrush(color))
            painter.drawEllipse(point, _GOAL_DOT_PX, _GOAL_DOT_PX)
            ring_px = tolerance * self.scale_factor if tolerance is not None else _GOAL_RING_PX
            painter.setBrush(QBrush())  # hollow ring
            painter.drawEllipse(point, ring_px, ring_px)
        else:
            pen = QPen(color)
            pen.setStyle(Qt.PenStyle.DashLine)
            painter.setPen(pen)
            painter.setBrush(QBrush())
            painter.drawEllipse(point, _GOAL_RING_PX, _GOAL_RING_PX)

    def _draw_path(
        self,
        painter: QPainter,
        points: NDArray[np.float64],
        cx: float,
        cy: float,
        color: QColor,
    ) -> None:
        """Draw a world-space polyline (a reference curve / trajectory) as a dashed line."""
        if len(points) < 2:  # noqa: PLR2004 — need at least a segment to draw
            return
        pen = QPen(color)
        pen.setStyle(Qt.PenStyle.DashLine)
        pen.setWidth(2)
        painter.setPen(pen)
        painter.setBrush(QBrush())
        screen = [self._world_to_screen(float(p[0]), float(p[1]), cx, cy) for p in points]
        for start, end in itertools.pairwise(screen):
            painter.drawLine(start, end)

    def _draw_task_overlays(self, painter: QPainter, cx: float, cy: float) -> None:
        """Draw the optional reference path and task target markers (each toggleable)."""
        if self.overlay_path is not None and self.show_overlay_path:
            self._draw_path(painter, self.overlay_path, cx, cy, self.overlay_path_color)
        if self.show_overlay_targets:
            for pos, color, tolerance, active in self.overlay_targets:
                self._draw_target_marker(painter, pos, cx, cy, color=color, tolerance=tolerance, active=active)

    def update_skeleton(self) -> None:
        """Trigger a repaint."""
        self.update()

    def _world_from_screen(self, screen_pos: QPointF) -> tuple[float, float]:
        """Convert a widget pixel position to world coordinates (meters)."""
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
            Target endpoint position in world coordinates (meters).
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
    """A minimal viewer: the arm canvas plus one slider per joint.

    Drag a slider to set a joint angle (forward kinematics), or click and drag in
    the canvas to move the tip (inverse kinematics). Subclasses can add extra
    controls with :meth:`add_control` and react to pose changes via the
    :attr:`pose_updated` signal.
    """

    pose_updated = pyqtSignal()  # emitted after any FK (slider) or IK (canvas) pose change

    def __init__(self, skeleton: Skeleton) -> None:
        """Initialize the viewer."""
        super().__init__()
        self.skeleton = skeleton

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
        controls_panel.setFixedWidth(_PANEL_WIDTH_PX)  # keep a constant width regardless of content
        controls_layout = QVBoxLayout(controls_panel)
        self.controls_panel = controls_panel  # exposed for sizing (fixed width) and tests
        self.controls_layout = controls_layout  # exposed so subclasses can add_control

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

        controls_layout.addStretch()
        main_layout.addWidget(controls_panel, stretch=1)

    def add_control(self, widget: QWidget) -> None:
        """Add an extra control widget above the trailing stretch (for subclasses)."""
        self.controls_layout.insertWidget(self.controls_layout.count() - 1, widget)

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
        self.pose_updated.emit()

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
        self.pose_updated.emit()
