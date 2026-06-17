"""Provides a PyQt6 widget for visualizing the robot arm."""

from __future__ import annotations

import functools
import math
from typing import TYPE_CHECKING

from PyQt6.QtCore import QPointF, QSignalBlocker, Qt
from PyQt6.QtGui import QBrush, QColor, QPainter, QPaintEvent, QPen
from PyQt6.QtWidgets import (
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QSlider,
    QVBoxLayout,
    QWidget,
)

if TYPE_CHECKING:
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

    _JOINT_RADIUS_PX = 6  # also used for the origin, so the two match in size
    _LINK_WIDTH_PX = 4
    _LINK_COLOR = QColor(0, 100, 200)  # blue (movable links)
    _BASE_LINK_COLOR = QColor(150, 150, 150)  # gray (fixed base link)
    _JOINT_COLOR = QColor(200, 0, 0)  # red

    def __init__(self, skeleton: Skeleton, parent: QWidget | None = None) -> None:
        """Initialize the canvas."""
        super().__init__(parent)
        self.skeleton = skeleton
        self.scale_factor = _DEFAULT_SCALE  # Pixels per metre; re-fit on each paint.
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

        # Draw the origin (black) and every joint (red) on top, all the same size.
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QBrush(Qt.GlobalColor.black))
        origin = self._world_to_screen(0, 0, center_x, center_y)
        painter.drawEllipse(origin, self._JOINT_RADIUS_PX, self._JOINT_RADIUS_PX)
        painter.setBrush(QBrush(self._JOINT_COLOR))
        for link in self.skeleton.links:
            p2 = self._world_to_screen(link.xe, link.ye, center_x, center_y)
            painter.drawEllipse(p2, self._JOINT_RADIUS_PX, self._JOINT_RADIUS_PX)

    def _world_to_screen(self, wx: float, wy: float, cx: float, cy: float) -> QPointF:
        """Convert world coordinates (meters) to screen coordinates (pixels)."""
        sx = cx + wx * self.scale_factor
        # Invert Y because screen Y is down
        sy = cy - wy * self.scale_factor
        return QPointF(sx, sy)

    def update_skeleton(self) -> None:
        """Trigger a repaint."""
        self.update()


class SkelarmViewer(QMainWindow):
    """Main window for the Skelarm visualizer."""

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

        # Add canvas
        main_layout.addWidget(self.canvas, stretch=3)

        # Controls panel
        controls_panel = QWidget()
        controls_layout = QVBoxLayout(controls_panel)

        controls_label = QLabel("<b>Joint Controls</b>")
        controls_layout.addWidget(controls_label)

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
        self.canvas.update_skeleton()

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
