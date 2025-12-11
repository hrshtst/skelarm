"""Provides a PyQt6 widget for visualizing the robot arm."""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import numpy as np
from PyQt6.QtCore import QPointF, Qt
from PyQt6.QtGui import QBrush, QColor, QPainter, QPen
from PyQt6.QtWidgets import (
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QSlider,
    QVBoxLayout,
    QWidget,
)

from skelarm.kinematics import compute_forward_kinematics

if TYPE_CHECKING:
    from skelarm.skeleton import Skeleton


class SkelarmCanvas(QWidget):
    """A widget to draw the robot arm skeleton."""

    def __init__(self, skeleton: Skeleton, parent: QWidget | None = None) -> None:
        """Initialize the canvas."""
        super().__init__(parent)
        self.skeleton = skeleton
        self.scale_factor = 100.0  # Pixels per meter
        # Set background color to white
        self.setAutoFillBackground(True)
        p = self.palette()
        p.setColor(self.backgroundRole(), QColor("white"))
        self.setPalette(p)

    def paintEvent(self, a0) -> None:  # noqa: ANN001, N802, ARG002
        """Paint the robot arm."""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        # Center of the widget
        center_x = self.width() / 2
        center_y = self.height() / 2

        # Draw Base
        painter.setBrush(QBrush(Qt.GlobalColor.black))
        base_screen = self.world_to_screen(0, 0, center_x, center_y)
        painter.drawEllipse(base_screen, 10, 10)

        # Draw Links
        pen_link = QPen(QColor(0, 100, 200))  # Blue-ish
        pen_link.setWidth(6)
        pen_link.setCapStyle(Qt.PenCapStyle.RoundCap)
        painter.setPen(pen_link)

        # Draw Joints
        brush_joint = QBrush(QColor(200, 0, 0))  # Red

        # Iterate through links
        # Note: We assume compute_forward_kinematics has been called externally
        # or we could call it here, but typically the state is updated elsewhere.

        for link in self.skeleton.links:
            p1 = self.world_to_screen(link.x, link.y, center_x, center_y)
            p2 = self.world_to_screen(link.xe, link.ye, center_x, center_y)

            # Draw link segment
            painter.drawLine(p1, p2)

            # Draw joint at the end of the link (tip or next joint)
            painter.setPen(Qt.PenStyle.NoPen)
            painter.setBrush(brush_joint)
            painter.drawEllipse(p2, 6, 6)

            # Reset pen for next link
            painter.setPen(pen_link)

    def world_to_screen(self, wx: float, wy: float, cx: float, cy: float) -> QPointF:
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

        # Ensure kinematics are computed initially
        compute_forward_kinematics(self.skeleton)

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

        for i, link in enumerate(skeleton.links):
            row_layout = QVBoxLayout()

            header_layout = QHBoxLayout()
            label = QLabel(f"Joint {i + 1}")
            value_label = QLabel("0.0°")
            header_layout.addWidget(label)
            header_layout.addStretch()
            header_layout.addWidget(value_label)

            slider = QSlider(Qt.Orientation.Horizontal)
            # Map -180..180 degrees to slider values
            # Using link limits if available would be better, but fixed for now
            slider.setRange(-180, 180)
            # Set initial value
            initial_deg = int(math.degrees(link.q))
            slider.setValue(initial_deg)
            slider.valueChanged.connect(self.on_slider_change)

            row_layout.addLayout(header_layout)
            row_layout.addWidget(slider)

            controls_layout.addLayout(row_layout)
            controls_layout.addSpacing(10)

            self.sliders.append(slider)
            self.angle_labels.append(value_label)

            # Initialize label text
            value_label.setText(f"{initial_deg}°")

        controls_layout.addStretch()
        main_layout.addWidget(controls_panel, stretch=1)

    def on_slider_change(self) -> None:
        """Handle slider value changes."""
        new_q = []
        for i, slider in enumerate(self.sliders):
            angle_deg = slider.value()
            self.angle_labels[i].setText(f"{angle_deg}°")
            new_q.append(math.radians(angle_deg))

        self.skeleton.q = np.array(new_q)
        compute_forward_kinematics(self.skeleton)
        self.canvas.update_skeleton()
