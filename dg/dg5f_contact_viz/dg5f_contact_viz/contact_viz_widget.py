"""Qt widget showing a 5-finger x 4-joint grid colored by contact level.

Subscribes to std_msgs/Float32MultiArray (20 values in [0,1]) and colors
each joint cell from green (0.0) through yellow to red (1.0) using HSV
interpolation (hue 120 -> 0 degrees).
"""

import signal
import sys
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QColor, QPalette
from PyQt5.QtWidgets import (
    QApplication, QGridLayout, QLabel, QVBoxLayout, QWidget
)


FINGER_LABELS = ["F1 (Thumb)", "F2 (Index)", "F3 (Middle)", "F4 (Ring)", "F5 (Pinky)"]
JOINT_LABELS = ["J1", "J2", "J3", "J4"]
CELL_MIN_PX = 56


class ContactBridge(Node):
    def __init__(self):
        super().__init__("contact_viz")
        self.declare_parameter("contact_level_topic", "/dg5f_right/contact_level")
        topic = self.get_parameter("contact_level_topic").value
        self._levels = [0.0] * 20
        self._lock = threading.Lock()
        self._sub = self.create_subscription(
            Float32MultiArray, topic, self._on_levels, 10
        )
        self.get_logger().info(f"contact_viz: listening on {topic}")

    def _on_levels(self, msg: Float32MultiArray):
        data = list(msg.data)
        if len(data) < 20:
            data = data + [0.0] * (20 - len(data))
        with self._lock:
            self._levels = [float(max(0.0, min(1.0, v))) for v in data[:20]]

    def snapshot(self):
        with self._lock:
            return list(self._levels)


class HandGrid(QWidget):
    def __init__(self, bridge: ContactBridge):
        super().__init__()
        self.setWindowTitle("DG5F Contact (right hand)")
        self._bridge = bridge
        self._cells = [[None] * 4 for _ in range(5)]

        root = QVBoxLayout(self)
        grid = QGridLayout()
        grid.setHorizontalSpacing(6)
        grid.setVerticalSpacing(6)

        grid.addWidget(QLabel(""), 0, 0)
        for j, label in enumerate(JOINT_LABELS):
            lab = QLabel(label)
            lab.setAlignment(Qt.AlignCenter)
            grid.addWidget(lab, 0, j + 1)

        for f, flabel in enumerate(FINGER_LABELS):
            lab = QLabel(flabel)
            lab.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            grid.addWidget(lab, f + 1, 0)
            for j in range(4):
                cell = QLabel("")
                cell.setMinimumSize(CELL_MIN_PX, CELL_MIN_PX)
                cell.setAutoFillBackground(True)
                cell.setAlignment(Qt.AlignCenter)
                self._cells[f][j] = cell
                grid.addWidget(cell, f + 1, j + 1)

        root.addLayout(grid)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._refresh)
        self._timer.start(50)

    def _refresh(self):
        levels = self._bridge.snapshot()
        for f in range(5):
            for j in range(4):
                self._set_cell_color(self._cells[f][j], levels[f * 4 + j])

    @staticmethod
    def _set_cell_color(cell: QLabel, level: float):
        hue = int(round((1.0 - level) * 120.0))
        color = QColor.fromHsv(hue, 230, 230)
        pal = cell.palette()
        pal.setColor(QPalette.Window, color)
        cell.setPalette(pal)


def _spin_ros(node: Node):
    try:
        rclpy.spin(node)
    except Exception:
        pass


def main(args=None):
    rclpy.init(args=args)
    bridge = ContactBridge()

    app = QApplication(sys.argv)
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    win = HandGrid(bridge)
    win.resize(520, 380)
    win.show()

    ros_thread = threading.Thread(target=_spin_ros, args=(bridge,), daemon=True)
    ros_thread.start()

    exit_code = app.exec_()

    bridge.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
