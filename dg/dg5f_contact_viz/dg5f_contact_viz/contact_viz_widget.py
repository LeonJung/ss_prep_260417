"""Qt widget showing a right-hand silhouette with per-joint contact lamps.

Subscribes to std_msgs/Float32MultiArray (20 values in [0,1]) and colors
each of the 20 joint dots from green (0.0) through yellow to red (1.0)
via HSV interpolation (hue 120 -> 0 degrees). Layout is a simple
palm-facing-viewer schematic of the right hand: wrist at the bottom,
fingertips at the top, thumb angled outward to the left.
"""

import signal
import sys
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from PyQt5.QtCore import Qt, QTimer, QPointF, QRectF
from PyQt5.QtGui import QColor, QPainter, QPen, QBrush, QFont
from PyQt5.QtWidgets import QApplication, QWidget


FINGER_NAMES = ["Thumb", "Index", "Middle", "Ring", "Pinky"]

# Joint positions in canvas-relative coordinates (x, y) in [0, 1].
# Each finger is 4 joints [J1 base .. J4 tip]. Thumb is diagonal,
# other fingers are vertical columns with middle tallest.
JOINT_POS_RATIO = [
    # Thumb (F1): lower-left, angled up-left
    (0.22, 0.64), (0.15, 0.54), (0.09, 0.44), (0.04, 0.34),
    # Index (F2)
    (0.32, 0.56), (0.32, 0.44), (0.32, 0.32), (0.32, 0.20),
    # Middle (F3) — tallest
    (0.47, 0.56), (0.47, 0.42), (0.47, 0.28), (0.47, 0.14),
    # Ring (F4)
    (0.62, 0.56), (0.62, 0.44), (0.62, 0.32), (0.62, 0.20),
    # Pinky (F5) — shortest
    (0.76, 0.59), (0.76, 0.48), (0.76, 0.37), (0.76, 0.26),
]
assert len(JOINT_POS_RATIO) == 20

# Palm polygon ratio points (bottom to top, closed).
PALM_RATIO = [
    (0.18, 0.92),  # lower-left (wrist L)
    (0.82, 0.92),  # lower-right (wrist R)
    (0.82, 0.66),  # right side just under pinky base
    (0.28, 0.70),  # left side (thumb web)
]


def level_to_color(level: float) -> QColor:
    level = max(0.0, min(1.0, float(level)))
    hue = int(round((1.0 - level) * 120.0))
    return QColor.fromHsv(hue, 230, 230)


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


class HandWidget(QWidget):
    LAMP_DIAMETER = 32
    LINK_WIDTH = 4
    MIN_W = 460
    MIN_H = 580

    def __init__(self, bridge: ContactBridge):
        super().__init__()
        self.setWindowTitle("DG5F Contact (right hand)")
        self._bridge = bridge
        self._levels = [0.0] * 20
        self.setMinimumSize(self.MIN_W, self.MIN_H)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._refresh)
        self._timer.start(50)

    def _refresh(self):
        self._levels = self._bridge.snapshot()
        self.update()

    # ------------------------- paint -------------------------

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        w, h = self.width(), self.height()

        # Background
        p.fillRect(self.rect(), QColor("#2b2b2b"))

        # Palm
        palm_color = QColor("#444")
        p.setPen(QPen(QColor("#666"), 2))
        p.setBrush(QBrush(palm_color))
        palm_pts = [QPointF(x * w, y * h) for (x, y) in PALM_RATIO]
        p.drawPolygon(*palm_pts)

        # Finger structure lines (J1 -> J2 -> J3 -> J4) per finger
        link_pen = QPen(QColor("#5a5a5a"), self.LINK_WIDTH,
                        Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin)
        p.setPen(link_pen)
        for f in range(5):
            pts = [self._joint_xy(f * 4 + j, w, h) for j in range(4)]
            for a, b in zip(pts, pts[1:]):
                p.drawLine(a, b)

        # Link from palm anchor to J1 of each non-thumb finger
        anchor_pen = QPen(QColor("#4a4a4a"), self.LINK_WIDTH - 1,
                          Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin)
        p.setPen(anchor_pen)
        for f, anchor_ratio in enumerate([
            (0.30, 0.72),  # thumb base anchor at palm
            (0.32, 0.66),  # index
            (0.47, 0.66),  # middle
            (0.62, 0.66),  # ring
            (0.76, 0.68),  # pinky
        ]):
            ax, ay = anchor_ratio
            p.drawLine(QPointF(ax * w, ay * h),
                       self._joint_xy(f * 4, w, h))

        # Finger name labels (near palm, one per finger)
        p.setPen(QColor("#cfcfcf"))
        label_font = QFont()
        label_font.setPointSize(10)
        p.setFont(label_font)
        for f, name in enumerate(FINGER_NAMES):
            base = self._joint_xy(f * 4, w, h)
            p.drawText(QRectF(base.x() - 40, base.y() + 16, 80, 18),
                       Qt.AlignCenter, name)

        # Joint lamps (20 circles, color by level)
        small_font = QFont()
        small_font.setPointSize(8)
        p.setFont(small_font)
        d = self.LAMP_DIAMETER
        r = d / 2.0
        outline = QPen(QColor("#111"), 2)
        for idx in range(20):
            cx, cy = self._joint_xy(idx, w, h).x(), self._joint_xy(idx, w, h).y()
            color = level_to_color(self._levels[idx])
            p.setPen(outline)
            p.setBrush(QBrush(color))
            p.drawEllipse(QPointF(cx, cy), r, r)
            # label J1..J4 slightly offset to the right of each lamp
            j_label = f"J{(idx % 4) + 1}"
            p.setPen(QColor("#8e8e8e"))
            label_x_offset = 14 if (idx // 4) != 0 else -28  # thumb label to left
            p.drawText(QRectF(cx + label_x_offset, cy - 8, 28, 16),
                       Qt.AlignLeft | Qt.AlignVCenter, j_label)

    def _joint_xy(self, idx: int, w: int, h: int) -> QPointF:
        rx, ry = JOINT_POS_RATIO[idx]
        return QPointF(rx * w, ry * h)


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

    win = HandWidget(bridge)
    win.resize(HandWidget.MIN_W, HandWidget.MIN_H)
    win.show()

    ros_thread = threading.Thread(target=_spin_ros, args=(bridge,), daemon=True)
    ros_thread.start()

    exit_code = app.exec_()

    bridge.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
