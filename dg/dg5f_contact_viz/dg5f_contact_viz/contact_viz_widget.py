"""Qt widget showing an anatomical hand (right / left / both) with contact lamps.

Subscribes to one or two std_msgs/Float32MultiArray topics (20 values each):
  * /dg5f_right/contact_level  (when display_mode is 'right' or 'both')
  * /dg5f_left/contact_level   (when display_mode is 'left' or 'both')

Lamps are placed at the actual joint centers: J1 abduction marker inside
the palm, J2 at the MCP knuckle, J3 at the PIP, J4 at the DIP. Colors use
HSV hue 120° (green) -> 0° (red) for level 0..1.

display_mode=left renders the same hand silhouette horizontally mirrored
(palm view of the opposite hand). display_mode=both splits the window
into two side-by-side sections — left hand on the left, right on the
right — with thumbs facing inward.
"""

import math
import signal
import sys
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from PyQt5.QtCore import Qt, QTimer, QPointF, QRectF
from PyQt5.QtGui import (
    QColor, QPainter, QPen, QBrush, QFont, QPainterPath,
    QLinearGradient
)
from PyQt5.QtWidgets import QApplication, QWidget


FINGER_NAMES = ["Thumb", "Index", "Middle", "Ring", "Pinky"]

# Canonical (right-hand, palm view) geometry. Mirrored at paint time
# when the drawing is for the left hand.
FINGERS = [
    ("Thumb",  (0.27, 0.72), (0.07, 0.38), 0.075, (-0.08, 0.05, 0.40, 0.75)),
    ("Index",  (0.33, 0.66), (0.33, 0.18), 0.070, (-0.05, 0.05, 0.40, 0.75)),
    ("Middle", (0.48, 0.66), (0.48, 0.10), 0.072, (-0.05, 0.05, 0.40, 0.75)),
    ("Ring",   (0.62, 0.66), (0.62, 0.17), 0.070, (-0.05, 0.05, 0.40, 0.75)),
    ("Pinky",  (0.76, 0.69), (0.76, 0.27), 0.060, (-0.05, 0.05, 0.40, 0.75)),
]

PALM_PATH_POINTS = [
    ("M", (0.28, 0.94)),
    ("L", (0.72, 0.94)),
    ("C", (0.82, 0.94), (0.86, 0.82), (0.83, 0.72)),
    ("C", (0.82, 0.68), (0.80, 0.66), (0.76, 0.66)),
    ("L", (0.72, 0.65)),
    ("L", (0.66, 0.65)),
    ("L", (0.58, 0.65)),
    ("L", (0.52, 0.65)),
    ("L", (0.44, 0.65)),
    ("L", (0.37, 0.65)),
    ("L", (0.30, 0.66)),
    ("C", (0.27, 0.68), (0.26, 0.70), (0.25, 0.73)),
    ("C", (0.24, 0.77), (0.22, 0.82), (0.24, 0.86)),
    ("C", (0.25, 0.90), (0.26, 0.93), (0.28, 0.94)),
]


def level_to_color(level: float) -> QColor:
    level = max(0.0, min(1.0, float(level)))
    hue = int(round((1.0 - level) * 120.0))
    return QColor.fromHsv(hue, 230, 235)


def lerp(a, b, t):
    return a + (b - a) * t


# ------------------------ ROS bridge ------------------------

class ContactBridge(Node):
    def __init__(self):
        super().__init__("contact_viz")
        self.declare_parameter("display_mode", "right")       # right | left | both
        self.declare_parameter(
            "right_level_topic", "/dg5f_right/contact_level")
        self.declare_parameter(
            "left_level_topic", "/dg5f_left/contact_level")

        self.mode = str(self.get_parameter("display_mode").value).lower()
        if self.mode not in ("right", "left", "both"):
            raise ValueError(
                f"display_mode must be right | left | both, got {self.mode!r}")

        self._right_levels = [0.0] * 20
        self._left_levels = [0.0] * 20
        self._lock = threading.Lock()

        if self.mode in ("right", "both"):
            t = self.get_parameter("right_level_topic").value
            self.create_subscription(
                Float32MultiArray, t, self._on_right, 10)
            self.get_logger().info(f"contact_viz: subscribed right on {t}")
        if self.mode in ("left", "both"):
            t = self.get_parameter("left_level_topic").value
            self.create_subscription(
                Float32MultiArray, t, self._on_left, 10)
            self.get_logger().info(f"contact_viz: subscribed left on {t}")

    def _on_right(self, msg):
        self._store(self._right_levels, msg)

    def _on_left(self, msg):
        self._store(self._left_levels, msg)

    def _store(self, target, msg):
        data = list(msg.data)
        if len(data) < 20:
            data = data + [0.0] * (20 - len(data))
        with self._lock:
            for i in range(20):
                target[i] = float(max(0.0, min(1.0, data[i])))

    def snapshot(self):
        with self._lock:
            return list(self._right_levels), list(self._left_levels)


# ------------------------ HandWidget ------------------------

class HandWidget(QWidget):
    LAMP_DIAMETER = 26
    MIN_W = 520
    MIN_H = 640

    COLOR_BG = QColor("#1a1a1a")
    COLOR_HAND_FILL = QColor("#5d4a3c")
    COLOR_HAND_FILL_LIGHT = QColor("#6d5a4a")
    COLOR_HAND_EDGE = QColor("#8a715e")
    COLOR_NAIL = QColor("#d8c4b0")
    COLOR_LABEL = QColor("#cfcfcf")
    COLOR_BADGE = QColor("#8acafa")
    COLOR_LAMP_OUTLINE = QColor("#111")

    def __init__(self, bridge: ContactBridge):
        super().__init__()
        self._bridge = bridge
        self._mode = bridge.mode

        if self._mode == "both":
            self.setWindowTitle("DG5F Contact (bimanual)")
            self.setMinimumSize(self.MIN_W * 2, self.MIN_H)
        elif self._mode == "left":
            self.setWindowTitle("DG5F Contact (left hand)")
            self.setMinimumSize(self.MIN_W, self.MIN_H)
        else:
            self.setWindowTitle("DG5F Contact (right hand)")
            self.setMinimumSize(self.MIN_W, self.MIN_H)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self.update)
        self._timer.start(50)

    # --------------- geometry helpers ---------------

    @staticmethod
    def _rx(x, mirror):
        return (1.0 - x) if mirror else x

    def _pt(self, x, y, w, h, mirror):
        return QPointF(self._rx(x, mirror) * w, y * h)

    def _finger_axis_point(self, finger_idx, frac, w, h, mirror):
        base = FINGERS[finger_idx][1]
        tip = FINGERS[finger_idx][2]
        x = lerp(base[0], tip[0], frac)
        y = lerp(base[1], tip[1], frac)
        return QPointF(self._rx(x, mirror) * w, y * h)

    # --------------- paint ---------------

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        p.fillRect(self.rect(), self.COLOR_BG)

        right_levels, left_levels = self._bridge.snapshot()

        if self._mode == "both":
            half = self.width() // 2
            self._paint_one_hand(
                p, 0, 0, half, self.height(),
                levels=left_levels, mirror=True, badge="LEFT")
            self._paint_one_hand(
                p, half, 0, self.width() - half, self.height(),
                levels=right_levels, mirror=False, badge="RIGHT")
            # Separator line
            p.setPen(QPen(QColor("#333"), 1, Qt.DotLine))
            p.drawLine(half, 12, half, self.height() - 12)
        elif self._mode == "left":
            self._paint_one_hand(
                p, 0, 0, self.width(), self.height(),
                levels=left_levels, mirror=True)
        else:
            self._paint_one_hand(
                p, 0, 0, self.width(), self.height(),
                levels=right_levels, mirror=False)

    # ------ one section ------

    def _paint_one_hand(self, p, ox, oy, w, h, levels, mirror, badge=None):
        p.save()
        p.translate(ox, oy)

        self._paint_hand(p, w, h, mirror)
        self._paint_finger_labels(p, w, h, mirror)
        self._paint_lamps(p, w, h, levels, mirror)
        if badge:
            self._paint_badge(p, w, h, badge)

        p.restore()

    def _paint_hand(self, p, w, h, mirror):
        palm = self._build_palm_path(w, h, mirror)
        grad = QLinearGradient(QPointF(0.5 * w, 0.66 * h),
                               QPointF(0.5 * w, 0.94 * h))
        grad.setColorAt(0.0, self.COLOR_HAND_FILL_LIGHT)
        grad.setColorAt(1.0, self.COLOR_HAND_FILL)
        p.setPen(QPen(self.COLOR_HAND_EDGE, 2))
        p.setBrush(QBrush(grad))
        p.drawPath(palm)

        for f in range(5):
            self._paint_finger(p, w, h, f, mirror)

        p.setPen(QPen(QColor("#715a48"), 1))
        for f in range(5):
            self._paint_nail(p, w, h, f, mirror)

    def _build_palm_path(self, w, h, mirror):
        path = QPainterPath()
        for entry in PALM_PATH_POINTS:
            cmd = entry[0]
            if cmd == "M":
                path.moveTo(self._pt(*entry[1], w, h, mirror))
            elif cmd == "L":
                path.lineTo(self._pt(*entry[1], w, h, mirror))
            elif cmd == "C":
                c1 = self._pt(*entry[1], w, h, mirror)
                c2 = self._pt(*entry[2], w, h, mirror)
                end = self._pt(*entry[3], w, h, mirror)
                path.cubicTo(c1, c2, end)
        path.closeSubpath()
        return path

    def _paint_finger(self, p, w, h, f, mirror):
        _, base, tip, width_ratio, _ = FINGERS[f]
        bx = self._rx(base[0], mirror) * w
        by = base[1] * h
        tx = self._rx(tip[0], mirror) * w
        ty = tip[1] * h
        dx, dy = tx - bx, ty - by
        length = (dx * dx + dy * dy) ** 0.5
        if length < 1.0:
            return
        ux, uy = dx / length, dy / length
        px, py = -uy, ux
        half_w_px = (width_ratio * w) / 2.0

        knuckle_fracs = [0.40, 0.75]

        bl = QPointF(bx + px * half_w_px, by + py * half_w_px)
        br = QPointF(bx - px * half_w_px, by - py * half_w_px)

        taper = 0.82
        taper_l = QPointF(tx + px * half_w_px * taper, ty + py * half_w_px * taper)
        taper_r = QPointF(tx - px * half_w_px * taper, ty - py * half_w_px * taper)

        tip_pad_len = half_w_px * 1.1
        tip_outer = QPointF(tx + ux * tip_pad_len, ty + uy * tip_pad_len)

        path = QPainterPath()
        path.moveTo(bl)
        path.lineTo(taper_l)
        path.cubicTo(
            QPointF(taper_l.x() + ux * half_w_px, taper_l.y() + uy * half_w_px),
            QPointF(tip_outer.x() + px * half_w_px * 0.3,
                    tip_outer.y() + py * half_w_px * 0.3),
            tip_outer,
        )
        path.cubicTo(
            QPointF(tip_outer.x() - px * half_w_px * 0.3,
                    tip_outer.y() - py * half_w_px * 0.3),
            QPointF(taper_r.x() + ux * half_w_px, taper_r.y() + uy * half_w_px),
            taper_r,
        )
        path.lineTo(br)
        path.closeSubpath()

        grad = QLinearGradient(bl, QPointF(tx, ty))
        grad.setColorAt(0.0, self.COLOR_HAND_FILL_LIGHT)
        grad.setColorAt(1.0, self.COLOR_HAND_FILL)
        p.setPen(QPen(self.COLOR_HAND_EDGE, 2))
        p.setBrush(QBrush(grad))
        p.drawPath(path)

        p.setPen(QPen(QColor("#4a3a2e"), 1, Qt.SolidLine))
        for kf in knuckle_fracs:
            kx = bx + dx * kf
            ky = by + dy * kf
            k_w_px = half_w_px * 0.92
            a = QPointF(kx + px * k_w_px, ky + py * k_w_px)
            b = QPointF(kx - px * k_w_px, ky - py * k_w_px)
            p.drawLine(a, b)

    def _paint_nail(self, p, w, h, f, mirror):
        _, base, tip, width_ratio, _ = FINGERS[f]
        tx = self._rx(tip[0], mirror) * w
        ty = tip[1] * h
        bx = self._rx(base[0], mirror) * w
        by = base[1] * h
        dx, dy = tx - bx, ty - by
        length = (dx * dx + dy * dy) ** 0.5
        if length < 1.0:
            return
        ux, uy = dx / length, dy / length
        half_w_px = (width_ratio * w) / 2.0
        center = QPointF(tx - ux * half_w_px * 0.25,
                         ty - uy * half_w_px * 0.25)
        nail_w = half_w_px * 1.0
        nail_h = half_w_px * 0.55
        p.save()
        p.translate(center)
        angle_deg = math.degrees(math.atan2(dy, dx)) - 90.0
        p.rotate(angle_deg)
        p.setPen(QPen(QColor("#a58876"), 1))
        p.setBrush(QBrush(self.COLOR_NAIL))
        p.drawEllipse(QRectF(-nail_w, -nail_h, 2 * nail_w, 2 * nail_h))
        p.restore()

    def _paint_finger_labels(self, p, w, h, mirror):
        p.setPen(self.COLOR_LABEL)
        font = QFont()
        font.setPointSize(9)
        p.setFont(font)
        label_positions = [
            (0.22, 0.82),
            (0.33, 0.98),
            (0.48, 0.98),
            (0.62, 0.98),
            (0.76, 0.98),
        ]
        for (rx, ry), name in zip(label_positions, FINGER_NAMES):
            x = self._rx(rx, mirror) * w
            y = ry * h
            p.drawText(QRectF(x - 40, y - 9, 80, 18),
                       Qt.AlignCenter, name)

    def _paint_lamps(self, p, w, h, levels, mirror):
        font = QFont()
        font.setPointSize(8)
        p.setFont(font)
        for f in range(5):
            jfracs = FINGERS[f][4]
            for j in range(4):
                center = self._finger_axis_point(f, jfracs[j], w, h, mirror)
                self._paint_lamp(
                    p, center, levels[f * 4 + j],
                    label=f"J{j + 1}",
                    is_abduction=(j == 0),
                    mirror=mirror,
                )

    def _paint_lamp(self, p, center, level, label, is_abduction, mirror):
        r = self.LAMP_DIAMETER / 2.0
        if is_abduction:
            r *= 0.70
            p.setPen(QPen(QColor("#333"), 1, Qt.DashLine))
        else:
            p.setPen(QPen(self.COLOR_LAMP_OUTLINE, 2))
        p.setBrush(QBrush(level_to_color(level)))
        p.drawEllipse(center, r, r)
        if not is_abduction:
            p.setPen(QColor("#9a9a9a"))
            # Label floats opposite-side of center from the lamp so it does not
            # land on top of an adjacent finger's lamp after mirroring.
            xoff = -(r + 3 + 20) if mirror else (r + 3)
            p.drawText(QRectF(center.x() + xoff, center.y() - 8, 24, 16),
                       Qt.AlignLeft | Qt.AlignVCenter, label)

    def _paint_badge(self, p, w, h, text):
        # Top-center hand label when running in bimanual mode.
        p.setPen(self.COLOR_BADGE)
        font = QFont()
        font.setPointSize(11)
        font.setBold(True)
        p.setFont(font)
        p.drawText(QRectF(0, 6, w, 22), Qt.AlignCenter, text)


# ------------------------ main ------------------------

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
    w = HandWidget.MIN_W * (2 if bridge.mode == "both" else 1)
    win.resize(w, HandWidget.MIN_H)
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
