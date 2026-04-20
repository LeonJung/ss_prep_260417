"""Qt widget showing an anatomical right-hand rendering with 25 contact lamps.

Subscribes to std_msgs/Float32MultiArray on two topics:
  * contact_level_topic   — 20 values, motor-current-derived (4 joints * 5 fingers)
  * fingertip_level_topic —  5 values, F/T-sensor-derived fingertip pressure

Lamps are placed at the actual anatomical joint centers (MCP abduction
marker inside the palm, MCP flex at the knuckle, PIP mid-finger, DIP
near the tip) and an additional 5 lamps sit on the distal phalanx for
the fingertip F/T contact.

Color mapping: HSV hue 120° (green) at level 0 -> 0° (red) at level 1.
"""

import signal
import sys
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import WrenchStamped

from PyQt5.QtCore import Qt, QTimer, QPointF, QRectF
from PyQt5.QtGui import (
    QColor, QPainter, QPen, QBrush, QFont, QPainterPath,
    QLinearGradient, QRadialGradient
)
from PyQt5.QtWidgets import QApplication, QWidget


FINGER_NAMES = ["Thumb", "Index", "Middle", "Ring", "Pinky"]

# ----- Finger geometry (palm-view, right hand, wrist at bottom) -----
# Each entry: (base_xy_ratio, tip_xy_ratio, width_ratio, knuckle_frac_list)
# knuckle_frac_list gives normalized fractions along the finger axis for
# J1, J2, J3, J4 lamps (0 = base, 1 = tip). The fingertip F/T lamp sits
# at the very tip (drawn slightly inside so the lamp is on the pad).
#
# J1 is the abduction axis (same anatomical joint as J2). We draw it as a
# subtle marker just below the knuckle (on the palm side) so the user
# can distinguish it; J2 is the visible MCP knuckle.
FINGERS = [
    # name,     base(ratio x,y),  tip(ratio x,y), width, [J1..J4 fracs], tip_frac
    # Thumb: angled outward (base at thumb web, tip up-left).
    ("Thumb",  (0.27, 0.72),     (0.07, 0.38),   0.075, (-0.08, 0.05, 0.40, 0.75), 0.96),
    ("Index",  (0.33, 0.66),     (0.33, 0.18),   0.070, (-0.05, 0.05, 0.40, 0.75), 0.95),
    ("Middle", (0.48, 0.66),     (0.48, 0.10),   0.072, (-0.05, 0.05, 0.40, 0.75), 0.95),
    ("Ring",   (0.62, 0.66),     (0.62, 0.17),   0.070, (-0.05, 0.05, 0.40, 0.75), 0.95),
    ("Pinky",  (0.76, 0.69),     (0.76, 0.27),   0.060, (-0.05, 0.05, 0.40, 0.75), 0.95),
]

# Palm outline — cubic-bezier path in ratio coords (wrist bottom).
PALM_PATH_POINTS = [
    # (cmd, (x, y), optional control pts)
    ("M", (0.28, 0.94)),                                        # start: wrist L
    ("L", (0.72, 0.94)),                                        # wrist R
    ("C", (0.82, 0.94), (0.86, 0.82), (0.83, 0.72)),            # right side up
    ("C", (0.82, 0.68), (0.80, 0.66), (0.76, 0.66)),            # pinky base
    ("L", (0.72, 0.65)),
    ("L", (0.66, 0.65)),                                        # ring base
    ("L", (0.58, 0.65)),
    ("L", (0.52, 0.65)),                                        # middle base
    ("L", (0.44, 0.65)),
    ("L", (0.37, 0.65)),                                        # index base
    ("L", (0.30, 0.66)),                                        # toward thumb web
    ("C", (0.27, 0.68), (0.26, 0.70), (0.25, 0.73)),            # thumb web crease
    ("C", (0.24, 0.77), (0.22, 0.82), (0.24, 0.86)),            # left side
    ("C", (0.25, 0.90), (0.26, 0.93), (0.28, 0.94)),            # back to wrist
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
        self.declare_parameter("contact_level_topic", "/dg5f_right/contact_level")
        self.declare_parameter("fingertip_level_topic",
                               "/dg5f_right/fingertip_contact_level")
        joint_topic = self.get_parameter("contact_level_topic").value
        tip_topic = self.get_parameter("fingertip_level_topic").value

        self._levels = [0.0] * 20
        self._tip_levels = [0.0] * 5
        self._lock = threading.Lock()

        self._sub_joints = self.create_subscription(
            Float32MultiArray, joint_topic, self._on_levels, 10)
        self._sub_tips = self.create_subscription(
            Float32MultiArray, tip_topic, self._on_tip_levels, 10)
        self.get_logger().info(
            f"contact_viz: joints={joint_topic} tips={tip_topic}")

    def _on_levels(self, msg):
        data = list(msg.data)
        if len(data) < 20:
            data = data + [0.0] * (20 - len(data))
        with self._lock:
            self._levels = [float(max(0.0, min(1.0, v))) for v in data[:20]]

    def _on_tip_levels(self, msg):
        data = list(msg.data)
        if len(data) < 5:
            data = data + [0.0] * (5 - len(data))
        with self._lock:
            self._tip_levels = [float(max(0.0, min(1.0, v))) for v in data[:5]]

    def snapshot(self):
        with self._lock:
            return list(self._levels), list(self._tip_levels)


# ------------------------ HandWidget ------------------------

class HandWidget(QWidget):
    LAMP_DIAMETER = 26
    TIP_LAMP_DIAMETER = 22
    MIN_W = 520
    MIN_H = 640

    COLOR_BG = QColor("#1a1a1a")
    COLOR_HAND_FILL = QColor("#5d4a3c")
    COLOR_HAND_FILL_LIGHT = QColor("#6d5a4a")
    COLOR_HAND_EDGE = QColor("#8a715e")
    COLOR_NAIL = QColor("#d8c4b0")
    COLOR_LABEL = QColor("#cfcfcf")
    COLOR_LAMP_OUTLINE = QColor("#111")

    def __init__(self, bridge: ContactBridge):
        super().__init__()
        self.setWindowTitle("DG5F Contact (right hand)")
        self._bridge = bridge
        self._levels = [0.0] * 20
        self._tip_levels = [0.0] * 5
        self.setMinimumSize(self.MIN_W, self.MIN_H)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._refresh)
        self._timer.start(50)

    def _refresh(self):
        self._levels, self._tip_levels = self._bridge.snapshot()
        self.update()

    # --------------- geometry helpers ---------------

    def _pt(self, x, y, w, h):
        return QPointF(x * w, y * h)

    def _finger_axis_point(self, finger_idx, frac, w, h):
        """Point along finger axis at fraction 0..1 (base..tip)."""
        base, tip, *_ = FINGERS[finger_idx][1], FINGERS[finger_idx][2]
        x = lerp(base[0], tip[0], frac)
        y = lerp(base[1], tip[1], frac)
        return QPointF(x * w, y * h)

    # --------------- paint ---------------

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        w, h = self.width(), self.height()

        p.fillRect(self.rect(), self.COLOR_BG)

        self._paint_hand(p, w, h)
        self._paint_finger_labels(p, w, h)
        self._paint_lamps(p, w, h)

    def _paint_hand(self, p, w, h):
        # Palm (with gradient shading for some depth)
        palm = self._build_palm_path(w, h)
        grad = QLinearGradient(QPointF(0.5 * w, 0.66 * h),
                               QPointF(0.5 * w, 0.94 * h))
        grad.setColorAt(0.0, self.COLOR_HAND_FILL_LIGHT)
        grad.setColorAt(1.0, self.COLOR_HAND_FILL)
        p.setPen(QPen(self.COLOR_HAND_EDGE, 2))
        p.setBrush(QBrush(grad))
        p.drawPath(palm)

        # Fingers
        for f in range(5):
            self._paint_finger(p, w, h, f)

        # Fingernails (subtle ovals at tip)
        p.setPen(QPen(QColor("#715a48"), 1))
        for f in range(5):
            self._paint_nail(p, w, h, f)

    def _build_palm_path(self, w, h):
        path = QPainterPath()
        for i, entry in enumerate(PALM_PATH_POINTS):
            cmd = entry[0]
            if cmd == "M":
                path.moveTo(self._pt(*entry[1], w, h))
            elif cmd == "L":
                path.lineTo(self._pt(*entry[1], w, h))
            elif cmd == "C":
                c1 = self._pt(*entry[1], w, h)
                c2 = self._pt(*entry[2], w, h)
                end = self._pt(*entry[3], w, h)
                path.cubicTo(c1, c2, end)
        path.closeSubpath()
        return path

    def _paint_finger(self, p, w, h, f):
        _, base, tip, width_ratio, _, tip_frac = FINGERS[f]
        bx, by = base[0] * w, base[1] * h
        tx, ty = tip[0] * w, tip[1] * h
        # Finger direction and perpendicular
        dx, dy = tx - bx, ty - by
        length = (dx * dx + dy * dy) ** 0.5
        if length < 1.0:
            return
        ux, uy = dx / length, dy / length
        # Perpendicular
        px, py = -uy, ux
        half_w_px = (width_ratio * w) / 2.0

        # Phalange boundaries for subtle knuckle shading (fracs along axis).
        # Boundaries at PIP and DIP (0.40, 0.75 approx from J3, J4 fractions).
        knuckle_fracs = [0.40, 0.75]

        # Build finger path (pill with rounded tip)
        path = QPainterPath()
        # Four corners (base-left, base-right, tip-left, tip-right)
        bl = QPointF(bx + px * half_w_px, by + py * half_w_px)
        br = QPointF(bx - px * half_w_px, by - py * half_w_px)
        tl = QPointF(tx + px * half_w_px, ty + py * half_w_px)
        tr = QPointF(tx - px * half_w_px, ty - py * half_w_px)

        # Tapered width near tip
        taper = 0.82
        taper_l = QPointF(tx + px * half_w_px * taper, ty + py * half_w_px * taper)
        taper_r = QPointF(tx - px * half_w_px * taper, ty - py * half_w_px * taper)

        # Cubic-bezier rounded tip cap
        tip_pad_len = half_w_px * 1.1
        tip_outer = QPointF(tx + ux * tip_pad_len, ty + uy * tip_pad_len)

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

        grad = QLinearGradient(bl, tr)
        grad.setColorAt(0.0, self.COLOR_HAND_FILL_LIGHT)
        grad.setColorAt(1.0, self.COLOR_HAND_FILL)
        p.setPen(QPen(self.COLOR_HAND_EDGE, 2))
        p.setBrush(QBrush(grad))
        p.drawPath(path)

        # Knuckle shading lines (thin darker lines across finger at PIP/DIP)
        p.setPen(QPen(QColor("#4a3a2e"), 1, Qt.SolidLine))
        for kf in knuckle_fracs:
            kx = bx + dx * kf
            ky = by + dy * kf
            k_w_px = half_w_px * 0.92
            a = QPointF(kx + px * k_w_px, ky + py * k_w_px)
            b = QPointF(kx - px * k_w_px, ky - py * k_w_px)
            p.drawLine(a, b)

    def _paint_nail(self, p, w, h, f):
        _, _, tip, width_ratio, _, tip_frac = FINGERS[f]
        tx, ty = tip[0] * w, tip[1] * h
        base = FINGERS[f][1]
        bx, by = base[0] * w, base[1] * h
        dx, dy = tx - bx, ty - by
        length = (dx * dx + dy * dy) ** 0.5
        ux, uy = dx / length, dy / length
        px, py = -uy, ux
        half_w_px = (width_ratio * w) / 2.0
        # Nail: small oval just below the tip pad.
        center = QPointF(tx - ux * half_w_px * 0.25,
                         ty - uy * half_w_px * 0.25)
        nail_w = half_w_px * 1.0
        nail_h = half_w_px * 0.55
        # Make an ellipse aligned with finger axis
        p.save()
        p.translate(center)
        # Rotate so the "width" axis of ellipse is perpendicular to finger axis
        import math
        angle_deg = math.degrees(math.atan2(dy, dx)) - 90.0
        p.rotate(angle_deg)
        p.setPen(QPen(QColor("#a58876"), 1))
        p.setBrush(QBrush(self.COLOR_NAIL))
        p.drawEllipse(QRectF(-nail_w, -nail_h, 2 * nail_w, 2 * nail_h))
        p.restore()

    def _paint_finger_labels(self, p, w, h):
        p.setPen(self.COLOR_LABEL)
        font = QFont()
        font.setPointSize(9)
        p.setFont(font)
        label_positions = [
            (0.22, 0.82),  # Thumb label below thumb base
            (0.33, 0.98),  # Index
            (0.48, 0.98),  # Middle
            (0.62, 0.98),  # Ring
            (0.76, 0.98),  # Pinky
        ]
        for (rx, ry), name in zip(label_positions, FINGER_NAMES):
            p.drawText(QRectF(rx * w - 40, ry * h - 9, 80, 18),
                       Qt.AlignCenter, name)

    def _paint_lamps(self, p, w, h):
        font = QFont()
        font.setPointSize(8)
        p.setFont(font)

        for f in range(5):
            _, _, _, _, jfracs, tip_frac = FINGERS[f]
            for j in range(4):
                center = self._finger_axis_point(f, jfracs[j], w, h)
                level = self._levels[f * 4 + j]
                self._paint_lamp(p, center, level,
                                 label=f"J{j + 1}",
                                 diameter=self.LAMP_DIAMETER,
                                 is_abduction=(j == 0))
            # Fingertip F/T lamp
            tip_center = self._finger_axis_point(f, tip_frac, w, h)
            p.setPen(QPen(self.COLOR_LAMP_OUTLINE, 2))
            p.setBrush(QBrush(level_to_color(self._tip_levels[f])))
            r = self.TIP_LAMP_DIAMETER / 2.0
            p.drawEllipse(tip_center, r, r)
            # "T" label for tip
            p.setPen(QColor("#eaeaea"))
            p.drawText(QRectF(tip_center.x() - 6, tip_center.y() - 8, 12, 16),
                       Qt.AlignCenter, "T")

    def _paint_lamp(self, p, center, level, label, diameter, is_abduction=False):
        r = diameter / 2.0
        if is_abduction:
            # smaller, outlined differently so user can tell it's the
            # abduction axis (anatomically same joint as J2).
            r *= 0.70
            p.setPen(QPen(QColor("#333"), 1, Qt.DashLine))
        else:
            p.setPen(QPen(self.COLOR_LAMP_OUTLINE, 2))
        p.setBrush(QBrush(level_to_color(level)))
        p.drawEllipse(center, r, r)
        # Tiny label to the right for non-abduction lamps
        if not is_abduction:
            p.setPen(QColor("#9a9a9a"))
            p.drawText(
                QRectF(center.x() + r + 3, center.y() - 8, 24, 16),
                Qt.AlignLeft | Qt.AlignVCenter, label)


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
