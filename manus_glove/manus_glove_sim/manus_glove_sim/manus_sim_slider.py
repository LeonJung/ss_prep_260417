"""Qt slider window that publishes a fake manus_ros2_msgs/ManusGlove.

5 finger columns x 4 ergonomics rows = 20 sliders. Range [-30 deg, 90 deg].
Presets: Open / Fist / Pinch.
"""

import signal
import sys
import threading
from typing import Dict, List

import rclpy
from rclpy.node import Node

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication, QGridLayout, QHBoxLayout, QLabel, QPushButton, QSlider,
    QVBoxLayout, QWidget
)

from .msg_builder import ERGO_KEYS, build_manus_glove_msg
from manus_ros2_msgs.msg import ManusGlove


FINGERS = [
    ("Thumb",  "Thumb"),
    ("Index",  "Index"),
    ("Middle", "Middle"),
    ("Ring",   "Ring"),
    ("Pinky",  "Pinky"),
]
ROWS = [
    ("MCPSpread",  "MCPSpread"),     # Thumb uses "ThumbMCPSpread", others use "<F>Spread"
    ("MCPStretch", "MCPStretch"),
    ("PIPStretch", "PIPStretch"),
    ("DIPStretch", "DIPStretch"),
]
SLIDER_MIN_DEG = -30
SLIDER_MAX_DEG = 90

PRESET_OPEN = {k: 0.0 for k in ERGO_KEYS}
PRESET_FIST = {
    **{k: 0.0 for k in ERGO_KEYS},
    "ThumbMCPStretch": 40.0, "ThumbPIPStretch": 40.0, "ThumbDIPStretch": 40.0,
    "IndexMCPStretch": 70.0, "IndexPIPStretch": 80.0, "IndexDIPStretch": 60.0,
    "MiddleMCPStretch": 70.0, "MiddlePIPStretch": 80.0, "MiddleDIPStretch": 60.0,
    "RingMCPStretch": 70.0,  "RingPIPStretch": 80.0,  "RingDIPStretch": 60.0,
    "PinkyMCPStretch": 70.0, "PinkyPIPStretch": 80.0, "PinkyDIPStretch": 60.0,
}
PRESET_PINCH = {
    **{k: 0.0 for k in ERGO_KEYS},
    "ThumbMCPSpread": 30.0, "ThumbMCPStretch": 35.0,
    "ThumbPIPStretch": 35.0, "ThumbDIPStretch": 25.0,
    "IndexMCPStretch": 55.0, "IndexPIPStretch": 55.0, "IndexDIPStretch": 25.0,
}


def ergo_key(finger_label: str, row_label: str) -> str:
    # Thumb uses "ThumbMCPSpread"; others use "<Finger>Spread".
    if row_label == "MCPSpread" and finger_label != "Thumb":
        return f"{finger_label}Spread"
    return f"{finger_label}{row_label}"


class GlovePublisher(Node):
    def __init__(self):
        super().__init__("manus_sim_slider")
        self.declare_parameter("topic", "/manus_glove_1")
        self.declare_parameter("side", "Right")
        self.declare_parameter("glove_id", 1)
        topic = self.get_parameter("topic").value
        self._side = self.get_parameter("side").value
        self._glove_id = int(self.get_parameter("glove_id").value)
        self._values: Dict[str, float] = {k: 0.0 for k in ERGO_KEYS}
        self._lock = threading.Lock()
        self._pub = self.create_publisher(ManusGlove, topic, 10)
        self._timer = self.create_timer(1.0 / 30.0, self._tick)
        self.get_logger().info(
            f"manus_sim_slider: publishing to {topic} "
            f"(side={self._side}, id={self._glove_id}) @ 30 Hz"
        )

    def update(self, values: Dict[str, float]):
        with self._lock:
            self._values.update(values)

    def snapshot(self) -> Dict[str, float]:
        with self._lock:
            return dict(self._values)

    def _tick(self):
        msg = build_manus_glove_msg(self.snapshot(), side=self._side,
                                    glove_id=self._glove_id)
        self._pub.publish(msg)


class SliderGrid(QWidget):
    def __init__(self, publisher: GlovePublisher):
        super().__init__()
        self.setWindowTitle("Manus Glove Sim — right hand")
        self._pub = publisher
        self._sliders: Dict[str, QSlider] = {}
        self._value_labels: Dict[str, QLabel] = {}

        root = QVBoxLayout(self)

        grid = QGridLayout()
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(6)
        grid.addWidget(QLabel(""), 0, 0)
        for col, (flab, _) in enumerate(FINGERS):
            lab = QLabel(f"<b>{flab}</b>")
            lab.setAlignment(Qt.AlignCenter)
            grid.addWidget(lab, 0, col + 1)

        for row, (rlab, _) in enumerate(ROWS):
            grid.addWidget(QLabel(rlab), row * 2 + 1, 0,
                           alignment=Qt.AlignRight | Qt.AlignVCenter)
            for col, (flab, _) in enumerate(FINGERS):
                key = ergo_key(flab, rlab)
                s = QSlider(Qt.Horizontal)
                s.setMinimum(SLIDER_MIN_DEG)
                s.setMaximum(SLIDER_MAX_DEG)
                s.setValue(0)
                s.setMinimumWidth(120)
                vlabel = QLabel("0°")
                vlabel.setMinimumWidth(40)
                vlabel.setAlignment(Qt.AlignCenter)
                s.valueChanged.connect(
                    lambda v, k=key, lab=vlabel: self._on_slider_change(k, v, lab)
                )
                self._sliders[key] = s
                self._value_labels[key] = vlabel
                grid.addWidget(s,      row * 2 + 1, col + 1)
                grid.addWidget(vlabel, row * 2 + 2, col + 1,
                               alignment=Qt.AlignCenter)
        root.addLayout(grid)

        presets = QHBoxLayout()
        for name, preset in (("Open", PRESET_OPEN),
                             ("Fist", PRESET_FIST),
                             ("Pinch", PRESET_PINCH)):
            btn = QPushButton(name)
            btn.clicked.connect(lambda _=False, p=preset: self._apply_preset(p))
            presets.addWidget(btn)
        root.addLayout(presets)

    def _on_slider_change(self, key: str, value: int, label: QLabel):
        label.setText(f"{value}°")
        self._pub.update({key: float(value)})

    def _apply_preset(self, preset: Dict[str, float]):
        for key, val in preset.items():
            s = self._sliders.get(key)
            if s is None:
                continue
            iv = int(round(max(SLIDER_MIN_DEG, min(SLIDER_MAX_DEG, val))))
            s.blockSignals(True)
            s.setValue(iv)
            s.blockSignals(False)
            self._value_labels[key].setText(f"{iv}°")
        self._pub.update(preset)


def _spin_ros(node: Node):
    try:
        rclpy.spin(node)
    except Exception:
        pass


def main(args=None):
    rclpy.init(args=args)
    bridge = GlovePublisher()

    app = QApplication(sys.argv)
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    win = SliderGrid(bridge)
    win.setWindowTitle(f"Manus Glove Sim — {bridge._side} (id {bridge._glove_id})")
    win.resize(820, 420)
    win.show()

    ros_thread = threading.Thread(target=_spin_ros, args=(bridge,), daemon=True)
    ros_thread.start()

    rc = app.exec_()

    bridge.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(rc)


if __name__ == "__main__":
    main()
