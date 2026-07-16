#!/usr/bin/env python3
"""
Sensor Data Viewer - Visualize sensor data from ROS2 bags

Supported data types:
    IMU     (sensor_msgs/msg/Imu)
    Pose    (geometry_msgs/msg/PoseStamped)
    Image   (sensor_msgs/msg/CompressedImage)

Usage:
    ./run_viewer.sh [bag_path]
    source /opt/ros/humble/setup.bash && python3 sensor_viewer.py [bag_path]

The config file (viewer_config.json) saves recent bags and last-used bag,
and is created automatically next to this script.  Edit it to set defaults.

Mouse interactions:
    Scroll wheel          - Zoom in/out centered on cursor
    Left-drag (zoom mode) - Rectangle zoom (use toolbar button)
    Left-drag (pan mode)  - Pan (use toolbar button)
    Right-click           - Context menu (home, etc.)

Keyboard shortcuts:
    Esc / q     - Quit
    Ctrl+O      - Open bag (directory dialog)
    Ctrl+P      - Open bag (type path)
    F5          - Reload current topic
    Left/Right  - Image: previous / next frame
    Space       - Image: play / pause
"""

import sys
import os
import io
import json
import numpy as np
from datetime import datetime
from PIL import Image

# --- ROS2 environment setup ------------------------------------------------
ROS_HUMBLE = '/opt/ros/humble'
_site_pkg = os.path.join(ROS_HUMBLE, 'local/lib/python3.10/dist-packages')
if _site_pkg not in sys.path:
    sys.path.insert(0, _site_pkg)
_ld_path = os.environ.get('LD_LIBRARY_PATH', '')
if os.path.join(ROS_HUMBLE, 'lib') not in _ld_path:
    os.environ['LD_LIBRARY_PATH'] = os.path.join(ROS_HUMBLE, 'lib') + (
        ':' + _ld_path if _ld_path else ''
    )
os.environ.setdefault('AMENT_PREFIX_PATH', ROS_HUMBLE)

try:
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from sensor_msgs.msg import Imu, CompressedImage
    from geometry_msgs.msg import PoseStamped
    from rclpy.serialization import deserialize_message
    HAS_ROSBAG2 = True
    ROSBAG_ERR = ''
except ImportError as e:
    HAS_ROSBAG2 = False
    ROSBAG_ERR = str(e)

from PyQt5.QtWidgets import (
    QMainWindow, QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QCheckBox, QPushButton, QFileDialog, QStatusBar,
    QLabel, QSplitter, QMessageBox, QComboBox, QFrame, QSlider,
    QInputDialog, QAction
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure

import matplotlib
matplotlib.rcParams['font.family'] = 'sans-serif'
matplotlib.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial', 'Helvetica']

# ===========================================================================
# Config
# ===========================================================================

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(SCRIPT_DIR, 'viewer_config.json')

def load_config():
    """Load viewer_config.json, return dict. Defaults applied for missing keys."""
    defaults = {'recent_bags': [], 'last_bag': ''}
    try:
        if os.path.exists(CONFIG_PATH):
            with open(CONFIG_PATH) as f:
                d = json.load(f)
            for k, v in defaults.items():
                d.setdefault(k, v)
            return d
    except Exception:
        pass
    return defaults

def save_config(cfg):
    try:
        with open(CONFIG_PATH, 'w') as f:
            json.dump(cfg, f, indent=2, ensure_ascii=False)
    except Exception:
        pass  # silently ignore permission errors

# ===========================================================================
# Data model
# ===========================================================================

DT_IMU   = 'IMU'
DT_POSE  = 'Pose'
DT_IMAGE = 'Image'

TYPE_MAP = {
    'sensor_msgs/msg/Imu':              DT_IMU,
    'geometry_msgs/msg/PoseStamped':     DT_POSE,
    'sensor_msgs/msg/CompressedImage':   DT_IMAGE,
}

CHANNEL_GROUPS = {
    DT_IMU: [
        ("Acceleration (m/s^2)", [("Accel X", "ax"), ("Accel Y", "ay"), ("Accel Z", "az")]),
        ("Ang Vel (rad/s)",     [("Gyro X",  "gx"), ("Gyro Y",  "gy"), ("Gyro Z",  "gz")]),
    ],
    DT_POSE: [
        ("Position (m)",       [("Pos X", "px"), ("Pos Y", "py"), ("Pos Z", "pz")]),
        ("Orientation (rad)",  [("Roll",  "roll"), ("Pitch", "pitch"), ("Yaw",  "yaw")]),
    ],
}

COLORS = ['#e74c3c', '#2ecc71', '#3498db']

# ===========================================================================
# Quaternion
# ===========================================================================

def quat_to_euler(qx, qy, qz, qw):
    qx, qy, qz, qw = np.asarray(qx), np.asarray(qy), np.asarray(qz), np.asarray(qw)
    roll  = np.arctan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy))
    sinp  = 2*(qw*qy - qz*qx)
    pitch = np.where(np.abs(sinp) >= 1, np.sign(sinp)*np.pi/2, np.arcsin(sinp))
    yaw   = np.arctan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
    return roll, pitch, yaw

# ===========================================================================
# Loaders
# ===========================================================================

def scan_bag(bag_path):
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag_path, storage_id='sqlite3'),
                ConverterOptions(input_serialization_format='cdr',
                                 output_serialization_format='cdr'))
    result = {}
    for t in reader.get_all_topics_and_types():
        dt = TYPE_MAP.get(t.type)
        if dt:
            result.setdefault(dt, []).append(t.name)
    return result


def _open_reader(bag_path):
    r = SequentialReader()
    r.open(StorageOptions(uri=bag_path, storage_id='sqlite3'),
           ConverterOptions(input_serialization_format='cdr',
                            output_serialization_format='cdr'))
    return r


def load_imu_data(bag_path, topic, progress_cb=None):
    reader = _open_reader(bag_path)
    ts, ax, ay, az, gx, gy, gz = [], [], [], [], [], [], []
    first_ns, count = None, 0
    while reader.has_next():
        tpc, data, t_ns = reader.read_next()
        if tpc != topic:
            continue
        if first_ns is None:
            first_ns = t_ns
        m = deserialize_message(data, Imu)
        ts.append((t_ns - first_ns) * 1e-9)
        ax.append(m.linear_acceleration.x); ay.append(m.linear_acceleration.y); az.append(m.linear_acceleration.z)
        gx.append(m.angular_velocity.x); gy.append(m.angular_velocity.y); gz.append(m.angular_velocity.z)
        count += 1
        if progress_cb and count % 5000 == 0:
            progress_cb(count)
    ts = np.array(ts, dtype=np.float32)
    dur = ts[-1] - ts[0] if len(ts) > 1 else 0
    return {'ts': ts, 'ax': np.array(ax, np.float32), 'ay': np.array(ay, np.float32),
            'az': np.array(az, np.float32), 'gx': np.array(gx, np.float32),
            'gy': np.array(gy, np.float32), 'gz': np.array(gz, np.float32),
            'n': count, 'duration': dur,
            'start_dt': datetime.fromtimestamp(first_ns/1e9) if first_ns else None}


def load_pose_data(bag_path, topic, progress_cb=None):
    reader = _open_reader(bag_path)
    ts, px, py, pz, qx, qy, qz, qw = [], [], [], [], [], [], [], []
    first_ns, count = None, 0
    while reader.has_next():
        tpc, data, t_ns = reader.read_next()
        if tpc != topic:
            continue
        if first_ns is None:
            first_ns = t_ns
        m = deserialize_message(data, PoseStamped)
        ts.append((t_ns - first_ns) * 1e-9)
        px.append(m.pose.position.x); py.append(m.pose.position.y); pz.append(m.pose.position.z)
        qx.append(m.pose.orientation.x); qy.append(m.pose.orientation.y)
        qz.append(m.pose.orientation.z); qw.append(m.pose.orientation.w)
        count += 1
        if progress_cb and count % 5000 == 0:
            progress_cb(count)
    roll, pitch, yaw = quat_to_euler(qx, qy, qz, qw)
    ts = np.array(ts, dtype=np.float32)
    dur = ts[-1] - ts[0] if len(ts) > 1 else 0
    return {'ts': ts, 'px': np.array(px, np.float32), 'py': np.array(py, np.float32),
            'pz': np.array(pz, np.float32), 'roll': roll.astype(np.float32),
            'pitch': pitch.astype(np.float32), 'yaw': yaw.astype(np.float32),
            'n': count, 'duration': dur,
            'start_dt': datetime.fromtimestamp(first_ns/1e9) if first_ns else None}


def load_image_data(bag_path, topic, progress_cb=None):
    reader = _open_reader(bag_path)
    ts, images = [], []
    first_ns, count = None, 0
    while reader.has_next():
        tpc, data, t_ns = reader.read_next()
        if tpc != topic:
            continue
        if first_ns is None:
            first_ns = t_ns
        m = deserialize_message(data, CompressedImage)
        ts.append((t_ns - first_ns) * 1e-9)
        images.append(bytes(m.data))
        count += 1
        if progress_cb and count % 100 == 0:
            progress_cb(count)
    ts = np.array(ts, dtype=np.float32)
    dur = ts[-1] - ts[0] if len(ts) > 1 else 0
    return {'ts': ts, 'images': images, 'n': count, 'duration': dur,
            'start_dt': datetime.fromtimestamp(first_ns/1e9) if first_ns else None}


DATA_LOADERS = {
    DT_IMU:   load_imu_data,
    DT_POSE:  load_pose_data,
    DT_IMAGE: load_image_data,
}

# ===========================================================================
# Main window
# ===========================================================================

class SensorViewer(QMainWindow):

    def __init__(self, bag_path=None, config=None):
        super().__init__()
        self._config = config or {}
        self._bag_path = None
        self._available = {}
        self._data = {}
        self._current_dt = None
        self._current_topic = None
        self._cbs = {}

        self._lod_lines = []         # (line, full_ts, full_data) for zoom-LOD
        self._lod_cid = None        # xlim-changed callback id
        self._ignore_lim = False    # guard against self-triggered xlim events

        self._img_frame_idx = 0
        self._img_playing = False
        self._img_timer = QTimer(self)
        self._img_timer.timeout.connect(self._img_next_frame)

        self.setWindowTitle("Sensor Data Viewer")
        self.setMinimumSize(1100, 650)
        self.resize(1400, 820)

        self._setup_ui()
        self._setup_menu()

        if bag_path:
            QTimer.singleShot(200, lambda: self._open_bag(bag_path))

    # -- UI ------------------------------------------------------------------

    def _setup_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(6, 6, 6, 6)

        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(self._build_control_panel())
        splitter.addWidget(self._build_chart_area())
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([240, 1100])
        root.addWidget(splitter)

        self._status = QStatusBar()
        self._status_label = QLabel("Ready - Open a bag (Ctrl+O) or type a path (Ctrl+P)")
        self._status.addWidget(self._status_label)
        self.setStatusBar(self._status)

    def _build_control_panel(self):
        panel = QWidget()
        self._panel_layout = QVBoxLayout(panel)
        self._panel_layout.setContentsMargins(4, 4, 4, 4)
        self._panel_layout.setSpacing(8)

        lbl = QLabel("Data Type"); lbl.setFont(QFont("", 11, QFont.Bold))
        self._panel_layout.addWidget(lbl)

        self._type_combo = QComboBox()
        self._type_combo.setMinimumHeight(28)
        self._type_combo.currentTextChanged.connect(self._on_type_changed)
        self._panel_layout.addWidget(self._type_combo)

        lbl2 = QLabel("Topic"); lbl2.setFont(QFont("", 11, QFont.Bold))
        self._panel_layout.addWidget(lbl2)

        self._topic_combo = QComboBox()
        self._topic_combo.setMinimumHeight(28)
        self._topic_combo.currentTextChanged.connect(self._on_topic_changed)
        self._panel_layout.addWidget(self._topic_combo)

        line = QFrame(); line.setFrameShape(QFrame.HLine); line.setFrameShadow(QFrame.Sunken)
        self._panel_layout.addWidget(line)

        self._section_label = QLabel("Channels")
        self._section_label.setFont(QFont("", 11, QFont.Bold))
        self._panel_layout.addWidget(self._section_label)

        self._ch_container = QWidget()
        self._ch_container_layout = QVBoxLayout(self._ch_container)
        self._ch_container_layout.setContentsMargins(0, 0, 0, 0)
        self._ch_container_layout.setSpacing(6)
        self._panel_layout.addWidget(self._ch_container)

        self._btn_row = QWidget()
        brl = QHBoxLayout(self._btn_row); brl.setContentsMargins(0, 0, 0, 0)
        b_all = QPushButton("All");   b_all.clicked.connect(lambda: self._set_all(True))
        b_none = QPushButton("None"); b_none.clicked.connect(lambda: self._set_all(False))
        brl.addWidget(b_all); brl.addWidget(b_none)
        self._panel_layout.addWidget(self._btn_row)

        # Image nav
        self._img_nav = QWidget()
        nav_l = QVBoxLayout(self._img_nav)
        nav_l.setContentsMargins(0, 0, 0, 0); nav_l.setSpacing(4)

        self._img_frame_label = QLabel("Frame: 0 / 0")
        self._img_frame_label.setAlignment(Qt.AlignCenter)
        nav_l.addWidget(self._img_frame_label)

        self._img_slider = QSlider(Qt.Horizontal)
        self._img_slider.setMinimum(0)
        self._img_slider.setTracking(False)
        self._img_slider.valueChanged.connect(self._on_img_slider)
        nav_l.addWidget(self._img_slider)

        img_btn_row = QHBoxLayout(); img_btn_row.setSpacing(4)
        for text, slot in [("|<", self._img_first), ("<", self._img_prev),
                           ("Play", self._img_toggle_play), (">", self._img_next),
                           (">|", self._img_last)]:
            b = QPushButton(text); b.setFixedWidth(40)
            b.clicked.connect(slot); img_btn_row.addWidget(b)
        nav_l.addLayout(img_btn_row)

        self._img_ts_label = QLabel("t = --")
        self._img_ts_label.setAlignment(Qt.AlignCenter)
        self._img_ts_label.setStyleSheet("color: #555;")
        nav_l.addWidget(self._img_ts_label)

        self._panel_layout.addWidget(self._img_nav)
        self._img_nav.hide()

        line2 = QFrame(); line2.setFrameShape(QFrame.HLine); line2.setFrameShadow(QFrame.Sunken)
        self._panel_layout.addWidget(line2)

        self._info_label = QLabel("No data loaded")
        self._info_label.setWordWrap(True)
        self._info_label.setStyleSheet("color: #888;")
        self._panel_layout.addWidget(self._info_label)

        self._panel_layout.addStretch()
        return panel

    def _build_chart_area(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(0, 0, 0, 0)

        self._fig = Figure(figsize=(10, 8), dpi=100)
        self._canvas = FigureCanvas(self._fig)
        self._axes = []
        self._toolbar = NavigationToolbar(self._canvas, self)

        self._canvas.mpl_connect('scroll_event', self._on_scroll)

        layout.addWidget(self._toolbar)
        layout.addWidget(self._canvas)
        return widget

    def _setup_menu(self):
        m = self.menuBar()
        f = m.addMenu("&File")
        f.addAction("&Open Bag...\tCtrl+O", self._on_open_dir)
        f.addAction("Open &Path...\tCtrl+P", self._on_open_path)

        self._recent_menu = f.addMenu("Recent Bags")
        self._rebuild_recent_menu()

        f.addSeparator()
        f.addAction("&Reload Topic\tF5", self._on_reload)
        f.addSeparator()
        f.addAction("&Quit\tCtrl+Q", self.close)

    def _rebuild_recent_menu(self):
        self._recent_menu.clear()
        recent = self._config.get('recent_bags', [])
        if recent:
            for path in recent:
                label = os.path.basename(path)
                act = self._recent_menu.addAction(f"{label}  ({path})")
                act.triggered.connect(lambda checked, p=path: self._open_bag(p))
        else:
            self._recent_menu.addAction("(empty)").setEnabled(False)

    # -- Keyboard -------------------------------------------------------------

    def keyPressEvent(self, event):
        key = event.key()
        if key in (Qt.Key_Escape, Qt.Key_Q):
            self.close()
        elif key == Qt.Key_F5:
            self._on_reload()
        elif self._current_dt == DT_IMAGE:
            if key == Qt.Key_Left:   self._img_prev()
            elif key == Qt.Key_Right: self._img_next()
            elif key == Qt.Key_Space: self._img_toggle_play()
            else: super().keyPressEvent(event)
        else:
            super().keyPressEvent(event)

    def closeEvent(self, event):
        self._img_timer.stop()
        QApplication.quit()
        super().closeEvent(event)

    # -- Bag opening ----------------------------------------------------------

    def _open_bag(self, bag_path):
        if not os.path.isdir(bag_path):
            QMessageBox.warning(self, "Error", f"Path not found:\n{bag_path}")
            return
        if not HAS_ROSBAG2:
            QMessageBox.critical(self, "Error",
                "Cannot import rosbag2_py.\nSource ROS2 environment first.\n"
                f"Detail: {ROSBAG_ERR}")
            return

        self._status_label.setText("Scanning bag topics ...")
        QApplication.processEvents()

        try:
            available = scan_bag(bag_path)
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to scan bag:\n{e}")
            self._status_label.setText("Scan failed")
            return

        if not available:
            QMessageBox.warning(self, "Warning",
                "No supported data types found in this bag.\n"
                "Supported: Imu, PoseStamped, CompressedImage")
            self._status_label.setText("No supported topics")
            return

        self._bag_path = bag_path
        self._available = available
        self._data = {}; self._cbs = {}
        self._current_dt = None; self._current_topic = None
        self._img_timer.stop(); self._img_playing = False

        self.setWindowTitle(f"Sensor Data Viewer - {os.path.basename(bag_path)}")

        # Update config
        self._save_to_recent(bag_path)

        data_types = sorted(available.keys())
        self._type_combo.blockSignals(True)
        self._type_combo.clear()
        self._type_combo.addItems(data_types)
        default_type = DT_IMU if DT_IMU in data_types else data_types[0]
        self._type_combo.setCurrentText(default_type)
        self._type_combo.blockSignals(False)

        self._on_type_changed(default_type)

    def _save_to_recent(self, bag_path):
        bag_path = os.path.abspath(bag_path)
        self._config['last_bag'] = bag_path
        recent = self._config.setdefault('recent_bags', [])
        if bag_path in recent:
            recent.remove(bag_path)
        recent.insert(0, bag_path)
        self._config['recent_bags'] = recent[:10]
        save_config(self._config)
        self._rebuild_recent_menu()

    def _on_type_changed(self, data_type):
        if not data_type or data_type not in self._available:
            return

        self._img_timer.stop(); self._img_playing = False
        self._current_dt = data_type

        topics = self._available[data_type]
        self._topic_combo.blockSignals(True)
        self._topic_combo.clear()
        self._topic_combo.addItems(topics)
        default = next((t for t in topics if '/odin1/' in t), topics[0] if topics else None)
        if default:
            self._topic_combo.setCurrentText(default)
        self._topic_combo.blockSignals(False)

        if data_type == DT_IMAGE:
            self._section_label.setText("Frame")
            self._ch_container.hide(); self._btn_row.hide()
            self._img_nav.show()
        else:
            self._section_label.setText("Channels")
            self._ch_container.show(); self._btn_row.show()
            self._img_nav.hide()
            self._rebuild_channels(data_type)

        if default:
            self._load_topic(default)

    def _rebuild_channels(self, data_type):
        while self._ch_container_layout.count():
            item = self._ch_container_layout.takeAt(0)
            if item.widget():
                item.widget().setParent(None)
        self._cbs = {}
        for group_title, fields in CHANNEL_GROUPS.get(data_type, []):
            grp = QGroupBox(group_title)
            gl = QVBoxLayout(grp)
            for cb_label, key in fields:
                cb = QCheckBox(cb_label)
                cb.setChecked(True)
                cb.stateChanged.connect(self._on_cb)
                gl.addWidget(cb)
                self._cbs[key] = cb
            self._ch_container_layout.addWidget(grp)

    # -- Data loading ---------------------------------------------------------

    def _on_topic_changed(self, topic):
        if topic and self._bag_path and self._current_dt:
            self._load_topic(topic)

    def _load_topic(self, topic):
        if not self._bag_path or not self._current_dt:
            return
        self._current_topic = topic
        if topic in self._data:
            self._display_data(topic)
            return

        loader = DATA_LOADERS.get(self._current_dt)
        if not loader:
            return
        self._status_label.setText(f"Loading {topic} ...")
        QApplication.processEvents()

        try:
            def cb(n):
                self._status_label.setText(f"Loading {topic} ... {n} msgs")
                QApplication.processEvents()
            data = loader(self._bag_path, topic, progress_cb=cb)
            self._data[topic] = data
            self._display_data(topic)
        except Exception as e:
            QMessageBox.critical(self, "Load Error", str(e))
            self._status_label.setText(f"Failed: {topic}")

    def _display_data(self, topic):
        data = self._data[topic]
        n = data['n']; dur = data['duration']
        freq = n / dur if dur > 0 else 0
        ts_str = data['start_dt'].strftime('%Y-%m-%d %H:%M:%S') if data.get('start_dt') else '?'
        self._status_label.setText(
            f"{topic}  |  {n} msgs  |  {dur:.1f}s  |  ~{freq:.0f} Hz")

        if self._current_dt == DT_IMAGE:
            self._img_frame_idx = 0
            self._img_slider.blockSignals(True)
            self._img_slider.setMaximum(n - 1)
            self._img_slider.setValue(0)
            self._img_slider.blockSignals(False)
            total_mb = sum(len(img) for img in data['images']) / 1e6
            self._info_label.setText(
                f"Start: {ts_str}\nFrames: {n}\nDuration: {dur:.1f}s\n"
                f"Rate: ~{freq:.0f} Hz\nData: ~{total_mb:.0f} MB")
        else:
            lines = [f"Start: {ts_str}", f"Msgs: {n}",
                     f"Duration: {dur:.1f}s", f"Rate: ~{freq:.0f} Hz", ""]
            for grp_title, fields in CHANNEL_GROUPS.get(self._current_dt, []):
                lines.append(f"{grp_title}:")
                for _, key in fields:
                    arr = data.get(key)
                    if arr is not None and len(arr) > 0:
                        lines.append(f"  {key}: [{arr.min():.3f}, {arr.max():.3f}]")
                    else:
                        lines.append(f"  {key}: N/A")
                lines.append("")
            self._info_label.setText("\n".join(lines))
        self._redraw()

    # -- Plotting -------------------------------------------------------------

    def _redraw(self):
        if self._current_dt == DT_IMAGE:
            return self._redraw_image()
        if not self._current_topic or self._current_topic not in self._data:
            return

        data = self._data[self._current_topic]
        dt = self._current_dt
        ts = data['ts']
        groups = CHANNEL_GROUPS.get(dt, [])

        # Save current view limits so zoom/pan survives checkbox toggles
        saved_lim = None
        if self._axes:
            saved_lim = [(ax.get_xlim(), ax.get_ylim()) for ax in self._axes]

        MAX_PTS = 6000
        stride = max(1, len(ts) // MAX_PTS)

        self._fig.clear()
        n_grp = len(groups)
        if n_grp == 0:
            self._canvas.draw_idle()
            return

        self._axes = []
        for i in range(n_grp):
            ax = self._fig.add_subplot(n_grp, 1, i + 1,
                                       sharex=self._axes[0] if self._axes else None)
            self._axes.append(ax)

        self._lod_lines = []

        for gi, (grp_title, fields) in enumerate(groups):
            ax = self._axes[gi]
            ax.grid(True, alpha=0.3)
            ax.set_ylabel(grp_title)
            visible, labels = [], []
            for fi, (_, key) in enumerate(fields):
                if key in self._cbs and self._cbs[key].isChecked():
                    arr = data.get(key)
                    if arr is not None:
                        line, = ax.plot(ts[::stride], arr[::stride],
                                        color=COLORS[fi % 3], linewidth=0.5,
                                        alpha=0.85, label=key)
                        visible.append(line); labels.append(key)
                        self._lod_lines.append((line, ts, arr))
            if visible:
                ax.legend(visible, labels, loc='upper right',
                          fontsize=8, ncol=3, framealpha=0.6)

        self._axes[-1].set_xlabel("Time (s)")
        self._axes[0].set_title(f"{self._current_topic}")

        # Connect xlim callback for LOD (reconnect on every redraw since axes change)
        if self._lod_cid is not None:
            try:
                self._axes[0].callbacks.disconnect(self._lod_cid)
            except Exception:
                pass
        if self._axes:
            self._lod_cid = self._axes[0].callbacks.connect('xlim_changed', self._on_lod_update)

        if saved_lim:
            self._ignore_lim = True
            for ax, (xl, yl) in zip(self._axes, saved_lim):
                ax.set_xlim(xl); ax.set_ylim(yl)
            self._ignore_lim = False
        else:
            self._fig.tight_layout()

        self._canvas.draw_idle()

    # -- Scroll zoom ----------------------------------------------------------

    def _on_scroll(self, event):
        """Mouse wheel zoom centered on cursor position."""
        if event.inaxes is None or self._current_dt == DT_IMAGE:
            return
        scale = 1.0 / 1.25 if event.button == 'up' else 1.25

        # Zoom x-axis (shared, so one axis covers all)
        ax = event.inaxes
        xd, yd = event.xdata, event.ydata
        if xd is None or yd is None:
            return

        x0, x1 = ax.get_xlim()
        ax.set_xlim(xd - (xd - x0) * scale, xd + (x1 - xd) * scale)

        # Zoom y-axis only for the axis under cursor
        y0, y1 = ax.get_ylim()
        ax.set_ylim(yd - (yd - y0) * scale, yd + (y1 - yd) * scale)

        self._canvas.draw_idle()

    # -- Level-of-detail on zoom ----------------------------------------------

    def _on_lod_update(self, ax):
        """When x-axis limits change, update line data resolution to match
        the visible range.  Zoomed out => downsampled; zoomed in => full res."""
        if self._ignore_lim or self._current_dt == DT_IMAGE:
            return
        if not self._lod_lines:
            return

        xmin, xmax = ax.get_xlim()
        for line, full_ts, full_data in self._lod_lines:
            mask = (full_ts >= xmin) & (full_ts <= xmax)
            n_vis = np.count_nonzero(mask)
            if n_vis <= 0:
                line.set_data([], [])
                continue
            if n_vis > 8000:
                s = max(1, n_vis // 8000)
                idx = np.where(mask)[0][::s]
            else:
                idx = np.where(mask)[0]
            line.set_data(full_ts[idx], full_data[idx])
        self._canvas.draw_idle()

    def _redraw_image(self):
        if not self._current_topic or self._current_topic not in self._data:
            return
        data = self._data[self._current_topic]
        n = data['n']
        idx = min(self._img_frame_idx, n - 1)

        try:
            pil_img = Image.open(io.BytesIO(data['images'][idx]))
            img_arr = np.array(pil_img)
        except Exception as e:
            self._status_label.setText(f"Image decode error: {e}")
            return

        self._fig.clear()
        ax = self._fig.add_subplot(1, 1, 1)
        ax.imshow(img_arr)
        ax.set_title(f"{self._current_topic}  |  Frame {idx+1}/{n}  |  t = {data['ts'][idx]:.3f}s")
        ax.axis('off')
        self._fig.tight_layout(pad=0.5)
        self._canvas.draw_idle()

        self._img_slider.blockSignals(True)
        self._img_slider.setValue(idx)
        self._img_slider.blockSignals(False)
        self._img_frame_label.setText(f"Frame: {idx+1} / {n}")
        self._img_ts_label.setText(
            f"t = {data['ts'][idx]:.3f}s"
            f"  ({data['start_dt'].strftime('%H:%M:%S.%f')[:-3] if data.get('start_dt') else '?'})")

    # -- Image navigation -----------------------------------------------------

    def _on_img_slider(self, value):
        self._img_frame_idx = value
        self._redraw_image()

    def _img_prev(self):
        if self._current_dt != DT_IMAGE: return
        self._img_frame_idx = max(0, self._img_frame_idx - 1)
        self._redraw_image()

    def _img_next(self):
        if self._current_dt != DT_IMAGE: return
        n = self._data[self._current_topic]['n']
        self._img_frame_idx = min(n - 1, self._img_frame_idx + 1)
        self._redraw_image()

    def _img_first(self):
        if self._current_dt != DT_IMAGE: return
        self._img_frame_idx = 0
        self._redraw_image()

    def _img_last(self):
        if self._current_dt != DT_IMAGE: return
        self._img_frame_idx = self._data[self._current_topic]['n'] - 1
        self._redraw_image()

    def _img_next_frame(self):
        if self._current_dt != DT_IMAGE: self._img_timer.stop(); return
        n = self._data[self._current_topic]['n']
        if self._img_frame_idx < n - 1:
            self._img_frame_idx += 1
            self._redraw_image()
        else:
            self._img_timer.stop(); self._img_playing = False

    def _img_toggle_play(self):
        if self._current_dt != DT_IMAGE: return
        self._img_playing = not self._img_playing
        self._img_timer.start(100) if self._img_playing else self._img_timer.stop()

    # -- Slots ---------------------------------------------------------------

    def _on_cb(self):
        self._redraw()

    def _set_all(self, state):
        for cb in self._cbs.values():
            cb.setChecked(state)

    def _on_open_dir(self):
        path = QFileDialog.getExistingDirectory(
            self, "Select ROS2 Bag Directory", SCRIPT_DIR, QFileDialog.ShowDirsOnly)
        if path:
            self._open_bag(path)

    def _on_open_path(self):
        text, ok = QInputDialog.getText(
            self, "Open Bag Path", "Enter the path to the bag directory:",
            text=self._bag_path or "")
        if ok and text.strip():
            self._open_bag(text.strip())

    def _on_reload(self):
        if self._current_topic:
            self._data.pop(self._current_topic, None)
            self._load_topic(self._current_topic)


# ===========================================================================
# Entry point
# ===========================================================================

def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    app.setQuitOnLastWindowClosed(True)

    config = load_config()

    # Priority: CLI arg > config last_bag > default dir
    if len(sys.argv) > 1:
        bag = sys.argv[1]
        if not os.path.isdir(bag):
            print(f"[WARN] Directory not found: {bag}", file=sys.stderr)
            bag = None
    elif config.get('last_bag') and os.path.isdir(config['last_bag']):
        bag = config['last_bag']
    else:
        default = os.path.join(SCRIPT_DIR, 'odin_bag', 'D10R', '20260714160813')
        bag = default if os.path.isdir(default) else None

    viewer = SensorViewer(bag_path=bag, config=config)
    viewer.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
