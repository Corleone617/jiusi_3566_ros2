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
import glob
import sqlite3
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

# GPS (mavros_msgs/GPSRAW) is optional: if mavros_msgs is unavailable, the
# viewer still works for IMU/Pose/Image and simply hides the GPS option.
try:
    from mavros_msgs.msg import GPSRAW
    HAS_GPSRAW = True
    GPSRAW_ERR = ''
except ImportError as e:
    HAS_GPSRAW = False
    GPSRAW_ERR = str(e)

from PyQt5.QtWidgets import (
    QMainWindow, QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QCheckBox, QPushButton, QFileDialog, QStatusBar,
    QLabel, QSplitter, QMessageBox, QComboBox, QFrame, QSlider,
    QInputDialog, QAction, QSizePolicy
)
from PyQt5.QtCore import Qt, QTimer, QSize
from PyQt5.QtGui import QFont, QIcon

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure

import matplotlib
matplotlib.rcParams['font.family'] = 'sans-serif'
matplotlib.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial', 'Helvetica']
# Larger, bolder fonts for readability
matplotlib.rcParams.update({
    'font.size': 12,
    'axes.labelsize': 13,
    'axes.titlesize': 14,
    'axes.titleweight': 'bold',
    'axes.labelweight': 'bold',
    'xtick.labelsize': 11,
    'ytick.labelsize': 11,
    'legend.fontsize': 11,
    'figure.titlesize': 14,
})

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
DT_GPS   = 'GPS'

TYPE_MAP = {
    'sensor_msgs/msg/Imu':              DT_IMU,
    'geometry_msgs/msg/PoseStamped':     DT_POSE,
    'sensor_msgs/msg/CompressedImage':   DT_IMAGE,
}
if HAS_GPSRAW:
    TYPE_MAP['mavros_msgs/msg/GPSRAW'] = DT_GPS

CHANNEL_GROUPS = {
    DT_IMU: [
        ("Acceleration (m/s^2)", [("Accel X", "ax"), ("Accel Y", "ay"), ("Accel Z", "az")]),
        ("Ang Vel (rad/s)",     [("Gyro X",  "gx"), ("Gyro Y",  "gy"), ("Gyro Z",  "gz")]),
    ],
    DT_POSE: [
        ("Position (m)",       [("Pos X", "px"), ("Pos Y", "py"), ("Pos Z", "pz")]),
        ("Orientation (rad)",  [("Roll",  "roll"), ("Pitch", "pitch"), ("Yaw",  "yaw")]),
    ],
    DT_GPS: [
        ("Position",            [("Lat (deg)", "lat"), ("Lon (deg)", "lon"), ("Alt (m)", "alt")]),
        ("Heading (deg)",       [("Yaw", "yaw"), ("CoG", "cog")]),
        ("Signal",              [("Satellites", "sat"), ("Fix Type", "fix")]),
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

def _find_db3(bag_path):
    files = sorted(glob.glob(os.path.join(bag_path, '*.db3')))
    if not files:
        files = sorted(glob.glob(os.path.join(bag_path, '*.mcap')))
    return files[0] if files else None


def _try_rosbag2_reader(bag_path):
    r = SequentialReader()
    r.open(StorageOptions(uri=bag_path, storage_id='sqlite3'),
           ConverterOptions(input_serialization_format='cdr',
                            output_serialization_format='cdr'))
    return r


class _SqliteBagReader:
    """Fallback reader: streams messages directly from the .db3 SQLite file.

    Used when metadata.yaml is missing/empty (e.g. recording was interrupted or
    is still in progress) so rosbag2_py cannot open the bag. The `data` BLOB is
    the raw CDR payload, identical to what rosbag2_py returns, so messages can be
    decoded with rclpy.serialization.deserialize_message unchanged.

    Exposes the same minimal API used by the loaders: has_next()/read_next(),
    where read_next() returns (topic_name, data_bytes, timestamp_ns).
    """
    def __init__(self, bag_path, topic_filter=None):
        db = _find_db3(bag_path)
        if not db:
            raise RuntimeError(f'No .db3/.mcap file found in {bag_path}')
        self._conn = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
        self._cur = self._conn.cursor()
        # topic id -> name
        try:
            self._cur.execute('SELECT id, name FROM topics')
        except sqlite3.DatabaseError as e:
            raise RuntimeError(f'Cannot read topics table: {e}')
        self._id2name = {row[0]: row[1] for row in self._cur.fetchall()}
        if topic_filter:
            ids = [tid for tid, nm in self._id2name.items() if nm == topic_filter]
            if ids:
                ph = ','.join('?' * len(ids))
                self._cur.execute(
                    f'SELECT topic_id, data, timestamp FROM messages '
                    f'WHERE topic_id IN ({ph}) ORDER BY timestamp', ids)
            else:
                self._cur.execute(
                    'SELECT topic_id, data, timestamp FROM messages WHERE 0')
        else:
            self._cur.execute(
                'SELECT topic_id, data, timestamp FROM messages ORDER BY timestamp')
        self._next = self._fetch()

    def _fetch(self):
        row = self._cur.fetchone()
        if row is None:
            return None
        tid, data, t_ns = row
        return (self._id2name.get(tid, ''), data, t_ns)

    def has_next(self):
        return self._next is not None

    def read_next(self):
        cur = self._next
        self._next = self._fetch()
        return cur


def scan_bag(bag_path):
    # Preferred path: rosbag2_py (needs a valid metadata.yaml)
    try:
        reader = _try_rosbag2_reader(bag_path)
        result = {}
        for t in reader.get_all_topics_and_types():
            dt = TYPE_MAP.get(t.type)
            if dt:
                result.setdefault(dt, []).append(t.name)
        return result
    except Exception:
        pass
    # Fallback: read topics straight from the .db3 SQLite file
    db = _find_db3(bag_path)
    if not db:
        raise RuntimeError(f'No .db3/.mcap storage file found in {bag_path}')
    result = {}
    conn = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
    try:
        for _id, name, mtype in conn.execute('SELECT id, name, type FROM topics'):
            dt = TYPE_MAP.get(mtype)
            if dt:
                result.setdefault(dt, []).append(name)
    finally:
        conn.close()
    if not result:
        raise RuntimeError('No supported topics found in bag storage.')
    return result


def _open_reader(bag_path, topic=None):
    """Open a bag for reading. Tries rosbag2_py first; if that fails (e.g. empty
    metadata.yaml), falls back to direct SQLite access filtered to `topic`."""
    try:
        return _try_rosbag2_reader(bag_path)
    except Exception:
        if not _find_db3(bag_path):
            raise
        return _SqliteBagReader(bag_path, topic_filter=topic)


def load_imu_data(bag_path, topic, progress_cb=None):
    reader = _open_reader(bag_path, topic)
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
    reader = _open_reader(bag_path, topic)
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
    reader = _open_reader(bag_path, topic)
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


def load_gps_data(bag_path, topic, progress_cb=None):
    """Load mavros_msgs/GPSRAW: lat/lon/alt (position) + yaw/CoG (heading)."""
    reader = _open_reader(bag_path, topic)
    ts, lat, lon, alt, yaw, cog, sat, fix = [], [], [], [], [], [], [], []
    first_ns, count = None, 0
    while reader.has_next():
        tpc, data, t_ns = reader.read_next()
        if tpc != topic:
            continue
        if first_ns is None:
            first_ns = t_ns
        m = deserialize_message(data, GPSRAW)
        ts.append((t_ns - first_ns) * 1e-9)
        # lat/lon in degE7 -> degrees; alt in mm -> metres
        lat.append(m.lat / 1e7)
        lon.append(m.lon / 1e7)
        alt.append(m.alt / 1e3)
        # yaw/cog in centidegrees -> degrees; sentinel UINT16_MAX (65535) => NaN
        yaw.append(m.yaw / 100.0 if m.yaw != 65535 else float('nan'))
        cog.append(m.cog / 100.0 if m.cog != 65535 else float('nan'))
        sat.append(m.satellites_visible if m.satellites_visible != 255 else 0)
        fix.append(m.fix_type)
        count += 1
        if progress_cb and count % 5000 == 0:
            progress_cb(count)
    ts = np.array(ts, dtype=np.float32)
    dur = ts[-1] - ts[0] if len(ts) > 1 else 0
    return {'ts': ts,
            'lat': np.array(lat, np.float32), 'lon': np.array(lon, np.float32),
            'alt': np.array(alt, np.float32), 'yaw': np.array(yaw, np.float32),
            'cog': np.array(cog, np.float32), 'sat': np.array(sat, np.float32),
            'fix': np.array(fix, np.float32),
            'n': count, 'duration': dur,
            'start_dt': datetime.fromtimestamp(first_ns/1e9) if first_ns else None}


DATA_LOADERS = {
    DT_IMU:   load_imu_data,
    DT_POSE:  load_pose_data,
    DT_IMAGE: load_image_data,
    DT_GPS:   load_gps_data,
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

        # GPS overlay (GPS1 vs GPS2) state
        self._overlay_topic = None

        self.setWindowTitle("Sensor Data Viewer")
        self.setMinimumSize(1100, 650)
        self.resize(1500, 880)

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
        splitter.setSizes([300, 1180])
        root.addWidget(splitter)

        self._status = QStatusBar()
        self._status_label = QLabel("Ready - Open a bag (Ctrl+O) or type a path (Ctrl+P)")
        self._status.addWidget(self._status_label)
        self.setStatusBar(self._status)

    def _build_control_panel(self):
        panel = QWidget()
        self._panel_layout = QVBoxLayout(panel)
        self._panel_layout.setContentsMargins(6, 6, 6, 6)
        self._panel_layout.setSpacing(10)

        lbl = QLabel("Data Type"); lbl.setFont(QFont("", 13, QFont.Black))
        self._panel_layout.addWidget(lbl)

        self._type_combo = QComboBox()
        self._type_combo.setMinimumHeight(34)
        self._type_combo.currentTextChanged.connect(self._on_type_changed)
        self._panel_layout.addWidget(self._type_combo)

        lbl2 = QLabel("Topic"); lbl2.setFont(QFont("", 13, QFont.Black))
        self._panel_layout.addWidget(lbl2)

        self._topic_combo = QComboBox()
        self._topic_combo.setMinimumHeight(34)
        self._topic_combo.currentTextChanged.connect(self._on_topic_changed)
        self._panel_layout.addWidget(self._topic_combo)

        # GPS overlay toggle (GPS1 vs GPS2)
        self._gps_overlay_cb = QCheckBox("Overlay GPS1 & GPS2")
        self._gps_overlay_cb.setFont(QFont("", 11, QFont.Bold))
        self._gps_overlay_cb.stateChanged.connect(self._on_overlay_toggled)
        self._gps_overlay_cb.hide()
        self._panel_layout.addWidget(self._gps_overlay_cb)

        line = QFrame(); line.setFrameShape(QFrame.HLine); line.setFrameShadow(QFrame.Sunken)
        self._panel_layout.addWidget(line)

        self._section_label = QLabel("Channels")
        self._section_label.setFont(QFont("", 13, QFont.Black))
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
        nav_l.setContentsMargins(0, 0, 0, 0); nav_l.setSpacing(6)

        self._img_frame_label = QLabel("Frame: 0 / 0")
        self._img_frame_label.setFont(QFont("", 11, QFont.Bold))
        self._img_frame_label.setAlignment(Qt.AlignCenter)
        nav_l.addWidget(self._img_frame_label)

        self._img_slider = QSlider(Qt.Horizontal)
        self._img_slider.setMinimum(0)
        self._img_slider.setMinimumHeight(26)
        self._img_slider.setTracking(False)
        self._img_slider.valueChanged.connect(self._on_img_slider)
        nav_l.addWidget(self._img_slider)

        img_btn_row = QHBoxLayout(); img_btn_row.setSpacing(6)
        for text, slot in [("|<", self._img_first), ("<", self._img_prev),
                           ("Play", self._img_toggle_play), (">", self._img_next),
                           (">|", self._img_last)]:
            b = QPushButton(text); b.setFixedSize(50, 34)
            b.clicked.connect(slot); img_btn_row.addWidget(b)
        nav_l.addLayout(img_btn_row)

        self._img_ts_label = QLabel("t = --")
        self._img_ts_label.setFont(QFont("", 10, QFont.Bold))
        self._img_ts_label.setAlignment(Qt.AlignCenter)
        self._img_ts_label.setStyleSheet("color: #555;")
        nav_l.addWidget(self._img_ts_label)

        self._panel_layout.addWidget(self._img_nav)
        self._img_nav.hide()

        line2 = QFrame(); line2.setFrameShape(QFrame.HLine); line2.setFrameShadow(QFrame.Sunken)
        self._panel_layout.addWidget(line2)

        self._info_label = QLabel("No data loaded")
        self._info_label.setWordWrap(True)
        self._info_label.setFont(QFont("", 10, QFont.Bold))
        self._info_label.setStyleSheet("color: #666;")
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
        self._toolbar.setIconSize(QSize(28, 28))

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
        if data_type == DT_GPS:
            default = next((t for t in topics if 'gps1' in t), default)
        if default:
            self._topic_combo.setCurrentText(default)
        self._topic_combo.blockSignals(False)

        # GPS overlay checkbox: only when there are two GPS topics (gps1 + gps2)
        if data_type == DT_GPS:
            has1 = any('gps1' in t for t in topics)
            has2 = any('gps2' in t for t in topics)
            if has1 and has2:
                self._gps_overlay_cb.show()
            else:
                self._gps_overlay_cb.hide()
                self._gps_overlay_cb.setChecked(False)
        else:
            self._gps_overlay_cb.hide()
            self._gps_overlay_cb.setChecked(False)
        self._overlay_topic = None

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
            grp.setFont(QFont("", 11, QFont.Bold))
            gl = QVBoxLayout(grp)
            for cb_label, key in fields:
                cb = QCheckBox(cb_label)
                cb.setFont(QFont("", 10, QFont.Bold))
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
        # For GPS, refresh the overlay partner before display so it shows together
        if self._current_dt == DT_GPS:
            self._refresh_overlay()
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

    # -- GPS overlay ----------------------------------------------------------

    def _gps_partner_topic(self):
        """Return the complementary GPS topic (gps1<->gps2) if present."""
        topics = self._available.get(DT_GPS, [])
        cur = self._current_topic
        if not cur:
            return None
        if 'gps1' in cur:
            return next((t for t in topics if 'gps2' in t), None)
        if 'gps2' in cur:
            return next((t for t in topics if 'gps1' in t), None)
        return None

    def _refresh_overlay(self):
        """Sync self._overlay_topic with the checkbox state; load partner data
        lazily so it is ready for plotting."""
        if (self._current_dt != DT_GPS
                or not getattr(self, '_gps_overlay_cb', None)
                or not self._gps_overlay_cb.isChecked()):
            self._overlay_topic = None
            return
        partner = self._gps_partner_topic()
        self._overlay_topic = partner
        if partner and partner not in self._data:
            loader = DATA_LOADERS.get(DT_GPS)
            if loader:
                try:
                    self._status_label.setText(f"Loading overlay {partner} ...")
                    QApplication.processEvents()
                    self._data[partner] = loader(self._bag_path, partner)
                except Exception:
                    self._overlay_topic = None

    def _on_overlay_toggled(self):
        self._refresh_overlay()
        self._redraw()

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
                        valid = arr[~np.isnan(arr)] if np.issubdtype(arr.dtype, np.floating) else arr
                        if len(valid) > 0:
                            lines.append(f"  {key}: [{valid.min():.3f}, {valid.max():.3f}]")
                        else:
                            lines.append(f"  {key}: N/A")
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

        # Datasets to plot: list of (data_dict, label_suffix, linestyle).
        # GPS overlay adds the complementary GPS topic (dashed) for comparison.
        datasets = [(data, '', '-')]
        if dt == DT_GPS and self._overlay_topic and self._overlay_topic in self._data:
            od = self._data[self._overlay_topic]
            tag = '2' if 'gps2' in self._overlay_topic else '1'
            datasets.append((od, f' [{tag}]', '--'))

        # Save current view limits so zoom/pan survives checkbox toggles
        saved_lim = None
        if self._axes:
            saved_lim = [(ax.get_xlim(), ax.get_ylim()) for ax in self._axes]

        MAX_PTS = 6000

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
            for d_data, prefix, ls in datasets:
                d_ts = d_data['ts']
                d_stride = max(1, len(d_ts) // MAX_PTS)
                for fi, (_, key) in enumerate(fields):
                    if key in self._cbs and self._cbs[key].isChecked():
                        arr = d_data.get(key)
                        if arr is not None:
                            lbl = f"{key}{prefix}"
                            line, = ax.plot(d_ts[::d_stride], arr[::d_stride],
                                            color=COLORS[fi % 3], linewidth=1.2,
                                            linestyle=ls, alpha=0.9, label=lbl)
                            visible.append(line); labels.append(lbl)
                            self._lod_lines.append((line, d_ts, arr))
            if visible:
                ax.legend(visible, labels, loc='upper right',
                          fontsize=10, ncol=2, framealpha=0.7)

        self._axes[-1].set_xlabel("Time (s)")
        title = self._current_topic or ''
        if dt == DT_GPS and self._overlay_topic:
            tag = '2' if 'gps2' in self._overlay_topic else '1'
            title += f"   (+ GPS{tag} overlay)"
        self._axes[0].set_title(title)

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

APP_STYLE = """
QWidget { font-size: 11pt; }
QLabel { font-weight: 600; }
QPushButton { min-height: 28px; padding: 5px 14px; font-weight: 700; }
QComboBox { min-height: 30px; }
QComboBox QAbstractItemView { font-size: 11pt; }
QCheckBox { font-weight: 600; spacing: 8px; }
QCheckBox::indicator { width: 20px; height: 20px; }
QGroupBox { font-weight: 700; font-size: 12pt; }
QStatusBar { font-weight: 700; font-size: 11pt; }
QStatusBar::item { border: none; }
QMenuBar { font-size: 11pt; font-weight: 600; }
QMenu { font-size: 11pt; }
QSlider { min-height: 22px; }
"""

def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    app.setFont(QFont("Sans Serif", 11))
    app.setStyleSheet(APP_STYLE)
    app.setQuitOnLastWindowClosed(True)

    # Startup diagnostics (printed to the launching terminal)
    print(f"[viewer] rosbag2: {'OK' if HAS_ROSBAG2 else 'MISSING - ' + ROSBAG_ERR}")
    print(f"[viewer] GPS (mavros_msgs/GPSRAW): "
          f"{'OK' if HAS_GPSRAW else 'MISSING - ' + GPSRAW_ERR}")

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
