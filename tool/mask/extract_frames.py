#!/usr/bin/env python3
"""
Extract frames from /odin1/image/compressed in a ROS2 bag to PNG files.
Used to prepare images for the Mask Editor to draw D10R masks.

Usage:
    ./run_viewer.sh  (then select Image -> /odin1/image/compressed)
    # or use this extractor:
    source /opt/ros/humble/setup.bash && python3 extract_frames.py [bag_path] [output_dir]

Default: extracts 1 frame per 100 (about 20 frames total for D10R)
"""

import sys, os, io
import numpy as np
from PIL import Image

ROS_HUMBLE = '/opt/ros/humble'
_site_pkg = os.path.join(ROS_HUMBLE, 'local/lib/python3.10/dist-packages')
if _site_pkg not in sys.path:
    sys.path.insert(0, _site_pkg)
os.environ.setdefault('LD_LIBRARY_PATH',
    os.path.join(ROS_HUMBLE, 'lib') + ':' + os.environ.get('LD_LIBRARY_PATH', ''))
os.environ.setdefault('AMENT_PREFIX_PATH', ROS_HUMBLE)

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from sensor_msgs.msg import CompressedImage
from rclpy.serialization import deserialize_message

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_BAG = os.path.join(SCRIPT_DIR, 'odin_bag', 'D10R', '20260714160813')
DEFAULT_OUT = os.path.join(SCRIPT_DIR, 'mask', 'd10r_frames')
STRIDE = 100  # extract every Nth frame


def main():
    bag_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_BAG
    out_dir  = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_OUT

    if not os.path.isdir(bag_path):
        print(f"ERROR: bag not found: {bag_path}")
        sys.exit(1)

    os.makedirs(out_dir, exist_ok=True)

    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag_path, storage_id='sqlite3'),
                ConverterOptions(input_serialization_format='cdr',
                                 output_serialization_format='cdr'))

    count, saved = 0, 0
    first_ns = None

    print(f"Extracting frames to: {out_dir}")
    while reader.has_next():
        tpc, data, t_ns = reader.read_next()
        if tpc != '/odin1/image/compressed':
            continue
        if count % STRIDE == 0:
            if first_ns is None:
                first_ns = t_ns
            rel_t = (t_ns - first_ns) * 1e-9
            msg = deserialize_message(data, CompressedImage)
            img = Image.open(io.BytesIO(bytes(msg.data)))
            fname = os.path.join(out_dir, f"frame_{saved:04d}_t{rel_t:.3f}s.png")
            img.save(fname)
            saved += 1
            print(f"  [{saved:3d}] {fname}")
        count += 1

    print(f"\nDone. {saved} frames extracted (every {STRIDE}th of {count} total).")


if __name__ == '__main__':
    main()
