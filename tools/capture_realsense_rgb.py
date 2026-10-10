#!/usr/bin/env python3
"""Save a RealSense RGB frame by its SDK camera serial, without robot control."""

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs


DEFAULT_SERIAL = "406122071837"


def available_cameras():
    cameras = {}
    for device in rs.context().devices:
        serial = device.get_info(rs.camera_info.serial_number)
        asic = device.get_info(rs.camera_info.asic_serial_number) if device.supports(rs.camera_info.asic_serial_number) else "unknown"
        port = device.get_info(rs.camera_info.physical_port) if device.supports(rs.camera_info.physical_port) else "unknown"
        cameras[serial] = (asic, port)
    return dict(sorted(cameras.items()))


def capture(serial, output_dir, width, height):
    config = rs.config()
    config.enable_device(serial)
    config.enable_stream(rs.stream.color, width, height, rs.format.rgb8, 30)
    pipeline = rs.pipeline()
    pipeline.start(config)
    try:
        color_frame = None
        for _ in range(20):  # Give automatic exposure time to settle.
            frames = pipeline.wait_for_frames(5000)
            color_frame = frames.get_color_frame()
        if not color_frame:
            raise RuntimeError("No RGB frame received")
        bgr = cv2.cvtColor(np.asanyarray(color_frame.get_data()), cv2.COLOR_RGB2BGR)
        output_dir.mkdir(parents=True, exist_ok=True)
        filename = output_dir / "realsense_{}_rgb.png".format(serial)
        if not cv2.imwrite(str(filename), bgr):
            raise RuntimeError("Could not write {}".format(filename))
        print("{} -> {} ({}x{})".format(serial, filename, bgr.shape[1], bgr.shape[0]))
    finally:
        pipeline.stop()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    choice = parser.add_mutually_exclusive_group()
    choice.add_argument("--serial", default=DEFAULT_SERIAL, help="RealSense SDK camera serial")
    choice.add_argument("--all-available", action="store_true")
    parser.add_argument("--output-dir", type=Path, default=Path("/tmp/ur10e-rgb-sdk-check"))
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    args = parser.parse_args()

    cameras = available_cameras()
    for serial, (asic, port) in cameras.items():
        print("SDK serial: {}; ASIC/USB serial: {}; port: {}".format(serial, asic, port))
    targets = list(cameras) if args.all_available else [args.serial]
    if not args.all_available and args.serial not in cameras:
        print("Requested SDK camera {} is not visible".format(args.serial), file=sys.stderr)
        return 2
    for serial in targets:
        try:
            capture(serial, args.output_dir, args.width, args.height)
        except Exception as error:
            print("{}: {}".format(serial, error), file=sys.stderr)
            return 1
    return 0 if targets else 2


if __name__ == "__main__":
    sys.exit(main())
