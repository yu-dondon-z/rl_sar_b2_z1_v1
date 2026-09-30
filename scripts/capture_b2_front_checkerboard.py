#!/usr/bin/env python3
"""Capture B2 built-in front-camera checkerboard images for calibration."""

from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("network_interface")
    parser.add_argument("--unitree-sdk2-python", default="/home/dong/unitree_sdk2_python")
    parser.add_argument("--columns", type=int, default=9, help="checkerboard inner-corner columns")
    parser.add_argument("--rows", type=int, default=6, help="checkerboard inner-corner rows")
    parser.add_argument("--square-size-mm", type=float, default=25.0)
    parser.add_argument("--output-dir", default="")
    parser.add_argument("--preview-scale", type=float, default=0.75)
    return parser.parse_args()


def detect_checkerboard(image, pattern):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # Detect at half resolution for a responsive live preview, then refine on
    # the original JPEG so saved calibration points retain full resolution.
    small = cv2.resize(gray, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(small, pattern, flags)
    if not found:
        return False, None
    corners = corners * 2.0
    criteria = (
        cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER,
        40,
        0.01,
    )
    corners = cv2.cornerSubPix(gray, corners, (7, 7), (-1, -1), criteria)
    return True, corners


def main():
    args = parse_args()
    if args.columns < 2 or args.rows < 2 or args.square_size_mm <= 0.0:
        raise ValueError("columns, rows and square size must be positive")
    sdk_root = str(Path(args.unitree_sdk2_python).expanduser().resolve())
    if sdk_root not in sys.path:
        sys.path.insert(0, sdk_root)

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.b2.front_video.front_video_client import FrontVideoClient

    if args.output_dir:
        output = Path(args.output_dir).expanduser().resolve()
    else:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output = (Path.cwd() / "calibration" / "b2_front" /
                  f"intrinsics_{stamp}").resolve()
    output.mkdir(parents=True, exist_ok=True)

    pattern = (args.columns, args.rows)
    metadata = {
        "camera": "B2 FrontVideoClient",
        "network_interface": args.network_interface,
        "inner_corners": [args.columns, args.rows],
        "square_size_mm": args.square_size_mm,
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "images": [],
    }
    metadata_path = output / "capture_metadata.json"

    ChannelFactoryInitialize(0, args.network_interface)
    client = FrontVideoClient()
    client.SetTimeout(2.0)
    client.Init()

    saved = 0
    last_status = time.monotonic()
    window = "B2 front calibration - SPACE/S save, Q/ESC quit"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    print(f"[CalibrationCapture] output={output}")
    print("[CalibrationCapture] SPACE/S save when corners are green; Q/ESC quit")
    try:
        while True:
            code, encoded = client.GetImageSample()
            if code != 0 or not encoded:
                if time.monotonic() - last_status > 1.0:
                    print(f"[CalibrationCapture] camera read failed: code={code}")
                    last_status = time.monotonic()
                continue
            image = cv2.imdecode(
                np.frombuffer(bytes(encoded), dtype=np.uint8), cv2.IMREAD_COLOR)
            if image is None:
                continue
            found, corners = detect_checkerboard(image, pattern)
            preview = image.copy()
            if found:
                cv2.drawChessboardCorners(preview, pattern, corners, True)
            color = (20, 220, 20) if found else (20, 20, 240)
            status = "DETECTED - press SPACE/S" if found else "NOT DETECTED"
            cv2.putText(preview, status, (20, 42), cv2.FONT_HERSHEY_SIMPLEX,
                        1.0, color, 2, cv2.LINE_AA)
            cv2.putText(
                        preview,
                        (f"saved {saved}/30   {args.columns}x{args.rows} inner corners"
                         f"   {args.square_size_mm:g} mm"),
                        (20, 82), cv2.FONT_HERSHEY_SIMPLEX, 0.85,
                        (255, 255, 255), 2, cv2.LINE_AA)
            if args.preview_scale != 1.0:
                preview = cv2.resize(preview, None,
                                     fx=args.preview_scale,
                                     fy=args.preview_scale,
                                     interpolation=cv2.INTER_AREA)
            cv2.imshow(window, preview)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q"), ord("Q")):
                break
            if key in (32, ord("s"), ord("S")):
                if not found:
                    print("[CalibrationCapture] not saved: all 9x6 corners must be detected")
                    continue
                filename = f"calib_{saved:03d}.jpg"
                target = output / filename
                if not cv2.imwrite(str(target), image,
                                   [cv2.IMWRITE_JPEG_QUALITY, 98]):
                    print(f"[CalibrationCapture] failed to write {target}")
                    continue
                saved += 1
                metadata["resolution"] = [int(image.shape[1]), int(image.shape[0])]
                metadata["images"].append(filename)
                metadata_path.write_text(
                    json.dumps(metadata, indent=2), encoding="utf-8")
                print(f"[CalibrationCapture] saved {saved}: {target}")
    finally:
        metadata_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
        cv2.destroyAllWindows()
        print(f"[CalibrationCapture] finished; saved={saved}, output={output}")


if __name__ == "__main__":
    main()
