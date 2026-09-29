#!/usr/bin/env python3
"""Camera calibration for the K10's GC2145 camera -- a prerequisite for
vo.py, which needs an intrinsics matrix K for findEssentialMat/solvePnP.

Calibration is per GC2145_FOV_MODE: each mode selects a different sensor
window/crop (see ../../lib/gc2145wide/gc2145_wide.cpp and
../../firmware/k10-fast-corners/platformio.ini's per-mode envs), so K
differs by mode even though the *streamed* ORB coordinate space (120x160)
doesn't -- see DET_W/DET_H's note in firmware/k10-fast-corners/src/main.cpp.
Flash the board with the env you're calibrating for (e.g. `pio run -e
wide_fov -t upload` for FOV_MODE=1) before running this, and pass the same
--fov-mode so the output filename/field matches. Use that same env when
actually running VO afterwards -- a mismatched K will silently produce bad
poses, not an error.

Reuses the firmware's existing 'D' raw-RGB565 serial dump command and
../../firmware/k10-fast-corners/tools/capture.py's decoder -- no new
firmware needed. Print a checkerboard (default 9x6 internal corners, 25mm
squares -- both configurable) and hold it in view of the K10's camera at
varied angles/distances/positions for each capture.

Usage:
    python3 calibrate.py                          # FOV_MODE=0, 15 frames
    python3 calibrate.py --fov-mode 1              # -> calib_fov1.json
    python3 calibrate.py -p /dev/cu.usbmodem1101 --count 20
    python3 calibrate.py --cols 7 --rows 5 --square-mm 20

Writes calib_fov<N>.json: K (3x3, at the captured 240x320 resolution),
distortion coefficients, reprojection error, and stream_scale=0.5 -- ORBF
features are streamed in the half-res 120x160 detection frame regardless of
FOV mode (see firmware/k10-fast-corners/src/main.cpp's DET_W/DET_H), and
vo.CameraIntrinsics.load() applies this scale when loading K for VO.
"""

import argparse
import json
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                 "..", "..", "firmware", "k10-fast-corners", "tools"))
import capture as k10capture  # noqa: E402  (reuse capture()/find_port())


def _rgb565_to_gray(data, w, h):
    # Camera bytes are swapped vs. CPU uint16_t (see capture.py's own
    # RGB565 decode and k10image's byte-swap fix) -- reading as big-endian
    # u16 undoes that in one step.
    px = np.frombuffer(data, dtype=">u2").reshape(h, w)
    r = ((px >> 11) & 0x1F).astype(np.float32) * (255.0 / 31)
    g = ((px >> 5) & 0x3F).astype(np.float32) * (255.0 / 63)
    b = (px & 0x1F).astype(np.float32) * (255.0 / 31)
    return (0.299 * r + 0.587 * g + 0.114 * b).astype(np.uint8)


def grab_checkerboard_frames(port, count, cols, rows, delay):
    objp = np.zeros((cols * rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)

    pts_obj, pts_img = [], []
    w = h = None
    got = 0
    while got < count:
        input("place the checkerboard (frame %d/%d), then press Enter... "
              % (got + 1, count))
        head, data = k10capture.capture(port, "D", 30.0)
        tag, w, h = head[1], int(head[2]), int(head[3])
        if tag != "RGB565":
            print("unexpected dump tag %s, skipping" % tag, file=sys.stderr)
            continue
        gray = _rgb565_to_gray(data, w, h)
        found, corners = cv2.findChessboardCorners(gray, (cols, rows))
        if not found:
            print("checkerboard not found -- reposition and retry", file=sys.stderr)
            continue
        corners = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        pts_obj.append(objp)
        pts_img.append(corners)
        got += 1
        print("captured %d/%d" % (got, count))
        time.sleep(delay)
    return pts_obj, pts_img, w, h


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("-p", "--port", default=None)
    ap.add_argument("--count", type=int, default=15)
    ap.add_argument("--cols", type=int, default=9,
                     help="checkerboard internal corners, columns")
    ap.add_argument("--rows", type=int, default=6,
                     help="checkerboard internal corners, rows")
    ap.add_argument("--square-mm", type=float, default=25.0)
    ap.add_argument("--delay", type=float, default=0.5,
                     help="pause after each capture (let the board settle)")
    ap.add_argument("--fov-mode", type=int, default=0, choices=(0, 1, 2, 3),
                     help="GC2145_FOV_MODE the board is currently flashed "
                          "with -- must match, see the module docstring")
    ap.add_argument("-o", "--out", default=None,
                     help="default: calib_fov<fov-mode>.json")
    a = ap.parse_args()
    out_path = a.out or ("calib_fov%d.json" % a.fov_mode)

    port = a.port or k10capture.find_port()
    pts_obj, pts_img, w, h = grab_checkerboard_frames(
        port, a.count, a.cols, a.rows, a.delay)
    if not pts_obj or w is None or h is None:
        sys.exit("no checkerboard frames captured")
    pts_obj = [p * a.square_mm for p in pts_obj]

    rms, K, dist, _rvecs, _tvecs = cv2.calibrateCamera(
        pts_obj, pts_img, (w, h), None, None)
    print("reprojection RMS error: %.3f px" % rms)

    out = {
        "fov_mode": a.fov_mode,
        "calib_w": w, "calib_h": h,
        "K": K.tolist(),
        "dist": dist.ravel().tolist(),
        "reproj_error_px": rms,
        "stream_scale": 0.5,
        "n_frames": len(pts_obj),
    }
    with open(out_path, "w") as f:
        json.dump(out, f, indent=2)
    print("wrote", out_path)
    if rms > 0.8:
        print("warning: reprojection error is high (>0.8px) -- consider "
              "recapturing with more varied angles/distances/positions",
              file=sys.stderr)


if __name__ == "__main__":
    main()
