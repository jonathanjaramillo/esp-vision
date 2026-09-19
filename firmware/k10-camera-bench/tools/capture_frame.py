#!/usr/bin/env python3
"""Capture the benchmark's post-result --FOV base64 frame as a PNG."""

import argparse
import base64
import re
import time

import serial
from PIL import Image

HEADER = re.compile(r"--FOV (\S+) (\d+) (\d+) (\d+)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--port", required=True)
    ap.add_argument("-o", "--output", required=True)
    ap.add_argument("-t", "--timeout", type=float, default=150)
    args = ap.parse_args()
    with serial.Serial(args.port, 115200, timeout=.25) as ser:
        end = time.time() + args.timeout
        meta = None
        encoded = []
        while time.time() < end:
            line = ser.readline().decode("ascii", "replace").strip()
            match = HEADER.fullmatch(line)
            if match:
                meta = (match.group(1), int(match.group(2)), int(match.group(3)),
                        int(match.group(4)))
                encoded = []
            elif meta and line == "--END":
                break
            elif meta and re.fullmatch(r"[A-Za-z0-9+/]+={0,2}", line):
                encoded.append(line)
        if not meta:
            raise SystemExit("timed out before --FOV header")
    raw = base64.b64decode("".join(encoded))
    tag, w, h, expected = meta
    if len(raw) != expected:
        raise SystemExit(f"truncated frame: {len(raw)} of {expected} bytes")
    rgb = bytearray(w * h * 3)
    for i in range(w * h):
        value = raw[2*i] | raw[2*i+1] << 8
        r, g, b = (value >> 11) & 31, (value >> 5) & 63, value & 31
        rgb[3*i:3*i+3] = bytes(((r << 3) | (r >> 2),
                                (g << 2) | (g >> 4),
                                (b << 3) | (b >> 2)))
    Image.frombytes("RGB", (w, h), bytes(rgb)).save(args.output)
    print(f"wrote {args.output}: {tag} {w}x{h} {len(raw)} bytes")


if __name__ == "__main__":
    main()
