#!/usr/bin/env python3
"""Build, flash, and collect one RESULT block for every benchmark mode."""

import argparse
import glob
import re
import subprocess
import time

import serial

ENVS = ("stock_qvga", "wide_qvga", "vga_software")


def find_port():
    hits = sorted(glob.glob("/dev/cu.usbmodem*"))
    if not hits:
        raise SystemExit("No /dev/cu.usbmodem* device found")
    return hits[0]


def capture(port, timeout=100):
    lines = []
    inside = False
    with serial.Serial(port, 115200, timeout=0.25) as ser:
        end = time.time() + timeout
        while time.time() < end:
            line = ser.readline().decode("utf-8", "replace").strip()
            if line == "RESULT_BEGIN":
                inside = True
                lines = []
            elif line == "RESULT_END" and inside:
                return lines
            elif inside:
                lines.append(re.sub(r"\x1b\[[0-9;]*m", "", line))
    raise RuntimeError("timed out waiting for RESULT_END")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--port", default=None)
    ap.add_argument("--pio", default="pio")
    args = ap.parse_args()
    port = args.port or find_port()

    print("environment,results")
    for env in ENVS:
        subprocess.run(
            [args.pio, "run", "-e", env, "-t", "upload", "--upload-port", port],
            check=True,
        )
        block = capture(port)
        print(env + "," + " | ".join(block))


if __name__ == "__main__":
    main()
